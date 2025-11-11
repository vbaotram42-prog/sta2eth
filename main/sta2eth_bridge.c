/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/**
 * sta2eth L2 Bridge with Dynamic MAC Learning
 * 
 * Implementation for single PC scenario:
 * 1. Initialize Ethernet and wait for link
 * 2. Capture first packet to learn PC MAC
 * 3. Initialize WiFi with PC MAC
 * 4. Connect WiFi with static IP
 * 5. Create bridge after WiFi connects
 * 6. Enable transparent L2 forwarding
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_br_glue.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_remote.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ethernet_init.h"
#include "wifi_config_portal.h"
#include "c6_ota.h"

static const char *TAG = "sta2eth";

// Event flags
static EventGroupHandle_t s_event_flags;
#define ETH_LINK_UP_BIT      BIT0
#define MAC_LEARNED_BIT      BIT1
#define WIFI_CONNECTED_BIT   BIT2
#define WIFI_DISCONNECTED_BIT BIT3
#define BRIDGE_READY_BIT     BIT4
#define PROV_SUCCESS_BIT     BIT5
#define PROV_FAIL_BIT        BIT6
#define RECONFIGURE_BIT      BIT7
#define WIFI_FAIL_BIT        BIT8

// WiFi connection retry configuration
#define WIFI_MAXIMUM_RETRY  5
static int s_wifi_retry_num = 0;

// Global state
static esp_netif_t *s_eth_netif = NULL;
static esp_netif_t *s_wifi_netif = NULL;
static esp_netif_t *s_br_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static uint8_t s_pc_mac[6] = {0};
static bool s_mac_learned = false;

// Link down timer - restarts device after prolonged cable disconnection (3+ seconds)
// This distinguishes between brief network glitches vs. intentional cable removal
static esp_timer_handle_t s_link_down_timer = NULL;
#define LINK_DOWN_RESET_TIMEOUT_SEC 3

/**
 * Timer callback for prolonged Ethernet link down
 * 
 * This callback is triggered when Ethernet link stays down for 3+ seconds,
 * indicating the cable was physically removed (not just a brief network glitch).
 * 
 * Action: Restart device to re-initialize and re-learn PC MAC address
 */
static void link_down_timer_callback(void *arg)
{
    ESP_LOGW(TAG, "Ethernet link down for %d seconds - cable appears to be disconnected", 
             LINK_DOWN_RESET_TIMEOUT_SEC);
    ESP_LOGW(TAG, "Restarting device to re-learn PC MAC address on next cable reconnection");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/**
 * Ethernet packet receive callback for MAC learning (ONE-TIME ONLY)
 * This callback is removed immediately after learning the first packet
 */
static esp_err_t eth_packet_receive_cb(esp_eth_handle_t hdl, uint8_t *buffer, uint32_t length, void *priv)
{
    if (!s_mac_learned && length >= 14) {
        // Extract source MAC from Ethernet frame (bytes 6-11)
        memcpy(s_pc_mac, buffer + 6, 6);
        s_mac_learned = true;
        
        ESP_LOGI(TAG, "===========================================");
        ESP_LOGI(TAG, "PC MAC Address Learned!");
        ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
                 s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
        ESP_LOGI(TAG, "===========================================");
        
        // CRITICAL: Signal that MAC is learned so callback can be removed
        xEventGroupSetBits(s_event_flags, MAC_LEARNED_BIT);
    }
    
    // Free the buffer - we're just learning MAC, not forwarding yet
    free(buffer);
    return ESP_OK;
}

/**
 * Ethernet event handler
 */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
        
        // Stop link down timer if it was running
        // Cable was replugged within 3 seconds - just a brief glitch, no restart needed
        if (s_link_down_timer) {
            esp_timer_stop(s_link_down_timer);
        }
        
        // CRITICAL: Stop DHCP client on link up to prevent "dhcp client start failed" error
        // When Ethernet link comes back up (e.g., cable replugged), netif layer
        // automatically tries to start DHCP client. But bridge ports should NOT run DHCP.
        // Only the bridge interface itself runs DHCP client to get IP for the entire bridge.
        if (s_eth_netif) {
            esp_netif_dhcpc_stop(s_eth_netif);
        }
        xEventGroupSetBits(s_event_flags, ETH_LINK_UP_BIT);
        break;
        
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");
        xEventGroupClearBits(s_event_flags, ETH_LINK_UP_BIT);
        
        // Start timer: if link stays down for 3+ seconds, restart device
        // This distinguishes brief network glitches from intentional cable removal
        // On restart, device will re-learn PC MAC from first packet
        if (s_link_down_timer && s_mac_learned) {
            ESP_LOGI(TAG, "Starting %ds timer - will restart if link stays down", 
                     LINK_DOWN_RESET_TIMEOUT_SEC);
            esp_timer_start_once(s_link_down_timer, LINK_DOWN_RESET_TIMEOUT_SEC * 1000000ULL);
        }
        break;
        
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/**
 * WiFi event handler
 * Following ESP-IDF standard pattern with retry limit
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_REMOTE_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started, initiating connection...");
            esp_wifi_remote_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi Connected to AP");
            s_wifi_retry_num = 0;  // Reset retry counter on successful connection
            xEventGroupSetBits(s_event_flags, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_event_flags, WIFI_DISCONNECTED_BIT);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            // CRITICAL: Re-set MAC address before reconnection
            // MAC is stored in C6's RAM (WIFI_STORAGE_RAM) and may be lost on disconnect/reset
            // This ensures PC MAC is maintained across reconnections
            if (s_mac_learned) {
                ESP_LOGI(TAG, "Re-setting WiFi MAC to PC MAC before reconnect: %02x:%02x:%02x:%02x:%02x:%02x",
                         s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
                         s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
                esp_wifi_set_mac(WIFI_IF_STA, s_pc_mac);
            }
            
            if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
                esp_wifi_remote_connect();
                s_wifi_retry_num++;
                ESP_LOGI(TAG, "Retry to connect to AP (attempt %d/%d)", s_wifi_retry_num, WIFI_MAXIMUM_RETRY);
            } else {
                xEventGroupSetBits(s_event_flags, WIFI_FAIL_BIT);
                ESP_LOGW(TAG, "Failed to connect after %d attempts", WIFI_MAXIMUM_RETRY);
            }
            xEventGroupClearBits(s_event_flags, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_event_flags, WIFI_DISCONNECTED_BIT);
            break;
        default:
            break;
        }
    }
}

/**
 * IP event handler (unused in pure L2 bridge mode)
 * 
 * Note: In this L2 bridge implementation, the bridge interface does not need
 * an IP address. The PC connected via Ethernet will obtain its IP directly 
 * from the router through transparent bridging.
 */
/*
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    // Not used in current L2 bridging implementation
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        const esp_netif_ip_info_t *ip_info = &event->ip_info;
        
        ESP_LOGI(TAG, "~~~~~~~~~~~");
        ESP_LOGI(TAG, "Bridge Got IP Address");
        ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&ip_info->ip));
        ESP_LOGI(TAG, "MASK:" IPSTR, IP2STR(&ip_info->netmask));
        ESP_LOGI(TAG, "GW:" IPSTR, IP2STR(&ip_info->gw));
        ESP_LOGI(TAG, "~~~~~~~~~~~");
    }
}
*/

/**
 * Step 1: Initialize Ethernet with IP101 PHY
 */
static esp_err_t init_ethernet(void)
{
    ESP_LOGI(TAG, "Step 1: Initializing Ethernet (IP101 PHY)...");
    
    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create Ethernet netif (will be added to bridge later)
    esp_netif_inherent_config_t eth_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    eth_cfg.flags = 0;  // No flags for bridged port
    esp_netif_config_t netif_cfg = {
        .base = &eth_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    s_eth_netif = esp_netif_new(&netif_cfg);
    
    // Initialize Ethernet driver
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(ethernet_init_all(&eth_handles, &eth_port_cnt));
    
    if (eth_port_cnt == 0) {
        ESP_LOGE(TAG, "No Ethernet interface found!");
        return ESP_FAIL;
    }
    
    s_eth_handle = eth_handles[0];
    free(eth_handles);
    
    // Attach Ethernet driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
    
    // Assign static link-local IP to Ethernet
    esp_netif_dhcpc_stop(s_eth_netif);
    esp_netif_ip_info_t eth_ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(169, 254, 0, 3) },
        .gw = { .addr = ESP_IP4TOADDR(169, 254, 0, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
    };
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &eth_ip_info));
    
    // Create link down timer - restarts device after prolonged disconnection
    // This ensures proper re-initialization when cable is removed and replugged
    const esp_timer_create_args_t timer_args = {
        .callback = &link_down_timer_callback,
        .name = "link_down_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_link_down_timer));
    
    // Enable promiscuous mode for packet capture
    bool promiscuous = true;
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous));
    ESP_LOGI(TAG, "Ethernet promiscuous mode enabled for MAC learning");
    
    // Register packet receive callback for MAC learning (will be removed after first packet)
    ESP_ERROR_CHECK(esp_eth_update_input_path(s_eth_handle, eth_packet_receive_cb, NULL));
    ESP_LOGI(TAG, "MAC learning callback registered (one-time use)");
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    
    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    
    ESP_LOGI(TAG, "Ethernet initialized successfully");
    return ESP_OK;
}

/**
 * Step 2: Wait for PC MAC learning and cleanup Ethernet
 */
static esp_err_t wait_for_pc_mac_and_cleanup(void)
{
    ESP_LOGI(TAG, "Step 2: Waiting for Ethernet link and PC packet...");
    
    // Wait for Ethernet link up
    ESP_LOGI(TAG, "Waiting for Ethernet link up...");
    xEventGroupWaitBits(s_event_flags, ETH_LINK_UP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Ethernet link is up!");
    
    // Wait for MAC learning from first packet
    ESP_LOGI(TAG, "Waiting for first packet from PC to learn MAC address...");
    ESP_LOGI(TAG, "(Please ensure PC is connected and sending traffic)");
    xEventGroupWaitBits(s_event_flags, MAC_LEARNED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    
    ESP_LOGI(TAG, "PC MAC learned successfully!");
    
    // CRITICAL: Completely deinitialize Ethernet to ensure clean state for bridge
    // This ensures both netifs are absolutely clean when bridge is created
    ESP_LOGI(TAG, "Deinitializing Ethernet to ensure clean state for bridge...");
    
    // Stop link down timer if it's running (could have started if link went down briefly)
    if (s_link_down_timer) {
        esp_timer_stop(s_link_down_timer);
    }
    
    // CRITICAL: Unregister event handlers BEFORE stopping Ethernet
    // This prevents the LINK_DOWN event from esp_eth_stop() from starting the timer
    ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler));
    ESP_LOGI(TAG, "Ethernet event handlers unregistered");
    
    // Stop Ethernet (will trigger LINK_DOWN event, but handler is already unregistered)
    ESP_ERROR_CHECK(esp_eth_stop(s_eth_handle));
    ESP_LOGI(TAG, "Ethernet stopped");
    
    // Destroy Ethernet netif (will be recreated clean for bridge)
    esp_netif_destroy(s_eth_netif);
    s_eth_netif = NULL;
    ESP_LOGI(TAG, "Ethernet netif destroyed");
    
    // Driver and timer remain valid for re-initialization
    ESP_LOGI(TAG, "Ethernet deinitialization complete - ready for clean bridge init");
    
    return ESP_OK;
}

/**
 * Step 3: Initialize WiFi with PC MAC
 */
static esp_err_t init_wifi_with_pc_mac(void)
{
    ESP_LOGI(TAG, "Step 3: Initializing WiFi with PC MAC...");
    
    // Initialize WiFi Remote
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_remote_init(&cfg));
    
    // CRITICAL: Set WiFi storage to RAM only
    // Credentials are stored on P4's NVS, never on C6's flash
    ESP_LOGI(TAG, "Setting WiFi storage to RAM only (C6 won't save credentials)");
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // Set WiFi mode to STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // CRITICAL: Set WiFi STA MAC = PC MAC
    ESP_LOGI(TAG, "Setting WiFi STA MAC to PC MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, s_pc_mac));
    
    // Create WiFi STA netif with static IP
    esp_netif_inherent_config_t wifi_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    wifi_cfg.flags = 0;  // No flags for bridged port
    s_wifi_netif = esp_netif_create_wifi(WIFI_IF_STA, &wifi_cfg);
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    
    // Assign static link-local IP to WiFi STA
    esp_netif_dhcpc_stop(s_wifi_netif);
    esp_netif_ip_info_t wifi_ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(169, 254, 0, 2) },
        .gw = { .addr = ESP_IP4TOADDR(169, 254, 0, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
    };
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_wifi_netif, &wifi_ip_info));
    ESP_LOGI(TAG, "WiFi STA assigned static IP: 169.254.0.2");
    
    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_REMOTE_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized with PC MAC");
    return ESP_OK;
}

/**
 * Step 4: Configure and connect WiFi to AP
 * 
 * Following ESP-IDF standard WiFi station pattern:
 * - Load credentials from P4's NVS
 * - Set WiFi configuration
 * - Connection is initiated automatically by WIFI_EVENT_STA_START event
 * - Wait for connection result
 */
static esp_err_t connect_wifi(void)
{
    ESP_LOGI(TAG, "Step 4: Configuring WiFi connection...");
    
    // Load credentials from P4's NVS
    char ssid[33] = {0};
    char password[65] = {0};
    esp_err_t err = load_wifi_credentials(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load WiFi credentials: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "WiFi credentials loaded: SSID=%s", ssid);
    
    // Set WiFi configuration (credentials passed to C6's RAM)
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(err));
        return err;
    }
    
    // Note: Connection will be initiated by WIFI_EVENT_STA_START event handler
    // This follows the standard ESP-IDF WiFi station pattern
    
    // Wait for connection result (success or failure after retries)
    // Following ESP-IDF example: wait for either WIFI_CONNECTED_BIT or WIFI_FAIL_BIT
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(s_event_flags, 
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully!");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi after %d retries", WIFI_MAXIMUM_RETRY);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Unexpected WiFi connection result");
        return ESP_FAIL;
    }
}

/**
 * Reinitialize Ethernet cleanly for bridge
 * Called after WiFi is connected to ensure Ethernet netif is in clean state
 */
static esp_err_t reinit_ethernet_for_bridge(void)
{
    ESP_LOGI(TAG, "Re-initializing Ethernet in clean state for bridge...");
    
    // Create clean Ethernet netif for bridge
    esp_netif_inherent_config_t eth_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    eth_cfg.flags = 0;  // No flags for bridged port
    esp_netif_config_t netif_cfg = {
        .base = &eth_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    s_eth_netif = esp_netif_new(&netif_cfg);
    
    // Attach Ethernet driver to netif (driver handle is still valid from initial setup)
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
    
    // Assign static link-local IP to Ethernet
    esp_netif_dhcpc_stop(s_eth_netif);
    esp_netif_ip_info_t eth_ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(169, 254, 0, 3) },
        .gw = { .addr = ESP_IP4TOADDR(169, 254, 0, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
    };
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &eth_ip_info));
    
    // Re-enable promiscuous mode (needed for bridge)
    bool promiscuous = true;
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous));
    
    // Re-register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    
    ESP_LOGI(TAG, "Ethernet re-initialized successfully in clean state");
    ESP_LOGI(TAG, "Note: Ethernet will be started after being added to bridge");
    return ESP_OK;
}

/**
 * Step 5: Create bridge
 */
static esp_err_t create_bridge(void)
{
    ESP_LOGI(TAG, "Step 5: Creating L2 bridge...");
    
    // Create bridge netif configuration
    esp_netif_inherent_config_t br_cfg = ESP_NETIF_INHERENT_DEFAULT_BR();
    esp_netif_config_t br_netif_cfg = {
        .base = &br_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_BR,
    };
    
    // Bridge configuration
    bridgeif_config_t bridgeif_config = {
        .max_fdb_dyn_entries = 10,
        .max_fdb_sta_entries = 2,
        .max_ports = 2
    };
    br_cfg.bridge_info = &bridgeif_config;
    
    // Set bridge MAC to PC MAC (for consistency)
    memcpy(br_cfg.mac, s_pc_mac, 6);
    s_br_netif = esp_netif_new(&br_netif_cfg);
    
    // Create bridge glue and add ports
    esp_netif_br_glue_handle_t br_glue = esp_netif_br_glue_new();
    
    // Add Ethernet port
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_eth_netif));
    ESP_LOGI(TAG, "Added Ethernet port to bridge");
    
    // Add WiFi STA port
    // Note: Use generic add_port for WiFi Remote instead of add_wifi_port
    // WiFi Remote over SDIO doesn't support the specialized add_wifi_port function
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_wifi_netif));
    ESP_LOGI(TAG, "Added WiFi STA port to bridge");
    
    // Attach bridge glue to bridge netif
    ESP_ERROR_CHECK(esp_netif_attach(s_br_netif, br_glue));
    
    // Now that bridge event handlers are registered, start Ethernet
    // This ensures bridge glue's port_action_start() will receive ETHERNET_EVENT_START
    // and properly redirect netif input to the bridge
    ESP_LOGI(TAG, "Starting Ethernet driver with bridge glue registered...");
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    ESP_LOGI(TAG, "Ethernet started - bridge will intercept ETHERNET_EVENT_START");
    
    // Note: Bridge operates at L2, no IP event handling needed
    // PC will obtain IP directly from router via transparent bridging
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Bridge Created Successfully!");
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  PC MAC:     %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ESP_LOGI(TAG, "  WiFi MAC:   %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ESP_LOGI(TAG, "  Bridge MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ESP_LOGI(TAG, "===========================================");
    
    ESP_LOGI(TAG, "Transparent L2 bridging now active!");
    ESP_LOGI(TAG, "PC should be able to get DHCP and access network");
    
    xEventGroupSetBits(s_event_flags, BRIDGE_READY_BIT);
    return ESP_OK;
}

/**
 * Reconfigure button handler task
 */
// Reconfigure button GPIO (Boot button)
#define RECONFIGURE_BUTTON_GPIO 2

static void reconfigure_button_task(void *arg)
{
    // Configure GPIO for reconfigure button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RECONFIGURE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Reconfigure button monitoring started (GPIO%d)", RECONFIGURE_BUTTON_GPIO);
    ESP_LOGI(TAG, "Long press (2 seconds) to reset WiFi credentials");
    
    uint32_t press_start = 0;
    bool button_pressed = false;
    
    while (1) {
        int level = gpio_get_level(RECONFIGURE_BUTTON_GPIO);
        
        if (level == 0 && !button_pressed) {
            // Button pressed
            button_pressed = true;
            press_start = esp_timer_get_time() / 1000;  // Convert to ms
        } else if (level == 1 && button_pressed) {
            // Button released
            button_pressed = false;
            uint32_t press_duration = (esp_timer_get_time() / 1000) - press_start;
            
            if (press_duration >= 2000) {
                ESP_LOGW(TAG, "Reconfigure button long-pressed! Clearing WiFi credentials...");
                clear_wifi_credentials();
                ESP_LOGW(TAG, "WiFi credentials cleared. Restarting...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "STA2ETH L2 Bridge with Dynamic MAC Learning");
    ESP_LOGI(TAG, "===========================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create event group
    s_event_flags = xEventGroupCreate();
    
    // ========================================================================
    // CRITICAL: C6 OTA Check MUST happen FIRST before any other functionality
    // ========================================================================
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Step 0: Checking C6 firmware status...");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "C6 firmware check will allow adequate time for initialization");
    ESP_LOGI(TAG, "");
    
    if (c6_ota_should_enter_mode()) {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "===========================================");
        ESP_LOGW(TAG, "  C6 FIRMWARE UPDATE REQUIRED!");
        ESP_LOGW(TAG, "===========================================");
        ESP_LOGW(TAG, "C6 is missing, not responding, or version mismatch");
        ESP_LOGW(TAG, "Entering standalone C6 OTA mode...");
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "Instructions:");
        ESP_LOGW(TAG, "  1. Connect PC to Ethernet port");
        ESP_LOGW(TAG, "  2. PC will auto-get IP via DHCP (192.168.100.x)");
        ESP_LOGW(TAG, "  3. Visit http://192.168.100.1 in browser");
        ESP_LOGW(TAG, "  4. Upload C6 firmware (.bin file)");
        ESP_LOGW(TAG, "  5. System will restart after successful update");
        ESP_LOGW(TAG, "===========================================");
        ESP_LOGW(TAG, "");
        
        // Enter OTA mode - this blocks until firmware is updated
        // PC can connect via Ethernet and get IP automatically
        ESP_ERROR_CHECK(c6_ota_start_mode());
        
        // Should not reach here - c6_ota_start_mode restarts after update
        ESP_LOGE(TAG, "C6 OTA mode exited unexpectedly!");
        esp_restart();
    }
    
    ESP_LOGI(TAG, "✓ C6 firmware check passed - version compatible");
    ESP_LOGI(TAG, "✓ C6 is present and working");
    ESP_LOGI(TAG, "Proceeding with normal bridge initialization...");
    ESP_LOGI(TAG, "");
    
    // ========================================================================
    // Normal operation: C6 is present and firmware is compatible
    // ========================================================================
    
    // Step 1: Initialize Ethernet for MAC learning ONLY
    ESP_ERROR_CHECK(init_ethernet());
    
    // Step 2: Wait for PC MAC learning and cleanup Ethernet
    // After MAC is learned, Ethernet is completely deinitialized
    // to ensure clean state for bridge initialization later
    ESP_ERROR_CHECK(wait_for_pc_mac_and_cleanup());
    
    // Step 3: Check WiFi provisioning
    bool wifi_configured = is_wifi_provisioned();
    
    if (!wifi_configured) {
        ESP_LOGI(TAG, "WiFi not provisioned - Starting configuration portal...");
        
        // Start WiFi config portal (SoftAP with web interface)
        ESP_ERROR_CHECK(start_wifi_config_portal(&s_event_flags, PROV_SUCCESS_BIT, PROV_FAIL_BIT));
        
        // Wait for provisioning to complete
        EventBits_t bits = xEventGroupWaitBits(s_event_flags,
                                                PROV_SUCCESS_BIT | PROV_FAIL_BIT,
                                                pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (bits & PROV_FAIL_BIT) {
            ESP_LOGE(TAG, "WiFi provisioning failed - restarting...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();
        }
        
        ESP_LOGI(TAG, "WiFi provisioning successful! Credentials saved.");
        ESP_LOGI(TAG, "Restarting to apply new configuration...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        ESP_LOGI(TAG, "WiFi already provisioned, using saved credentials");
    }
    
    // Step 4: Start reconfigure button monitoring
    xTaskCreate(reconfigure_button_task, "recfg_btn", 4096, NULL, 5, NULL);
    
    // Step 5: Initialize WiFi with PC MAC
    ESP_ERROR_CHECK(init_wifi_with_pc_mac());
    
    // Step 6: Connect WiFi
    esp_err_t wifi_err = connect_wifi();
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "WiFi Connection Failed!");
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "Possible reasons:");
        ESP_LOGE(TAG, "  - Wrong WiFi password");
        ESP_LOGE(TAG, "  - WiFi network unavailable");
        ESP_LOGE(TAG, "  - Signal too weak");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "To reconfigure WiFi:");
        ESP_LOGE(TAG, "  Long-press Boot button (GPIO2) for 2 seconds");
        ESP_LOGE(TAG, "========================================");
        
        // Don't restart or abort - bridge can still work when WiFi becomes available
        // WiFi will automatically retry connection when available
        // User can trigger reconfiguration with button
    }
    
    // Step 7: Re-initialize Ethernet in clean state for bridge
    // After WiFi is connected, Ethernet is re-initialized fresh
    // This ensures both netifs (Ethernet and WiFi) are in clean state for bridge
    ESP_ERROR_CHECK(reinit_ethernet_for_bridge());
    
    // Step 8: Create bridge with both clean netifs
    ESP_ERROR_CHECK(create_bridge());
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  System Ready! Bridge is Operational");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Active Features:");
    ESP_LOGI(TAG, "  ✓ L2 Bridge (Ethernet ↔ WiFi)");
    ESP_LOGI(TAG, "  ✓ Transparent bridging with PC MAC");
    ESP_LOGI(TAG, "  ✓ WiFi Config Portal (SoftAP)");
    ESP_LOGI(TAG, "  ✓ Reconfigure Button (GPIO%d - 2s press)", RECONFIGURE_BUTTON_GPIO);
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "");
    
    // Monitor connection and handle reconnection
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(s_event_flags,
                                                WIFI_DISCONNECTED_BIT | RECONFIGURE_BIT,
                                                pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (bits & RECONFIGURE_BIT) {
            ESP_LOGI(TAG, "Reconfiguration requested - restarting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
        
        // WiFi disconnection is handled automatically by event handler
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
