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
static void *s_eth_glue = NULL;  // Store Ethernet netif glue to reuse it
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
    
    // Create Ethernet netif glue for initial MAC learning phase
    // Note: glue ownership will transfer to netif after esp_netif_attach()
    // The glue will be automatically freed when netif is destroyed
    void *eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    if (!eth_glue) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif glue");
        return ESP_FAIL;
    }
    
    // Attach Ethernet driver to netif (glue ownership transfers to netif)
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, eth_glue));
    
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
 * Step 2: Wait for PC MAC learning and complete Ethernet teardown
 * 
 * Completely tear down Ethernet to ensure absolutely clean state:
 * 1. Remove packet receive callback
 * 2. Stop Ethernet driver
 * 3. Destroy netif (auto-cleans glue)
 * 4. Uninstall Ethernet driver completely
 * 5. Everything will be recreated from scratch for bridge
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
    
    // CRITICAL: Remove the packet receive callback IMMEDIATELY
    ESP_LOGI(TAG, "Removing MAC learning packet callback...");
    ESP_ERROR_CHECK(esp_eth_update_input_path(s_eth_handle, NULL, NULL));
    ESP_LOGI(TAG, "✓ Packet callback removed");
    
    // Cleanup Ethernet netif/glue but KEEP driver running
    // Following official ESP-IDF pattern: drivers stay running, only netif/glue recreated
    ESP_LOGI(TAG, "Cleaning up Ethernet network layer (keeping driver running)...");
    
    // Stop link down timer
    if (s_link_down_timer) {
        esp_timer_stop(s_link_down_timer);
        esp_timer_delete(s_link_down_timer);
        s_link_down_timer = NULL;
    }
    
    // IMPORTANT: Unregister event handlers FIRST (before netif destroy)
    ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler));
    ESP_LOGI(TAG, "✓ Event handlers unregistered");
    
    // CRITICAL ORDER: Destroy netif BEFORE stopping driver
    // When netif is destroyed, it needs to clean up MAC filters, which requires driver to be running
    esp_netif_destroy(s_eth_netif);
    s_eth_netif = NULL;
    ESP_LOGI(TAG, "✓ Ethernet netif destroyed");
    
    // Delete glue explicitly (must be done after netif destroy)
    if (s_eth_glue) {
        esp_eth_del_netif_glue(s_eth_glue);
        s_eth_glue = NULL;
        ESP_LOGI(TAG, "✓ Ethernet glue deleted");
    }
    
    // NOTE: We do NOT stop the Ethernet driver!
    // Keep driver running so we can reattach a new netif without restarting
    ESP_LOGI(TAG, "✓ Ethernet driver kept running for reattachment")
    
    // NOTE: Driver remains installed (s_eth_handle valid) - will be reused in bridge mode
    ESP_LOGI(TAG, "Ethernet cleanup complete - driver ready for reuse");
    
    return ESP_OK;
}

/**
 * Step 3: Initialize WiFi with PC MAC and configure credentials
 * 
 * Following official ESP-IDF bridge example pattern:
 * - Initialize WiFi (but don't start)
 * - Create netif
 * - Set WiFi config (SSID/password)
 * - Register event handlers
 * - WiFi will be started later in create_bridge()
 * 
 * Prerequisites (guaranteed by app_main):
 * - PC MAC learned
 * - Event loop created
 * - NVS initialized
 * - WiFi credentials available
 * 
 * Returns:
 * - ESP_OK: WiFi initialized and configured successfully
 * - ESP_FAIL: Initialization failed (C6 not responding)
 */
static esp_err_t init_wifi_with_pc_mac(void)
{
    ESP_LOGI(TAG, "Step 3: Initializing WiFi with PC MAC...");
    
    // Initialize WiFi Remote - this checks if C6 is responding
    ESP_LOGI(TAG, "Initializing WiFi Remote (esp_wifi_remote)...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_remote_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi Remote: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "C6 not responding");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✓ WiFi Remote initialized - C6 is responding");
    
    // Set WiFi storage to RAM only
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi storage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    // Set WiFi mode to STA
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    // Set WiFi MAC = PC MAC
    ESP_LOGI(TAG, "Setting WiFi MAC to PC MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ret = esp_wifi_set_mac(WIFI_IF_STA, s_pc_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi MAC: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    // Load WiFi credentials from NVS
    char ssid[33] = {0};
    char password[65] = {0};
    ret = load_wifi_credentials(ssid, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load credentials: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Validate SSID
    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is empty");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "WiFi credentials loaded: SSID=%s", ssid);
    
    // Set WiFi configuration (will auto-connect when started)
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ WiFi configuration set");
    
    // Create WiFi netif - following official bridge example
    esp_netif_inherent_config_t wifi_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    wifi_cfg.flags = 0;  // Must be 0 for bridged ports (except AP needs AUTOUP)
    wifi_cfg.ip_info = NULL;  // No IP for physical interface when bridged
    s_wifi_netif = esp_netif_create_wifi(WIFI_IF_STA, &wifi_cfg);
    if (!s_wifi_netif) {
        ESP_LOGE(TAG, "Failed to create WiFi netif");
        return ESP_FAIL;
    }
    
    ret = esp_wifi_set_default_wifi_sta_handlers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set handlers: %s", esp_err_to_name(ret));
        esp_netif_destroy(s_wifi_netif);
        s_wifi_netif = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✓ WiFi netif created");
    
    // Register event handler
    ret = esp_event_handler_register(WIFI_REMOTE_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event handler: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    // NOTE: Do NOT start WiFi here!
    // Following official ESP-IDF bridge example:
    // WiFi will be started in create_bridge() after bridge is configured
    
    ESP_LOGI(TAG, "WiFi initialized and configured (not started yet)");
    ESP_LOGI(TAG, "WiFi will auto-connect when started in create_bridge()");
    return ESP_OK;
}

/**
 * Reinitialize Ethernet network layer for bridge
 * 
 * Driver is already installed and stopped - reuse it.
 * Following official ESP-IDF pattern:
 * 1. Create new netif (bridged config)
 * 2. Create new glue
 * 3. Attach to existing driver
 * 4. Configure and register handlers
 * 
 * Returns:
 * - ESP_OK: Ethernet ready for bridge
 * - ESP_FAIL: Failed to reinitialize
 */
static esp_err_t reinit_ethernet_for_bridge(void)
{
    ESP_LOGI(TAG, "Re-initializing Ethernet for bridge (reusing driver)...");
    
    // Verify driver is still valid
    if (!s_eth_handle) {
        ESP_LOGE(TAG, "Ethernet driver handle is NULL");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✓ Using existing Ethernet driver");
    
    // Create clean Ethernet netif for bridge
    esp_netif_inherent_config_t eth_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    eth_cfg.flags = 0;  // Flags must be 0 for bridged ports
    esp_netif_config_t netif_cfg = {
        .base = &eth_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✓ New Ethernet netif created");
    
    // Create fresh glue and attach to existing driver
    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    if (!s_eth_glue) {
        ESP_LOGE(TAG, "Failed to create Ethernet glue");
        esp_netif_destroy(s_eth_netif);
        s_eth_netif = NULL;
        return ESP_FAIL;
    }
    
    esp_err_t ret = esp_netif_attach(s_eth_netif, s_eth_glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach: %s", esp_err_to_name(ret));
        esp_netif_destroy(s_eth_netif);
        s_eth_netif = NULL;
        s_eth_glue = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "✓ Ethernet netif attached to glue");
    
    // NOTE: Bridged ports should NOT have IP configuration
    // IP is managed by the bridge netif, not individual ports
    // Do NOT call esp_netif_dhcpc_stop() or esp_netif_set_ip_info()
    
    // Enable promiscuous mode (required for bridge)
    bool promiscuous = true;
    ret = esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable promiscuous: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Recreate link down timer
    const esp_timer_create_args_t timer_args = {
        .callback = &link_down_timer_callback,
        .name = "link_down_timer"
    };
    ret = esp_timer_create(&timer_args, &s_link_down_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register event handlers
    ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register events: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // NOTE: Do NOT start Ethernet here!
    // Will be started in create_bridge() after bridge is configured
    
    ESP_LOGI(TAG, "Ethernet network layer re-initialized for bridge (driver reused)");
    ESP_LOGI(TAG, "✓ Ethernet re-initialized for bridge");
    return ESP_OK;
}

/**
 * Step 5: Create bridge and start interfaces
 * 
 * Following official ESP-IDF bridge example pattern:
 * 1. Create bridge netif
 * 2. Create bridge glue  
 * 3. Add all port netifs to bridge glue
 * 4. Attach bridge glue to bridge netif
 * 5. Start all drivers (Ethernet and WiFi)
 * 6. Wait for WiFi connection
 * 
 * Prerequisites (guaranteed by app_main):
 * - Both netifs created and attached to drivers
 * - WiFi configured with credentials
 * - Ethernet configured
 * - Neither started yet
 * 
 * Returns:
 * - ESP_OK: Bridge created and WiFi connected
 * - ESP_FAIL: Bridge creation or WiFi connection failed
 * - ESP_ERR_TIMEOUT: WiFi connection timeout
 */
static esp_err_t create_bridge(void)
{
    ESP_LOGI(TAG, "Step 5: Creating L2 bridge...");
    
    // Create bridge netif
    esp_netif_inherent_config_t br_cfg = ESP_NETIF_INHERENT_DEFAULT_BR();
    esp_netif_config_t br_netif_cfg = {
        .base = &br_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_BR,
    };
    
    bridgeif_config_t bridgeif_config = {
        .max_fdb_dyn_entries = 10,
        .max_fdb_sta_entries = 2,
        .max_ports = 2
    };
    br_cfg.bridge_info = &bridgeif_config;
    memcpy(br_cfg.mac, s_pc_mac, 6);
    
    s_br_netif = esp_netif_new(&br_netif_cfg);
    if (!s_br_netif) {
        ESP_LOGE(TAG, "Failed to create bridge netif");
        return ESP_FAIL;
    }
    
    // Create bridge glue
    esp_netif_br_glue_handle_t br_glue = esp_netif_br_glue_new();
    if (!br_glue) {
        ESP_LOGE(TAG, "Failed to create bridge glue");
        esp_netif_destroy(s_br_netif);
        s_br_netif = NULL;
        return ESP_FAIL;
    }
    
    // Add Ethernet port to bridge
    esp_err_t ret = esp_netif_br_glue_add_port(br_glue, s_eth_netif);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Ethernet port: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Added Ethernet port to bridge");
    
    // Add WiFi port to bridge
    ret = esp_netif_br_glue_add_port(br_glue, s_wifi_netif);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add WiFi port: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Added WiFi STA port to bridge");
    
    // Attach bridge glue to bridge netif
    ret = esp_netif_attach(s_br_netif, br_glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach glue: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Bridge glue attached successfully");
    
    // Clear connection state flags before starting WiFi
    xEventGroupClearBits(s_event_flags, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_DISCONNECTED_BIT);
    s_wifi_retry_num = 0;
    
    // Start WiFi driver
    // Note: Ethernet driver is already running - we only stopped/recreated netif
    ESP_LOGI(TAG, "Starting WiFi driver...");
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ WiFi started, will auto-connect to configured AP");
    
    // Wait for WiFi connection with timeout
    // WiFi will automatically connect based on config set in init_wifi_with_pc_mac()
    const TickType_t timeout = pdMS_TO_TICKS((WIFI_MAXIMUM_RETRY + 1) * 12000);
    ESP_LOGI(TAG, "Waiting for WiFi connection (timeout: %d sec, max retries: %d)...", 
             (WIFI_MAXIMUM_RETRY + 1) * 12, WIFI_MAXIMUM_RETRY);
    
    EventBits_t bits = xEventGroupWaitBits(s_event_flags, 
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, timeout);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✓ WiFi connected successfully");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAXIMUM_RETRY);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Bridge Created Successfully!");
    ESP_LOGI(TAG, "PC MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             s_pc_mac[0], s_pc_mac[1], s_pc_mac[2],
             s_pc_mac[3], s_pc_mac[4], s_pc_mac[5]);
    ESP_LOGI(TAG, "Bridge is operational with WiFi connected");
    ESP_LOGI(TAG, "===========================================");
    
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
    
    // Step 5: Initialize WiFi with PC MAC and configure credentials
    // WiFi will not be started yet - following official bridge example pattern
    esp_err_t wifi_init_err = init_wifi_with_pc_mac();
    if (wifi_init_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(wifi_init_err));
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  - C6 not responding");
        ESP_LOGE(TAG, "  - WiFi credentials invalid");
        ESP_LOGE(TAG, "Restarting...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "✓ WiFi initialized and configured");
    
    // Step 6: Re-initialize Ethernet for bridge
    // Ethernet will not be started yet - following official bridge example pattern
    esp_err_t eth_err = reinit_ethernet_for_bridge();
    if (eth_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-initialize Ethernet: %s", esp_err_to_name(eth_err));
        ESP_LOGE(TAG, "Cannot create bridge. Restarting...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "✓ Ethernet re-initialized for bridge");
    
    // Step 7: Create bridge and start both interfaces
    // Following official ESP-IDF bridge example pattern:
    // - Both netifs are created and configured
    // - Bridge is configured
    // - Both interfaces started together
    // - Wait for WiFi connection
    esp_err_t bridge_err = create_bridge();
    if (bridge_err != ESP_OK) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "Bridge Creation or WiFi Connection Failed!");
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "Error code: %s", esp_err_to_name(bridge_err));
        
        if (bridge_err == ESP_ERR_TIMEOUT || bridge_err == ESP_FAIL) {
            ESP_LOGE(TAG, "Possible reasons:");
            ESP_LOGE(TAG, "  - Wrong WiFi password");
            ESP_LOGE(TAG, "  - WiFi network unavailable");
            ESP_LOGE(TAG, "  - Signal too weak");
            ESP_LOGE(TAG, "");
            ESP_LOGE(TAG, "To reconfigure WiFi:");
            ESP_LOGE(TAG, "  Long-press Boot button (GPIO2) for 2 seconds");
            ESP_LOGE(TAG, "========================================");
            ESP_LOGE(TAG, "System will wait for manual reconfiguration...");
            
            // Cannot proceed - wait for manual intervention
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(10000));
                ESP_LOGW(TAG, "Waiting for manual reconfiguration...");
            }
        } else {
            ESP_LOGE(TAG, "System cannot operate without bridge. Restarting...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        }
    }
    
    ESP_LOGI(TAG, "✓ Bridge created and WiFi connected successfully");
    
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
