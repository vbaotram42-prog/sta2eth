/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/**
 * sta2eth L2 Bridge (ESP32-P4 + C6)
 * 
 * Following the official ESP-IDF bridge example:
 * https://github.com/espressif/esp-idf/tree/master/examples/network/bridge
 * 
 * Architecture:
 * 1. During first boot: Configuration portal learns Ethernet MAC and WiFi credentials
 * 2. Normal operation: Use saved MAC for both Ethernet and WiFi, create L2 bridge
 * 3. Transparent bridging between Ethernet and WiFi with same MAC address
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
#include "ethernet_init.h"
#include "wifi_config_portal.h"
#include "c6_ota.h"

static const char *TAG = "sta2eth";

// Event flags
static EventGroupHandle_t s_event_flags;
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_DISCONNECTED_BIT BIT1
#define PROV_SUCCESS_BIT     BIT2
#define PROV_FAIL_BIT        BIT3

// WiFi connection retry configuration
#define WIFI_MAXIMUM_RETRY  5
static int s_wifi_retry_num = 0;

// Global state
static esp_netif_t *s_eth_netif = NULL;
static esp_netif_t *s_wifi_netif = NULL;
static esp_netif_t *s_br_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static uint8_t s_common_mac[6] = {0};  // Saved MAC used for both interfaces

// Reconfigure button GPIO (Boot button)
#define RECONFIGURE_BUTTON_GPIO 2

// Bridge diagnostic configuration
#define ENABLE_BRIDGE_DIAGNOSTICS 1
#define DIAGNOSTIC_INTERVAL_MS 5000  // Send test packets every 5 seconds


/**
 * Ethernet event handler
 */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
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
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_REMOTE_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started, connecting...");
            esp_wifi_remote_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected to AP");
            s_wifi_retry_num = 0;
            xEventGroupSetBits(s_event_flags, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_event_flags, WIFI_DISCONNECTED_BIT);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            // Re-set MAC address before reconnection
            // MAC is stored in C6's RAM (WIFI_STORAGE_RAM) and may be lost
            ESP_LOGI(TAG, "WiFi disconnected, re-setting MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     s_common_mac[0], s_common_mac[1], s_common_mac[2],
                     s_common_mac[3], s_common_mac[4], s_common_mac[5]);
            esp_wifi_set_mac(WIFI_IF_STA, s_common_mac);
            
            if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
                esp_wifi_remote_connect();
                s_wifi_retry_num++;
                ESP_LOGI(TAG, "Retrying connection (%d/%d)", s_wifi_retry_num, WIFI_MAXIMUM_RETRY);
            } else {
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
 * IP event handler
 * Logs when bridge gets IP from router
 */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
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

/**
 * Reconfigure button handler task
 */
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
    
    ESP_LOGI(TAG, "Reconfigure button ready (GPIO%d, long-press 2s)", RECONFIGURE_BUTTON_GPIO);
    
    uint32_t press_start = 0;
    bool button_pressed = false;
    
    while (1) {
        int level = gpio_get_level(RECONFIGURE_BUTTON_GPIO);
        
        if (level == 0 && !button_pressed) {
            // Button pressed
            button_pressed = true;
            press_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
        } else if (level == 1 && button_pressed) {
            // Button released
            button_pressed = false;
            uint32_t press_duration = (xTaskGetTickCount() * portTICK_PERIOD_MS) - press_start;
            
            if (press_duration >= 2000) {
                ESP_LOGW(TAG, "Reconfigure button pressed! Clearing credentials...");
                clear_wifi_credentials();
                ESP_LOGW(TAG, "Restarting to enter configuration mode...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#if ENABLE_BRIDGE_DIAGNOSTICS
/**
 * Bridge Diagnostic Functions
 * Send broadcast packets to test packet flow through bridge, Ethernet, and WiFi
 */

// ARP packet structure for broadcast testing
typedef struct {
    uint8_t dst_mac[6];      // Destination MAC (broadcast: ff:ff:ff:ff:ff:ff)
    uint8_t src_mac[6];      // Source MAC
    uint16_t eth_type;       // Ethernet type (0x0806 for ARP)
    uint16_t hw_type;        // Hardware type (1 for Ethernet)
    uint16_t proto_type;     // Protocol type (0x0800 for IPv4)
    uint8_t hw_size;         // Hardware address size (6 for MAC)
    uint8_t proto_size;      // Protocol address size (4 for IPv4)
    uint16_t opcode;         // Operation (1 for request)
    uint8_t sender_mac[6];   // Sender MAC
    uint8_t sender_ip[4];    // Sender IP
    uint8_t target_mac[6];   // Target MAC (00:00:00:00:00:00 for broadcast)
    uint8_t target_ip[4];    // Target IP (broadcast query)
} __attribute__((packed)) arp_packet_t;

/**
 * Send ARP broadcast packet through specified interface
 */
static esp_err_t send_arp_broadcast(esp_netif_t *netif, const char *iface_name, const uint8_t *src_mac)
{
    if (!netif || !src_mac) {
        ESP_LOGE(TAG, "[DIAG] Invalid parameters for %s", iface_name);
        return ESP_ERR_INVALID_ARG;
    }

    // Build ARP request packet
    arp_packet_t arp_pkt;
    memset(&arp_pkt, 0, sizeof(arp_pkt));
    
    // Ethernet header
    memset(arp_pkt.dst_mac, 0xff, 6);  // Broadcast MAC
    memcpy(arp_pkt.src_mac, src_mac, 6);
    arp_pkt.eth_type = htons(0x0806);  // ARP
    
    // ARP header
    arp_pkt.hw_type = htons(1);        // Ethernet
    arp_pkt.proto_type = htons(0x0800); // IPv4
    arp_pkt.hw_size = 6;
    arp_pkt.proto_size = 4;
    arp_pkt.opcode = htons(1);          // ARP Request
    
    // Sender info
    memcpy(arp_pkt.sender_mac, src_mac, 6);
    arp_pkt.sender_ip[0] = 169;
    arp_pkt.sender_ip[1] = 254;
    arp_pkt.sender_ip[2] = 100;
    arp_pkt.sender_ip[3] = 1;
    
    // Target info (query for 169.254.100.100)
    memset(arp_pkt.target_mac, 0x00, 6);
    arp_pkt.target_ip[0] = 169;
    arp_pkt.target_ip[1] = 254;
    arp_pkt.target_ip[2] = 100;
    arp_pkt.target_ip[3] = 100;
    
    ESP_LOGI(TAG, "[DIAG] Sending ARP broadcast via %s:", iface_name);
    ESP_LOGI(TAG, "       Src MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    ESP_LOGI(TAG, "       Dst MAC: ff:ff:ff:ff:ff:ff (broadcast)");
    ESP_LOGI(TAG, "       Query: Who has 169.254.100.100?");
    
    // Try to transmit through netif
    esp_err_t ret = esp_netif_transmit(netif, &arp_pkt, sizeof(arp_pkt));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[DIAG] ✓ ARP packet transmitted via %s", iface_name);
    } else {
        ESP_LOGW(TAG, "[DIAG] ✗ Failed to transmit via %s: %s", iface_name, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Send raw Ethernet broadcast frame
 */
static esp_err_t send_raw_broadcast(esp_eth_handle_t eth_handle, const char *iface_name, const uint8_t *src_mac)
{
    if (!eth_handle || !src_mac) {
        ESP_LOGE(TAG, "[DIAG] Invalid parameters for raw %s", iface_name);
        return ESP_ERR_INVALID_ARG;
    }

    // Simple Ethernet broadcast frame with custom EtherType
    uint8_t frame[64];
    memset(frame, 0, sizeof(frame));
    
    // Destination MAC: broadcast
    memset(frame, 0xff, 6);
    // Source MAC
    memcpy(frame + 6, src_mac, 6);
    // EtherType: custom test type (0x88B5 - local experimental)
    frame[12] = 0x88;
    frame[13] = 0xB5;
    // Payload: "BRIDGE_TEST" + timestamp
    const char *payload = "BRIDGE_TEST_PACKET";
    memcpy(frame + 14, payload, strlen(payload));
    
    ESP_LOGI(TAG, "[DIAG] Sending raw broadcast via %s (EMAC):", iface_name);
    ESP_LOGI(TAG, "       Src MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    ESP_LOGI(TAG, "       EtherType: 0x88B5 (test), Payload: %s", payload);
    
    // Transmit directly through Ethernet driver
    esp_err_t ret = esp_eth_transmit(eth_handle, frame, sizeof(frame));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[DIAG] ✓ Raw frame transmitted via %s EMAC", iface_name);
    } else {
        ESP_LOGW(TAG, "[DIAG] ✗ Failed to transmit raw frame via %s: %s", iface_name, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Bridge diagnostic task
 * Periodically sends test packets to diagnose packet flow
 */
static void bridge_diagnostic_task(void *arg)
{
    ESP_LOGI(TAG, "[DIAG] ==========================================");
    ESP_LOGI(TAG, "[DIAG] Bridge Diagnostic Task Started");
    ESP_LOGI(TAG, "[DIAG] Will send test packets every %d ms", DIAGNOSTIC_INTERVAL_MS);
    ESP_LOGI(TAG, "[DIAG] ==========================================");
    
    // Wait for bridge to be fully operational
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint32_t test_count = 0;
    
    while (1) {
        test_count++;
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "[DIAG] ========== Test Round #%lu ==========", test_count);
        
        // Test 1: Send ARP through bridge netif
        if (s_br_netif) {
            ESP_LOGI(TAG, "[DIAG] Test 1: Sending via BRIDGE netif");
            send_arp_broadcast(s_br_netif, "BRIDGE", s_common_mac);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Test 2: Send ARP through Ethernet netif directly
        if (s_eth_netif) {
            ESP_LOGI(TAG, "[DIAG] Test 2: Sending via ETHERNET netif");
            send_arp_broadcast(s_eth_netif, "ETHERNET", s_common_mac);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Test 3: Send ARP through WiFi netif directly
        if (s_wifi_netif) {
            ESP_LOGI(TAG, "[DIAG] Test 3: Sending via WIFI netif");
            send_arp_broadcast(s_wifi_netif, "WIFI", s_common_mac);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Test 4: Send raw frame through Ethernet driver (EMAC)
        if (s_eth_handle) {
            ESP_LOGI(TAG, "[DIAG] Test 4: Sending raw via ETHERNET driver (EMAC/IP101)");
            send_raw_broadcast(s_eth_handle, "ETHERNET", s_common_mac);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "[DIAG] Test round #%lu completed", test_count);
        ESP_LOGI(TAG, "[DIAG] ======================================");
        
        // Wait before next round
        vTaskDelay(pdMS_TO_TICKS(DIAGNOSTIC_INTERVAL_MS));
    }
}
#endif // ENABLE_BRIDGE_DIAGNOSTICS

/**
 * Main application entry point
 * 
 * Following the official ESP-IDF bridge example pattern:
 * https://github.com/espressif/esp-idf/tree/master/examples/network/bridge
 */
void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "STA2ETH L2 Bridge (ESP32-P4 + C6)");
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
    // C6 OTA Check - ensure C6 firmware is compatible
    // ========================================================================
    ESP_LOGI(TAG, "Step 0: Checking C6 firmware status...");
    
    if (c6_ota_should_enter_mode()) {
        ESP_LOGW(TAG, "===========================================");
        ESP_LOGW(TAG, "  C6 FIRMWARE UPDATE REQUIRED!");
        ESP_LOGW(TAG, "===========================================");
        ESP_LOGW(TAG, "Entering C6 OTA mode...");
        ESP_ERROR_CHECK(c6_ota_start_mode());
        // Should not reach here
        esp_restart();
    }
    
    ESP_LOGI(TAG, "✓ C6 firmware compatible");
    
    // ========================================================================
    // Initialize event loop and network stack (required for both config and bridge)
    // ========================================================================
    ESP_LOGI(TAG, "Initializing event loop and network stack...");
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    ESP_LOGI(TAG, "✓ Event loop and network stack initialized");
    
    // ========================================================================
    // Check if Ethernet MAC and WiFi credentials are saved
    // If not, enter configuration mode
    // ========================================================================
    bool eth_mac_saved = is_eth_mac_saved();
    bool wifi_configured = is_wifi_provisioned();
    
    if (!eth_mac_saved || !wifi_configured) {
        ESP_LOGI(TAG, "===========================================");
        ESP_LOGI(TAG, "  Configuration Required");
        ESP_LOGI(TAG, "===========================================");
        if (!eth_mac_saved) {
            ESP_LOGI(TAG, "✗ Ethernet MAC not saved");
        }
        if (!wifi_configured) {
            ESP_LOGI(TAG, "✗ WiFi not configured");
        }
        ESP_LOGI(TAG, "Starting configuration portal...");
        
        // Start configuration portal (SoftAP + Ethernet)
        // This will save both Ethernet MAC and WiFi credentials
        ESP_ERROR_CHECK(start_wifi_config_portal(&s_event_flags, PROV_SUCCESS_BIT, PROV_FAIL_BIT));
        
        // Wait for configuration to complete
        EventBits_t bits = xEventGroupWaitBits(s_event_flags,
                                                PROV_SUCCESS_BIT | PROV_FAIL_BIT,
                                                pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (bits & PROV_FAIL_BIT) {
            ESP_LOGE(TAG, "Configuration failed - restarting...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();
        }
        
        ESP_LOGI(TAG, "✓ Configuration successful! Restarting...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "✓ Ethernet MAC and WiFi credentials found");
    
    // ========================================================================
    // Load saved Ethernet MAC address
    // ========================================================================
    ret = load_eth_mac(s_common_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load Ethernet MAC - entering config mode");
        ESP_LOGI(TAG, "Please long-press button to reconfigure");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "Using MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
             s_common_mac[0], s_common_mac[1], s_common_mac[2],
             s_common_mac[3], s_common_mac[4], s_common_mac[5]);
    
    // ========================================================================
    // Step 1: Initialize Ethernet
    // ========================================================================
    ESP_LOGI(TAG, "Step 1: Initializing Ethernet...");
    
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(ethernet_init_all(&eth_handles, &eth_port_cnt));
    
    if (eth_port_cnt == 0) {
        ESP_LOGE(TAG, "No Ethernet interface found!");
        return;
    }
    
    s_eth_handle = eth_handles[0];
    free(eth_handles);
    
    // Set Ethernet MAC to the saved common MAC
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, s_common_mac));
    
    // Create Ethernet netif (flags = 0 for bridged port)
    esp_netif_inherent_config_t eth_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    eth_cfg.flags = 0;  // No flags for bridged port
    esp_netif_config_t eth_netif_cfg = {
        .base = &eth_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    s_eth_netif = esp_netif_new(&eth_netif_cfg);
    
    // Attach Ethernet driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
    
    ESP_LOGI(TAG, "✓ Ethernet initialized");
    
    // ========================================================================
    // Step 2: Initialize WiFi
    // ========================================================================
    ESP_LOGI(TAG, "Step 2: Initializing WiFi...");
    
    // Display esp-hosted network split configuration
    ESP_LOGI(TAG, "ESP-Hosted Network Split Configuration:");
#ifdef CONFIG_ESP_HOSTED_NETWORK_SPLIT_ENABLED
    ESP_LOGI(TAG, "  Network Split: ENABLED");
    #ifdef CONFIG_ESP_DEFAULT_LWIP_HOST
        ESP_LOGI(TAG, "  Default LWIP: HOST (all packets processed on P4)");
    #elif defined(CONFIG_ESP_DEFAULT_LWIP_SLAVE)
        ESP_LOGW(TAG, "  Default LWIP: SLAVE (packets processed on C6) - NOT RECOMMENDED FOR BRIDGE!");
    #elif defined(CONFIG_ESP_DEFAULT_LWIP_BOTH)
        ESP_LOGW(TAG, "  Default LWIP: BOTH (packets processed on both) - NOT RECOMMENDED FOR BRIDGE!");
    #else
        ESP_LOGW(TAG, "  Default LWIP: NOT CONFIGURED - May use slave default!");
    #endif
#else
    ESP_LOGW(TAG, "  Network Split: DISABLED - Bridge may not work correctly!");
#endif
    
    // Initialize WiFi Remote
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_remote_init(&wifi_cfg));
    
    // Set WiFi storage to RAM only (credentials on P4's NVS)
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // Set WiFi mode to STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Set WiFi STA MAC to the same common MAC
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, s_common_mac));
    
    // Create WiFi STA netif (flags = 0 for bridged port)
    // When using esp_wifi_remote, the standard esp_netif_create_wifi() is overridden
    // by the component's injected headers to work correctly with remote WiFi
    esp_netif_inherent_config_t wifi_sta_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    wifi_sta_cfg.flags = 0;  // No flags for bridged port
    s_wifi_netif = esp_netif_create_wifi(WIFI_IF_STA, &wifi_sta_cfg);
    
    // Attach WiFi STA netif to WiFi driver (required when using esp_netif_create_wifi)
    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(s_wifi_netif));
    
    // Register default event handlers
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    
    // Load and set WiFi credentials
    char ssid[33] = {0};
    char password[65] = {0};
    ret = load_wifi_credentials(ssid, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load WiFi credentials");
        return;
    }
    
    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    ESP_LOGI(TAG, "✓ WiFi initialized (SSID: %s)", ssid);
    
    // ========================================================================
    // Step 3: Create Bridge
    // ========================================================================
    ESP_LOGI(TAG, "Step 3: Creating bridge...");
    
    // Create bridge netif configuration
    esp_netif_inherent_config_t br_cfg = ESP_NETIF_INHERENT_DEFAULT_BR();
    
    // CRITICAL: Remove DHCP client flag - bridge should NOT get IP
    // Only PC should get IP from router
    br_cfg.flags &= ~ESP_NETIF_DHCP_CLIENT;
    
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
    
    // Set bridge MAC to the common MAC (PC's MAC)
    memcpy(br_cfg.mac, s_common_mac, 6);
    s_br_netif = esp_netif_new(&br_netif_cfg);
    
    // Explicitly stop DHCP client on bridge (safety measure)
    esp_netif_dhcpc_stop(s_br_netif);
    
    // Create bridge glue and add ports
    esp_netif_br_glue_handle_t br_glue = esp_netif_br_glue_new();
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_eth_netif));
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_wifi_netif));
    
    // Attach bridge glue to bridge netif
    ESP_ERROR_CHECK(esp_netif_attach(s_br_netif, br_glue));
    
    ESP_LOGI(TAG, "✓ Bridge created (DHCP disabled, L2 only)");
    
    // ========================================================================
    // Step 4: Register event handlers
    // ========================================================================
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_REMOTE_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    
    // ========================================================================
    // Step 5: Start interfaces
    // ========================================================================
    ESP_LOGI(TAG, "Step 4: Starting interfaces...");
    
    // Assign link-local static IPs to bridge ports BEFORE starting interfaces
    // These IPs (169.254.x.x) are for internal bridge management only
    esp_netif_ip_info_t eth_ip = {
        .ip = { .addr = ESP_IP4TOADDR(169, 254, 1, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
        .gw = { .addr = ESP_IP4TOADDR(169, 254, 1, 1) }
    };
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_eth_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &eth_ip));
    ESP_LOGI(TAG, "Ethernet port IP: 169.254.1.1/16");

    esp_netif_ip_info_t wifi_ip = {
        .ip = { .addr = ESP_IP4TOADDR(169, 254, 1, 2) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
        .gw = { .addr = ESP_IP4TOADDR(169, 254, 1, 2) }
    };
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_wifi_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_wifi_netif, &wifi_ip));
    ESP_LOGI(TAG, "WiFi port IP: 169.254.1.2/16");
    
    // Enable promiscuous mode on Ethernet for bridge
    bool promiscuous = true;
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous));
    
    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    ESP_LOGI(TAG, "✓ Ethernet started");
    
    // Start WiFi (will connect automatically)
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "✓ WiFi started");
    
    // ========================================================================
    // Step 6: Start reconfigure button monitoring
    // ========================================================================
    xTaskCreate(reconfigure_button_task, "recfg_btn", 4096, NULL, 5, NULL);
    
#if ENABLE_BRIDGE_DIAGNOSTICS
    // ========================================================================
    // Step 7: Start bridge diagnostic task
    // ========================================================================
    ESP_LOGI(TAG, "Starting bridge diagnostic task...");
    xTaskCreate(bridge_diagnostic_task, "br_diag", 8192, NULL, 5, NULL);
#endif
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Bridge Operational!");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
             s_common_mac[0], s_common_mac[1], s_common_mac[2],
             s_common_mac[3], s_common_mac[4], s_common_mac[5]);
    ESP_LOGI(TAG, "WiFi: %s", ssid);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "To reconfigure: Long-press button for 2 seconds");
#if ENABLE_BRIDGE_DIAGNOSTICS
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "⚠️  DIAGNOSTIC MODE ENABLED");
    ESP_LOGI(TAG, "Sending test packets every %d ms to trace packet flow", DIAGNOSTIC_INTERVAL_MS);
#endif
    ESP_LOGI(TAG, "===========================================");
    
    // Monitor and handle reconnection
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
