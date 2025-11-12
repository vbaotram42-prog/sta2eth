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
    // Initialize bridge following official ESP-IDF pattern
    // ========================================================================
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
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
    esp_netif_inherent_config_t wifi_sta_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    wifi_sta_cfg.flags = 0;  // No flags for bridged port
    s_wifi_netif = esp_netif_create_wifi(WIFI_IF_STA, &wifi_sta_cfg);
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
    
    // Set bridge MAC to the common MAC
    memcpy(br_cfg.mac, s_common_mac, 6);
    s_br_netif = esp_netif_new(&br_netif_cfg);
    
    // Create bridge glue and add ports
    esp_netif_br_glue_handle_t br_glue = esp_netif_br_glue_new();
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_eth_netif));
    ESP_ERROR_CHECK(esp_netif_br_glue_add_port(br_glue, s_wifi_netif));
    
    // Attach bridge glue to bridge netif
    ESP_ERROR_CHECK(esp_netif_attach(s_br_netif, br_glue));
    
    ESP_LOGI(TAG, "✓ Bridge created");
    
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
    ESP_LOGI(TAG, "===========================================");
    
    // Monitor and handle reconnection
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
