/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * WiFi Configuration Portal for ESP32-P4 + C6
 * 
 * Uses SoftAP on C6 for configuration via mobile phone
 * Separate compilation unit to use WiFi remote types
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Include injected headers BEFORE esp_wifi_remote.h
#include "injected/esp_wifi.h"
#include "esp_wifi_remote.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "dns_server.h"
#include "wifi_config_portal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "ethernet_init.h"

static const char *TAG = "wifi_config_portal";

static httpd_handle_t s_web_server = NULL;
static EventGroupHandle_t *s_event_flags = NULL;
static int s_success_bit;
static esp_netif_t *s_ap_netif = NULL;
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;

// SoftAP configuration
#define SOFTAP_SSID       "ESP32-P4-Config"
#define SOFTAP_CHANNEL    6
#define SOFTAP_MAX_CONN   4

// HTML configuration page with WiFi scanning
static const char config_page_html[] = 
"<!DOCTYPE html><html><head>"
"<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32-P4 WiFi Setup</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:20px}"
".container{max-width:500px;margin:0 auto;background:white;border-radius:20px;box-shadow:0 20px 60px rgba(0,0,0,0.3);overflow:hidden}"
".header{background:linear-gradient(135deg,#667eea,#764ba2);color:white;padding:30px;text-align:center}"
".header h1{font-size:24px;margin-bottom:10px}"
".header p{opacity:0.9;font-size:14px}"
".content{padding:30px}"
".network-list{margin:20px 0}"
".network-item{display:flex;align-items:center;padding:15px;margin:10px 0;background:#f8f9fa;border:2px solid transparent;border-radius:10px;cursor:pointer;transition:all 0.3s}"
".network-item:hover{background:#e9ecef;border-color:#667eea}"
".network-item.selected{background:#667eea;color:white;border-color:#667eea}"
".network-item .signal{margin-right:15px;font-size:20px}"
".network-item .ssid{flex:1;font-weight:500}"
".network-item .lock{font-size:16px;opacity:0.6}"
".form-group{margin:20px 0}"
".form-group label{display:block;margin-bottom:8px;font-weight:600;color:#333}"
".form-group input{width:100%;padding:12px;border:2px solid #e0e0e0;border-radius:8px;font-size:16px;transition:border 0.3s}"
".form-group input:focus{outline:none;border-color:#667eea}"
".btn{width:100%;padding:15px;background:linear-gradient(135deg,#667eea,#764ba2);color:white;border:none;border-radius:10px;font-size:16px;font-weight:600;cursor:pointer;transition:transform 0.2s}"
".btn:hover{transform:translateY(-2px)}"
".btn:active{transform:translateY(0)}"
".btn:disabled{opacity:0.6;cursor:not-allowed}"
".status{margin:20px 0;padding:15px;border-radius:10px;text-align:center;display:none}"
".status.show{display:block}"
".status.info{background:#e3f2fd;color:#1976d2}"
".status.success{background:#e8f5e9;color:#388e3c}"
".status.error{background:#ffebee;color:#c62828}"
".status.warning{background:#fff3e0;color:#f57c00}"
".eth-prompt{margin:0 0 20px 0;padding:15px;border-radius:10px;background:#fff3e0;color:#f57c00;border-left:4px solid #f57c00}"
".eth-prompt h3{font-size:16px;margin-bottom:8px;display:flex;align-items:center}"
".eth-prompt h3 span{margin-right:8px}"
".eth-prompt p{font-size:14px;line-height:1.5;margin:5px 0}"
".eth-prompt .eth-mac{font-family:monospace;font-weight:bold;background:rgba(0,0,0,0.1);padding:4px 8px;border-radius:4px;display:inline-block;margin-top:5px}"
".loading{display:inline-block;width:20px;height:20px;border:3px solid rgba(255,255,255,0.3);border-radius:50%;border-top-color:white;animation:spin 1s linear infinite}"
"@keyframes spin{to{transform:rotate(360deg)}}"
"</style></head><body>"
"<div class='container'>"
"<div class='header'><h1>🌐 WiFi Configuration</h1><p>ESP32-P4 + C6 Bridge</p></div>"
"<div class='content'>"
"<div class='eth-prompt' id='ethPrompt'>"
"<h3><span>🔌</span>Important: Connect Ethernet Cable</h3>"
"<p>Please connect your PC/device to the Ethernet port of this device.</p>"
"<p>The system will learn the MAC address from your Ethernet connection.</p>"
"<p id='ethMacStatus'>Checking Ethernet connection...</p>"
"</div>"
"<button class='btn' onclick='scanNetworks()' id='scanBtn'>📡 Scan Networks</button>"
"<div class='network-list' id='networkList'></div>"
"<div class='form-group'><label>WiFi Name (SSID)</label>"
"<input type='text' id='ssid' placeholder='Select network or enter SSID'></div>"
"<div class='form-group'><label>Password</label>"
"<input type='password' id='password' placeholder='Enter WiFi password'></div>"
"<button class='btn' onclick='connectWiFi()' id='connectBtn'>✓ Connect</button>"
"<div class='status' id='status'></div>"
"</div></div>"
"<script>"
"let selectedSSID='';"
"function selectNetwork(ssid,btn){selectedSSID=ssid;document.getElementById('ssid').value=ssid;"
"document.querySelectorAll('.network-item').forEach(el=>el.classList.remove('selected'));"
"btn.classList.add('selected');document.getElementById('password').focus();}"
"function showStatus(msg,type){const s=document.getElementById('status');s.className='status show '+type;s.innerHTML=msg;}"
"function scanNetworks(){const btn=document.getElementById('scanBtn');"
"btn.disabled=true;btn.innerHTML='<span class=\"loading\"></span> Scanning...';"
"fetch('/scan').then(r=>r.json()).then(data=>{"
"const list=document.getElementById('networkList');"
"if(data.networks.length===0){list.innerHTML='<p style=\"text-align:center;color:#999;padding:20px\">No networks found</p>';}"
"else{list.innerHTML=data.networks.map(n=>"
"'<div class=\"network-item\" onclick=\"selectNetwork(\\''+n.ssid+'\\',this)\">"
"<span class=\"signal\">'+getSignalIcon(n.rssi)+'</span>"
"<span class=\"ssid\">'+n.ssid+'</span>"
"<span class=\"lock\">'+(n.auth!==0?'🔒':'')+'</span></div>').join('');}"
"btn.disabled=false;btn.innerHTML='📡 Scan Networks';}).catch(e=>{"
"showStatus('Scan failed: '+e,'error');btn.disabled=false;btn.innerHTML='📡 Scan Networks';});}"
"function getSignalIcon(rssi){if(rssi>-50)return'📶';if(rssi>-70)return'📶';return'📶';}"
"function connectWiFi(){const ssid=document.getElementById('ssid').value.trim();"
"const pwd=document.getElementById('password').value;"
"if(!ssid){showStatus('Please select or enter WiFi network','error');return;}"
"if(!pwd){showStatus('Please enter password','error');return;}"
"const btn=document.getElementById('connectBtn');"
"btn.disabled=true;btn.innerHTML='<span class=\"loading\"></span> Connecting...';"
"showStatus('Connecting to '+ssid+'...','info');"
"fetch('/connect?ssid='+encodeURIComponent(ssid)+'&password='+encodeURIComponent(pwd))"
".then(r=>r.json()).then(data=>{"
"if(data.success){showStatus('✓ Connected! Device will restart...','success');"
"setTimeout(()=>window.location.reload(),3000);}"
"else{showStatus('✗ Connection failed: '+data.message,'error');"
"btn.disabled=false;btn.innerHTML='✓ Connect';}}).catch(e=>{"
"showStatus('✗ Error: '+e,'error');btn.disabled=false;btn.innerHTML='✓ Connect';});}"
"function checkEthMac(){fetch('/eth_mac_status').then(r=>r.json()).then(data=>{"
"const status=document.getElementById('ethMacStatus');"
"if(data.success){"
"status.innerHTML='✓ Ethernet connected! MAC: <span class=\"eth-mac\">'+data.mac+'</span>';"
"document.getElementById('ethPrompt').style.background='#e8f5e9';"
"document.getElementById('ethPrompt').style.color='#388e3c';"
"document.getElementById('ethPrompt').style.borderColor='#388e3c';"
"}else{"
"status.innerHTML='⚠ '+data.message;"
"setTimeout(checkEthMac,2000);"
"}}).catch(()=>setTimeout(checkEthMac,2000));}"
"window.onload=function(){scanNetworks();checkEthMac();};"
"</script></body></html>";

/**
 * Root page handler
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, config_page_html, strlen(config_page_html));
    return ESP_OK;
}

/**
 * Captive portal redirect handler for common detection URLs
 * This handles requests to domains like:
 * - connectivitycheck.gstatic.com (Android)
 * - captive.apple.com (iOS)
 * - www.msftconnecttest.com (Windows)
 */
static esp_err_t captive_portal_redirect_handler(httpd_req_t *req)
{
    // Get the Host header to redirect to the actual domain requested
    char host[128] = "192.168.4.1";
    size_t host_len = sizeof(host);
    if (httpd_req_get_hdr_value_str(req, "Host", host, host_len) == ESP_OK) {
        ESP_LOGI(TAG, "Captive portal redirect from: %s", host);
    }
    
    // Return 302 redirect to our config page using the requested host
    httpd_resp_set_status(req, "302 Found");
    char location[256];
    snprintf(location, sizeof(location), "http://%s/", host);
    httpd_resp_set_hdr(req, "Location", location);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/**
 * WiFi scan handler
 */
static esp_err_t scan_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Scanning WiFi networks...");
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false
    };
    
    // Start scan
    // Regular esp_wifi API - forwarded to C6 via esp_wifi_remote
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(ret));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"networks\":[]}");
        return ESP_OK;
    }
    
    // Get scan results
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    ESP_LOGI(TAG, "Found %d networks", ap_count);
    
    // Allocate JSON buffer on heap to avoid stack overflow
    char *json = malloc(4096);
    if (!json) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"networks\":[]}");
        return ESP_OK;
    }
    
    strcpy(json, "{\"networks\":[");
    size_t offset = strlen(json);
    
    if (ap_count > 0) {
        wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
        if (ap_records) {
            ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
            if (ret == ESP_OK) {
                for (int i = 0; i < ap_count && offset < 4096 - 200; i++) {
                    if (i > 0) {
                        offset += snprintf(json + offset, 4096 - offset, ",");
                    }
                    offset += snprintf(json + offset, 4096 - offset,
                        "{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
                        ap_records[i].ssid,
                        ap_records[i].rssi,
                        ap_records[i].authmode);
                }
            }
            free(ap_records);
        }
    }
    
    snprintf(json + offset, 4096 - offset, "]}");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    
    free(json);
    return ESP_OK;
}

/**
 * WiFi connect handler
 */
static esp_err_t connect_handler(httpd_req_t *req)
{
    char ssid[33] = {0};
    char password[65] = {0};
    
    // Parse query
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "ssid", ssid, sizeof(ssid));
        httpd_query_key_value(query, "password", password, sizeof(password));
    }
    
    if (strlen(ssid) == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"SSID empty\"}");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Testing connection to WiFi: %s", ssid);
    
    // Configure WiFi credentials for STA interface
    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    
    // Set configuration (in RAM first, not flash yet)
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"Failed to set config\"}");
        return ESP_OK;
    }
    
    // Disconnect if already connected
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to connect to verify credentials
    ESP_LOGI(TAG, "Attempting connection test...");
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start connection: %s", esp_err_to_name(ret));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"Connection failed to start\"}");
        return ESP_OK;
    }
    
    // Wait for connection result (with timeout)
    // In APSTA mode, we can briefly test connection while AP stays active
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait up to 5 seconds for connection
    
    // Check if connected
    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret == ESP_OK) {
        // Connection successful - save to P4's NVS (not C6's flash)
        ESP_LOGI(TAG, "Connection test successful, saving configuration to P4's NVS");
        ret = save_wifi_credentials(ssid, password);
        
        if (ret == ESP_OK) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"Connected successfully\"}");
            
            // Disconnect from test connection
            esp_wifi_disconnect();
            
            // Signal success to restart in bridge mode
            if (s_event_flags) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                xEventGroupSetBits(*s_event_flags, s_success_bit);
            }
        } else {
            ESP_LOGE(TAG, "Failed to save credentials to NVS");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"Failed to save credentials\"}");
        }
    } else {
        ESP_LOGW(TAG, "Connection test failed: %s", esp_err_to_name(ret));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"Wrong password or network unavailable\"}");
    }
    
    return ESP_OK;
}

/**
 * Ethernet MAC status handler
 * Returns the status of Ethernet MAC learning
 */
static esp_err_t eth_mac_status_handler(httpd_req_t *req)
{
    uint8_t eth_mac[6] = {0};
    bool mac_saved = (load_eth_mac(eth_mac) == ESP_OK);
    
    char json[256];
    if (mac_saved) {
        snprintf(json, sizeof(json),
                 "{\"success\":true,\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"message\":\"Ethernet MAC detected\"}",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
    } else {
        snprintf(json, sizeof(json),
                 "{\"success\":false,\"message\":\"Please connect Ethernet cable and wait...\"}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

static const httpd_uri_t uri_root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
static const httpd_uri_t uri_scan = {.uri = "/scan", .method = HTTP_GET, .handler = scan_handler};
static const httpd_uri_t uri_connect = {.uri = "/connect", .method = HTTP_GET, .handler = connect_handler};
static const httpd_uri_t uri_eth_mac = {.uri = "/eth_mac_status", .method = HTTP_GET, .handler = eth_mac_status_handler};

// Captive portal detection URLs
static const httpd_uri_t uri_generate_204 = {.uri = "/generate_204", .method = HTTP_GET, .handler = captive_portal_redirect_handler};
static const httpd_uri_t uri_hotspot = {.uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_portal_redirect_handler};
static const httpd_uri_t uri_connecttest = {.uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_portal_redirect_handler};
static const httpd_uri_t uri_redirect = {.uri = "/redirect", .method = HTTP_GET, .handler = captive_portal_redirect_handler};
static const httpd_uri_t uri_success = {.uri = "/success.txt", .method = HTTP_GET, .handler = captive_portal_redirect_handler};

/**
 * Start web server
 */
static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting web server");
    if (httpd_start(&s_web_server, &config) == ESP_OK) {
        httpd_register_uri_handler(s_web_server, &uri_root);
        httpd_register_uri_handler(s_web_server, &uri_scan);
        httpd_register_uri_handler(s_web_server, &uri_connect);
        httpd_register_uri_handler(s_web_server, &uri_eth_mac);
        
        // Register captive portal detection URLs for auto-popup
        httpd_register_uri_handler(s_web_server, &uri_generate_204);  // Android
        httpd_register_uri_handler(s_web_server, &uri_hotspot);        // iOS
        httpd_register_uri_handler(s_web_server, &uri_connecttest);    // Windows
        httpd_register_uri_handler(s_web_server, &uri_redirect);       // Generic
        httpd_register_uri_handler(s_web_server, &uri_success);        // Generic
        
        ESP_LOGI(TAG, "Captive portal handlers registered for auto-popup");
    }
}

/**
 * WiFi event handler for AP mode
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station connected, MAC:" MACSTR, MAC2STR(event->mac));
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station disconnected, MAC:" MACSTR, MAC2STR(event->mac));
    }
}

/**
 * Start WiFi configuration portal
 */
esp_err_t start_wifi_config_portal(EventGroupHandle_t *flags, int success_bit, int fail_bit)
{
    ESP_LOGI(TAG, "Starting WiFi configuration portal");
    
    s_event_flags = flags;
    s_success_bit = success_bit;
    
    // ========================================================================
    // Initialize Ethernet to get MAC address
    // Following official ESP-IDF bridge example pattern
    // ========================================================================
    ESP_LOGI(TAG, "Initializing Ethernet to obtain MAC address...");
    
    // Create Ethernet netif (same as official bridge example)
    esp_netif_inherent_config_t eth_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t netif_cfg = {
        .base = &eth_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif");
    } else {
        // Initialize Ethernet driver
        uint8_t eth_port_cnt = 0;
        esp_eth_handle_t *eth_handles;
        esp_err_t ret = ethernet_init_all(&eth_handles, &eth_port_cnt);
        
        if (ret == ESP_OK && eth_port_cnt > 0) {
            s_eth_handle = eth_handles[0];
            free(eth_handles);
            
            // Attach Ethernet driver to netif
            ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
            
            // Start Ethernet
            ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
            
            // Wait briefly for Ethernet link to come up
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Get Ethernet MAC address
            uint8_t eth_mac[6];
            ret = esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, eth_mac);
            if (ret == ESP_OK) {
                // Save MAC to NVS
                save_eth_mac(eth_mac);
                ESP_LOGI(TAG, "Ethernet MAC saved: %02x:%02x:%02x:%02x:%02x:%02x",
                         eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
            } else {
                ESP_LOGW(TAG, "Failed to get Ethernet MAC: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "No Ethernet interface found or initialization failed");
        }
    }
    
    // ========================================================================
    // Initialize WiFi for configuration portal
    // ========================================================================
    
    // Create AP netif
    s_ap_netif = esp_netif_create_default_wifi_ap();
    
    // Create STA netif for scanning and testing connection
    // Note: We don't need to store the netif pointer - it's automatically registered
    (void)esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi in APSTA mode (AP + STA coexistence)
    // This allows C6 to run SoftAP while also scanning and testing connections
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_remote_init(&cfg));
    
    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_REMOTE_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    
    // Clear any stored credentials on C6 to ensure clean state
    ESP_LOGI(TAG, "Clearing any stored credentials on C6");
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    wifi_config_t empty_cfg = {0};
    esp_wifi_set_config(WIFI_IF_STA, &empty_cfg);
    
    // Set storage to RAM only - credentials will be stored on P4's NVS
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    
    // Configure AP
    wifi_config_t ap_config = {
        .ap = {
            .ssid = SOFTAP_SSID,
            .ssid_len = strlen(SOFTAP_SSID),
            .channel = SOFTAP_CHANNEL,
            .password = "",
            .max_connection = SOFTAP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    
    // Use APSTA mode to allow scanning and connection testing while running SoftAP
    // This is essential for WiFi configuration portal functionality
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "SoftAP started in APSTA mode: SSID=%s", SOFTAP_SSID);
    ESP_LOGI(TAG, "STA interface enabled for WiFi scanning and testing");
    
    // Start web server
    start_webserver();
    
    // Start DNS server for captive portal
    dns_server_config_t dns_config = DNS_SERVER_CONFIG_SINGLE("*", "ap");
    start_dns_server(&dns_config);
    
    ESP_LOGI(TAG, "Configuration portal ready:");
    ESP_LOGI(TAG, "  1. Connect PC to Ethernet port (to learn MAC address)");
    ESP_LOGI(TAG, "  2. Connect phone to WiFi: %s (no password)", SOFTAP_SSID);
    ESP_LOGI(TAG, "  3. Browser will auto-open or go to: http://192.168.4.1");
    ESP_LOGI(TAG, "  4. Wait for Ethernet MAC detection, then configure WiFi");
    
    return ESP_OK;
}

/**
 * Save WiFi credentials to P4's NVS (not C6's flash)
 */
esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save SSID
    ret = nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Save password
    ret = nvs_set_str(nvs_handle, "wifi_pass", password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials saved to P4's NVS: SSID=%s", ssid);
    }
    
    return ret;
}

/**
 * Load WiFi credentials from P4's NVS
 */
esp_err_t load_wifi_credentials(char *ssid, char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Load SSID
    size_t ssid_len = 33;
    ret = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load SSID: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Load password
    size_t pass_len = 65;
    ret = nvs_get_str(nvs_handle, "wifi_pass", password, &pass_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load password: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi credentials loaded from P4's NVS: SSID=%s", ssid);
    
    return ESP_OK;
}

/**
 * Clear WiFi credentials from P4's NVS
 */
esp_err_t clear_wifi_credentials(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    nvs_erase_key(nvs_handle, "wifi_ssid");
    nvs_erase_key(nvs_handle, "wifi_pass");
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "WiFi credentials cleared from P4's NVS");
    return ret;
}

/**
 * Check if WiFi is provisioned
 * Reads directly from P4's NVS without requiring WiFi initialization
 */
bool is_wifi_provisioned(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "NVS not opened, WiFi not provisioned");
        return false;
    }
    
    // Try to read WiFi SSID from P4's NVS
    char ssid[33] = {0};
    size_t len = sizeof(ssid);
    ret = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &len);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK && ssid[0] != 0) {
        ESP_LOGI(TAG, "WiFi credentials found in P4's NVS");
        return true;
    }
    
    ESP_LOGI(TAG, "No WiFi credentials in P4's NVS");
    return false;
}

/**
 * Save Ethernet MAC address to P4's NVS
 */
esp_err_t save_eth_mac(const uint8_t *eth_mac)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for ETH MAC save: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save MAC as blob (6 bytes)
    ret = nvs_set_blob(nvs_handle, "eth_mac", eth_mac, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save ETH MAC: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Ethernet MAC saved to P4's NVS: %02x:%02x:%02x:%02x:%02x:%02x",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
    }
    
    return ret;
}

/**
 * Load Ethernet MAC address from P4's NVS
 */
esp_err_t load_eth_mac(uint8_t *eth_mac)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("sta2eth", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for ETH MAC load: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Load MAC as blob
    size_t mac_len = 6;
    ret = nvs_get_blob(nvs_handle, "eth_mac", eth_mac, &mac_len);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK && mac_len == 6) {
        ESP_LOGI(TAG, "Ethernet MAC loaded from P4's NVS: %02x:%02x:%02x:%02x:%02x:%02x",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
        return ESP_OK;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load ETH MAC: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Check if Ethernet MAC is saved
 */
bool is_eth_mac_saved(void)
{
    uint8_t mac[6];
    return (load_eth_mac(mac) == ESP_OK);
}
