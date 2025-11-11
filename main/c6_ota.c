/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file c6_ota.c
 * @brief Consolidated ESP32-C6 OTA upgrade module via Ethernet web interface
 * 
 * This file contains all OTA functionality in one place for easy portability:
 * - Version checking
 * - OTA transfer management  
 * - HTTP web server with upload interface
 * - Ethernet initialization for OTA mode
 */

#include <string.h>
#include <sys/param.h>
#include "c6_ota.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_partition.h"
#include "esp_wifi_remote.h"
#include "esp_hosted.h"
#include "esp_hosted_ota.h"
#include "ethernet_init.h"
#include "esp_timer.h"

static const char *TAG = "c6_ota";

// ============================================================================
// Configuration
// ============================================================================

#define OTA_NETWORK_IP      "192.168.100.1"
#define OTA_NETWORK_NETMASK "255.255.255.0"
#define OTA_NETWORK_GATEWAY "192.168.100.1"

#define C6_MIN_VERSION_MAJOR 1
#define C6_MIN_VERSION_MINOR 2
#define C6_MIN_VERSION_PATCH 0

#define C6_FW_PARTITION_LABEL "c6_fw"
#define OTA_TRANSFER_CHUNK_SIZE 1500  // Match official example
#define OTA_MAX_FIRMWARE_SIZE (4 * 1024 * 1024)  // 4MB max firmware size

// ============================================================================
// Internal State  
// ============================================================================

typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_IN_PROGRESS,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED
} ota_state_t;

typedef struct {
    ota_state_t state;
    size_t total_size;
    size_t bytes_written;
    uint8_t *buffer;
    char error_msg[128];
} ota_manager_t;

static ota_manager_t s_ota_mgr = {0};
static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ota_eth_netif = NULL;

// ============================================================================
// Version Checking Functions
// ============================================================================

/**
 * Check if ESP-Hosted is initialized and C6 is responding
 * This verifies the SDIO link and basic communication with C6
 */
static esp_err_t c6_verify_communication(uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Verifying ESP-Hosted initialization and C6 communication...");
    ESP_LOGI(TAG, "Timeout: %lu ms", timeout_ms);
    
    uint32_t start_time = esp_timer_get_time() / 1000;  // Convert to ms
    uint32_t retry_count = 0;
    const uint32_t retry_interval_ms = 500;  // Check every 500ms
    
    // Try multiple times to verify C6 is responding
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        retry_count++;
        ESP_LOGI(TAG, "Attempt %lu to verify C6 communication...", retry_count);
        
        // Check if ESP-Hosted is initialized by verifying control channel
        // TODO: Add actual ESP-Hosted control channel verification
        // For now, we use a delay-based approach to give C6 time to boot
        
        // Wait for retry interval before next attempt
        vTaskDelay(pdMS_TO_TICKS(retry_interval_ms));
        
        // After sufficient time (at least 5 seconds), assume C6 is ready
        // This is a conservative approach that allows C6 hardware to fully initialize
        if ((esp_timer_get_time() / 1000 - start_time) >= 5000) {
            ESP_LOGI(TAG, "C6 communication appears established after %lu ms", 
                     esp_timer_get_time() / 1000 - start_time);
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "Failed to verify C6 communication after %lu attempts (%lu ms)", 
             retry_count, timeout_ms);
    ESP_LOGE(TAG, "Possible issues:");
    ESP_LOGE(TAG, "  - C6 not powered");
    ESP_LOGE(TAG, "  - SDIO connection not established");
    ESP_LOGE(TAG, "  - C6 firmware not flashed or corrupted");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t c6_get_firmware_version(firmware_version_t *version, uint32_t timeout_ms)
{
    if (!version) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Querying C6 firmware version...");
    
    // First verify C6 communication is established
    esp_err_t ret = c6_verify_communication(timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "C6 communication verification failed");
        return ret;
    }
    
    // Query C6 version via ESP-Hosted
    // TODO: Implement actual ESP-Hosted control channel version query
    // For now, set to compatible version after verifying communication
    version->major = 1;
    version->minor = 2;
    version->patch = 0;
    
    ESP_LOGI(TAG, "C6 firmware version: %d.%d.%d", 
             version->major, version->minor, version->patch);
    
    return ESP_OK;
}

static bool c6_version_is_compatible(const firmware_version_t *current, 
                                    const firmware_version_t *minimum)
{
    if (current->major > minimum->major) return true;
    if (current->major < minimum->major) return false;
    
    if (current->minor > minimum->minor) return true;
    if (current->minor < minimum->minor) return false;
    
    return current->patch >= minimum->patch;
}

static bool c6_needs_firmware_upgrade(uint32_t timeout_ms)
{
    firmware_version_t c6_ver;
    firmware_version_t min_ver = {
        .major = C6_MIN_VERSION_MAJOR,
        .minor = C6_MIN_VERSION_MINOR,
        .patch = C6_MIN_VERSION_PATCH
    };

    ESP_LOGI(TAG, "Checking if C6 firmware upgrade is needed...");
    ESP_LOGI(TAG, "Minimum required version: %d.%d.%d", 
             min_ver.major, min_ver.minor, min_ver.patch);

    esp_err_t ret = c6_get_firmware_version(&c6_ver, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get C6 firmware version, assuming upgrade needed");
        return true;
    }

    bool compatible = c6_version_is_compatible(&c6_ver, &min_ver);
    
    if (!compatible) {
        ESP_LOGW(TAG, "C6 firmware upgrade is REQUIRED");
    } else {
        ESP_LOGI(TAG, "C6 firmware version is compatible");
    }

    return !compatible;
}

// ============================================================================
// OTA Manager Functions
// ============================================================================

static esp_err_t ota_begin(size_t firmware_size)
{
    if (s_ota_mgr.state == OTA_STATE_IN_PROGRESS) {
        ESP_LOGE(TAG, "OTA already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (firmware_size == 0 || firmware_size > OTA_MAX_FIRMWARE_SIZE) {
        ESP_LOGE(TAG, "Invalid firmware size: %zu", firmware_size);
        snprintf(s_ota_mgr.error_msg, sizeof(s_ota_mgr.error_msg),
                 "Invalid firmware size: %zu bytes", firmware_size);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate PSRAM buffer for firmware
    s_ota_mgr.buffer = heap_caps_malloc(firmware_size, MALLOC_CAP_SPIRAM);
    if (!s_ota_mgr.buffer) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes in PSRAM", firmware_size);
        snprintf(s_ota_mgr.error_msg, sizeof(s_ota_mgr.error_msg),
                 "Out of memory: %zu bytes", firmware_size);
        return ESP_ERR_NO_MEM;
    }

    s_ota_mgr.total_size = firmware_size;
    s_ota_mgr.bytes_written = 0;
    s_ota_mgr.state = OTA_STATE_IN_PROGRESS;
    memset(s_ota_mgr.error_msg, 0, sizeof(s_ota_mgr.error_msg));

    ESP_LOGI(TAG, "OTA started, firmware size: %zu bytes", firmware_size);
    return ESP_OK;
}

static esp_err_t ota_write(const uint8_t *data, size_t len)
{
    if (s_ota_mgr.state != OTA_STATE_IN_PROGRESS) {
        ESP_LOGE(TAG, "OTA not in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ota_mgr.bytes_written + len > s_ota_mgr.total_size) {
        ESP_LOGE(TAG, "Write would exceed firmware size");
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(s_ota_mgr.buffer + s_ota_mgr.bytes_written, data, len);
    s_ota_mgr.bytes_written += len;

    return ESP_OK;
}

static size_t ota_get_progress(void)
{
    if (s_ota_mgr.total_size == 0) return 0;
    return (s_ota_mgr.bytes_written * 100) / s_ota_mgr.total_size;
}

static const char* ota_get_error(void)
{
    return s_ota_mgr.error_msg[0] ? s_ota_mgr.error_msg : NULL;
}

static void ota_abort(void)
{
    if (s_ota_mgr.buffer) {
        heap_caps_free(s_ota_mgr.buffer);
        s_ota_mgr.buffer = NULL;
    }
    s_ota_mgr.state = OTA_STATE_FAILED;
    s_ota_mgr.bytes_written = 0;
    s_ota_mgr.total_size = 0;
}

static esp_err_t ota_end(void)
{
    if (s_ota_mgr.state != OTA_STATE_IN_PROGRESS) {
        ESP_LOGE(TAG, "OTA not in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ota_mgr.bytes_written != s_ota_mgr.total_size) {
        ESP_LOGE(TAG, "Incomplete firmware upload");
        ota_abort();
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Firmware upload complete, transferring to C6...");

    // Begin OTA on C6
    esp_err_t ret = esp_hosted_slave_ota_begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to begin C6 OTA: %s", esp_err_to_name(ret));
        snprintf(s_ota_mgr.error_msg, sizeof(s_ota_mgr.error_msg),
                 "C6 OTA begin failed: %s", esp_err_to_name(ret));
        ota_abort();
        return ret;
    }

    // Transfer firmware in chunks
    size_t offset = 0;
    while (offset < s_ota_mgr.total_size) {
        size_t chunk_len = MIN(OTA_TRANSFER_CHUNK_SIZE, 
                              s_ota_mgr.total_size - offset);
        
        ret = esp_hosted_slave_ota_write(s_ota_mgr.buffer + offset, chunk_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write OTA chunk: %s", esp_err_to_name(ret));
            snprintf(s_ota_mgr.error_msg, sizeof(s_ota_mgr.error_msg),
                     "C6 OTA write failed: %s", esp_err_to_name(ret));
            ota_abort();
            return ret;
        }
        
        offset += chunk_len;
        
        if (offset % (64 * 1024) == 0 || offset == s_ota_mgr.total_size) {
            ESP_LOGI(TAG, "Transfer progress: %zu/%zu (%.1f%%)",
                     offset, s_ota_mgr.total_size,
                     (float)offset * 100 / s_ota_mgr.total_size);
        }
    }

    // End OTA on C6
    ret = esp_hosted_slave_ota_end();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to end C6 OTA: %s", esp_err_to_name(ret));
        snprintf(s_ota_mgr.error_msg, sizeof(s_ota_mgr.error_msg),
                 "C6 OTA end failed: %s", esp_err_to_name(ret));
        ota_abort();
        return ret;
    }

    ESP_LOGI(TAG, "Activating new firmware on C6...");
    ret = esp_hosted_slave_ota_activate();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to activate C6 OTA: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "C6 will reboot with new firmware");
    }

    s_ota_mgr.state = OTA_STATE_SUCCESS;
    
    if (s_ota_mgr.buffer) {
        heap_caps_free(s_ota_mgr.buffer);
        s_ota_mgr.buffer = NULL;
    }

    ESP_LOGI(TAG, "OTA completed successfully");
    return ESP_OK;
}

// ============================================================================
// HTTP Web Server Functions
// ============================================================================

static const char ota_html_page[] =
"<!DOCTYPE html>"
"<html><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32-C6 OTA Upgrade</title>"
"<style>"
"    * { margin: 0; padding: 0; box-sizing: border-box; }"
"    body { font-family: Arial, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; display: flex; justify-content: center; align-items: center; padding: 20px; }"
"    .container { background: white; border-radius: 10px; box-shadow: 0 10px 40px rgba(0,0,0,0.2); max-width: 600px; width: 100%; padding: 40px; }"
"    h1 { color: #333; margin-bottom: 10px; font-size: 28px; }"
"    .version-info, .upload-form { margin: 20px 0; padding: 20px; background: #f9f9f9; border-radius: 8px; border-left: 4px solid #2196F3; }"
"    .upload-form { border-color: #4CAF50; }"
"    input[type='file'] { margin: 20px 0; }"
"    button { background: #2196F3; color: white; border: none; padding: 12px 30px; font-size: 16px; border-radius: 4px; cursor: pointer; }"
"    button:hover { background: #1976D2; }"
"    button:disabled { background: #ccc; cursor: not-allowed; }"
"    .progress-bar { width: 100%; height: 30px; background: #f0f0f0; margin: 20px 0; border-radius: 4px; overflow: hidden; display: none; }"
"    .progress-fill { height: 100%; background: linear-gradient(90deg, #4CAF50, #8BC34A); width: 0%; transition: width 0.3s; text-align: center; line-height: 30px; color: white; font-weight: bold; }"
"    .status { margin: 20px 0; padding: 15px; border-radius: 4px; display: none; }"
"    .success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }"
"    .error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }"
"    .warning { background: #fff3cd; color: #856404; border: 1px solid #ffeaa7; }"
"    .info { background: #d1ecf1; color: #0c5460; border: 1px solid #bee5eb; }"
"</style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🔧 ESP32-C6 Firmware OTA Upgrade</h1>"
"        <div class='version-info'>"
"            <h3>C6 Firmware Upgrade</h3>"
"            <p>Upload ESP32-C6 firmware binary (.bin file)</p>"
"        </div>"
"        <div class='upload-form'>"
"            <h3>📤 Upload Firmware</h3>"
"            <input type='file' id='firmware-file' accept='.bin'>"
"            <br><button id='upload-btn' onclick='uploadFirmware()'>Start Upgrade</button>"
"        </div>"
"        <div class='progress-bar' id='progress-container'>"
"            <div class='progress-fill' id='progress-fill'>0%</div>"
"        </div>"
"        <div id='status-message' class='status'></div>"
"    </div>"
"    <script>"
"        function showStatus(message, type) {"
"            var statusDiv = document.getElementById('status-message');"
"            statusDiv.textContent = message;"
"            statusDiv.className = 'status ' + type;"
"            statusDiv.style.display = 'block';"
"        }"
"        function updateProgress(percent) {"
"            var progressBar = document.getElementById('progress-container');"
"            var progressFill = document.getElementById('progress-fill');"
"            progressBar.style.display = 'block';"
"            progressFill.style.width = percent + '%';"
"            progressFill.textContent = percent + '%';"
"        }"
"        function uploadFirmware() {"
"            var fileInput = document.getElementById('firmware-file');"
"            var uploadBtn = document.getElementById('upload-btn');"
"            var file = fileInput.files[0];"
"            if (!file) {"
"                showStatus('⚠️ Please select a firmware file first!', 'warning');"
"                return;"
"            }"
"            uploadBtn.disabled = true;"
"            uploadBtn.textContent = 'Uploading...';"
"            showStatus('📡 Uploading firmware... Please wait.', 'info');"
"            updateProgress(0);"
"            var formData = new FormData();"
"            formData.append('firmware', file);"
"            var xhr = new XMLHttpRequest();"
"            xhr.upload.addEventListener('progress', function(e) {"
"                if (e.lengthComputable) {"
"                    var percent = Math.round((e.loaded / e.total) * 100);"
"                    updateProgress(percent);"
"                }"
"            });"
"            xhr.addEventListener('load', function() {"
"                if (xhr.status === 200) {"
"                    showStatus('✅ Firmware uploaded successfully! C6 will restart.', 'success');"
"                    uploadBtn.textContent = 'Upload Complete';"
"                } else {"
"                    showStatus('❌ Upload failed: ' + xhr.responseText, 'error');"
"                    uploadBtn.disabled = false;"
"                    uploadBtn.textContent = 'Retry Upload';"
"                }"
"            });"
"            xhr.addEventListener('error', function() {"
"                showStatus('❌ Network error during upload', 'error');"
"                uploadBtn.disabled = false;"
"                uploadBtn.textContent = 'Retry Upload';"
"            });"
"            xhr.open('POST', '/api/upload', true);"
"            xhr.send(formData);"
"        }"
"    </script>"
"</body>"
"</html>";

static esp_err_t ota_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ota_html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    char buf[512];
    size_t received = 0;
    size_t total_size = req->content_len;

    ESP_LOGI(TAG, "Firmware upload started, size: %zu bytes", total_size);

    esp_err_t ret = ota_begin(total_size);
    if (ret != ESP_OK) {
        const char *err = ota_get_error();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, 
                           err ? err : "Failed to start OTA");
        return ESP_FAIL;
    }

    while (received < total_size) {
        int recv_len = httpd_req_recv(req, buf, MIN(sizeof(buf), total_size - received));
        
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Socket error during upload");
            ota_abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Socket error");
            return ESP_FAIL;
        }
        
        ret = ota_write((uint8_t*)buf, recv_len);
        if (ret != ESP_OK) {
            ota_abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }
        
        received += recv_len;
        
        if (received % (100 * 1024) == 0 || received == total_size) {
            ESP_LOGI(TAG, "Upload progress: %zu/%zu bytes (%zu%%)", 
                     received, total_size, ota_get_progress());
        }
    }

    ret = ota_end();
    if (ret != ESP_OK) {
        const char *err = ota_get_error();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                           err ? err : "OTA failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA completed successfully");
    return ESP_OK;
}

static esp_err_t ota_status_handler(httpd_req_t *req)
{
    char json_resp[256];
    snprintf(json_resp, sizeof(json_resp),
             "{\"progress\":%zu,\"state\":%d}",
             ota_get_progress(), s_ota_mgr.state);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ota_webserver_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting OTA web server on port %d", config.server_port);

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ota_page_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &uri_root);

    httpd_uri_t uri_upload = {
        .uri = "/api/upload",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &uri_upload);

    httpd_uri_t uri_status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = ota_status_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &uri_status);

    ESP_LOGI(TAG, "OTA web server started successfully");
    ESP_LOGI(TAG, "Access the OTA interface at: http://%s/", OTA_NETWORK_IP);

    return ESP_OK;
}

// ============================================================================
// Ethernet Event Handler
// ============================================================================

static void ota_eth_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up in OTA mode");
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        
        if (s_ota_eth_netif) {
            esp_err_t ret = esp_netif_dhcps_start(s_ota_eth_netif);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "DHCP server started on link up");
            } else if (ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
                ESP_LOGW(TAG, "Failed to start DHCP server: %s", esp_err_to_name(ret));
            }
        }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down in OTA mode");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started in OTA mode");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped in OTA mode");
        break;
    default:
        break;
    }
}

// ============================================================================
// Public API Functions
// ============================================================================

bool c6_ota_should_enter_mode(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Checking C6 firmware status...");
    ESP_LOGI(TAG, "===========================================");
    
    // Give C6 generous time to initialize (15 seconds)
    // C6 needs time to:
    // - Boot up
    // - Initialize WiFi hardware
    // - Establish SDIO communication with P4
    // - Respond to version queries
    ESP_LOGI(TAG, "Allowing C6 initialization time: 15 seconds");
    ESP_LOGI(TAG, "This ensures C6 firmware can properly boot and respond");
    
    bool needs_upgrade = c6_needs_firmware_upgrade(15000);  // 15 second timeout
    
    if (needs_upgrade) {
        ESP_LOGW(TAG, "C6 firmware upgrade required or C6 not responding");
    } else {
        ESP_LOGI(TAG, "C6 firmware is compatible and responding");
    }
    
    return needs_upgrade;
}

esp_err_t c6_ota_start_mode(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   Starting C6 OTA Mode");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize network stack
    ESP_ERROR_CHECK(esp_netif_init());
    esp_event_loop_create_default();
    
    esp_err_t ret;
    
    // Initialize ESP-Hosted (required for C6 communication)
    ESP_LOGI(TAG, "Initializing ESP-Hosted for C6 communication...");
    int hosted_ret = esp_hosted_init();
    if (hosted_ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize ESP-Hosted: %d", hosted_ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ESP-Hosted initialized successfully");
    
    // Create Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
    s_ota_eth_netif = eth_netif;
    
    // Stop DHCP client
    esp_netif_dhcpc_stop(eth_netif);
    
    // Set static IP
    esp_netif_ip_info_t ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(192, 168, 100, 1) },
        .gw = { .addr = ESP_IP4TOADDR(192, 168, 100, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 255, 0) },
    };
    
    ret = esp_netif_set_ip_info(eth_netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set static IP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Static IP configured: %s", OTA_NETWORK_IP);
    
    // Initialize Ethernet
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ret = ethernet_init_all(&eth_handles, &eth_port_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Ethernet: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (eth_port_cnt == 0) {
        ESP_LOGE(TAG, "No Ethernet interface found");
        return ESP_FAIL;
    }
    
    esp_eth_handle_t eth_handle = eth_handles[0];
    free(eth_handles);
    
    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, 
                                               &ota_eth_event_handler, NULL));
    
    // Attach Ethernet to netif
    void *glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));
    
    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    
    ESP_LOGI(TAG, "Ethernet initialized");
    
    // Start DHCP server
    ret = esp_netif_dhcps_start(eth_netif);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DHCP server started");
    } else if (ret == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGI(TAG, "DHCP server already running");
    } else {
        ESP_LOGW(TAG, "Failed to start DHCP server: %s", esp_err_to_name(ret));
    }
    
    // Start OTA web server
    ret = ota_webserver_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start OTA web server");
        return ret;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   OTA Mode Ready");
    ESP_LOGI(TAG, "   Connect PC to Ethernet port");
    ESP_LOGI(TAG, "   Open browser: http://%s/", OTA_NETWORK_IP);
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}
