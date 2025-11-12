/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start WiFi configuration portal using SoftAP on C6
 * 
 * This creates a SoftAP on ESP32-C6 for WiFi configuration via mobile phone
 * 
 * @param flags Event group for synchronization
 * @param success_bit Bit to set when configuration succeeds
 * @param fail_bit Bit to set when configuration fails
 * @return ESP_OK on success
 */
esp_err_t start_wifi_config_portal(EventGroupHandle_t *flags, int success_bit, int fail_bit);

/**
 * @brief Check if WiFi is provisioned
 * 
 * @return true if provisioned
 */
bool is_wifi_provisioned(void);

/**
 * @brief Save WiFi credentials to P4's NVS
 * 
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t save_wifi_credentials(const char *ssid, const char *password);

/**
 * @brief Load WiFi credentials from P4's NVS
 * 
 * @param ssid Buffer to store SSID (must be at least 33 bytes)
 * @param password Buffer to store password (must be at least 65 bytes)
 * @return ESP_OK on success
 */
esp_err_t load_wifi_credentials(char *ssid, char *password);

/**
 * @brief Clear WiFi credentials from P4's NVS
 * 
 * @return ESP_OK on success
 */
esp_err_t clear_wifi_credentials(void);

/**
 * @brief Save Ethernet MAC address to P4's NVS
 * 
 * @param eth_mac Ethernet MAC address (6 bytes)
 * @return ESP_OK on success
 */
esp_err_t save_eth_mac(const uint8_t *eth_mac);

/**
 * @brief Load Ethernet MAC address from P4's NVS
 * 
 * @param eth_mac Buffer to store Ethernet MAC (must be at least 6 bytes)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not saved
 */
esp_err_t load_eth_mac(uint8_t *eth_mac);

/**
 * @brief Check if Ethernet MAC is saved
 * 
 * @return true if MAC is saved
 */
bool is_eth_mac_saved(void);

#ifdef __cplusplus
}
#endif
