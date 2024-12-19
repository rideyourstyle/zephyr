/** @file
 * @brief HL7800 modem public API header file.
 *
 * Allows an application to control the HL7800 modem.
 *
 * Copyright (c) 2020 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_UBXLIB_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_UBXLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

#include <u_device.h>
#include <u_network.h>

// typedef void ( *connectedCallback )( void );
typedef void (*networkStatusCallback)(uNetworkStatus_t *pStatus);

/**
 * @brief Connect to network
 *
 * @return 0 for success
 */
int32_t mdm_ubxlib_bring_interface_up(const void *pCfg);

/**
 * @brief Connect to network
 *
 * @return 0 for success
 */
uint32_t mdm_ubxlib_interface_down(void);

/**
 * @brief Disconnect from network
 *
 * @return int32_t 0 for success
 */
int32_t mdm_ubxlib_disconnect(void);

/**
 * @brief Switch on cellular modem
 *
 * @return int32_t 0 for success
 */
int32_t mdm_ubxlib_power_on(void);

/**
 * @brief Switch off cellular modem
 *
 * @return int32_t 0 for success
 */
int32_t mdm_ubxlib_power_off(void);
int64_t mdm_ubxlib_get_time_UTC(void);
int32_t mdm_ubxlib_get_rssi_dbm(void);
int32_t mdm_reset_modem(void);

bool mdm_get_handle(uDeviceHandle_t *cellHandle);
// bool mdm_ubxlib_register_connect_cb(connectedCallback cb);
bool mdm_ubxlib_register_network_status_cb(networkStatusCallback cb);

///**
// * @brief Power off the HL7800
// *
// * @return int32_t 0 for success
// */
// int32_t mdm_hl7800_power_off(void);
//
///**
// * @brief Reset the HL7800 (and allow it to reconfigure).
// *
// * @return int32_t 0 for success
// */
// int32_t mdm_hl7800_reset(void);
//
///**
// * @brief Control the wake signals to the HL7800.
// * @note this API should only be used for debug purposes.
// *
// * @param awake True to keep the HL7800 awake, False to allow sleep
// */
// void mdm_hl7800_wakeup(bool awake);
//
///**
// * @brief Send an AT command to the HL7800.
// * @note this API should only be used for debug purposes.
// *
// * @param data AT command string
// * @return int32_t 0 for success
// */
// int32_t mdm_hl7800_send_at_cmd(const uint8_t *data);
//
///**
// * @brief Get the signal quality of the HL7800.
// * If CONFIG_MODEM_HL7800_RSSI_RATE_SECONDS is non-zero, then
// * this function returns the value from the last periodic read.
// * If CONFIG_MODEM_HL7800_RSSI_RATE_SECONDS is 0, then this
// * may cause the modem to be woken so that the values can be queried.
// *
// * @param rsrp Reference Signals Received Power (dBm)
// *             Range = -140 dBm to -44 dBm
// * @param sinr Signal to Interference plus Noise Ratio (dB)
// *             Range = -128 dB to 40 dB
// */
// void mdm_hl7800_get_signal_quality(int *rsrp, int *sinr);
//
///**
// * @brief Get the SIM card ICCID
// *
// */
// char *mdm_hl7800_get_iccid(void);
//
///**
// * @brief Get the HL7800 serial number
// *
// */
// char *mdm_hl7800_get_sn(void);
//
///**
// * @brief Get the HL7800 IMEI
// *
// */
// char *mdm_hl7800_get_imei(void);
//
///**
// * @brief Get the HL7800 firmware version
// *
// */
// char *mdm_hl7800_get_fw_version(void);
//
///**
// * @brief Get the IMSI
// *
// */
// char *mdm_hl7800_get_imsi(void);
//
///**
// * @brief Update the Access Point Name in the modem.
// *
// * @retval 0 on success, negative on failure.
// */
// int32_t mdm_hl7800_update_apn(char *access_point_name);
//
///**
// * @brief Update the Radio Access Technology (mode).
// *
// * @retval 0 on success, negative on failure.
// */
// int32_t mdm_hl7800_update_rat(enum mdm_hl7800_radio_mode value);
//
///**
// * @retval true if RAT value is valid
// */
// bool mdm_hl7800_valid_rat(uint8_t value);
//
///**
// * @brief Register a function that is called when a modem event occurs.
// * Multiple users registering for callbacks is supported.
// *
// * @param agent event callback agent
// */
// void mdm_hl7800_register_event_callback(struct mdm_hl7800_callback_agent *agent);
//
///**
// * @brief Unregister a callback event function
// *
// * @param agent event callback agent
// */
// void mdm_hl7800_unregister_event_callback(struct mdm_hl7800_callback_agent *agent);
//
///**
// * @brief Force modem module to generate status events.
// *
// * @note This can be used to get the current state when a module initializes
// * later than the modem.
// */
// void mdm_hl7800_generate_status_events(void);
//
///**
// * @brief Get the local time from the modem's real time clock.
// *
// * @param tm time structure
// * @param offset The amount the local time is offset from GMT/UTC in seconds.
// * @return int32_t 0 if successful
// */
// int32_t mdm_hl7800_get_local_time(struct tm *tm, int32_t *offset);
//
//#ifdef CONFIG_MODEM_HL7800_FW_UPDATE
///**
// * @brief Update the HL7800 via XMODEM protocol.  During the firmware update
// * no other modem fuctions will be available.
// *
// * @param file_path Absolute path of the update file
// *
// * @param 0 if successful
// */
// int32_t mdm_hl7800_update_fw(char *file_path);
//#endif
//
///**
// * @brief Read the operator index from the modem.
// *
// * @retval negative error code, 0 on success
// */
// int32_t mdm_hl7800_get_operator_index(void);
//
///**
// * @brief Get modem functionality
// *
// * @return int32_t negative errno on failure, else mdm_hl7800_functionality
// */
// int32_t mdm_hl7800_get_functionality(void);
//
///**
// * @brief Set airplane, normal, or reduced functionality mode.
// * Airplane mode persists when reset.
// *
// * @note Boot functionality is also controlled by Kconfig
// * MODEM_HL7800_BOOT_IN_AIRPLANE_MODE.
// *
// * @param mode
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_set_functionality(enum mdm_hl7800_functionality mode);
//
///**
// * @brief When rate is non-zero: Put modem into Airplane mode. Enable GPS and
// * generate HL7800_EVENT_GPS events.
// * When zero: Disable GPS and put modem into normal mode.
// *
// * @note Airplane mode isn't cleared when the modem is reset.
// *
// * @param rate in seconds to query location
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_set_gps_rate(uint32_t rate);
//
///**
// * @brief Register modem/SIM with polte.io
// *
// * @note It takes around 30 seconds for HL7800_EVENT_POLTE_REGISTRATION to
// * be generated.  If the applications saves the user and password
// * information into non-volatile memory, then this command
// * only needs to be run once.
// *
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_polte_register(void);
//
///**
// * @brief Enable PoLTE.
// *
// * @param user from polte.io or register command callback
// * @param password from polte.io register command callback
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_polte_enable(char *user, char *password);
//
///**
// * @brief Locate device using PoLTE.
// *
// * @note The first HL7800_EVENT_POLTE_LOCATE_STATUS event indicates
// * the status of issuing the locate command. The second event
// * requires 20-120 seconds to be generated and it contains the
// * location information (or indicates server failure).
// *
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_polte_locate(void);
//
///**
// * @brief Perform a site survey.  This command may return different values
// * each time it is run (depending on what is in range).
// *
// * HL7800_EVENT_SITE_SURVEY is generated for each response received from modem.
// *
// * @retval negative error code, 0 on success
// */
// int32_t mdm_hl7800_perform_site_survey(void);
//
///**
// * @brief Set desired sleep level. Requires MODEM_HL7800_LOW_POWER_MODE
// *
// * @param level (sleep, lite hibernate, or hibernate)
// * @return int negative errno, 0 on success
// */
// int mdm_hl7800_set_desired_sleep_level(enum mdm_hl7800_sleep level);
//
///**
// * @brief Allows mapping of WAKE_UP signal
// * to a user accessible test point on the development board.
// *
// * @param func to be called when application requests modem wake/sleep.
// * The state parameter of the callback is 1 when modem should stay awake,
// * 0 when modem can sleep
// */
// void mdm_hl7800_register_wake_test_point_callback(void (*func)(int state));
//
///**
// * @brief Allows mapping of P1.12_GPIO6 signal
// * to a user accessible test point on the development board.
// *
// * @param func to be called when modem wakes/sleeps is sleep level is
// * hibernate or lite hibernate.
// * The state parameter of the callback follows gpio_pin_get definitions,
// * but will default high if there is an error reading pin
// */
// void mdm_hl7800_register_gpio6_callback(void (*func)(int state));
//
///**
// * @brief Allows mapping of UART1_CTS signal
// * to a user accessible test point on the development board.
// *
// * @param func to be called when CTS state changes if sleep level is sleep.
// * The state parameter of the callback follows gpio_pin_get definitions,
// * but will default low if there is an error reading pin
// */
// void mdm_hl7800_register_cts_callback(void (*func)(int state));
//
///**
// * @brief Set the bands available for the LTE connection
// *
// * @param bands Band bitmap in hexadecimal format without the 0x prefix.
// * Leading 0's for the value can be ommited.
// *
// * @return int32_t negative errno, 0 on success
// */
// int32_t mdm_hl7800_set_bands(const char *bands);
//
///**
// * @brief Set the log level for the modem.
// *
// * @note It cannot be set higher than CONFIG_MODEM_LOG_LEVEL.
// * If debug level is desired, then it must be compiled with that level.
// *
// * @param level 0 (None) - 4 (Debug)
// *
// * @retval new log level
// */
// uint32_t mdm_hl7800_log_filter_set(uint32_t level);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_UBXLIB_H_ */