/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>


#include <u_cfg_app_platform_specific.h>
#include <ubxlib.h>


static const uDeviceCfg_t gDeviceCfg = {
	.deviceType = U_DEVICE_TYPE_CELL,
	.deviceCfg  = {
		.cfgCell = {
			.moduleType        = U_CELL_MODULE_TYPE_SARA_R5,
			.pSimPinCode       = NULL, /* SIM pin */
			.pinEnablePower    = U_CFG_APP_PIN_CELL_ENABLE_POWER,
			.pinPwrOn          = U_CFG_APP_PIN_CELL_PWR_ON,
			.pinVInt           = U_CFG_APP_PIN_CELL_VINT,
			.pinDtrPowerSaving = U_CFG_APP_PIN_CELL_DTR },
	       },
	       .transportType = U_DEVICE_TRANSPORT_TYPE_UART,
	       .transportCfg  = {
		.cfgUart = {
			.uart = U_CFG_APP_CELL_UART, .baudRate = U_CELL_UART_BAUD_RATE, .pinTxd = U_CFG_APP_PIN_CELL_TXD, .pinRxd = U_CFG_APP_PIN_CELL_RXD, .pinCts = U_CFG_APP_PIN_CELL_CTS, .pinRts = U_CFG_APP_PIN_CELL_RTS,
			.pPrefix = NULL, // Relevant for Linux only
		   },
	       },
	   };

int main(void)
{
	uDeviceHandle_t devHandle = NULL;
	//struct sockaddr_in destinationAddress;
	int32_t sock;
	const char message[] = "The quick brown zephyr-fox jumps over the lazy dog.";
	size_t txSize = sizeof(message);
	char buffer[128];
	size_t rxSize = 0;
	int32_t returnCode;

	// Initialise the APIs we will need
	uPortInit();
	uDeviceInit();

	// Open the device
	returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
	uPortLog("Opened device with return code %d.\n", returnCode);
}
