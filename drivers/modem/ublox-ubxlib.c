/*
 * Copyright (c) 2019-2020 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_ubxlib

#define U_DEVICE_PRIVATE_I2C_MAX_NUM	    0
#define U_DEVICE_PRIVATE_DEVICE_I2C_MAX_NUM 0

// #define U_CFG_OS_APP_TASK_PRIORITY	   4
// #define U_AT_CLIENT_CALLBACK_TASK_PRIORITY 4
// #define U_CFG_OS_TIMER_EVENT_TASK_PRIORITY 1

#include <ctype.h>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include <fcntl.h>

#if defined(CONFIG_MODEM_UBLOX_SARA_AUTODETECT_APN)
#include <stdio.h>
#endif

#include "modem_context.h"
#include "modem_socket.h"

#if !defined(CONFIG_MODEM_UBLOX_SARA_R4_MANUAL_MCCMNO)
#define CONFIG_MODEM_UBLOX_SARA_R4_MANUAL_MCCMNO ""
#endif

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#include "tls_internal.h"

#include <zephyr/net/tls_credentials.h>
#endif

#define SARA_VCC_ID	DT_NODELABEL(do_sara_vcc)
#define SARA_N_POWER_ID DT_NODELABEL(do_sara_n_power)
#define SARA_RESET_ID	DT_NODELABEL(do_sara_reset)

#define U_PORT_UART_MAX_NUM 1
#define U_CFG_APP_CELL_UART 0

#define U_CFG_APP_PIN_CELL_PWR_ON	-1
#define U_CFG_APP_PIN_CELL_ENABLE_POWER -1
#define U_CFG_APP_PIN_CELL_VINT		-1
#define U_CFG_APP_PIN_CELL_DTR		-1

#include <zephyr/logging/log.h>

#include <u_device_shared.h>

LOG_MODULE_REGISTER(ubx_wrapper);

#define U_CELL_NET_SCAN_RETRIES 1

#include "u_cfg_app_platform_specific.h"
#include "ubxlib.h"

#include <zephyr/drivers/modem/modem_ubxlib.h>

// pin settings
#if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_vcc_gpios)
static const struct gpio_dt_spec vcc_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_vcc_gpios);
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_power_on_gpios)
static const struct gpio_dt_spec power_on_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_power_on_gpios);
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_vint_gpios)
static const struct gpio_dt_spec vint_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_vint_gpios);
#endif

// #define MDM_UART_NODE DT_INST_BUS(0)
// #define MDM_UART_DEV  DEVICE_DT_GET(MDM_UART_NODE)

#define MDM_RESET_NOT_ASSERTED 1
#define MDM_RESET_ASSERTED     0

#define MDM_NET_CONNECT_TIMEOUT_SECONDS (180 * 1)

#define MDM_CMD_TIMEOUT		 K_SECONDS(10)
#define MDM_DNS_TIMEOUT		 K_SECONDS(70)
#define MDM_CMD_CONN_TIMEOUT	 K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT K_SECONDS(180)
#define MDM_PROMPT_CMD_DELAY	 K_MSEC(50)

#define MDM_MAX_DATA_LENGTH 1024
#define MDM_RECV_MAX_BUF    30
#define MDM_RECV_BUF_SIZE   128

#define MDM_MAX_SOCKETS	    6
#define MDM_BASE_SOCKET_NUM 0

// #define MDM_NETWORK_RETRY_COUNT 3
// #define MDM_WAIT_FOR_RSSI_COUNT 10
// #define MDM_WAIT_FOR_RSSI_DELAY K_SECONDS(2)

#define MDM_MANUFACTURER_LENGTH 10
#define MDM_MODEL_LENGTH	16
#define MDM_REVISION_LENGTH	64
#define MDM_IMEI_LENGTH		16
#define MDM_IMSI_LENGTH		16
#define MDM_APN_LENGTH		32
#define MDM_MAX_CERT_LENGTH	8192
// #if defined(CONFIG_MODEM_UBLOX_SARA_AUTODETECT_VARIANT)
// #define MDM_VARIANT_UBLOX_R4 4
// #define MDM_VARIANT_UBLOX_U2 2
// #endif

NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

#if defined(CONFIG_MODEM_UBLOX_SARA_RSSI_WORK)
/* RX thread work queue */
K_KERNEL_STACK_DEFINE(modem_workq_stack, CONFIG_MODEM_UBLOX_SARA_R4_RX_WORKQ_STACK_SIZE);
static struct k_work_q modem_workq;
#endif

/* socket read callback data */
struct socket_read_data {
	char *recv_buf;
	size_t recv_buf_len;
	struct sockaddr *recv_addr;
	uint16_t recv_read_len;
};

/* driver data */
struct modem_data {
	struct net_if *net_iface;
	uint8_t mac_addr[6];

	/* modem interface */
	uDeviceHandle_t cellHandle;
	uint64_t startTimeMs;
	int32_t ubxSocketId;

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	//	connectedCallback connectedCallback;
	networkStatusCallback networkStatusCallback;

#if defined(CONFIG_MODEM_UBLOX_SARA_RSSI_WORK)
	/* RSSI work */
	struct k_work_delayable rssi_query_work;
#endif

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
	char mdm_imsi[MDM_IMSI_LENGTH];
	int mdm_rssi;

#if defined(CONFIG_MODEM_UBLOX_SARA_AUTODETECT_VARIANT)
	/* modem variant */
	int mdm_variant;
#endif
#if defined(CONFIG_MODEM_UBLOX_SARA_AUTODETECT_APN)
	/* APN */
	char mdm_apn[MDM_APN_LENGTH];
#endif

	/* modem state */
	int ev_creg;

	/* bytes written to socket in last transaction */
	int sock_written;

	/* response semaphore */
	struct k_sem sem_response;

	/* prompt semaphore */
	struct k_sem sem_prompt;
};

static struct modem_data mdata;
static struct modem_context mctx;

#if defined(CONFIG_DNS_RESOLVER)
static struct zsock_addrinfo result;
static struct sockaddr result_addr;
static char result_canonname[DNS_MAX_NAME_SIZE + 1];
#endif

/* send binary data via the +USO[ST/WR] commands */
static ssize_t send_socket_data(void *obj, const struct msghdr *msg, k_timeout_t timeout)
{
	int32_t retVal = 0;
	uint16_t dst_port = 0U;
	struct modem_socket *sock = (struct modem_socket *)obj;
	struct sockaddr *dst_addr = msg->msg_name;
	size_t buf_len = 0;
	static uint32_t cntEnter = 0;
	static uint32_t cntExit = 0;

	if (!sock) {
		return -EINVAL;
	}

	LOG_WRN("send_socket_data: Enter: cntEnter: %d, cntExit: %d", cntEnter, cntExit);
	LOG_WRN("send_socket_data: mdata.ubxSocketId: %d, sock->id:", mdata.ubxSocketId, sock->id);

	for (int i = 0; i < msg->msg_iovlen; i++) {
		if (!msg->msg_iov[i].iov_base || msg->msg_iov[i].iov_len == 0) {
			errno = EINVAL;
			return -1;
		}
		buf_len += msg->msg_iov[i].iov_len;
	}

	if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP) {
		LOG_WRN("send_socket_data: sock->is_connected %d, sock->ip_proto %d",
			sock->is_connected, sock->ip_proto);
		errno = ENOTCONN;
		return -1;
	}

	if (!dst_addr && sock->ip_proto == IPPROTO_UDP) {
		dst_addr = &sock->dst;
	}

	/*
	 * Binary and ASCII mode allows sending MDM_MAX_DATA_LENGTH bytes to
	 * the socket in one command
	 */
	if (buf_len > MDM_MAX_DATA_LENGTH) {
		if (sock->type == SOCK_DGRAM) {
			errno = EMSGSIZE;
			return -1;
		}
		LOG_WRN("send_socket_data: Limit max data length to %d", MDM_MAX_DATA_LENGTH);
		buf_len = MDM_MAX_DATA_LENGTH;
	}

	mdata.sock_written = 0;

	if (sock->ip_proto == IPPROTO_UDP) {
		// TODO: implement UDP
		//		char ip_str[NET_IPV6_ADDR_LEN];
		//
		//		ret = modem_context_sprint_ip_addr(dst_addr, ip_str,
		// sizeof(ip_str)); 		if (ret != 0) { uPortLog("Error formatting IP string
		// %d", ret); 			goto exit;
		//		}
		//
		//		ret = modem_context_get_addr_port(dst_addr, &dst_port);
		//		if (ret != 0) {
		//			uPortLog("Error getting port from IP address %d", ret);
		//			goto exit;
		//		}
		//
		//		snprintk(send_buf, sizeof(send_buf), "AT+USOST=%d,\"%s\",%u,%zu",
		// sock->id, ip_str, 			 dst_port, buf_len);

	} else {
		for (int i = 0; i < msg->msg_iovlen; i++) {
			uint8_t chunkCounter = 0;
			int len = MIN(buf_len, msg->msg_iov[i].iov_len);

			LOG_INF("send_socket_data: [%02d] uSockWrite with len: %d", chunkCounter++,
				len);

			if (len == 0) {
				break;
			}
			retVal = uSockWrite(mdata.ubxSocketId, msg->msg_iov[i].iov_base, len);
			if (retVal < 0) {
				LOG_ERR("send_socket_data: uSockWrite returned with %d", retVal);
			} else {
				mdata.sock_written += retVal;
			}
			buf_len -= len;
		}
	}

	// in case an error happens
	if (retVal < 0) {
		return retVal;
	}

	return mdata.sock_written;
}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
/* send binary data via the +USO[ST/WR] commands */
static ssize_t send_cert(struct modem_socket *sock, struct modem_cmd *handler_cmds,
			 size_t handler_cmds_len, const char *cert_data, size_t cert_len,
			 int cert_type)
{
	int ret;
	char *filename = "ca";
	char send_buf[sizeof("AT+USECMNG=#,#,!####!,####\r\n")];

	/* TODO support other cert types as well */
	if (cert_type != 0) {
		return -EINVAL;
	}

	if (!sock) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(cert_len <= MDM_MAX_CERT_LENGTH);

	snprintk(send_buf, sizeof(send_buf), "AT+USECMNG=0,%d,\"%s\",%d", cert_type, filename,
		 cert_len);

	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);

	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0U, send_buf, NULL,
				    K_NO_WAIT);
	if (ret < 0) {
		goto exit;
	}

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, handler_cmds, handler_cmds_len,
					    true);
	if (ret < 0) {
		goto exit;
	}

	/* slight pause per spec so that @ prompt is received */
	k_sleep(MDM_PROMPT_CMD_DELAY);
	mctx.iface.write(&mctx.iface, cert_data, cert_len);

	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, K_MSEC(1000));

	if (ret == 0) {
		ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	} else if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
	}

exit:
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data, NULL, 0U, false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	return ret;
}
#endif

static int pin_init(void)
{
	LOG_INF("Setting Modem Pins");
	gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&vcc_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&power_on_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&vint_gpio, GPIO_INPUT);

	k_sleep(K_MSEC(1));

	gpio_pin_set_dt(&vcc_gpio, 1);	    // 3.3 V
	gpio_pin_set_dt(&power_on_gpio, 1); // 3.3 V
	gpio_pin_set_dt(&reset_gpio, 0);    // 3.3 V

	//  TODO: finish me...
	// #if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	//	LOG_DBG("MDM_RESET_PIN -> NOT_ASSERTED");
	//	gpio_pin_set_dt(&reset_gpio, MDM_RESET_NOT_ASSERTED);
	// #endif
	//
	//	LOG_DBG("MDM_POWER_PIN -> ENABLE");
	//	gpio_pin_set_dt(&power_on_gpio, 1);
	//	k_sleep(K_SECONDS(4));
	//
	//	LOG_DBG("MDM_POWER_PIN -> DISABLE");
	//	gpio_pin_set_dt(&power_on_gpio, 0);
	// #if defined(CONFIG_MODEM_UBLOX_SARA_U2)
	//	k_sleep(K_SECONDS(1));
	// #else
	//	k_sleep(K_SECONDS(4));
	// #endif
	//	LOG_DBG("MDM_POWER_PIN -> ENABLE");
	//	gpio_pin_set_dt(&power_on_gpio, 1);
	//	k_sleep(K_SECONDS(1));
	//
	//	/* make sure module is powered off */
	// #if DT_INST_NODE_HAS_PROP(0, mdm_vint_gpios)
	//	LOG_DBG("Waiting for MDM_VINT_PIN = 0");
	//
	//	while (gpio_pin_get_dt(&vint_gpio) > 0) {
	// #if defined(CONFIG_MODEM_UBLOX_SARA_U2)
	//		/* try to power off again */
	//		LOG_DBG("MDM_POWER_PIN -> DISABLE");
	//		gpio_pin_set_dt(&power_gpio, 0);
	//		k_sleep(K_SECONDS(1));
	//		LOG_DBG("MDM_POWER_PIN -> ENABLE");
	//		gpio_pin_set_dt(&power_gpio, 1);
	// #endif
	//		k_sleep(K_MSEC(100));
	//	}
	// #else
	//	k_sleep(K_SECONDS(8));
	// #endif
	//

	LOG_DBG("pin_init: setup pins");

	unsigned int irq_lock_key = irq_lock();

	gpio_pin_set_dt(&power_on_gpio, 0);
#if defined(CONFIG_MODEM_UBLOX_SARA_U2)
	k_usleep(50); /* 50-80 microseconds */
#else
	k_sleep(K_SECONDS(1));
#endif
	gpio_pin_set_dt(&power_on_gpio, 1);

	irq_unlock(irq_lock_key);

	LOG_DBG("MDM_POWER_PIN -> ENABLE");

#if DT_INST_NODE_HAS_PROP(0, mdm_vint_gpios)
	LOG_DBG("Waiting for MDM_VINT_PIN = 1");
	do {
		k_sleep(K_MSEC(100));
	} while (gpio_pin_get_dt(&vint_gpio) == 0);
#else
	k_sleep(K_SECONDS(10));
#endif

	// gpio_pin_configure_dt(&power_on_gpio, GPIO_INPUT);

	LOG_DBG("pin_init: setup pins done");
	return 0;
}

static const uDeviceCfg_t gDeviceCfg = {
	.deviceType = U_DEVICE_TYPE_CELL,
	.deviceCfg =
		{
			.cfgCell =
				{
					.moduleType = U_CELL_MODULE_TYPE_SARA_R5,
					.pSimPinCode = NULL, /* SIM pin */
					.pinEnablePower = U_CFG_APP_PIN_CELL_ENABLE_POWER,
					.pinPwrOn = U_CFG_APP_PIN_CELL_PWR_ON,
					.pinVInt = U_CFG_APP_PIN_CELL_VINT,
					.pinDtrPowerSaving = U_CFG_APP_PIN_CELL_DTR,
				},
		},
	.transportType = U_DEVICE_TRANSPORT_TYPE_UART,
	.transportCfg =
		{
			.cfgUart =
				{
					.uart = U_CFG_APP_CELL_UART,
					.baudRate = U_CELL_UART_BAUD_RATE,
					.pinTxd = U_CFG_APP_PIN_CELL_TXD,
					.pinRxd = U_CFG_APP_PIN_CELL_RXD,
					.pinCts = U_CFG_APP_PIN_CELL_CTS,
					.pinRts = U_CFG_APP_PIN_CELL_RTS,
					.pPrefix = NULL, // Relevant for Linux only
				},
		},
};

static void modem_reset(void)
{
	uAtClientHandle_t atHandle;
	k_thread_system_pool_assign(k_current_get());

	// Initialise the APIs we will need
	// uPortDeinit();

	pin_init();
	uPortInit();

	uDeviceInit();

	int32_t retVal = uDeviceOpen(&gDeviceCfg, &mdata.cellHandle);
	if (retVal != 0) {
		LOG_INF("## Opened device with return code %d", retVal);
	}

	if (uCellAtClientHandleGet(mdata.cellHandle, &atHandle) == 0) {
		// Switch AT printing off
		// bool atPrintOn = uAtClientPrintAtGet(atHandle);
		// LOG_INF("%d", atPrintOn);
		uAtClientPrintAtSet(atHandle, false);
	}
}

void onSocketCloseCb(void *pCbData)
{
	static uint32_t nrOfClose = 0;
	LOG_DBG("CB: Socket with id %d closed", *((int32_t *)pCbData));
	LOG_DBG("    nrOfClose: %d", nrOfClose++);
}

void onDataReceivedCb(void *pCbData)
{
	LOG_DBG("CB: Data received from socket %d", *((int32_t *)pCbData));
}

/*
 * generic socket creation function
 * which can be called in bind() or connect()
 */
static int create_socket(struct modem_socket *sock, const struct sockaddr *addr)
{
	int32_t retVal;

	if (sock->ip_proto != IPPROTO_UDP && sock->ip_proto != IPPROTO_TCP) {
		LOG_ERR("Not supported protocol %d", sock->ip_proto);
		goto error;
	}

	uSockType_t socketType =
		U_SOCK_TYPE_STREAM ? sock->ip_proto == IPPROTO_TCP : U_SOCK_TYPE_DGRAM;

	retVal = uSockCreate(mdata.cellHandle, socketType, sock->ip_proto);
	if (retVal < 0) {
		LOG_ERR("uSockCreate failed: %d", retVal);
		goto error;
	}
	LOG_INF("Create socket with ubx id:", retVal);
	mdata.ubxSocketId = retVal;

	uSockRegisterCallbackData(retVal, onDataReceivedCb, &mdata.ubxSocketId);
	uSockRegisterCallbackClosed(retVal, onSocketCloseCb, &mdata.ubxSocketId);

	// TODO: Handle socket security
	//	if (sock->ip_proto == IPPROTO_TLS_1_2) {
	//		char buf[sizeof("AT+USECPRF=#,#,#######\r")];
	//
	//		/* Enable socket security */
	//		snprintk(buf, sizeof(buf), "AT+USOSEC=%d,1,%d", sock->id, sock->id);
	//		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
	//				     &mdata.sem_response, MDM_CMD_TIMEOUT);
	//		if (ret < 0) {
	//			goto error;
	//		}
	//		/* Reset the security profile */
	//		snprintk(buf, sizeof(buf), "AT+USECPRF=%d", sock->id);
	//		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
	//				     &mdata.sem_response, MDM_CMD_TIMEOUT);
	//		if (ret < 0) {
	//			goto error;
	//		}
	//		/* Validate server cert against the CA.  */
	//		snprintk(buf, sizeof(buf), "AT+USECPRF=%d,0,1", sock->id);
	//		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
	//				     &mdata.sem_response, MDM_CMD_TIMEOUT);
	//		if (ret < 0) {
	//			goto error;
	//		}
	//		/* Use TLSv1.2 only */
	//		snprintk(buf, sizeof(buf), "AT+USECPRF=%d,1,3", sock->id);
	//		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
	//				     &mdata.sem_response, MDM_CMD_TIMEOUT);
	//		if (ret < 0) {
	//			goto error;
	//		}
	//		/* Set root CA filename */
	//		snprintk(buf, sizeof(buf), "AT+USECPRF=%d,3,\"ca\"", sock->id);
	//		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
	//				     &mdata.sem_response, MDM_CMD_TIMEOUT);
	//		if (ret < 0) {
	//			goto error;
	//		}
	//	}
	//
	// errno = 0;
	return 0;

error:
	modem_socket_put(&mdata.socket_config, sock->sock_fd);
	// errno = -1;
	return -1;
}

/*
 * Socket Offload OPS
 */
static const struct socket_op_vtable offload_socket_fd_op_vtable;

static int offload_socket(int family, int type, int proto)
{
	int ret;

	/* defer modem's socket create call to bind() */
	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		// errno = -ret;
		return -1;
	}

	// errno = 0;
	return ret;
}

static int offload_close(void *obj)
{
	int32_t retVal = 0;
	struct modem_socket *sock = (struct modem_socket *)obj;

	static uint32_t cntEnter = 0;
	static uint32_t cntExit = 0;
	LOG_WRN("offload_close: Enter: cntEnter: %d, cntExit: %d", cntEnter, cntExit);
	LOG_WRN("offload_close: mdata.ubxSocketId: %d, sock->id: %d", mdata.ubxSocketId, sock->id);
	k_sleep(K_MSEC(1));

	/* make sure we assigned an id */
	if (sock->id < mdata.socket_config.base_socket_num) {
		LOG_WRN("sock->id < mdata.socket_config.base_socket_num (%d < %d)", sock->id,
			mdata.socket_config.base_socket_num);
		return 0;
	}

	LOG_WRN("offload_close: Proceed: cntEnter: %d, cntExit: %d", ++cntEnter, cntExit);

	if (sock->is_connected || sock->ip_proto == IPPROTO_UDP) {

		retVal = uSockShutdown(mdata.ubxSocketId, U_SOCK_SHUTDOWN_READ_WRITE);
		if (retVal != 0) {
			LOG_ERR("offload_close: uSockShutdown failed (%d)", retVal);
		}

		retVal = uSockClose(mdata.ubxSocketId);
		if (retVal != 0) {
			LOG_ERR("offload_close: uSockClose failed (%d)", retVal);
		}

		uSockCleanUp();
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);

	LOG_WRN("offload_close: Exit: cntEnter: %d, cntExit: %d", cntEnter, ++cntExit);

	return retVal;
}

static int offload_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* save bind address information */
	memcpy(&sock->src, addr, sizeof(*addr));

	/* make sure we've created the socket */
	// if (sock->id == mdata.socket_config.sockets_len + 1) {
	if (create_socket(sock, addr) < 0) {
		return -1;
	}
	//}

	return 0;
}

static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	uSockAddress_t address = {0};

	if (addr == NULL) {
		LOG_ERR("offload_connect: addr == NULL");
		errno = EINVAL;
		return -1;
	}

	LOG_INF("offload_connect sock->id: %d, mdata.ubxSocketId: %d", sock->id, mdata.ubxSocketId);

	int retVal = create_socket(sock, NULL);
	if (retVal != 0) {
		LOG_ERR("offload_connect: create_socket returned with: %d", retVal);
		return -1;
	}

	memcpy(&sock->dst, addr, sizeof(*addr));
	if (addr->sa_family == AF_INET6) {
		address.port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		address.port = ntohs(net_sin(addr)->sin_port);
	} else {
		errno = EAFNOSUPPORT;
		return -1;
	}

	/* skip socket connect if UDP */
	if (sock->ip_proto == IPPROTO_UDP) {
		errno = 0;
		return 0;
	}

	// TODO: add IPV6 support
	address.ipAddress.address.ipv4 = (addr->data[2] << 24) + (addr->data[3] << 16) +
					 (addr->data[4] << 8) + (addr->data[5] << 0);

	LOG_INF("offload_connect: mdata.ubxSocketId: %d", mdata.ubxSocketId);
	k_sleep(K_MSEC(1));

	if (mdata.ubxSocketId < 0) {
		LOG_ERR("offload_connect: No valid socket Id: %d", mdata.ubxSocketId);
		return -1;
	}

	retVal = uSockConnect(mdata.ubxSocketId, &address);
	if (retVal != 0) {
		LOG_ERR("offload_connect: uSockConnect returned with %d", retVal);
		errno = ENOTCONN;
		return -1;
	}

	sock->is_connected = true;
	errno = 0;
	return 0;
}

static ssize_t offload_recvfrom(void *obj, void *buf, size_t len, int flags, struct sockaddr *from,
				socklen_t *fromlen)
{
	int32_t retVal;
	struct modem_socket *sock = (struct modem_socket *)obj;
	// int next_packet_size;
	struct socket_read_data sock_data;

	if (!buf || len == 0) {
		// errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		// errno = ENOTSUP;
		return -1;
	}

	// next_packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	// if (!next_packet_size) {
	if (flags & ZSOCK_MSG_DONTWAIT) {
		// errno = EAGAIN;
		return -1;
	}

	if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP) {
		// errno = 0;
		return 0;
	}

	// modem_socket_wait_data(&mdata.socket_config, sock);
	// next_packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	//}

	/*
	 * Binary and ASCII mode allows sending MDM_MAX_DATA_LENGTH bytes to
	 * the socket in one command
	 */
	// if (next_packet_size > MDM_MAX_DATA_LENGTH) {
	//	next_packet_size = MDM_MAX_DATA_LENGTH;
	// }

	//	snprintk(sendbuf, sizeof(sendbuf), "AT+USO%s=%d,%zd",
	//		 sock->ip_proto == IPPROTO_UDP ? "RF" : "RD", sock->id,
	//		 len < next_packet_size ? len : next_packet_size);

	/* socket read settings */
	(void)memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf = buf;
	sock_data.recv_buf_len = len;
	sock_data.recv_addr = from;
	sock->data = &sock_data;

	retVal = uSockRead(mdata.ubxSocketId, buf, len);

	if (retVal < 0) {
		LOG_ERR("uSockRead failed: %d", retVal);
		// errno = -retVal;
		retVal = -1;
		goto exit;
	}
	sock_data.recv_read_len += retVal;

	/* HACK: use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	/* return length of received data */
	// errno = 0;
	retVal = sock_data.recv_read_len;

exit:
	/* clear socket data */
	sock->data = NULL;
	return retVal;
}

static ssize_t offload_sendto(void *obj, const void *buf, size_t len, int flags,
			      const struct sockaddr *to, socklen_t tolen)
{
	struct iovec msg_iov = {
		.iov_base = (void *)buf,
		.iov_len = len,
	};
	struct msghdr msg = {
		.msg_iovlen = 1,
		.msg_name = (struct sockaddr *)to,
		.msg_namelen = tolen,
		.msg_iov = &msg_iov,
	};

	int retVal = send_socket_data(obj, &msg, MDM_CMD_TIMEOUT);
	if (retVal < 0) {
		LOG_ERR("uSockWrite failed: %d", retVal);
		// errno = -retVal;
		return -1;
	}

	// errno = 0;
	return retVal;
}

static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		return modem_socket_poll_prepare(&mdata.socket_config, obj, pfd, pev, pev_end);
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);

		return modem_socket_poll_update(obj, pfd, pev);
	}

	case F_GETFL:
		return 0;

	default:
		// errno = EINVAL;
		return -1;
	}
}

static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int bkp_iovec_idx;
	struct iovec bkp_iovec = {0};
	struct msghdr crafted_msg = {
		.msg_name = msg->msg_name,
		.msg_namelen = msg->msg_namelen,
	};
	size_t full_len = 0;
	int ret;

	/* Compute the full length to be send and check for invalid values */
	for (int i = 0; i < msg->msg_iovlen; i++) {
		if (!msg->msg_iov[i].iov_base || msg->msg_iov[i].iov_len == 0) {
			// errno = EINVAL;
			return -1;
		}
		full_len += msg->msg_iov[i].iov_len;
	}

	LOG_INF("msg_iovlen:%zd flags:%d, full_len:%zd", msg->msg_iovlen, flags, full_len);

	while (full_len > sent) {
		int removed = 0;
		int i = 0;

		crafted_msg.msg_iovlen = msg->msg_iovlen;
		crafted_msg.msg_iov = &msg->msg_iov[0];

		bkp_iovec_idx = -1;
		/*  Iterate on iovec to remove the bytes already sent */
		while (removed < sent) {
			int to_removed = sent - removed;

			if (to_removed >= msg->msg_iov[i].iov_len) {
				crafted_msg.msg_iovlen -= 1;
				crafted_msg.msg_iov = &msg->msg_iov[i + 1];

				removed += msg->msg_iov[i].iov_len;
			} else {
				/* Backup msg->msg_iov[i] before "removing"
				 * starting bytes already send.
				 */
				bkp_iovec_idx = i;
				bkp_iovec.iov_len = msg->msg_iov[i].iov_len;
				bkp_iovec.iov_base = msg->msg_iov[i].iov_base;

				/* Update msg->msg_iov[i] to "remove"
				 * starting bytes already send.
				 */
				msg->msg_iov[i].iov_len -= to_removed;
				msg->msg_iov[i].iov_base =
					&(((uint8_t *)msg->msg_iov[i].iov_base)[to_removed]);

				removed += to_removed;
			}

			i++;
		}

		ret = send_socket_data(obj, &crafted_msg, MDM_CMD_TIMEOUT);

		/* Restore backup iovec when necessary */
		if (bkp_iovec_idx != -1) {
			msg->msg_iov[bkp_iovec_idx].iov_len = bkp_iovec.iov_len;
			msg->msg_iov[bkp_iovec_idx].iov_base = bkp_iovec.iov_base;
		}

		/* Handle send_socket_data() returned value */
		if (ret < 0) {
			// errno = -ret;
			return -1;
		}

		sent += ret;
	}

	return (ssize_t)sent;
}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
static int map_credentials(struct modem_socket *sock, const void *optval, socklen_t optlen)
{
	sec_tag_t *sec_tags = (sec_tag_t *)optval;
	int ret = 0;
	int tags_len;
	sec_tag_t tag;
	int id;
	int i;
	struct tls_credential *cert;

	if ((optlen % sizeof(sec_tag_t)) != 0 || (optlen == 0)) {
		return -EINVAL;
	}

	tags_len = optlen / sizeof(sec_tag_t);
	/* For each tag, retrieve the credentials value and type: */
	for (i = 0; i < tags_len; i++) {
		tag = sec_tags[i];
		cert = credential_next_get(tag, NULL);
		while (cert != NULL) {
			switch (cert->type) {
			case TLS_CREDENTIAL_CA_CERTIFICATE:
				id = 0;
				break;
			case TLS_CREDENTIAL_NONE:
			case TLS_CREDENTIAL_PSK:
			case TLS_CREDENTIAL_PSK_ID:
			default:
				/* Not handled */
				return -EINVAL;
			}
			struct modem_cmd cmd[] = {
				MODEM_CMD("+USECMNG: ", on_cmd_cert_write, 3U, ","),
			};
			ret = send_cert(sock, cmd, 1, cert->buf, cert->len, id);
			if (ret < 0) {
				return ret;
			}

			cert = credential_next_get(tag, cert);
		}
	}

	return 0;
}
#else

static int map_credentials(struct modem_socket *sock, const void *optval, socklen_t optlen)
{
	return -EINVAL;
}

#endif

static int offload_setsockopt(void *obj, int level, int optname, const void *optval,
			      socklen_t optlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	int ret;

	if (IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS) && level == SOL_TLS) {
		switch (optname) {
		case TLS_SEC_TAG_LIST:
			ret = map_credentials(sock, optval, optlen);
			break;
		case TLS_HOSTNAME:
			uPortLog("TLS_HOSTNAME option is not supported");
			return -EINVAL;
		case TLS_PEER_VERIFY:
			if (*(uint32_t *)optval != TLS_PEER_VERIFY_REQUIRED) {
				uPortLog("Disabling peer verification is not supported");
				return -EINVAL;
			}
			ret = 0;
			break;
		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return ret;
}

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable =
		{
			.read = offload_read,
			.write = offload_write,
			.close = offload_close,
			.ioctl = offload_ioctl,
		},
	.bind = offload_bind,
	.connect = offload_connect,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.listen = NULL,
	.accept = NULL,
	.sendmsg = offload_sendmsg,
	.getsockopt = NULL,
	.setsockopt = offload_setsockopt,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET && family != AF_INET6) {
		return false;
	}

	if (type != SOCK_DGRAM && type != SOCK_STREAM) {
		return false;
	}

	if (proto != IPPROTO_TCP && proto != IPPROTO_UDP && proto != IPPROTO_TLS_1_2) {
		return false;
	}

	return true;
}

NET_SOCKET_OFFLOAD_REGISTER(ublox_ubxlib, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, AF_UNSPEC,
			    offload_is_supported, offload_socket);

#if defined(CONFIG_DNS_RESOLVER)
/* TODO: This is a bare-bones implementation of DNS handling
 * We ignore most of the hints like ai_family, ai_protocol and ai_socktype.
 * Later, we can add additional handling if it makes sense.
 */
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	LOG_ERR("UBX_LIB DNS-LOOKUP");
	static uint8_t test = 0;
	test++;
	//	static const struct modem_cmd cmd = MODEM_CMD("+UDNSRN: ", on_cmd_dns, 1U,
	//","); 	uint32_t port = 0U; 	int ret;
	//	/* DNS command + 128 bytes for domain name parameter */
	//	char sendbuf[sizeof("AT+UDNSRN=#,'[]'\r") + 128];
	//
	//	/* init result */
	//	(void)memset(&result, 0, sizeof(result));
	//	(void)memset(&result_addr, 0, sizeof(result_addr));
	//	/* FIXME: Hard-code DNS to return only IPv4 */
	//	result.ai_family = AF_INET;
	//	result_addr.sa_family = AF_INET;
	//	result.ai_addr = &result_addr;
	//	result.ai_addrlen = sizeof(result_addr);
	//	result.ai_canonname = result_canonname;
	//	result_canonname[0] = '\0';
	//
	//	if (service) {
	//		port = ATOI(service, 0U, "port");
	//		if (port < 1 || port > USHRT_MAX) {
	//			return DNS_EAI_SERVICE;
	//		}
	//	}
	//
	//	if (port > 0U) {
	//		/* FIXME: DNS is hard-coded to return only IPv4 */
	//		if (result.ai_family == AF_INET) {
	//			net_sin(&result_addr)->sin_port = htons(port);
	//		}
	//	}
	//
	//	/* check to see if node is an IP address */
	//	if (net_addr_pton(result.ai_family, node,
	//			  &((struct sockaddr_in *)&result_addr)->sin_addr) == 0) {
	//		*res = &result;
	//		return 0;
	//	}
	//
	//	/* user flagged node as numeric host, but we failed net_addr_pton */
	//	if (hints && hints->ai_flags & AI_NUMERICHOST) {
	//		return DNS_EAI_NONAME;
	//	}
	//
	//	snprintk(sendbuf, sizeof(sendbuf), "AT+UDNSRN=0,\"%s\"", node);
	//	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, &cmd, 1U, sendbuf,
	//&mdata.sem_response, 			     MDM_DNS_TIMEOUT); 	if (ret < 0) {
	// return ret;
	//	}
	//
	//	LOG_DBG("DNS RESULT: %s", net_addr_ntop(result.ai_family,
	//&net_sin(&result_addr)->sin_addr, sendbuf,
	// NET_IPV4_ADDR_LEN));
	//
	//	*res = (struct zsock_addrinfo *)&result;
	//	return 0;
}

static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	/* using static result from offload_getaddrinfo() -- no need to free */
	res = NULL;
}

const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};
#endif

static int net_offload_dummy_get(sa_family_t family, enum net_sock_type type,
				 enum net_ip_protocol ip_proto, struct net_context **context)
{
	LOG_ERR("CONFIG_NET_SOCKETS_OFFLOAD must be enabled for this driver");
	return -ENOTSUP;
}

/* placeholders, until Zephyr IP stack updated to handle a NULL net_offload */
static struct net_offload modem_net_offload = {
	.get = net_offload_dummy_get,
};

#define HASH_MULTIPLIER 37

static uint32_t hash32(char *str, int len)
{
	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

static int offload_socket(int family, int type, int proto);

static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data = dev->data;

	/* Direct socket offload used instead of net offload: */
	iface->if_dev->offload = &modem_net_offload;
	net_if_set_link_addr(iface, modem_get_mac(dev), sizeof(data->mac_addr), NET_LINK_ETHERNET);
	data->net_iface = iface;
#ifdef CONFIG_DNS_RESOLVER
	socket_offload_dns_register(&offload_dns_ops);
#endif

	net_if_socket_offload_set(iface, offload_socket);
}

static struct net_if_api api_funcs = {
	.init = modem_net_iface_init,
};

static int modem_init(const struct device *dev)
{
	int ret;
	ARG_UNUSED(dev);

	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_prompt, 0, 1);

#if defined(CONFIG_MODEM_UBLOX_SARA_RSSI_WORK)
	/* initialize the work queue */
	k_work_queue_start(&modem_workq, modem_workq_stack,
			   K_KERNEL_STACK_SIZEOF(modem_workq_stack), K_PRIO_COOP(7), NULL);
#endif

	/* socket config */
	mdata.socket_config.sockets = &mdata.sockets[0];
	mdata.socket_config.sockets_len = ARRAY_SIZE(mdata.sockets);
	// mdata.socket_config.base_socket_num = MDM_BASE_SOCKET_NUM;
	mdata.ubxSocketId = -1;

	/* socket config */
	// ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0],
	// ARRAY_SIZE(mdata.sockets),
	//                         MDM_BASE_SOCKET_NUM, false,
	//                         &offload_socket_fd_op_vtable);

	ret = modem_socket_init(&mdata.socket_config, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		LOG_ERR("Error modem_socket_init: %d", ret);
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;
	mctx.data_rssi = &mdata.mdm_rssi;
	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("SKIP: Error modem_context_register: %d", ret);
		// goto error;
	}

#if defined(CONFIG_MODEM_UBLOX_SARA_RSSI_WORK)
	/* init RSSI query */
	k_work_init_delayable(&mdata.rssi_query_work, modem_rssi_query_work);
#endif

	modem_reset();

error:
	return ret;
}

bool keepGoingCallback(uDeviceHandle_t cellHandle)
{
	UNUSED(cellHandle);
	bool keepGoing = true;

	if ((mdata.startTimeMs > 0) &&
	    (k_uptime_get() - mdata.startTimeMs > (MDM_NET_CONNECT_TIMEOUT_SECONDS * 1000))) {
		keepGoing = false;
	}

	return keepGoing;
}

bool mdm_get_handle(uDeviceHandle_t *cellHandle)
{
	if (mdata.cellHandle == NULL) {
		return false;
	}
	*cellHandle = mdata.cellHandle;
	return true;
}

// bool mdm_ubxlib_register_connect_cb(connectedCallback cb)
//{
//	mdata.connectedCallback = cb;
//	return true;
// }

bool mdm_ubxlib_register_network_status_cb(networkStatusCallback cb)
{
	mdata.networkStatusCallback = cb;
	return true;
}

void networkStatusCb(uDeviceHandle_t devHandle, uNetworkType_t netType, bool isUp,
		     uNetworkStatus_t *pStatus, void *pParameter)
{
	if (mdata.networkStatusCallback != NULL) {
		mdata.networkStatusCallback(pStatus);
	}
	LOG_DBG("devHandle: %d", devHandle);
	LOG_DBG("netType:   %d", netType);
	LOG_DBG("isUp:      %d", isUp);
	LOG_DBG("pStatus:   %d", pStatus);
}

int32_t mdm_ubxlib_bring_interface_up(const void *pCfg)
{
	char buffer[U_CELL_NET_IP_ADDRESS_SIZE];
	int32_t mcc;
	int32_t mnc;
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	if (cellHandle == NULL) {
		LOG_ERR("Cell handle not registered");
		return U_ERROR_COMMON_PLATFORM;
	}

	// Connect to the cellular network with all default parameters
	mdata.startTimeMs = k_uptime_get();

	if (uNetworkInterfaceUp(cellHandle, U_NETWORK_TYPE_CELL, pCfg) != 0) {
		LOG_WRN("uNetworkInterfaceUp failed");
		return U_CELL_ERROR_NOT_CONNECTED;
	}

	if (uNetworkSetStatusCallback(cellHandle, U_NETWORK_TYPE_CELL, networkStatusCb, NULL) !=
	    0) {
		LOG_WRN("uNetworkSetStatusCallback failed");
	}

	//	uCellNetGetOperatorStr(cellHandle, buffer, sizeof(buffer));
	//	uCellNetGetMccMnc(cellHandle, &mcc, &mnc);
	//	uCellNetGetIpAddressStr(cellHandle, buffer);
	//	uCellNetGetApnStr(cellHandle, buffer, sizeof(buffer));

	//	if (mdata.connectedCallback != NULL) {
	//		mdata.connectedCallback();
	//	}
	return U_ERROR_COMMON_SUCCESS;
}

uint32_t mdm_ubxlib_interface_down(void)
{
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	if (cellHandle == NULL) {
		LOG_ERR("Cell handle not registered");
		return U_ERROR_COMMON_PLATFORM;
	}

	if (uNetworkInterfaceDown(cellHandle, U_NETWORK_TYPE_CELL) != 0) {
		LOG_WRN("uNetworkInterfaceDown failed");
		return U_ERROR_COMMON_PLATFORM;
	}

	LOG_INF("uSockCleanUp()");
	uSockCleanUp();
	LOG_INF("uSockDeinit()");
	uSockDeinit();

	return U_ERROR_COMMON_SUCCESS;
}

int32_t mdm_ubxlib_disconnect(void)
{
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	if (cellHandle == NULL) {
		LOG_ERR("Cell handle not registered");
		return U_ERROR_COMMON_PLATFORM;
	}
	if (uNetworkInterfaceDown(cellHandle, U_NETWORK_TYPE_CELL) != 0) {
		LOG_ERR("uNetworkInterfaceDown failed");
		return U_ERROR_COMMON_PLATFORM;
	}

	return U_ERROR_COMMON_SUCCESS;
}

int32_t mdm_ubxlib_power_on(void)
{
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	if (cellHandle == NULL) {
		LOG_ERR("mdm_ubxlib_power_on: cellHandle not registered");
		return U_ERROR_COMMON_PLATFORM;
	}

	if (uCellPwrOn(mdata.cellHandle, NULL, NULL) != 0) {
		LOG_ERR("mdm_ubxlib_power_on uCellPwrOn() failed");
		return U_ERROR_COMMON_PLATFORM;
	}

	return U_ERROR_COMMON_SUCCESS;
}

int32_t mdm_ubxlib_power_off(void)
{
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	if (cellHandle == NULL) {
		LOG_ERR("mdm_ubxlib_power_off: cellHandle not registered");
		return U_ERROR_COMMON_PLATFORM;
	}

	if (uCellPwrOff(mdata.cellHandle, NULL) != 0) {
		LOG_ERR("mdm_ubxlib_power_off uCellPwrOff() failed");
		return U_ERROR_COMMON_PLATFORM;
	}

	return U_ERROR_COMMON_SUCCESS;
}

int32_t mdm_ubxlib_get_rssi_dbm(void)
{
	uDeviceHandle_t cellHandle = mdata.cellHandle;

	uCellInfoRefreshRadioParameters(cellHandle);

	return uCellInfoGetRssiDbm(cellHandle);
}

int32_t mdm_reset_modem(void)
{
	modem_init(NULL);
	return 0;
}

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_UBLOX_UBXLIB_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);
