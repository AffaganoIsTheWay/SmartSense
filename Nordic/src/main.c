/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

// Initialization UUID

/** @brief Sensor Service UUID. */
#define BT_UUID_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xEF680200, 0x9B35, 0x4933, 0x9B10, 0x52FFA9740042)

/** @brief Characteristic UUID. */
#define BT_UUID_TEMPERATURE_CHARACTERISTIC_VAL \
	BT_UUID_128_ENCODE(0xEF680201, 0x9B35, 0x4933, 0x9B10, 0x52FFA9740042)


/** @brief Characteristic UUID. */
#define BT_UUID_HUMIDITY_CHARACTERISTIC_VAL \
	BT_UUID_128_ENCODE(0xEF680203, 0x9B35, 0x4933, 0x9B10, 0x52FFA9740042)

/** @brief Characteristic UUID. */
#define BT_UUID_AIR_CHARACTERISTIC_VAL \
	BT_UUID_128_ENCODE(0xEF680204, 0x9B35, 0x4933, 0x9B10, 0x52FFA9740042)

#define BT_UUID_SERVICE BT_UUID_DECLARE_128(BT_UUID_SERVICE_VAL)
#define BT_UUID_TEMPERATURE_CHARACTERISTIC BT_UUID_DECLARE_128(BT_UUID_TEMPERATURE_CHARACTERISTIC_VAL)
#define BT_UUID_HUMIDITY_CHARACTERISTIC BT_UUID_DECLARE_128(BT_UUID_HUMIDITY_CHARACTERISTIC_VAL)
#define BT_UUID_AIR_CHARACTERISTIC BT_UUID_DECLARE_128(BT_UUID_AIR_CHARACTERISTIC_VAL)

// Fixed value for UART
#define RECEIVE_BUFF_SIZE 10

#define RECEIVE_TIMEOUT 10 * 60 * 1000

// Initialization Finate state machine
typedef enum {
    TEMPERATURE,
    HUMIDITY,
    AIR
}State_t;


// Initialization inizial values
State_t current_state = TEMPERATURE;

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static uint8_t tx_buf[2] = {0};

static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

static void start_scan(void);

static struct bt_conn *default_conn;

static struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params temperature_subscribe_params;
static struct bt_gatt_subscribe_params humidity_subscribe_params;
static struct bt_gatt_subscribe_params air_subscribe_params;

/* Callback Temperature:
	Called when a Temperature Characteristic data arrive though BT. 
	if TEMPERATURE state send value over UART,
	then display recived data
*/
static uint8_t temperature_notify_func(struct bt_conn *conn,
						   struct bt_gatt_subscribe_params *params,
						   const void *data, uint16_t length)
{
	if (!data)
	{
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	int ret;

	// Print the received data byte by byte
	if (current_state == TEMPERATURE) {
		printk("TEMPERATURE\n");
		// Fill up TX buffer
		tx_buf[0] = ((uint8_t *)data)[0];
		tx_buf[1] = ((uint8_t *)data)[1];
		// Send Data
		ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
		if (ret) {
			return 1;
		}
	}
	printk("[NOTIFICATION] Temperature: ");
	printk("%d.%u ", ((uint8_t *)data)[0], ((uint8_t *)data)[1]);
	printk("\n");

	return BT_GATT_ITER_CONTINUE;
}

/* Callback Humidity:
	Called when a Humidity Characteristic data arrive though BT. 
	if Humidity state send value over UART,
	then display recived data
*/
static uint8_t humidity_notify_func(struct bt_conn *conn,
						   struct bt_gatt_subscribe_params *params,
						   const void *data, uint16_t length)
{
	if (!data)
	{
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	int ret;

	// Print the received data byte by byte
	if (current_state == HUMIDITY) {
		printk("HUMIDITY\n");
		// Fill up TX buffer
		tx_buf[0] = ((uint8_t *)data)[0];
		tx_buf[1] = 0;
		// Send Data
		ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
		if (ret) {
			return 1;
		}

	}
	printk("[NOTIFICATION] Humidity: ");
	printk("%u%%", ((uint8_t *)data)[0]);
	printk("\n");

	return BT_GATT_ITER_CONTINUE;
}

/* Callback AIR:
	Called when a Air Characteristic data arrive though BT. 
	if Air state send value over UART,
	then display recived data
*/
static uint8_t air_notify_func(struct bt_conn *conn,
						   struct bt_gatt_subscribe_params *params,
						   const void *data, uint16_t length)
{
	if (!data)
	{
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	int ret;

	// Print the received data byte by byte
	if (current_state == AIR) {
		printk("AIR\n");
		// Send Data
		tx_buf[0] = ((uint8_t *)data)[0];
		tx_buf[1] = ((uint8_t *)data)[1];
		// Send Data
		ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
		if (ret) {
			return 1;
		}
	}
	printk("[NOTIFICATION] Air: ");
	printk("%u ppm", ((uint8_t *)data)[0] + (((uint8_t *)data)[1]*256));
	printk("\n");

	return BT_GATT_ITER_CONTINUE;
}

/* Callback UART interrupt:
	Called when a data arrive at the RX or it stop listening due to timeout
*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
		/* Data arrived at the RX */
		case UART_RX_RDY:
			printk("Getting Value: %u\n", evt->data.rx.buf[evt->data.rx.offset]);
			if ((evt->data.rx.len) == 1) {
				if (evt->data.rx.buf[evt->data.rx.offset] == 1) {
					current_state = TEMPERATURE;
				} else if (evt->data.rx.buf[evt->data.rx.offset] == 2) {
					current_state = HUMIDITY;
				} else if (evt->data.rx.buf[evt->data.rx.offset] == 3) {
					current_state = AIR;
				}
			}
		break;
		/* RX Timeout: restart RX */
		case UART_RX_DISABLED:
			uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;

		default:
		break;
	}
}

uint16_t temperature_handle, humidity_handle;
/* Discover Function:
	It will search the attribute defined by UUID in the BT messages of the device.
	First will search the attribute handle of the ENVIROMENT SERVICE,
	then in the Enviroment Service slice it will search for the handle of
	Temperature, Humiodity and Air Characteristic,
	the subscrive to the notification service
*/
static uint8_t discover_func(struct bt_conn *conn,
							 const struct bt_gatt_attr *attr,
							 struct bt_gatt_discover_params *params)
{
	int err;
	

	if (!attr)
	{
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_SERVICE)) {
		memcpy(&uuid, BT_UUID_TEMPERATURE_CHARACTERISTIC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err)
		{
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
						  BT_UUID_TEMPERATURE_CHARACTERISTIC))
	{
		memcpy(&uuid, BT_UUID_HUMIDITY_CHARACTERISTIC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		temperature_handle = attr->handle + 2;
		discover_params.start_handle = attr->handle; 
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		temperature_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err)
		{
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
						  BT_UUID_HUMIDITY_CHARACTERISTIC))
	{
		memcpy(&uuid, BT_UUID_AIR_CHARACTERISTIC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		humidity_handle = attr->handle + 2;
		discover_params.start_handle = attr->handle;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		humidity_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err)
		{
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
						  BT_UUID_AIR_CHARACTERISTIC))
	{
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(struct bt_uuid_16));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		air_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err)
		{
			printk("Discover failed (err %d)\n", err);
		}
	}
	else
	{
		temperature_subscribe_params.notify = temperature_notify_func;
		temperature_subscribe_params.value = BT_GATT_CCC_NOTIFY;
		printk("Temperature handle: %u\n", temperature_handle);
		temperature_subscribe_params.ccc_handle = temperature_handle;

		err = bt_gatt_subscribe(conn, &temperature_subscribe_params);
		if (err && err != -EALREADY)
		{
			printk("Subscribe failed (err %d)\n", err);
		}
		else
		{
			printk("[SUBSCRIBED]\n");
		}

		humidity_subscribe_params.notify = humidity_notify_func;
		humidity_subscribe_params.value = BT_GATT_CCC_NOTIFY;
		humidity_subscribe_params.ccc_handle = humidity_handle;

		err = bt_gatt_subscribe(conn, &humidity_subscribe_params);
		if (err && err != -EALREADY)
		{
			printk("Subscribe failed (err %d)\n", err);
		}
		else
		{
			printk("[SUBSCRIBED]\n");
		}

		air_subscribe_params.notify = air_notify_func;
		air_subscribe_params.value = BT_GATT_CCC_NOTIFY;
		air_subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &air_subscribe_params);
		if (err && err != -EALREADY)
		{
			printk("Subscribe failed (err %d)\n", err);
		}
		else
		{
			printk("[SUBSCRIBED]\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

/* Callback function when a device is found:
	First will retrive device information,
	then chack if it is the thingy device,
	in case will stop the scan and establish a connection
*/
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* Retrive Device Information */
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	/* Check if is Thingy's address*/
	if (strncmp(addr_str, "CA:D7:29:21:6F:C1", 17)){
		return;
	}

	/* Stop the scan */
	if (bt_le_scan_stop()) {
		return;
	}

	/* Enstablish a connection */
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%d)\n", addr_str, err);
		start_scan();
	}
}

// Function to start the scan for Bluetooth device
static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

/* Callback function for connect interrupt:
	check if there was an error while connecting
	then initialize the discovery function
*/
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err)
	{
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn)
	{
		memcpy(&uuid, BT_UUID_SERVICE, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err)
		{
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

/* Callback in case of BLE disconnect interrupt:
	Will display a disconnection message with the address of the device
	then clear the connection parameter
	then start the scan to connect to another device	
*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

/* Set the callback for BLE to connected and disconnected functions
*/
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int err;
	int ret;
    
	// Inizialize UART and check if it works properly
	if (!device_is_ready(uart)) {
		printk("UART device not ready\n");
		return 1;
	}

	// Set UART callback interrupt function
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
		return 1;
	}

	// Enable Bluetooth
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	// Start scan for Bluetooth Device
	start_scan();

	// Initialize UART reciver (pointer to uart devicetree, reciver buffer, buffer size, how much to wait for incoming data)
	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (ret) {
		return 1;
	}

	return 0;
}
