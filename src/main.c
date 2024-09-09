#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "led.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

// Advertisement Data

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SERVICE_UUID_VAL),
};

// GAP callbacks

static void connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    LOG_ERR("Connection failed (err 0x%02x)\n", err);
  } else {
    LOG_INF("Connected\n");
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  LOG_INF("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int main(void) {
  int err;

  LOG_INF("Starting LED application");

  // make sure the LED device is ready
	err = led_init();
  if (err) {
		LOG_ERR("LED init failed (err %d)\n", err);
		return err;
	}

  // initialize BLE
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)\n", err);
    return err;
  }
  LOG_INF("Bluetooth initialized\n");

  // start avertising
  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_ERR("Advertising failed to start (err %d)\n", err);
    return err;
  }
  LOG_INF("Advertising successfully started\n");
	return 0;
}
