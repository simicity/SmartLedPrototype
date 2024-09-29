#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>

#include "led.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "proto/led/led.pb.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led, CONFIG_MODULE_LED_LOG_LEVEL);

#define BLINK_FREQ_MS					1000
#define LED_UNIT_MAX					10
#define NUM_COLORS_RGB				3
#define BASE_10								10
#define DT_LABEL_AND_COMMA(node_id)	    DT_PROP(node_id, label),
#define GPIO_DT_SPEC_GET_AND_COMMA(node_id) GPIO_DT_SPEC_GET(node_id, gpios),

/* The following arrays are populated compile time from the .dts*/
static const char *const led_labels[] = {DT_FOREACH_CHILD(DT_PATH(leds), DT_LABEL_AND_COMMA)};

static const struct gpio_dt_spec leds[] = {DT_FOREACH_CHILD(DT_PATH(leds), GPIO_DT_SPEC_GET_AND_COMMA)};

enum led_type {
	LED_MONOCHROME,
	LED_COLOR,
};

struct user_config {
	LedStateType state;
	LedColorType color;
};

struct led_unit_cfg {
	uint8_t led_no;
	enum led_type unit_type;
	union {
		const struct gpio_dt_spec *mono;
		const struct gpio_dt_spec *color[NUM_COLORS_RGB];
	} type;
	struct user_config user_cfg;
};

static uint8_t leds_num;
static bool initialized;
static struct led_unit_cfg led_units[LED_UNIT_MAX];

// Service and Characteristics UUIDs
static struct bt_uuid_128 get_led_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x9c85a726, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));
static struct bt_uuid_128 put_led_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x9c85a726, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120003));

static struct bt_uuid_128 led_svc_uuid = BT_UUID_INIT_128(LED_SERVICE_UUID_VAL);

/**
 * @brief Configures fields for a RGB LED
 */
static int configure_led_color(uint8_t led_unit, uint8_t led_color, uint8_t led)
{
	if (!device_is_ready(leds[led].port)) {
		LOG_ERR("LED GPIO controller not ready");
		return -ENODEV;
	}

	led_units[led_unit].type.color[led_color] = &leds[led];
	led_units[led_unit].unit_type = LED_COLOR;

	return gpio_pin_configure_dt(led_units[led_unit].type.color[led_color],
				     GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Configures fields for a monochrome LED
 */
static int config_led_monochrome(uint8_t led_unit, uint8_t led)
{
	if (!device_is_ready(leds[led].port)) {
		LOG_ERR("LED GPIO controller not ready");
		return -ENODEV;
	}

	led_units[led_unit].type.mono = &leds[led];
	led_units[led_unit].unit_type = LED_MONOCHROME;

	return gpio_pin_configure_dt(led_units[led_unit].type.mono, GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Parses the device tree for LED settings.
 */
static int led_device_tree_parse(void)
{
	int ret;

	for (uint8_t i = 0; i < leds_num; i++) {
		char *end_ptr = NULL;
		uint32_t led_unit = strtoul(led_labels[i], &end_ptr, BASE_10);

		if (led_labels[i] == end_ptr) {
			LOG_ERR("No match for led unit. The dts is likely not properly formatted");
			return -ENXIO;
		}

		if (strstr(led_labels[i], "LED_RGB_RED")) {
			ret = configure_led_color(led_unit, RED, i);
			if (ret) {
				return ret;
			}

		} else if (strstr(led_labels[i], "LED_RGB_GREEN")) {
			ret = configure_led_color(led_unit, GREEN, i);
			if (ret) {
				return ret;
			}

		} else if (strstr(led_labels[i], "LED_RGB_BLUE")) {
			ret = configure_led_color(led_unit, BLUE, i);
			if (ret) {
				return ret;
			}

		} else if (strstr(led_labels[i], "LED_MONO")) {
			ret = config_led_monochrome(led_unit, i);
			if (ret) {
				return ret;
			}
		} else {
			LOG_ERR("No color identifier for LED %d LED unit %d", i, led_unit);
			return -ENODEV;
		}
	}
	return 0;
}

/**
 * @brief Internal handling to set the status of a led unit
 */
static int led_set_int(uint8_t led_unit, LedColorType color)
{
	int ret;

	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		if (color) {
			ret = gpio_pin_set_dt(led_units[led_unit].type.mono, 1);
			if (ret) {
				return ret;
			}
		} else {
			ret = gpio_pin_set_dt(led_units[led_unit].type.mono, 0);
			if (ret) {
				return ret;
			}
		}
	} else {
		for (uint8_t i = 0; i < NUM_COLORS_RGB; i++) {
			if (color & BIT(i)) {
				ret = gpio_pin_set_dt(led_units[led_unit].type.color[i], 1);
				if (ret) {
					return ret;
				}
			} else {
				ret = gpio_pin_set_dt(led_units[led_unit].type.color[i], 0);
				if (ret) {
					return ret;
				}
			}
		}
	}

	return 0;
}

static void led_blink_work_handler(struct k_work *work);

K_WORK_DEFINE(led_blink_work, led_blink_work_handler);

/**
 * @brief Submit a k_work on timer expiry.
 */
void led_blink_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&led_blink_work);
}

K_TIMER_DEFINE(led_blink_timer, led_blink_timer_handler, NULL);

/**
 * @brief Periodically invoked by the timer to blink LEDs.
 */
static void led_blink_work_handler(struct k_work *work)
{
	int ret;
	static bool on_phase;

	for (uint8_t i = 0; i < leds_num; i++) {
		if (led_units[i].user_cfg.state == LedStateType_LED_STATE_TYPE_BLINK) {
			if (on_phase) {
				ret = led_set_int(i, led_units[i].user_cfg.color);
				if (ret) {
					LOG_ERR("LED set error: %d", ret);
				}
			} else {
				ret = led_set_int(i, LedStateType_LED_STATE_TYPE_OFF);
				if (ret) {
					LOG_ERR("LED set error: %d", ret);
				}
			}
		}
	}

	on_phase = !on_phase;
}

static int led_set(uint8_t led_unit, LedColorType color, LedStateType state)
{
	int ret;

	if (!initialized) {
		return -EPERM;
	}

	ret = led_set_int(led_unit, color);
	if (ret) {
		return ret;
	}

	led_units[led_unit].user_cfg.state = state;
	led_units[led_unit].user_cfg.color = color;

	return 0;
}

int led_on(uint8_t led_unit, ...)
{
	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		return led_set(led_unit, LedColorType_LED_COLOR_WHITE, LedStateType_LED_STATE_TYPE_SOLID);
	}

	va_list args;

	va_start(args, led_unit);
	int color = va_arg(args, int);

	va_end(args);

	if (color <= 0 || color >= _LedColorType_ARRAYSIZE) {
		LOG_ERR("Invalid color code %d", color);
		return -EINVAL;
	}
	return led_set(led_unit, color, LedStateType_LED_STATE_TYPE_SOLID);
}

int led_blink(uint8_t led_unit, ...)
{
	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		return led_set(led_unit, LedColorType_LED_COLOR_WHITE, LedStateType_LED_STATE_TYPE_BLINK);
	}

	va_list args;

	va_start(args, led_unit);

	int color = va_arg(args, int);

	va_end(args);

	if (color <= 0 || color >= _LedColorType_ARRAYSIZE) {
		LOG_ERR("Invalid color code %d", color);
		return -EINVAL;
	}

	return led_set(led_unit, color, LedStateType_LED_STATE_TYPE_BLINK);
}

int led_off(uint8_t led_unit)
{
	return led_set(led_unit, LedColorType_LED_COLOR_OFF, LedStateType_LED_STATE_TYPE_SOLID);
}

static ssize_t read_get_led(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
	return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
}

static ssize_t write_get_led(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, const void *buf,
                               uint16_t len, uint16_t offset, uint8_t flags) {
	GetLedRequest request = GetLedRequest_init_zero;
	pb_istream_t stream = pb_istream_from_buffer(buf, len);

	if (!pb_decode(&stream, GetLedRequest_fields, &request)) {
		LOG_ERR("Failed to decode GetLedRequest");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	// Prepare the GetLedResponse
	uint32_t led_id = request.led_id;
	GetLedResponse response = GetLedResponse_init_zero;
	response.led_id = led_id;
	response.led_control.led_state_type = led_units[led_id].user_cfg.state;
	response.led_control.led_color_type = led_units[led_id].user_cfg.color;

	// Send the response back to the central
	uint8_t responseData[GetLedResponse_size];
	pb_ostream_t ostream = pb_ostream_from_buffer(responseData, sizeof(responseData));
	
	if (!pb_encode(&ostream, GetLedResponse_fields, &response)) {
		LOG_ERR("Failed to encode GetLedResponse");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	bt_gatt_notify(conn, attr, responseData, ostream.bytes_written);
  return len;
}

static ssize_t read_put_led(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
  return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
}

static ssize_t write_put_led(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, const void *buf,
                               uint16_t len, uint16_t offset, uint8_t flags) {
	PutLedRequest request = PutLedRequest_init_zero;
	pb_istream_t stream = pb_istream_from_buffer(buf, len);

	if (!pb_decode(&stream, PutLedRequest_fields, &request)) {
		LOG_ERR("Failed to decode PutLedRequest");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t led_id = request.led_id;
	if (led_id < leds_num) {
		switch (request.led_control.led_state_type) {
			case LedStateType_LED_STATE_TYPE_BLINK:
				if (led_units[led_id].led_no == LedColorType_LED_COLOR_OFF) {
					led_blink(led_id);
				} else {
					led_blink(led_id, request.led_control.led_color_type);
				}
				break;
			case LedStateType_LED_STATE_TYPE_SOLID:
				if (led_units[led_id].led_no == LedColorType_LED_COLOR_OFF) {
					led_on(led_id);
				} else {
					led_on(led_id, request.led_control.led_color_type);
				}
				break;
			case LedStateType_LED_STATE_TYPE_OFF:
				led_off(led_id);
				break;
		}
	} else {
		LOG_ERR("Invalid LED ID: %d\n", (int)request.led_id);
	}

	// Prepare the PutLedResponse
	PutLedResponse response = PutLedResponse_init_zero;
	response.led_id = led_id;
	response.led_control.led_state_type = led_units[led_id].user_cfg.state;
	response.led_control.led_color_type = led_units[led_id].user_cfg.color;

	// Send the response back to the central
	uint8_t responseData[PutLedResponse_size];
	pb_ostream_t ostream = pb_ostream_from_buffer(responseData, sizeof(responseData));
	
	if (!pb_encode(&ostream, PutLedResponse_fields, &response)) {
		LOG_ERR("Failed to encode PutLedResponse");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	bt_gatt_notify(conn, attr, responseData, ostream.bytes_written);
  return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notifications_enabled) {
        LOG_INF("Notifications enabled");
    } else {
        LOG_INF("Notifications disabled");
    }
}

/**
 * @brief GATT service definition
 */
BT_GATT_SERVICE_DEFINE(
    led_svc, BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
    BT_GATT_CHARACTERISTIC(&get_led_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           NULL, write_get_led, NULL),
		BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&put_led_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           NULL, write_put_led, NULL), 
		BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),);

int led_init(void)
{
	int ret;

	if (initialized) {
		return -EPERM;
	}

	__ASSERT(ARRAY_SIZE(leds) != 0, "No LEDs found in dts");

	leds_num = ARRAY_SIZE(leds);

	ret = led_device_tree_parse();
	if (ret) {
		return ret;
	}

	k_timer_start(&led_blink_timer, K_MSEC(BLINK_FREQ_MS / 2), K_MSEC(BLINK_FREQ_MS / 2));
	initialized = true;
	return 0;
}
