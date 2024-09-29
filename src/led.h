#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

#define LED_APP_1 0
#define LED_APP_2 1
#define LED_APP_3 2
#define LED_APP_4 3

// RGB of LED
#define RED 0
#define GREEN 1
#define BLUE 2

#define LED_SERVICE_UUID_VAL BT_UUID_128_ENCODE(0xf7547938, 0x68ba, 0x11ec, 0x90d6, 0x0242ac120003)

/**
 * @brief Set the state of a given LED unit to blink.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
 *			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_blink(uint8_t led_unit, ...);

/**
 * @brief Turn the given LED unit on.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
*			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_on(uint8_t led_unit, ...);

/**
 * @brief Set the state of a given LED unit to off.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *		Using a LED unit assigned to another core will do nothing and return 0.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_off(uint8_t led_unit);

/**
 * @brief Initialise the LED module
 *
 * @note This will parse the .dts files and configure all LEDs.
 *
 * @return	0 on success
 *		-EPERM if already initialsed
 *		-ENXIO if a LED is missing unit number in dts
 *		-ENODEV if a LED is missing color identifier
 */
int led_init(void);

#endif /* _LED_H_ */
