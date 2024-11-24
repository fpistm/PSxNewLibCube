#ifndef __PSxNewLibCube_config_h__
#define __PSxNewLibCube_config_h__
/** \brief Pin used for Controller Attention (ATTN)
 *
 * This pin makes the controller pay attention to what we're saying. The shield
 * has pin 10 wired for this purpose.
 */
#define PS2_ATT_PORT GPIOB
#define PS2_ATT_PIN LL_GPIO_PIN_6

/** \brief Pin for Controller Presence Led
 *
 * This led will light up steadily whenever a controller is detected and be off
 * otherwise.
 */
#define HAVECONTROLLER_PORT GPIOA
#define HAVECONTROLLER_PIN LL_GPIO_PIN_9

/** \brief Pin for Button Press Led
 *
 * This led will light up whenever a button is pressed on the controller.
 */
// const byte PIN_BUTTONPRESS = 7;

/** \brief Pin for Analog Movement Detection
 *
 * This led will light up whenever the left or right analog sticks are moved.
 */
// const byte PIN_ANALOG = 6;

/** \brief Dead zone for analog sticks
 *
 * If the analog stick moves less than this value from the center position, it
 * is considered still.
 *
 * \sa ANALOG_IDLE_VALUE
 */
#define ANALOG_DEAD_ZONE  50U


// SPI pins for Nucleo F401RE
// MISO
#define DAT_PIN LL_GPIO_PIN_6
#define DAT_PORT GPIOA
// MOSI
#define CMD_PIN LL_GPIO_PIN_7
#define CMD_PORT GPIOA
// SCK
#define CLK_PIN LL_GPIO_PIN_5
#define CLK_PORT GPIOA


#endif // __PSxNewLibCube_config_h__