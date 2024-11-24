/*******************************************************************************
 * This file is part of PsxNewLib.                                             *
 *                                                                             *
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * PsxNewLib is free software: you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * PsxNewLib is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with PsxNewLib. If not, see http://www.gnu.org/licenses.              *
 ******************************************************************************/
/**
 * \file PsxNewLib.h
 * \author SukkoPera <software@sukkology.net>
 * \date 27 Jan 2020
 * \brief Playstation controller interface library for Arduino
 *
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/PsxNewLib
 */

#ifndef PSXNEWLIB_H_
#define PSXNEWLIB_H_

#include "PSxNewLib_config.h"
#include "delay.h"
#include "gpio.h"
#include "spi.h"
#include <stdlib.h>
typedef uint8_t byte;
typedef unsigned int word;
// Uncomment this to have all byte exchanges logged to serial
//~ #define DUMP_COMMS

extern bool haveController;

/** \brief Command Inter-Byte Delay (us)
 *
 * Commands are several bytes long. This is the time to wait between two
 * consecutive bytes.
 *
 * This should actually be done by watching the \a Acknowledge line, but we are
 * ignoring it at the moment.
 */
//static const byte INTER_CMD_BYTE_DELAY = 50;
#define INTER_CMD_BYTE_DELAY 50

/** \brief Command timeout (ms)
 *
 * Commands are sent to the controller repeatedly, until they succeed or time
 * out. This is the length of that timeout.
 *
 * \sa COMMAND_RETRY_INTERVAL
 */
//static const unsigned long COMMAND_TIMEOUT = 250;
#define COMMAND_TIMEOUT 250

/** \brief Command Retry Interval (ms)
 *
 * When sending a command to the controller, if it does not succeed, it is
 * retried after this amount of time.
 */
//const unsigned long COMMAND_RETRY_INTERVAL = 10;
#define COMMAND_RETRY_INTERVAL 10

/** \brief Mode switch delay (ms)
 *
 * After a command has been issued successfully to the controller, this amount
 * of time is waited to allow it to complete any internal procedures required to
 * execute the command.
 *
 * \todo This is probably unnecessary.
 */
//const unsigned long MODE_SWITCH_DELAY = 500;
#define MODE_SWITCH_DELAY 500

/** \brief Type that is used to represent a single button in most places
 */
typedef enum {
  PSB_NONE       = 0x0000,
  PSB_SELECT     = 0x0001,
  PSB_L3         = 0x0002,
  PSB_R3         = 0x0004,
  PSB_START      = 0x0008,
  PSB_PAD_UP     = 0x0010,
  PSB_PAD_RIGHT  = 0x0020,
  PSB_PAD_DOWN   = 0x0040,
  PSB_PAD_LEFT   = 0x0080,
  PSB_L2         = 0x0100,
  PSB_R2         = 0x0200,
  PSB_L1         = 0x0400,
  PSB_R1         = 0x0800,
  PSB_TRIANGLE   = 0x1000,
  PSB_CIRCLE     = 0x2000,
  PSB_CROSS      = 0x4000,
  PSB_SQUARE     = 0x8000
} PsxButton;

/** \brief Button names
 */
// const char buttonSelectName[] = "Select";
// const char buttonL3Name[] = "L3";
// const char buttonR3Name[] = "R3";
// const char buttonStartName[] = "Start";
// const char buttonUpName[] = "Up";
// const char buttonRightName[] = "Right";
// const char buttonDownName[] = "Down";
// const char buttonLeftName[] = "Left";
// const char buttonL2Name[] = "L2";
// const char buttonR2Name[] = "R2";
// const char buttonL1Name[] = "L1";
// const char buttonR1Name[] = "R1";
// const char buttonTriangleName[] = "Triangle";
// const char buttonCircleName[] = "Circle";
// const char buttonCrossName[] = "Cross";
// const char buttonSquareName[] = "Square";
#define buttonSelectName "Select"
#define buttonL3Name "L3"
#define buttonR3Name "R3"
#define buttonStartName "Start"
#define buttonUpName "Up"
#define buttonRightName "Right"
#define buttonDownName "Down"
#define buttonLeftName "Left"
#define buttonL2Name "L2"
#define buttonR2Name "R2"
#define buttonL1Name "L1"
#define buttonR1Name "R1"
#define buttonTriangleName "Triangle"
#define buttonCircleName "Circle"
#define buttonCrossName "Cross"
#define buttonSquareName "Square"


/** \brief Type that is used to represent a single button when retrieving
 *         analog pressure data
 *
 * \sa getAnalogButton()
 */
typedef enum {
  PSAB_PAD_RIGHT  = 0,
  PSAB_PAD_LEFT   = 1,
  PSAB_PAD_UP     = 2,
  PSAB_PAD_DOWN   = 3,
  PSAB_TRIANGLE   = 4,
  PSAB_CIRCLE     = 5,
  PSAB_CROSS      = 6,
  PSAB_SQUARE     = 7,
  PSAB_L1         = 8,
  PSAB_R1         = 9,
  PSAB_L2         = 10,
  PSAB_R2         = 11
} PsxAnalogButton;

/** \brief Number of digital buttons
 *
 * Includes \a everything, i.e.: 4 directions, Square, Cross, Circle, Triangle,
 * L1/2/3, R1/2/3, Select and Start.
 *
 * This is the number of entries in #PsxButton.
 */
#define PSX_BUTTONS_NO  16

/** \brief Type that is used to report button presses
 */
typedef uint16_t PsxButtons;

/** \brief Size of buffer holding analog button data
 *
 * This is the size of the array returned by getAnalogButtonData().
 */
#define PSX_ANALOG_BTN_DATA_SIZE 12

//! \name Controller Commands
//! @{
/** \brief Enter Configuration Mode
 *
 * Command used to enter the controller configuration (also known as \a escape)
 * mode
 */
static const byte enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const byte exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
/* These shorter versions of enter_ and exit_config are accepted by all
 * controllers I've tested, even in analog mode, EXCEPT SCPH-1200, so let's use
 * the longer ones
 */
//~ static byte enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
//~ static byte exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x00};

/** \brief Read Controller Type
 *
 * Command used to read the controller type.
 *
 * This does not seem to be 100% reliable, or at least we don't know how to tell
 * all the various controllers apart.
 */
static const byte type_read[] = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const byte set_mode[] = {0x01, 0x44, 0x00, /* enabled */ 0x01, /* locked */ 0x03, 0x00, 0x00, 0x00, 0x00};
static const byte enable_rumble[] = {0x01, 0x4D, 0x00, /* motor 1 on */ 0x00, /* motor 2 on*/ 0x01, 0xff, 0xff, 0xff, 0xff};
static const byte set_pressures[] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};

/** \brief Poll all buttons
 *
 * Command used to read the status of all buttons.
 */
static const byte poll[] = {0x01, 0x42, 0x00, 0xFF, 0xFF};
//! @}

/** \brief Controller Type
 *
 * This is somehow derived from the reply to the #type_read command. It is NOT
 * much trustworthy, so it might be removed in the future.
 *
 * \sa getControllerType
 */
typedef enum {
  PSCTRL_UNKNOWN = 0,     //!< No idea
  PSCTRL_DUALSHOCK,     //!< DualShock or compatible
  PSCTRL_DSWIRELESS,      //!< Sony DualShock Wireless
  PSCTRL_GUITHERO,      //!< Guitar Hero controller
  PSCTRL_MAX
} PsxControllerType;

/** \brief Controller Protocol
 *
 * Identifies the protocol the controller uses to report axes positions and
 * button presses. It's quite more reliable than #PsxControllerType, so use this
 * if you must.
 *
 * \sa getProtocol
 */
typedef enum {
  PSPROTO_UNKNOWN = 0,    //!< No idea
  PSPROTO_DIGITAL,      //!< Original controller (SCPH-1010) protocol (8 digital buttons + START + SELECT)
  PSPROTO_DUALSHOCK,      //!< DualShock (has analog axes)
  PSPROTO_DUALSHOCK2,     //!< DualShock 2 (has analog axes and buttons)
  PSPROTO_FLIGHTSTICK,    //!< Green-mode (like DualShock but missing SELECT, L3 and R3)
  PSPROTO_NEGCON,       //!< Namco neGcon (has 1 analog X axis and analog Square, Circle and L1 buttons)
  PSPROTO_JOGCON,       //!< Namco Jogcon (Wheel is mapped to analog X axis, half a rotation in each direction)
  PSPROTO_GUNCON,
  PSPROTO_MAX
} PsxControllerProtocol;

// Controller Type
// const char ctrlTypeUnknown[] = "Unknown";
// const char ctrlTypeDualShock[] = "Dual Shock";
// const char ctrlTypeDsWireless[] = "Dual Shock Wireless";
// const char ctrlTypeGuitHero[] = "Guitar Hero";
// const char ctrlTypeOutOfBounds[] = "(Out of bounds)";
#define ctrlTypeUnknown "Unknown"
#define ctrlTypeDualShock "Dual Shock"
#define ctrlTypeDsWireless "Dual Shock Wireless"
#define ctrlTypeGuitHero "Guitar Hero"
#define ctrlTypeOutOfBounds "(Out of bounds)"

extern const char *const controllerTypeStrings[PSCTRL_MAX + 1];

// Controller Protocol
// const char ctrlProtoUnknown[] = "Unknown";
// const char ctrlProtoDigital[] = "Digital";
// const char ctrlProtoDualShock[] = "Dual Shock";
// const char ctrlProtoDualShock2[] = "Dual Shock 2";
// const char ctrlProtoFlightstick[] = "Flightstick";
// const char ctrlProtoNegcon[] = "neGcon";
// const char ctrlProtoJogcon[] = "Jogcon";
// const char ctrlProtoOutOfBounds[] = "(Out of bounds)";
#define ctrlProtoUnknown "Unknown"
#define ctrlProtoDigital "Digital"
#define ctrlProtoDualShock "Dual Shock"
#define ctrlProtoDualShock2 "Dual Shock 2"
#define ctrlProtoFlightstick "Flightstick"
#define ctrlProtoNegcon "neGcon"
#define ctrlProtoJogcon "Jogcon"
#define ctrlProtoOutOfBounds "(Out of bounds)"

extern const char *const controllerProtoStrings[PSPROTO_MAX + 1];

/** \brief Analog sticks minimum value
 *
 * Minimum value reported by analog sticks. This usually means that the stick is
 * fully either at the top or left position. Note that some sticks might not get
 * fully down to this value.
 *
 * \sa ANALOG_MAX_VALUE
 * \sa ANALOG_IDLE_VALUE
 */
//const byte ANALOG_MIN_VALUE = 0U;
#define ANALOG_MIN_VALUE 0U

/** \brief Analog sticks maximum value
 *
 * Maximum value reported by analog sticks. This usually means that the stick is
 * fully either at the bottom or right position. Note that some sticks might not
 * get fully up to this value.
 *
 * \sa ANALOGI_MAX_VALUE
 * \sa ANALOG_IDLE_VALUE
 */
//const byte ANALOG_MAX_VALUE = 255U;
#define ANALOG_MAX_VALUE 255U

/** \brief Analog sticks idle value
 *
 * Value reported when an analog stick is in the (ideal) center position. Note
 * that old and worn-out sticks might not self-center perfectly when released,
 * so you should never rely on this precise value to be reported.
 *
 * Also note that the up/down and left/right ranges are off by one, since values
 * 0-127 represent up/left and 129-255 mean down/right. The former interval
 * contains 128 different values, while the latter only 127. Sometimes you will
 * need to take this in consideration.
 */
//const byte ANALOG_IDLE_VALUE = 128U;
#define ANALOG_IDLE_VALUE 128U

/** \brief neGcon I/II-button press threshold
 *
 * The neGcon does not report digital button press data for its analog buttons,
 * so we have to make it up. The Square, Cross digital buttons will be
 * reported as pressed when the analog value of the II and I buttons
 * (respectively), goes over this threshold.
 *
 * \sa NEGCON_L_BUTTON_THRESHOLD
 */
//const byte NEGCON_I_II_BUTTON_THRESHOLD = 128U;
#define NEGCON_I_II_BUTTON_THRESHOLD 128U

/** \brief neGcon L-button press threshold
 *
 * The neGcon does not report digital button press data for its analog buttons,
 * so we have to make it up. The L1 digital button will be reported as pressed
 * when the analog value of the L buttons goes over this threshold.
 *
 * This value has been tuned so that the L button gets digitally triggered at
 * about the same point as the non-analog R button. This is done "empirically"
 * and might need tuning on a different controller than the one I actually have.
 *
 * \sa NEGCON_I_II_BUTTON_THRESHOLD
 */
//const byte NEGCON_L_BUTTON_THRESHOLD = 240U;
#define NEGCON_L_BUTTON_THRESHOLD 240U

//! \brief Guncon read status
typedef enum {
  //! Guncon data is valid
  GUNCON_OK,

  /** "Unexpected light": sensed light during VSYNC (e.g. from a Bulb or
   * Sunlight)
   */
  GUNCON_UNEXPECTED_LIGHT,

  /** "No light", this can mean either no light sensed at all (not aimed at
   * screen, or screen too dark: ERROR) or no light sensed yet (when trying to
   * read during rendering: BUSY)
   */
  GUNCON_NO_LIGHT,

  /** Data is not valid for some other reason (i.e.: no Guncon, read failed,
   * etc...)
   */
  GUNCON_OTHER_ERROR
} GunconStatus;

/** \brief PSX Controller Interface
 */

/** \brief Size of internal communication buffer
 *
 * This can be sized after the longest command reply (which is 21 bytes for
 * 01 42 when in DualShock 2 mode), but we're better safe than sorry.
 */
//static const byte BUFFER_SIZE = 32;
#define BUFFER_SIZE 32

/** \brief Internal communication buffer
 *
 * This is used to hold replies received from the controller.
 */
extern byte inputBuffer[BUFFER_SIZE];

/** \brief Previous (Digital) Button Status
 *
 * The individual bits can be identified through #PsxButton.
 */
extern PsxButtons previousButtonWord;

/** \brief (Digital) Button Status
 *
 * The individual bits can be identified through #PsxButton.
 */
extern PsxButtons buttonWord;

/** \brief Controller Protocol
 *
 * The protocol controller data was interpreted with at the last call to
 * read()
 *
 * \sa getProtocol
 */
extern PsxControllerProtocol protocol;

//! \name Analog Stick Data
//! @{
extern byte lx;   //!< Horizontal axis of left stick [0-255, L to R]
extern byte ly;   //!< Vertical axis of left stick [0-255, U to D]
extern byte rx;   //!< Horizontal axis of right stick [0-255, L to R]
extern byte ry;   //!< Vertical axis of right stick [0-255, U to D]

extern bool analogSticksValid;  //!< True if the above were valid at the last call to read()
//! @}

/** \brief Analog Button Data
 *
 * \todo What's the meaning of every individual byte?
 */
extern byte analogButtonData[PSX_ANALOG_BTN_DATA_SIZE];

/** \brief Analog Button Data Validity
 *
 * True if the #analogButtonData were valid in last call to read()
 */
extern bool analogButtonDataValid;

/** \brief Rumble feature enabled or disabled.
 *
 * True if rumble has been turned on with command 0x4d, false otherwise.
 *  Rumble must be enabled and 7.5v supplied to pin 3!
 */
extern bool rumbleEnabled;

/** \brief requested left motor (motor 1) power.
 *
 * 0xff for on, 0x00 for off, motor does not support partial activation.
 *  Rumble must be enabled and 7.5v supplied to pin 3!
 */
extern byte motor1Level;

/** \brief requested right motor (motor 2) power.
 *
 * 0x00 to 0xFF -> 0 to 100% power.
 *  Rumble must be enabled and 7.5v supplied to pin 3!
 */
extern byte motor2Level;

static inline bool isValidReply(const byte *status)
{
  //~ return status[0] != 0xFF || status[1] != 0xFF || status[2] != 0xFF;
  return status[1] != 0xFF && (status[2] == 0x5A || status[2] == 0x00);
  //~ return /* status[0] == 0xFF && */ status[1] != 0xFF && status[2] == 0x5A;
}

// Green Mode controllers
static inline bool isFlightstickReply(const byte *status)
{
  return (status[1] & 0xF0) == 0x50;
}

static inline bool isDualShockReply(const byte *status)
{
  return (status[1] & 0xF0) == 0x70;
}

static inline bool isDualShock2Reply(const byte *status)
{
  return status[1] == 0x79;
}

static inline bool isDigitalReply(const byte *status)
{
  return (status[1] & 0xF0) == 0x40;
}

static inline bool isConfigReply(const byte *status)
{
  return (status[1] & 0xF0) == 0xF0;
}

static inline bool isNegconReply(const byte *status)
{
  return status[1] == 0x23;
}

static inline bool isJogconReply(const byte *status)
{
  return (status[1] & 0xF0) == 0xE0;
}

static inline bool isGunconReply(const byte *status)
{
  return status[1] == 0x63;
}

/** \brief Assert the Attention line
 *
 * This function must be implemented by derived classes and must set the
 * Attention line \a low so that the controller will pay attention to what
 * we will send.
 */
void psxc_attention(void);

/** \brief Deassert the Attention line
 *
 * This function must be implemented by derived classes and must set the
 * Attention line \a high so that the controller will no longer pay
 * attention to what we will send.
 */
void psxc_noAttention();

//public:
/** \brief Initialize library
 *
 * This function shall be called before any others, it will initialize the
 * communication and return if a supported controller was found. It shall
 * also be called to reinitialize the communication whenever the controller
 * is unplugged.
 *
 * Derived classes can override this function if they need to perform
 * additional initializations, but shall call it on return.
 *
 * \return true if a supported controller was found, false otherwise
 */
bool psxc_begin();

//! \name Configuration Mode Functions
//! @{

/** \brief Enter Configuration Mode
 *
 * Some controllers can be configured in several aspects. For instance,
 * DualShock controllers can return analog stick data. This function puts
 * the controller in configuration mode.
 *
 * Note that <i>Configuration Mode</i> is sometimes called <i>Escape Mode</i>.
 *
 * \return true if Configuration Mode was entered successfully
 */
bool psxc_enterConfigMode();

/** \brief Enable (or disable) analog sticks
 *
 * This function enables or disables the analog sticks that were introduced
 * with DualShock controllers. When they are enabled, the getLeftAnalog()
 * and getRightAnalog() functions can be used to retrieve their positions.
 * Also, button presses for L3 and R3 will be available through the
 * buttonPressed() and similar functions.
 *
 * When analog sticks are enabled, the \a ANALOG led will light up (in red)
 * on the controller.
 *
 * Note that on some third-party controllers, when analog sticks are
 * disabled the analog levers will "emulate" the D-Pad and possibly the
 * []/^/O/X buttons. This does not happen on official Sony controllers.
 *
 * This function will only work if when the controller is in Configuration
 * Mode.
 *
 * \param[in] enabled true to enable, false to disable
 * \param[in] locked If true, the \a ANALOG button on the controller will be
 *                   disabled and the user will not be able to turn off the
 *                   analog sticks.
 * \return true if the command was ackowledged by the controller. Note that
 *         this does not fully guarantee that the analog sticks were enabled
 *         as this can only be checked after Configuration Mode is exited.
 */
// bool psxc_enableAnalogSticks (bool enabled = true, bool locked = false);
bool psxc_enableAnalogSticks(bool enabled, bool locked);

/** \brief Enable (or disable) the vibration capability of the DualShock / DualShock 2
 *
 * This function enables or disables the rumble feature of the DualShock / DualShock 2 controllers.
 *  NOTE that this function does nothing on its own - the vibration on/off must be set using
 *  setRumble() and the controller will begin to vibrate when the read() function is
 *  next called.
 *
 * This function will only work if when the controller is in Configuration
 * Mode.
 *
 * \param[in] enabled true to enable both motors, false to disable them.
 *
 * \return true if we got bytes back. Eventually we should wait for ACK from the controller.
 */
//bool psxc_enableRumble(bool enabled = true)
bool psxc_enableRumble(bool enabled);

/** \brief Set the requested power output of the rumble motors on DualShock / DualShock 2 controllers.
 *
 * This function sets internal variables that set the requested motor power of the rumble motors.
 *  NOTE this does nothing if rumble has not been enabled with enableRumble(), rumble motors will
 *  activate or deactivate to match the arguments of this function with the next call to read()
 *
 * \param[in] enabled true to activate motor 1, false to deactivate.
 * \param[in] requested motor power of motor 2, where 0x00 to 0xFF corresponds to 0 to 100%.
 */
//void setRumble(bool motor1Active = true, byte motor2Power = 0xff)
void psxc_setRumble(bool motor1Active, byte motor2Power);

/** \brief Enable (or disable) analog buttons
 *
 * This function enables or disables the analog buttons that were introduced
 * with DualShock 2 controllers. When they are enabled, the
 * getAnalogButton() functions can be used to retrieve how deep/strongly
 * they are pressed. This applies to the D-Pad buttons, []/^/O/X, L1/2 and
 * R1/2
 *
 * This function will only work if when the controller is in Configuration
 * Mode.
 *
 * \param[in] enabled true to enable, false to disable
 * \return true if the command was ackowledged by the controller. Note that
 *         this does not fully guarantee that the analog sticks were enabled
 *         as this can only be checked after Configuration Mode is exited.
 */
// bool enableAnalogButtons (bool enabled = true)
bool psxc_enableAnalogButtons(bool enabled);

/** \brief Retrieve the controller type
 *
 * This function retrieves the controller type. It is not 100% reliable, so
 * do not rely on it for anything other than a vague indication (for
 * instance, the DualShock SCPH-1200 controller gets reported as the Guitar
 * Hero controller...).
 *
 * This function will only work if when the controller is in Configuration
 * Mode.
 *
 * \return The (tentative) controller type
 */
PsxControllerType psxc_getControllerType();

bool psxc_exitConfigMode();


//! @}    // Configuration Mode Functions

//! \name Polling Functions
//! @{

/** \brief Retrieve the controller protocol
 *
 * This function retrieves the protocol that was used to interpret
 * controller data at the last call to read().
 *
 * \return The controller protocol
 */
static inline PsxControllerProtocol psxc_getProtocol()
{
  return protocol;
}

/** \brief Poll the controller
 *
 * This function polls the controller for button and stick data. It self-
 * adapts to all the supported controller types and populates internal
 * variables with the retrieved information, which can be later accessed
 * through the inspection functions.
 *
 * This function must be called quite often in order to keep the controller
 * alive. Most controllers have some kind of watchdog that will reset them
 * if they don't get polled at least every so often (like a couple dozen
 * times per seconds).
 *
 * If this function fails repeatedly, it can safely be assumed that the
 * controller has been disconnected (or that it is not supported if it
 * failed right from the beginning).
 *
 * \return true if the read was successful, false otherwise
 */
bool psxc_read();

#if 0
/** \brief Check if any button has changed state
 *
 * \return true if any button has changed state with regard to the previous
 *         call to read(), false otherwise
 */
static inline bool buttonsChanged()
{
  return ((previousButtonWord ^ buttonWord) > 0);
}

/** \brief Check if a button has changed state
 *
 * \return true if \a button has changed state with regard to the previous
 *         call to read(), false otherwise
 */
static inline bool buttonChanged(const PsxButtons button)
{
  return (((previousButtonWord ^ buttonWord) & button) > 0);
}

/** \brief Check if a button is currently pressed
 *
 * \param[in] button The button to be checked
 * \return true if \a button was pressed in last call to read(), false
 *         otherwise
 */
static inline bool buttonPressed(const PsxButton button)
{
  return buttonPressed(~buttonWord, button);
}

/** \brief Check if a button is pressed in a Button Word
 *
 * \param[in] buttons The button word to check in
 * \param[in] button The button to be checked
 * \return true if \a button is pressed in \a buttons, false otherwise
 */
static inline bool buttonPressed(const PsxButtons buttons, const PsxButton button)
{
  return ((buttons & (const PsxButtons)(button)) > 0);
}

/** \brief Check if a button has just been pressed
 *
 * \param[in] button The button to be checked
 * \return true if \a button was not pressed in the previous call to read()
 *         and is now, false otherwise
 */
static inline bool buttonJustPressed(const PsxButton button)
{
  return (buttonChanged(button) & buttonPressed(button));
}

/** \brief Check if a button has just been released
 *
 * \param[in] button The button to be checked
 * \return true if \a button was pressed in the previous call to read() and
 *         is not now, false otherwise
 */
static inline bool buttonJustReleased(const PsxButton button)
{
  return (buttonChanged(button) & ((~previousButtonWord & button) > 0));
}

/** \brief Check if NO button is pressed in a Button Word
 *
 * \param[in] buttons The button word to check in
 * \return true if all buttons in \a buttons are released, false otherwise
 */
static inline bool noButtonPressed(const PsxButtons buttons)
{
  return buttons == PSB_NONE;
}

/** \brief Check if NO button is currently pressed
 *
 * \return true if all buttons were released in the last call to read(),
 *         false otherwise
 */
static inline bool noButtonPressed(void)
{
  return buttonWord == ~PSB_NONE;
}
#endif
/** \brief Retrieve the <em>Button Word</em>
 *
 * The button word contains the status of all digital buttons and can be
 * retrieved so that it can be inspected later.
 *
 * \sa buttonPressed
 * \sa noButtonPressed
 *
 * \return the Button Word
 */
static inline PsxButtons psxc_getButtonWord()
{
  return ~buttonWord;
}

/** \brief Retrieve button pressure depth/strength
 *
 * This function will return how deeply/strongly a button is pressed. It
 * will only work on DualShock 2 controllers after enabling this feature
 * with enableAnalogButtons().
 *
 * Note that button pressure depth/strength is only available for the D-Pad
 * buttons, []/^/O/X, L1/2 and R1/2.
 *
 * \param[in] button the button the retrieve the pressure depth/strength of
 * \return the pressure depth/strength [0-255, Fully released to fully
 *         pressed]
 */
static inline byte getAnalogButton(const PsxAnalogButton button)
{
  byte ret = 0;

  if (analogButtonDataValid) {
    ret = analogButtonData[button];
    //~ } else if (buttonPressed (button)) {    // FIXME
    //~ // No analog data, assume fully pressed or fully released
    //~ ret = 0xFF;
  }

  return ret;
}

/** \brief Retrieve all analog button data
 */
static inline const byte *getAnalogButtonData()
{
  return analogButtonDataValid ? analogButtonData : NULL;
}

/** \brief Retrieve position of the \a left analog stick
 *
 * This function will return the absolute position of the left analog stick.
 *
 * Note that not all controllers have analog sticks, in which case this
 * function will return false.
 *
 * \param[in] x A variable where the horizontal position will be stored
 *              [0-255, L to R]
 * \param[in] y A variable where the vertical position will be stored
 *              [0-255, U to D]
 * \return true if the returned position is valid, false otherwise
 */
static inline bool getLeftAnalog(byte *x, byte *y)
{
  *x = lx;
  *y = ly;

  return analogSticksValid;
}

/** \brief Retrieve position of the \a right analog stick
 *
 * This function will return the absolute position of the right analog
 * stick.
 *
 * Note that not all controllers have analog sticks, in which case this
 * function will return false.
 *
 * \param[in] x A variable where the horizontal position will be stored
 *              [0-255, L to R]
 * \param[in] y A variable where the vertical position will be stored
 *              [0-255, U to D]
 * \return true if the returned position is valid, false otherwise
 */
static inline bool getRightAnalog(byte *x, byte *y)
{
  *x = rx;
  *y = ry;

  return analogSticksValid;
}

/** \brief Retrieve Guncon X/Y readings
 *
 * According to the Nocash PSX Specifications, the Guncon returns 16-bit X/Y
 * coordinates of the screen it is aimed at.
 *
 * The coordinates are updated in all frames. The absolute min/max may vary
 * from TV set to TV set.
 *
 * Vertical coordinates are counted in scanlines (ie. equal to pixels).
 * Horizontal coordinates are counted in 8MHz units (which would equal a
 * resolution of 385 pixels; which can be, for example, converted to 320
 * pixel resolution as X=X*320/385).
 *
 * <em>Caution:</em> The gun only returns meaningful data when read shortly
 * after begin of VBLANK (ie. AFTER rendering, but still BEFORE vsync), so
 * make sure to only consider readings returning \a GUNCON_OK;
 *
 * \sa GunconStatus
 */
static inline GunconStatus getGunconCoordinates(word *x, word *y)
{
  GunconStatus status = GUNCON_OTHER_ERROR;

  if (protocol == PSPROTO_GUNCON && analogSticksValid) {
    status = GUNCON_OK;

    *x = (((word) ry) << 8) | rx;
    *y = (((word) ly) << 8) | lx;

    if (*x == 0x0001) {
      if (*y == 0x0005) {
        status = GUNCON_UNEXPECTED_LIGHT;
      } else if (*y == 0x000A) {
        status = GUNCON_NO_LIGHT;
      }
    }
  }

  return status;
}

void psxc_dumpButtons(PsxButtons psxButtons);
void psxc_dumpAnalog(const char *str, const int8_t x, const int8_t y);
bool psxc_rightAnalogMoved(int8_t *x, int8_t *y);
bool psxc_leftAnalogMoved(int8_t *x, int8_t *y);
#endif
