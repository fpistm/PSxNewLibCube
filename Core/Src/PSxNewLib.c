#include "PSxNewLib.h"
#include <string.h>

bool haveController = false;
PsxControllerProtocol protocol;
const char *const psxButtonNames[PSX_BUTTONS_NO] = {
  buttonSelectName,
  buttonL3Name,
  buttonR3Name,
  buttonStartName,
  buttonUpName,
  buttonRightName,
  buttonDownName,
  buttonLeftName,
  buttonL2Name,
  buttonR2Name,
  buttonL1Name,
  buttonR1Name,
  buttonTriangleName,
  buttonCircleName,
  buttonCrossName,
  buttonSquareName
};

const char *const controllerTypeStrings[PSCTRL_MAX + 1] = {
  ctrlTypeUnknown,
  ctrlTypeDualShock,
  ctrlTypeDsWireless,
  ctrlTypeGuitHero,
  ctrlTypeOutOfBounds
};

const char *const controllerProtoStrings[PSPROTO_MAX + 1] = {
  ctrlProtoUnknown,
  ctrlProtoDigital,
  ctrlProtoDualShock,
  ctrlProtoDualShock2,
  ctrlProtoFlightstick,
  ctrlProtoNegcon,
  ctrlProtoJogcon,
  ctrlTypeOutOfBounds
};
byte inputBuffer[BUFFER_SIZE];
PsxButtons previousButtonWord;
PsxButtons buttonWord;
bool analogSticksValid;

byte lx;    //!< Horizontal axis of left stick [0-255, L to R]
byte ly;    //!< Vertical axis of left stick [0-255, U to D]
byte rx;    //!< Horizontal axis of right stick [0-255, L to R]
byte ry;    //!< Vertical axis of right stick [0-255, U to D]

byte analogButtonData[PSX_ANALOG_BTN_DATA_SIZE];
bool analogButtonDataValid;
bool rumbleEnabled;
byte motor1Level;
byte motor2Level;
/** \brief Attention Delay
 *
 * Time between attention being issued to the controller and the first clock
 * edge (us).
 */
const byte ATTN_DELAY = 50;

/** \brief Transfer several bytes to/from the controller
 *
 * This function transfers an array of <i>command</i> bytes to the
 * controller and reads back an equally sized array of <i>data</i> bytes.
 *
 * \param[in] out The command bytes to send the controller
 * \param[out] in The data bytes returned by the controller, must be sized
 *                 to hold at least \a len bytes
 * \param[in] len The amount of bytes to be exchanged
 */
void shiftInOut(const byte *out, byte *in, const byte len)
{
#ifdef DUMP_COMMS
  byte inbuf[len];
#endif

  for (byte i = 0; i < len; ++i) {
    byte tmp = transfer(out != NULL ? out[i] : 0x5A);
#ifdef DUMP_COMMS
    inbuf[i] = tmp;
#endif
    if (in != NULL) {
      in[i] = tmp;
    }

    delayMicroseconds(INTER_CMD_BYTE_DELAY);    // Very important!
  }

#ifdef DUMP_COMMS
  printf("<-- ");
  for (byte i = 0; i < len; ++i) {
    if (out && out[i] < 0x10) {
      printf("%i", 0);
    }
    printf("%x ", out ? out[i] : 0x5A);
  }
  printf("\n");
  printf("--> ");
  for (byte i = 0; i < len; ++i) {
    if (inbuf[i] < 0x10) {
      printf("%i", 0);
    }
    printf("%x ", inbuf[i]);
  }
  printf("\n");
#endif
}

/** \brief Get reply length
 *
 * Calculates the length of a command reply, in bytes
 *
 * \param[in] buf The buffer containing the reply, must be at least 2 bytes
 *                long
 * \return The calculated length
 */
static inline byte getReplyLength(const byte *buf)
{
  return (buf[1] & 0x0F) * 2;
}

/** \brief Transfer several bytes to/from the controller
 *
 * This function transfers an array of <i>command</i> bytes to the
 * controller and reads back the full reply of <i>data</i> bytes. The size
 * of the reply is calculated automatically and padding bytes (0x5A) are
 * appended to the outgoing message if it is shorter.
 *
 * The reply is stored in an internal buffer and will be valid until the
 * next call to this function, so make sure to save anything if is needed.
 *
 * \param[out] out The data bytes returned by the controller, must be sized
 *                 to hold at least \a len bytes
 * \param[in] len The amount of bytes to be exchanged
 * \return A pointer to a buffer containing the reply, whose size can be
 *         calculated with getReplyLength()
 */
byte *autoShift(const byte *out, const byte len)
{
  byte *ret = NULL;

  if (len >= 3 && len <= BUFFER_SIZE) {
    // All commands have at least 3 bytes, so shift out those first
    shiftInOut(out, inputBuffer, 3);
    if (isValidReply(inputBuffer)) {
      // Reply is good, get full length
      byte replyLen = getReplyLength(inputBuffer);

      // Shift out rest of command
      if (len > 3) {
        shiftInOut(out + 3, inputBuffer + 3, len - 3);
      }

      byte left = replyLen - len + 3;
      //~ Serial.print ("len = ");
      //~ Serial.print (replyLen);
      //~ Serial.print (", left = ");
      //~ Serial.println (left);
      if (left == 0) {
        // The whole reply was gathered
        ret = inputBuffer;
      } else if (len + left <= BUFFER_SIZE) {
        // Part of reply is still missing and we have space for it
        shiftInOut(NULL, inputBuffer + len, left);
        ret = inputBuffer;
      } else {
        // Reply incomplete but not enough space provided
      }
    }
  }

  return ret;
}

void attention()
{
  //att.low ();
  write_gpio(PS2_ATT_PORT, PS2_ATT_PIN, false);

  //SPI.beginTransaction (spiSettings);
  delayMicroseconds(ATTN_DELAY);
}

void noAttention()
{
  //~ delayMicroseconds (5);

  // SPI.endTransaction ();

  // Make sure CMD and CLK sit high
  // cmd.high ();
  write_gpio(CMD_PORT, CMD_PIN, true);
  // clk.high ();
  write_gpio(CLK_PORT, CLK_PIN, true);
  // att.high ();
  write_gpio(PS2_ATT_PORT, PS2_ATT_PIN, true);
  delayMicroseconds(ATTN_DELAY);
}

// Public functions
bool psxc_begin(void)
{
  //att.config (OUTPUT, HIGH);    // HIGH -> Controller not selected
  configure_gpio(PS2_ATT_PORT, PS2_ATT_PIN, LL_GPIO_MODE_OUTPUT, LL_GPIO_PULL_NO);
  write_gpio(PS2_ATT_PORT, PS2_ATT_PIN, true);
  /* We need to force these at startup, that's why we need to know which
    * pins are used for HW SPI. It's a sort of "start condition" the
    * controller needs.
    */

  //  DigitalPin<MOSI> cmd;
  //  DigitalPin<MISO> dat;
  //  DigitalPin<SCK> clk;
  // cmd.config (OUTPUT, HIGH);
  configure_gpio(CMD_PORT, CMD_PIN, LL_GPIO_MODE_OUTPUT, LL_GPIO_PULL_NO);
  write_gpio(CMD_PORT, CMD_PIN, true);
  // clk.config (OUTPUT, HIGH);
  configure_gpio(CLK_PORT, CLK_PIN, LL_GPIO_MODE_OUTPUT, LL_GPIO_PULL_UP);
  write_gpio(CLK_PORT, CLK_PIN, true);
  // dat.config (INPUT, HIGH);     // Enable pull-up
  configure_gpio(DAT_PORT, DAT_PIN, LL_GPIO_MODE_INPUT, LL_GPIO_PULL_UP);
  // SPI.begin ();
  MX_SPI1_Init();

  // Start with all analog axes at midway position
  lx = ANALOG_IDLE_VALUE;
  ly = ANALOG_IDLE_VALUE;
  rx = ANALOG_IDLE_VALUE;
  ry = ANALOG_IDLE_VALUE;

  analogSticksValid = false;
  memset(analogButtonData, 0, sizeof(analogButtonData));

  protocol = PSPROTO_UNKNOWN;

  rumbleEnabled = false;
  motor1Level = 0x00;
  motor2Level = 0x00;

  // Some disposable readings to let the controller know we are here
  for (byte i = 0; i < 5; ++i) {
    psxc_read();
    delay(1);
  }

  return psxc_read();
}

bool psxc_enterConfigMode()
{
  bool ret = false;

  unsigned long start = millis();
  do {
    attention();
    byte *in = autoShift(enter_config, 4);
    noAttention();

    ret = in != NULL && isConfigReply(in);

    if (!ret) {
      delay(COMMAND_RETRY_INTERVAL);
    }
  } while (!ret && millis() - start <= COMMAND_TIMEOUT);
  delay(MODE_SWITCH_DELAY);

  return ret;
}

bool psxc_enableAnalogSticks(bool enabled, bool locked)
{
  bool ret = false;
  byte out[sizeof(set_mode)];

  memcpy(out, set_mode, sizeof(set_mode));
  out[3] = enabled ? 0x01 : 0x00;
  out[4] = locked ? 0x03 : 0x00;

  unsigned long start = millis();
  byte cnt = 0;
  do {
    attention();
    byte *in = autoShift(out, 5);
    noAttention();

    /* We can't know if we have successfully enabled analog mode until
      * we get out of config mode, so let's just be happy if we get a few
      * consecutive valid replies
      */
    if (in != NULL) {
      ++cnt;
    }
    ret = cnt >= 3;

    if (!ret) {
      delay(COMMAND_RETRY_INTERVAL);
    }
  } while (!ret && millis() - start <= COMMAND_TIMEOUT);
  delay(MODE_SWITCH_DELAY);

  return ret;
}

bool psxc_enableRumble(bool enabled)
{
  bool ret = true;
  byte out[sizeof(enable_rumble)];

  memcpy(out, enable_rumble, sizeof(enable_rumble));
  out[3] = enabled ? 0x00 : 0xff;
  out[4] = enabled ? 0x01 : 0xff;

  unsigned long start = millis();
  byte cnt = 0;
  do {
    attention();
    byte *in = autoShift(out, 5);
    noAttention();

    /* The real way to check if the command was successful is to wait for ACK.
      *  Currently the library doesn't support the pin, so I will just assume success.
      */
    if (in != NULL) {
      ++cnt;
    }
    ret = cnt >= 3;

    if (!ret) {
      delay(COMMAND_RETRY_INTERVAL);
    }
  } while (!ret && millis() - start <= COMMAND_TIMEOUT);
  delay(MODE_SWITCH_DELAY);

  rumbleEnabled = true;
  return ret;
}

void psxc_setRumble(bool motor1Active, byte motor2Power)
{
  motor1Level = motor1Active ? 0xff : 0x00;
  motor2Level = motor2Power;
}

bool psxc_enableAnalogButtons(bool enabled)
{
  bool ret = false;
  byte out[sizeof(set_mode)];

  memcpy(out, set_pressures, sizeof(set_pressures));
  if (!enabled) {
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x00;
  }

  unsigned long start = millis();
  byte cnt = 0;
  do {
    attention();
    byte *in = autoShift(out, sizeof(set_pressures));
    noAttention();

    /* We can't know if we have successfully enabled analog mode until
      * we get out of config mode, so let's just be happy if we get a few
      * consecutive valid replies
      */
    if (in != NULL) {
      ++cnt;
    }
    ret = cnt >= 3;

    if (!ret) {
      delay(COMMAND_RETRY_INTERVAL);
    }
  } while (!ret && millis() - start <= COMMAND_TIMEOUT);
  delay(MODE_SWITCH_DELAY);

  return ret;
}

PsxControllerType psxc_getControllerType()
{
  PsxControllerType ret = PSCTRL_UNKNOWN;

  attention();
  byte *in = autoShift(type_read, 3);
  noAttention();

  if (in != NULL) {
    byte controllerType = in[3];
    if (controllerType == 0x03) {
      ret = PSCTRL_DUALSHOCK;
      //~ } else if (controllerType == 0x01 && in[1] == 0x42) {
      //~ return 4;   // ???
    }  else if (controllerType == 0x01 && in[1] != 0x42) {
      ret = PSCTRL_GUITHERO;
    } else if (controllerType == 0x0C) {
      ret = PSCTRL_DSWIRELESS;
    }
  }

  return ret;
}

bool psxc_exitConfigMode()
{
  bool ret = false;

  unsigned long start = millis();
  do {
    attention();
    //~ shiftInOut (poll, in, sizeof (poll));
    //~ shiftInOut (exit_config, in, sizeof (exit_config));
    byte *in = autoShift(exit_config, 4);
    noAttention();

    ret = in != NULL && !isConfigReply(in);

    if (!ret) {
      delay(COMMAND_RETRY_INTERVAL);
    }
  } while (!ret && millis() - start <= COMMAND_TIMEOUT);
  delay(MODE_SWITCH_DELAY);

  return ret;
}

bool psxc_read()
{
  bool ret = false;

  analogSticksValid = false;
  analogButtonDataValid = false;

  attention();
  byte *in = NULL;
  if (rumbleEnabled) {
    byte out[sizeof(poll)];
    memcpy(out, poll, sizeof(poll));
    out[3] = motor1Level;
    out[4] = motor2Level;
    in = autoShift(out, sizeof(poll));
  } else {
    in = autoShift(poll, 3);
  }
  noAttention();

  if (in != NULL) {
    if (isConfigReply(in)) {
      // We're stuck in config mode, try to get out
      psxc_exitConfigMode();
    } else {
      // We surely have buttons
      previousButtonWord = buttonWord;
      buttonWord = ((PsxButtons) in[4] << 8) | in[3];

      // See if we have anything more to read
      if (isDualShock2Reply(in)) {
        protocol = PSPROTO_DUALSHOCK2;
      } else if (isDualShockReply(in)) {
        protocol = PSPROTO_DUALSHOCK;
      } else if (isFlightstickReply(in)) {
        protocol = PSPROTO_FLIGHTSTICK;
      } else if (isNegconReply(in)) {
        protocol = PSPROTO_NEGCON;
      } else if (isJogconReply(in)) {
        protocol = PSPROTO_JOGCON;
      } else if (isGunconReply(in)) {
        protocol = PSPROTO_GUNCON;
      } else {
        protocol = PSPROTO_DIGITAL;
      }

      switch (protocol) {
        case PSPROTO_DUALSHOCK2:
          // We also have analog button data
          analogButtonDataValid = true;
          for (int i = 0; i < PSX_ANALOG_BTN_DATA_SIZE; ++i) {
            analogButtonData[i] = in[i + 9];
          }
        /* Now fall through to DualShock case, the next line
          * avoids GCC warning
          */
        /* FALLTHRU */
        case PSPROTO_GUNCON:
        /* The Guncon uses the same reply format as DualShocks,
          * by just falling through we'll end up with:
          * - A (Left side) -> Start
          * - B (Right side) -> Cross
          * - Trigger -> Circle
          * - Low byte of HSYNC -> RX
          * - High byte of HSYNC -> RY
          * - Low byte of VSYNC -> LX
          * - High byte of VSYNC -> LY
          */
        case PSPROTO_DUALSHOCK:
        case PSPROTO_FLIGHTSTICK:
          // We have analog stick data
          analogSticksValid = true;
          rx = in[5];
          ry = in[6];
          lx = in[7];
          ly = in[8];
          break;
        case PSPROTO_NEGCON:
          // Map the twist axis to X axis of left analog
          analogSticksValid = true;
          lx = in[5];

          // Map analog button data to their reasonable counterparts
          analogButtonDataValid = true;
          analogButtonData[PSAB_CROSS] = in[6];
          analogButtonData[PSAB_SQUARE] = in[7];
          analogButtonData[PSAB_L1] = in[8];

          // Make up "missing" digital data
          if (analogButtonData[PSAB_SQUARE] >= NEGCON_I_II_BUTTON_THRESHOLD) {
            buttonWord &= ~PSB_SQUARE;
          }
          if (analogButtonData[PSAB_CROSS] >= NEGCON_I_II_BUTTON_THRESHOLD) {
            buttonWord &= ~PSB_CROSS;
          }
          if (analogButtonData[PSAB_L1] >= NEGCON_L_BUTTON_THRESHOLD) {
            buttonWord &= ~PSB_L1;
          }
          break;
        case PSPROTO_JOGCON:
          /* Map the wheel X axis of left analog, half a rotation
            * per direction: byte 5 has the wheel position, it is
            * 0 at startup, then we have 0xFF down to 0x80 for
            * left/CCW, and 0x01 up to 0x80 for right/CW
            *
            * byte 6 is the number of full CW rotations
            * byte 7 is 0 if wheel is still, 1 if it is rotating CW
            *        and 2 if rotation CCW
            * byte 8 seems to stay at 0
            *
            * We'll want to cap the movement halfway in each
            * direction, for ease of use/implementation.
            */
          analogSticksValid = true;
          if (in[6] < 0x80) {
            // CW up to half
            lx = in[5] < 0x80 ? in[5] : (0x80 - 1);
          } else {
            // CCW down to half
            lx = in[5] > 0x80 ? in[5] : (0x80 + 1);
          }

          // Bring to the usual 0-255 range
          lx += 0x80;
          break;
        default:
          // We are already done
          break;
      }

      ret = true;
    }
  }

  return ret;
}

byte psxButtonToIndex(PsxButtons psxButtons)
{
  byte i;

  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }

    psxButtons >>= 1U;
  }

  return i;
}

const char *getButtonName(PsxButtons psxButton)
{
  const char *bName = "";

  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    bName = psxButtonNames[b];
  }
  return bName;
}

void psxc_dumpButtons(PsxButtons psxButtons)
{
  static PsxButtons lastB = 0;

  if (psxButtons != lastB) {
    lastB = psxButtons;     // Save it before we alter it

    printf("Pressed: ");

    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);
      if (b < PSX_BUTTONS_NO) {
        const char *bName = psxButtonNames[b];
        printf("%s", bName);
      }

      psxButtons &= ~(1 << b);

      if (psxButtons != 0) {
        printf(", ");
      }
    }

    printf("\n");
  }
}

void psxc_dumpAnalog(const char *str, const int8_t x, const int8_t y)
{
  printf("%s", str);
  printf(" analog: x = ");
  printf("%d", x);
  printf(", y = ");
  printf("%d\n", y);
}

// We like analog sticks to return something in the [-127, +127] range
bool psxc_rightAnalogMoved(int8_t *x, int8_t *y)
{
  bool ret = false;
  byte rx, ry;

  if (getRightAnalog(&rx, &ry)) {         // [0 ... 255]
    int8_t deltaRX = rx - ANALOG_IDLE_VALUE;  // [-128 ... 127]
    if (abs(deltaRX) > ANALOG_DEAD_ZONE) {
      *x = deltaRX;
      if (*x == -128) {
        *x = -127;
      }
      ret = true;
    } else {
      *x = 0;
    }

    int8_t deltaRY = ry - ANALOG_IDLE_VALUE;
    if (abs(deltaRY) > ANALOG_DEAD_ZONE) {
      *y = deltaRY;
      if (*y == -128) {
        *y = -127;
      }
      ret = true;
    } else {
      *y = 0;
    }
  }

  return ret;
}

bool psxc_leftAnalogMoved(int8_t *x, int8_t *y)
{
  bool ret = false;
  byte lx, ly;

  if (getLeftAnalog(&lx, &ly)) {        // [0 ... 255]
    if ((psxc_getProtocol() != PSPROTO_NEGCON) && (psxc_getProtocol() != PSPROTO_JOGCON)) {
      int8_t deltaLX = lx - ANALOG_IDLE_VALUE;  // [-128 ... 127]
      uint8_t deltaLXabs = abs(deltaLX);
      if (deltaLXabs > ANALOG_DEAD_ZONE) {
        *x = deltaLX;
        if (*x == -128) {
          *x = -127;
        }
        ret = true;
      } else {
        *x = 0;
      }

      int8_t deltaLY = ly - ANALOG_IDLE_VALUE;
      uint8_t deltaLYabs = abs(deltaLY);
      if (deltaLYabs > ANALOG_DEAD_ZONE) {
        *y = deltaLY;
        if (*y == -128) {
          *y = -127;
        }
        ret = true;
      } else {
        *y = 0;
      }
    } else {
      // The neGcon and JogCon are more precise and work better without any dead zone
      *x = lx, *y = ly;
    }
  }

  return ret;
}
