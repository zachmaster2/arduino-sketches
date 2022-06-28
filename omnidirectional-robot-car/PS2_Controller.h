#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <avr/io.h>

#define CLK_DELAY_MICROS     4
#define CTRL_DELAY_MICROS    3
#define READ_DELAY_MICROS 1000

// These are the button constants to be used with is_button_pressed(...)
#define PSB_SELECT    0x0001
#define PSB_L3        0x0002
#define PSB_R3        0x0004
#define PSB_START     0x0008
#define PSB_PAD_UP    0x0010
#define PSB_PAD_RIGHT 0x0020
#define PSB_PAD_DOWN  0x0040
#define PSB_PAD_LEFT  0x0080
#define PSB_L2        0x0100
#define PSB_R2        0x0200
#define PSB_L1        0x0400
#define PSB_R1        0x0800
#define PSB_GREEN     0x1000
#define PSB_RED       0x2000
#define PSB_BLUE      0x4000
#define PSB_PINK      0x8000
#define PSB_TRIANGLE  0x1000
#define PSB_CIRCLE    0x2000
#define PSB_CROSS     0x4000
#define PSB_SQUARE    0x8000

// These are the joystick constants to be used with get_joystick_value(...)
#define PSS_RX 2
#define PSS_RY 3
#define PSS_LX 4
#define PSS_LY 5

class PS2_Controller
{
public:

  /*
   * Sets up the pin modes and port registers for each of the data links
   * connected from the PS2 controller's receiver to the Arduino board.
   *
   * Example mapping from a PS2 controller's receiver to an Arduino board:
   *
   * PS2 Controller Receiver Wires
   * -----------------------------
   * | Wire      | Color  | Pin  |
   * -----------------------------
   * | clock     | white  | A0   |
   * | attention | green  | A1   |
   * | command   | blue   | A2   |
   * | data      | yellow | A3   |
   * | power     | red    | 3.3V |
   * | ground    | black  | GND  |
   * -----------------------------
   *
   * Note: Current wires are soldered onto the motor driver shield (which sits
   * on top of the Arduino board).
   */
  void
  configure(
      uint8_t clock_pin,
      uint8_t attention_pin,
      uint8_t command_pin,
      uint8_t data_pin);

  /*
   * Returns true if the controller reports back a supported type, false
   * otherwise.
   */
  bool
  is_supported();

  /*
   * Returns true if the provided button is being pressed, false otherwise.
   */
  bool
  is_button_pressed(
      uint16_t button);

  /*
   * Returns a value in the range [0, 255] corresponding to the position of
   * the provided joystick (either X or Y axis) within its range of motion.
   */
  uint8_t
  get_joystick_value(
      uint8_t joystick);

  /*
   * Queries the controller for the current state of all buttons and both
   * joysticks. The data is stored internal to this class. After calling
   * read_data(), use is_button_pressed(...) and get_joystick_value(...) to
   * retrieve the state of individual buttons and/or joysticks.
   *
   * Returns true if the queried button and joystick data is valid, false
   * otherwise.
   */
  bool
  read_data();

private:

  uint8_t
  get_type();

  void
  set_mode();

  void
  enter_config_mode();

  void
  exit_config_mode();

  void
  send_receive_bytes(
      const uint8_t   num_bytes,
      const uint8_t * bytes_in,
            uint8_t * bytes_out = 0x0);

  inline void
  CLK_BitSet();

  inline void
  CLK_BitClear();

  inline void
  ATT_BitSet();

  inline void
  ATT_BitClear();

  inline void
  CMD_BitSet();

  inline void
  CMD_BitClear();

  inline void
  CMD_BitWrite(
      const bool bitValue);

  inline bool
  DAT_BitRead();

  uint8_t _clk_mask; 
  volatile uint8_t *_clk_oreg;
  uint8_t _att_mask; 
  volatile uint8_t *_att_oreg;
  uint8_t _cmd_mask; 
  volatile uint8_t *_cmd_oreg;
  uint8_t _dat_mask; 
  volatile uint8_t *_dat_ireg;

  // Holds 16 on/off buttons (1 bit each) and 4 joystick values (8 bits each).
  uint8_t data_bytes[6];
};

#endif /* PS2_CONTROLLER_H */
