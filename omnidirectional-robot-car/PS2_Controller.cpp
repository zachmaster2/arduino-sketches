#include "PS2_Controller.h"

#include <Arduino.h>

void
PS2_Controller::configure(
    uint8_t clock_pin,
    uint8_t attention_pin,
    uint8_t command_pin,
    uint8_t data_pin)
{
  _clk_mask = digitalPinToBitMask(clock_pin);
  _clk_oreg = portOutputRegister(digitalPinToPort(clock_pin));
  _att_mask = digitalPinToBitMask(attention_pin);
  _att_oreg = portOutputRegister(digitalPinToPort(attention_pin));
  _cmd_mask = digitalPinToBitMask(command_pin);
  _cmd_oreg = portOutputRegister(digitalPinToPort(command_pin));
  _dat_mask = digitalPinToBitMask(data_pin);
  _dat_ireg = portInputRegister(digitalPinToPort(data_pin));

  pinMode(clock_pin, OUTPUT);
  pinMode(attention_pin, OUTPUT);
  pinMode(command_pin, OUTPUT);
  pinMode(data_pin, INPUT);

  digitalWrite(data_pin, HIGH); // enable pull-up
}

bool
PS2_Controller::is_supported()
{
  bool is_type_supported  = false;

  const uint8_t type = get_type();

  // Determine if the controller type is supported (ie. Guitar Hero controller
  // is not supported).
  if (type == 0x03 || type == 0x0C)
  {
    is_type_supported = true;
  }

  return is_type_supported;
}

bool
PS2_Controller::is_button_pressed(
    uint16_t button)
{
  uint16_t button_bitset = *((uint16_t *) data_bytes);
  return (~button_bitset & button);
}

uint8_t
PS2_Controller::get_joystick_value(
    uint8_t joystick)
{
  return data_bytes[joystick];
}

bool
PS2_Controller::read_data()
{
  const uint8_t read_data_input[] =
      { 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  const uint8_t num_bytes = sizeof(read_data_input);

  uint8_t read_data_output[num_bytes];

  send_receive_bytes(num_bytes, read_data_input, read_data_output);

  const bool needs_mode_reset = (read_data_output[1] & 0xF0) == 0x40;
  bool is_expected_mode = (read_data_output[1] & 0xF0) == 0x70;

  if (needs_mode_reset)
  {
    // After a period of time, the controller switches itself to a different
    // mode and returns invalid data. When this happens, we just have to reset
    // the controller's mode and then try to read data again. If the second
    // data read attempt doesn't report the expected mode, then something else
    // is probably messed up.
    set_mode();
    send_receive_bytes(num_bytes, read_data_input, read_data_output);
  }
  else if (!is_expected_mode)
  {
    // If the controller is not in the expected mode, but also doesn't need a
    // mode reset, then it's probably either unresponsive (ie. idle timeout)
    // or powered off. If that happens, then resetting the mode won't help. In
    // fact, if we reset the mode while it's unresponsive (but still powered
    // on), then the next data read will report the expected mode, even though
    // the data will still be invalid.
    return false;
  }

  const uint8_t data_byte_offset = 3;

  for (uint8_t byte_index = 0; byte_index < sizeof(data_bytes); byte_index++)
  {
    data_bytes[byte_index] = read_data_output[data_byte_offset + byte_index];
  }

  // If the controller is in the expected mode, then it's fairly safe to
  // assume that the data is valid.
  is_expected_mode = (read_data_output[1] & 0xF0) == 0x70;

  return is_expected_mode;
}

uint8_t
PS2_Controller::get_type()
{
  const uint8_t get_type_input[] =
      { 0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };

  const uint8_t num_bytes = sizeof(get_type_input);

  uint8_t get_type_output[num_bytes];

  enter_config_mode();

  send_receive_bytes(num_bytes, get_type_input, get_type_output);

  exit_config_mode();

  return get_type_output[3];
}

void
PS2_Controller::set_mode()
{
  const uint8_t set_mode_input[] =
      { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };

  const uint8_t num_bytes = sizeof(set_mode_input);

  enter_config_mode();

  send_receive_bytes(num_bytes, set_mode_input);

  exit_config_mode();

  // Extra delay to ensure the new mode takes full effect before reading data.
  delay(10);
}

void
PS2_Controller::enter_config_mode()
{
  const uint8_t enter_config_input[] =
      { 0x01, 0x43, 0x00, 0x01, 0x00 };

  const uint8_t num_bytes = sizeof(enter_config_input);

  send_receive_bytes(num_bytes, enter_config_input);
}

void
PS2_Controller::exit_config_mode()
{
  const uint8_t exit_config_input[] =
      { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };

  const uint8_t num_bytes = sizeof(exit_config_input);

  send_receive_bytes(num_bytes, exit_config_input);
}

void
PS2_Controller::send_receive_bytes(
    const uint8_t   num_bytes,
    const uint8_t * bytes_in,
          uint8_t * bytes_out)
{  
  CMD_BitSet();
  CLK_BitSet();
  ATT_BitClear();
  delayMicroseconds(CTRL_DELAY_MICROS);

  for (uint8_t byte_index = 0; byte_index < num_bytes; byte_index++)
  {
    const uint8_t byte_in = bytes_in[byte_index];
    uint8_t byte_out;

    for (uint8_t bit_index = 0; bit_index < 8; bit_index++)
    {
      CMD_BitWrite(bitRead(byte_in, bit_index));
      CLK_BitClear();
      delayMicroseconds(CLK_DELAY_MICROS);

      bitWrite(byte_out, bit_index, DAT_BitRead());
      CLK_BitSet();
    }

    if (bytes_out)
    {
      bytes_out[byte_index] = byte_out;
    }

    CMD_BitSet();
    delayMicroseconds(CTRL_DELAY_MICROS);
  }

  ATT_BitSet();
  delayMicroseconds(READ_DELAY_MICROS);
}

inline void
PS2_Controller::CLK_BitSet()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_clk_oreg |= _clk_mask;
  SREG = old_sreg;
}

inline void
PS2_Controller::CLK_BitClear()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_clk_oreg &= ~_clk_mask;
  SREG = old_sreg;
}

inline void
PS2_Controller::ATT_BitSet()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_att_oreg |= _att_mask ;
  SREG = old_sreg;
}

inline void
PS2_Controller::ATT_BitClear()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_att_oreg &= ~_att_mask;
  SREG = old_sreg;
}

inline void
PS2_Controller::CMD_BitSet()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_cmd_oreg |= _cmd_mask;
  SREG = old_sreg;
}

inline void
PS2_Controller::CMD_BitClear()
{
  register uint8_t old_sreg = SREG;
  cli();
  *_cmd_oreg &= ~_cmd_mask;
  SREG = old_sreg;
}

inline void
PS2_Controller::CMD_BitWrite(
    const bool bitValue)
{
  (bitValue) ? CMD_BitSet() : CMD_BitClear();
}

inline bool
PS2_Controller::DAT_BitRead()
{
  return (*_dat_ireg & _dat_mask) ? true : false;
}
