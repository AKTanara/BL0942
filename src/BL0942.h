/*
BL0942 Library
Author: Santeri Lindfors
Description: Interface for the BL0942 single-phase energy metering chip via
UART.
*/
#pragma once

#include <Arduino.h>

namespace bl0942 {

// Calibration values
// Currently set for Tongou DIN rail power meter unit
// https://github.com/esphome/esphome-docs/blob/current/components/sensor/bl0942.rst
static const float BL0942_PREF = 613.7;
static const float BL0942_UREF = 15850;
static const float BL0942_IREF = 248360;
static const float BL0942_EREF = 5268;

struct SensorData {
  float voltage;   // Voltage RMS
  float current;   // Current RMS
  float watt;      // Power in watts
  float energy;    // Energy in kWh:
                   // - If CNT_CLR_SEL_ENABLE → delta since last read
                   // - If CNT_CLR_SEL_DISABLE → total accumulated
  float frequency; // Frequency in Hz
};

enum UpdateFrequency : uint8_t {
  UPDATE_FREQUENCY_400MS = 0x00, // 400ms update frequency
  UPDATE_FREQUENCY_800MS = 0x08  // 800ms update frequency
};

enum RmsWaveform : uint8_t {
  RMS_WAVEFORM_FULL = 0x00, // Full waveform
  RMS_WAVEFORM_AC = 0x10    // AC waveform
};

enum LineFrequency : uint8_t {
  LINE_FREQUENCY_50HZ = 0x00, // 50Hz
  LINE_FREQUENCY_60HZ = 0x20  // 60Hz
};

enum ClearMode : uint8_t {
  CNT_CLR_SEL_DISABLE = 0x00, // Disable (do not clear after read)
  CNT_CLR_SEL_ENABLE = 0x40   // Enable (clear after read)
};

enum AccumulationMode : uint8_t {
  ACCUMULATION_MODE_ALGEBRAIC = 0x00, // Algebraic accumulation mode
  ACCUMULATION_MODE_ABSOLUTE = 0x80   // Absolute accumulation mode
};

enum UartRate : uint8_t {
  UART_RATE_4800 = 0x00,  // 4800bps
  UART_RATE_9600 = 0x01,  // 9600bps
  UART_RATE_19200 = 0x02, // 19200bps
  UART_RATE_38400 = 0x03  // 38400bps
};

// Selects which internal signal is routed to the CF1 logic output pin
// (register OT_FUNX, bits [1:0]). Only one function can be active on CF1
// at any given moment - it's a multiplexer, not a combiner. Switching this
// does NOT pause or affect anything computed internally (CF_CNT keeps
// accumulating, I_FAST_RMS keeps updating, etc. regardless of what's
// currently routed to the pin) - it only changes what reaches the pin
// itself. Values match the OT_FUNX CF1_FUNX_SEL bit encoding directly.
enum CF1Function : uint8_t {
  CF1_FUNC_ENERGY_PULSE = 0x00,       // CF: active energy calibration pulse (factory default)
  CF1_FUNC_OVERCURRENT = 0x01,        // O_C: over-current event logic output
  CF1_FUNC_ZERO_CROSS_VOLTAGE = 0x02, // ZX_V: zero-crossing voltage
  CF1_FUNC_ZERO_CROSS_CURRENT = 0x03  // ZX_I: zero-crossing current
};

struct ModeConfig {
  UpdateFrequency rms_update_freq = UPDATE_FREQUENCY_400MS;
  RmsWaveform rms_waveform = RMS_WAVEFORM_FULL;
  LineFrequency ac_freq = LINE_FREQUENCY_50HZ;
  ClearMode clear_mode = CNT_CLR_SEL_DISABLE;
  AccumulationMode accumulation_mode = ACCUMULATION_MODE_ABSOLUTE;
  UartRate uart_rate = UART_RATE_4800;
};

class BL0942 {
public:
  using OnDataReceivedCallback = std::function<void(SensorData &data)>;

  BL0942(HardwareSerial &serial, uint8_t address = 0);
  void setup(const ModeConfig &config = ModeConfig{});
  void reset();

  void onDataReceived(OnDataReceivedCallback);
  bool loop();
  void update();

  void print_registers();

  // Changes which signal is routed to the CF1 pin at runtime (OT_FUNX
  // register). Safe to call repeatedly/at any time after setup() - e.g.
  // switching to CF1_FUNC_ZERO_CROSS_CURRENT briefly to time a relay
  // latch at a current zero-crossing, then back to
  // CF1_FUNC_ENERGY_PULSE for a normal power-proportional LED blink.
  // Read-modify-write: only bits [1:0] of OT_FUNX are touched, so this
  // doesn't disturb CF2/ZX pin routing on packages that have them
  // (SSOP10L has no CF2/ZX pins, so this is the only OT_FUNX field that
  // matters on that package).
  void setCF1Function(CF1Function function);

protected:
  struct DataPacket {
    uint8_t frame_header;
    uint32_t i_rms : 24;
    uint32_t v_rms : 24;
    uint32_t i_fast_rms : 24;
    int32_t watt : 24;
    uint32_t cf_cnt : 24;
    uint16_t frequency;
    uint8_t reserved1;
    uint8_t status;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t checksum;
  } __attribute__((packed));

  HardwareSerial &serial_;
  OnDataReceivedCallback dataCallback;
  uint8_t address_;
  bool use_delta_energy_;
  uint32_t prev_cf_cnt_ = 0;

  int read_reg_(uint8_t reg);
  void write_reg_(uint8_t reg, uint32_t val);
  bool validate_checksum_(DataPacket *data);
  void received_package_(DataPacket *data);
};
} // namespace bl0942
