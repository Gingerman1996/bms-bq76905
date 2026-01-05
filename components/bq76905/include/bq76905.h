#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c.h"
#include "esp_err.h"

class BQ76905 {
public:
  // Direct command register addresses (BQ76905)
  enum class Reg : uint8_t {
    // Status registers
    ControlStatus = 0x00, // Control and status register
    SafetyAlertA = 0x02,  // Safety alert A
    SafetyStatusA = 0x03, // Safety status A
    SafetyAlertB = 0x04,  // Safety alert B
    SafetyStatusB = 0x05, // Safety status B
    BatteryStatus = 0x12, // Battery status

    // Voltage measurement registers
    Cell1Voltage = 0x14, // Cell 1 voltage
    Cell2Voltage = 0x16, // Cell 2 voltage
    Cell3Voltage = 0x18, // Cell 3 voltage
    Cell4Voltage = 0x1A, // Cell 4 voltage
    Cell5Voltage = 0x1C, // Cell 5 voltage
    Reg18Voltage = 0x22, // REG18 voltage
    VssVoltage = 0x24,   // VSS voltage
    StackVoltage = 0x26, // Stack (pack) voltage

    // Temperature and current registers
    IntTemperature = 0x28, // Internal temperature
    TsMeasurement = 0x2A,  // TS (NTC thermistor) measurement
    RawCurrent = 0x36,     // Raw current (24-bit)
    Current = 0x3A,        // Current (16-bit)
    Cc1Current = 0x3C,     // CC1 current

    // Subcommand and Data Memory access registers
    SubcommandLow = 0x3E,  // Subcommand address LSB / Data Memory access
    SubcommandHigh = 0x3F, // Subcommand address MSB
    TransferBuffer = 0x40, // Transfer buffer start (0x40-0x5F)
    Checksum = 0x60,       // Checksum for Data Memory write
    DataLength = 0x61,     // Data length for Data Memory writeƒƒƒ

    // Alarm and control registers
    AlarmStatus = 0x62,   // Alarm status
    AlarmRaw = 0x64,      // Alarm raw
    AlarmEnable = 0x66,   // Alarm enable
    FetControl = 0x68,    // FET control
    RegoutControl = 0x69, // REGOUT control
    DsgPwm = 0x6A,        // Discharge PWM
    ChgPwm = 0x6C,        // Charge PWM
  };

  // Safety Status A bit definitions (voltage/current faults)
  static constexpr uint8_t SAFETY_A_COV = (1 << 7);      // Cell Overvoltage
  static constexpr uint8_t SAFETY_A_CUV = (1 << 6);      // Cell Undervoltage
  static constexpr uint8_t SAFETY_A_SCD = (1 << 5);      // Short Circuit Discharge
  static constexpr uint8_t SAFETY_A_OCD1 = (1 << 4);     // Overcurrent Discharge 1
  static constexpr uint8_t SAFETY_A_OCD2 = (1 << 3);     // Overcurrent Discharge 2
  static constexpr uint8_t SAFETY_A_OCC = (1 << 2);      // Overcurrent Charge
  static constexpr uint8_t SAFETY_A_CURLATCH = (1 << 1); // Current Protection Latched

  // Safety Status B bit definitions (temperature/system faults)
  static constexpr uint8_t SAFETY_B_OTD = (1 << 7); // Overtemperature Discharge
  static constexpr uint8_t SAFETY_B_OTC = (1 << 6); // Overtemperature Charge
  static constexpr uint8_t SAFETY_B_UTD = (1 << 5); // Undertemperature Discharge
  static constexpr uint8_t SAFETY_B_UTC = (1 << 4); // Undertemperature Charge
  static constexpr uint8_t SAFETY_B_HWD = (1 << 2); // Host Watchdog Timeout

  // Battery Status bit definitions
  static constexpr uint16_t BATT_SS = (1 << 12);     // Safety Status Fault
  static constexpr uint16_t BATT_FET_EN = (1 << 8);  // Autonomous FET Control Active
  static constexpr uint16_t BATT_CHG_FET = (1 << 3); // Charge FET State
  static constexpr uint16_t BATT_DSG_FET = (1 << 2); // Discharge FET State

  static constexpr uint8_t SLEEP_CHG = (1 << 4); // Sleep Charge bit in Control and Status register

  // Subcommand addresses for Data Memory access
  enum class SubCmd : uint16_t {
    SET_CFGUPDATE = 0x0090,
    EXIT_CFGUPDATE = 0x0092,
  };

  // Data Memory addresses - System Configuration
  static constexpr uint16_t ADDR_DA_CONFIG = 0x9019;   // DA Configuration
  static constexpr uint16_t ADDR_VCELL_MODE = 0x901B;  // Cell count configuration
  static constexpr uint16_t ADDR_FET_OPTIONS = 0x901E; // FET control options

  // Data Memory addresses - Protection Enable
  static constexpr uint16_t ADDR_ENABLE_PROT_A =
      0x9024; // Enable Protections A (COV, CUV, SCD, OCD, OCC)
  static constexpr uint16_t ADDR_ENABLE_PROT_B =
      0x9025; // Enable Protections B (OTC, OTD, UTC, UTD)

  // Data Memory addresses - Voltage Protection
  static constexpr uint16_t ADDR_CUV_THRESHOLD = 0x902E; // Cell Undervoltage
  static constexpr uint16_t ADDR_COV_THRESHOLD = 0x9032; // Cell Overvoltage

  // Data Memory addresses - Current Protection
  static constexpr uint16_t ADDR_OCC_THRESHOLD = 0x9036;  // Overcurrent in Charge
  static constexpr uint16_t ADDR_OCD1_THRESHOLD = 0x9038; // Overcurrent in Discharge 1
  static constexpr uint16_t ADDR_OCD2_THRESHOLD = 0x903A; // Overcurrent in Discharge 2
  static constexpr uint16_t ADDR_SCD_THRESHOLD = 0x903C;  // Short Circuit in Discharge

  // Data Memory addresses - Temperature Protection (in ADC counts)
  static constexpr uint16_t ADDR_OTC_THRESHOLD = 0x9040; // Overtemperature in Charge
  static constexpr uint16_t ADDR_OTD_THRESHOLD = 0x9046; // Overtemperature in Discharge
  static constexpr uint16_t ADDR_UTC_THRESHOLD = 0x904C; // Undertemperature in Charge
  static constexpr uint16_t ADDR_UTD_THRESHOLD = 0x9052; // Undertemperature in Discharge

  // Battery chemistry types
  enum class BatteryType {
    LFP,   // LiFePO4 (3.65V max, 2.0V min)
    LiIon, // Li-ion NMC/LCO (4.2V max, 3.0V min)
  };

  static constexpr uint8_t kMaxCells = 5;

  BQ76905(i2c_port_t port, uint8_t addr = 0x08);

  // Probe the device with a basic read.
  esp_err_t begin();

  void setTimeout(uint32_t timeout_ms);

  // High-level BMS functions (units follow datasheet definitions).
  esp_err_t getCellVoltage(uint8_t cell_idx, uint16_t &voltage_mv);
  esp_err_t getPackVoltage(uint16_t &voltage_mv);
  esp_err_t getCurrent(int16_t &current_ma);
  esp_err_t getTemperature(float &temp_c);
  esp_err_t getTSTemperature(float &temp_c); // External NTC thermistor
  esp_err_t getRawCurrent(int32_t &raw_current);

  // Battery configuration functions
  esp_err_t setConfigUpdateMode(bool enable);
  esp_err_t writeDataMemory(uint16_t address, uint16_t value);
  esp_err_t writeDataMemory8(uint16_t address, uint8_t value); // For 8-bit values
  esp_err_t setupBattery(BatteryType type);
  esp_err_t fullConfiguration(BatteryType type, uint8_t cell_count = 5);

  // Low level access.
  esp_err_t readReg(Reg reg, uint8_t *data, size_t len);
  esp_err_t writeReg(Reg reg, const uint8_t *data, size_t len);
  esp_err_t readU8(Reg reg, uint8_t &value);
  esp_err_t writeU8(Reg reg, uint8_t value);
  esp_err_t readU16LE(Reg reg, uint16_t &value);
  esp_err_t writeU16LE(Reg reg, uint16_t value);

  // Subcommand access
  esp_err_t sendSubcommand(SubCmd cmd);
  esp_err_t writeSubcommandData(uint16_t address, const uint8_t *data, size_t len);

  esp_err_t clearAlarmStatus(uint8_t mask);

  // Status checking and diagnostics
  esp_err_t checkStatus(); // Check BMS status and log any faults
  esp_err_t getBatteryStatus(uint16_t &status);
  esp_err_t getSafetyStatusA(uint8_t &status);
  esp_err_t getSafetyStatusB(uint8_t &status);

private:
  i2c_port_t _port;
  uint8_t _addr;
  uint32_t _timeout_ms;

  uint32_t _timeoutTicks() const;
};
