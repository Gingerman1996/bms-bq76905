#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "BQ25672.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstdint>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

#define PERIOD_UPDATE_REG2E_MS 10000

// I2C Address for BQ25672
#define BQ25672_I2C_ADDRESS 0x6B

// Register Addresses
#define REG00_MINIMAL_SYSTEM_VOLTAGE 0x00
#define REG01_CHARGE_VOLTAGE_LIMIT 0x01
#define REG03_CHARGE_CURRENT_LIMIT 0x03
#define REG05_INPUT_VOLTAGE_LIMIT 0x05
#define REG06_INPUT_CURRENT_LIMIT 0x06
#define REG1B_CHG_STAT 0x1C // Register Address for Charger Status 0
#define REG2E_ADC_CTRL 0x2E
#define REG3B_VBAT_ADC 0x3B
#define REG3D_VSYS_ADC 0x3D
#define REG3C_BAT_PCT 0x3F
#define REG3F_TEMP_ADC 0x41
#define REG33_IBAT_ADC 0x33 // Register Address for IBAT ADC
#define REG37_VBUS_ADC1 0x37 // Register Address for VBUS ADC1
#define REG39_VBUS_ADC2 0x39 // Register Address for VBUS ADC2
#define REG15_MPPT 0x15
#define REG_SYSTEM_STATUS 0x0A
#define REG_FAULT_STATUS 0x0B
#define REG_CHARGE_STATUS 0x0C
#define REG_INPUT_STATUS 0x0D

namespace {
constexpr uint16_t kChargeVoltageStepMv = 10;
constexpr uint16_t kChargeCurrentStepMa = 10;
constexpr uint16_t kChargeCurrentMinMa = 50;
constexpr uint16_t kChargeCurrentMaxMa = 5000;

constexpr uint16_t kLfpCellMaxMv = 3650;
constexpr uint16_t kLfpCellMinMv = 2000;
constexpr uint16_t kLiIonCellMaxMv = 4200;
constexpr uint16_t kLiIonCellMinMv = 3000;

uint16_t clampU16(uint16_t value, uint16_t min_value, uint16_t max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}
} // namespace

BQ25672::BQ25672() {}
BQ25672::~BQ25672() {}

esp_err_t BQ25672::begin(i2c_port_t port) {
  _port = port;
  esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  // To hold read register output
  uint16_t value;

  // Probe device by reading a status register
  ESP_RETURN_ON_ERROR(writeReadRegister(REG_SYSTEM_STATUS, 1, &value), TAG,
                      "BQ25672 address (0x%.2x) not found", BQ25672_I2C_ADDRESS);

  // Enable MPPT
  ESP_RETURN_ON_ERROR(writeRegister(0x15, 0xAB), TAG, "Failed write to enable MPPT");
  ESP_RETURN_ON_ERROR(writeReadRegister(0x15, 1, &value), TAG, "Failed read MPPT status");
  ESP_LOGI(TAG, "MPPT Status (0x15): 0x%.2x", value);
  if (value != 0xAB) {
    ESP_LOGE(TAG, "MPPT status is not as expected (0x%.2x)", (uint8_t)value);
    // return ESP_FAIL;
  }

  // Enable ADC with 15-bit resolution and Continuous Mode
  // ADC_EN = 1, 15-bit, Continuous
  ESP_RETURN_ON_ERROR(writeRegister(REG2E_ADC_CTRL, 0x80), TAG,
                      "Failed write to enable ADC 15-bit resolution with Continuous Mode");
  // Check if ADC is successfully enabled
  ESP_RETURN_ON_ERROR(writeReadRegister(REG2E_ADC_CTRL, 1, &value), TAG,
                      "Failed read to enable ADC 15-bit resolution with Continuous Mode");
  if (value & 0x80) {
    return ESP_OK;
  }

  ESP_LOGE(TAG, "ADC control value is not as expected (0x%.2x)", (uint8_t)value);

  // return ESP_FAIL;
  return ESP_OK;
}

esp_err_t BQ25672::configureBattery(BatteryType type, uint8_t cell_count,
                                    uint16_t charge_current_ma) {
  if (cell_count == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const char *type_name = (type == BatteryType::LFP) ? "LFP" : "Li-ion";
  uint16_t cell_max_mv = (type == BatteryType::LFP) ? kLfpCellMaxMv : kLiIonCellMaxMv;
  uint16_t cell_min_mv = (type == BatteryType::LFP) ? kLfpCellMinMv : kLiIonCellMinMv;

  uint32_t full_mv = static_cast<uint32_t>(cell_max_mv) * cell_count;
  uint32_t empty_mv = static_cast<uint32_t>(cell_min_mv) * cell_count;
  if (full_mv > 0xFFFFu || empty_mv > 0xFFFFu) {
    return ESP_ERR_INVALID_ARG;
  }

  _battery_full_mv = static_cast<uint16_t>(full_mv);
  _battery_empty_mv = static_cast<uint16_t>(empty_mv);

  ESP_LOGI(TAG, "Configuring charger: type=%s, cells=%u, target CV=%u mV",
           type_name, cell_count, _battery_full_mv);
  ESP_LOGI(TAG, "SOC range: %u-%u mV", _battery_empty_mv, _battery_full_mv);
  if (charge_current_ma > 0) {
    ESP_LOGI(TAG, "Setting charge current limit: %u mA", charge_current_ma);
  } else {
    ESP_LOGI(TAG, "Keeping existing charge current limit");
  }

  esp_err_t err = setChargeVoltageLimit(_battery_full_mv);
  if (err != ESP_OK) {
    return err;
  }

  if (charge_current_ma > 0) {
    err = setChargeCurrentLimit(charge_current_ma);
    if (err != ESP_OK) {
      return err;
    }
  }

  uint16_t reg_value = 0;
  if (writeReadRegister(REG01_CHARGE_VOLTAGE_LIMIT, 2, &reg_value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Voltage Limit: raw=0x%04X, %u mV", reg_value,
             reg_value * kChargeVoltageStepMv);
  } else {
    ESP_LOGW(TAG, "Failed to read back charge voltage limit");
  }

  if (writeReadRegister(REG03_CHARGE_CURRENT_LIMIT, 2, &reg_value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Current Limit: raw=0x%04X, %u mA", reg_value,
             reg_value * kChargeCurrentStepMa);
  } else {
    ESP_LOGW(TAG, "Failed to read back charge current limit");
  }

  return ESP_OK;
}

esp_err_t BQ25672::setChargeVoltageLimit(uint16_t voltage_mv) {
  uint32_t reg_value = voltage_mv / kChargeVoltageStepMv;
  if (reg_value > 0xFFFFu) {
    reg_value = 0xFFFFu;
  }
  return writeRegister16(REG01_CHARGE_VOLTAGE_LIMIT, static_cast<uint16_t>(reg_value));
}

esp_err_t BQ25672::setChargeCurrentLimit(uint16_t current_ma) {
  uint16_t clamped_current =
      clampU16(current_ma, kChargeCurrentMinMa, kChargeCurrentMaxMa);
  if (clamped_current != current_ma) {
    ESP_LOGW(TAG, "Charge current clamped from %u mA to %u mA", current_ma,
             clamped_current);
  }

  uint16_t reg_value = clamped_current / kChargeCurrentStepMa;
  return writeRegister16(REG03_CHARGE_CURRENT_LIMIT, reg_value);
}

esp_err_t BQ25672::update() {
  // Attempt enable MPTT and read the status
  uint16_t value = 0;
  writeRegister(0x15, 0xAB);
  writeReadRegister(0x15, 1, &value);
  ESP_LOGD(TAG, "MPPT Status (0x15): 0x%.2x", value);

  // Update REG2E every PERIOD_UPDATE_REG2E_MS
  esp_err_t err = ESP_OK;
  if ((MILLIS() - lastUpdateTime) > PERIOD_UPDATE_REG2E_MS) {
    // Write REG2E_ADC_CTRL to reset WD_RST
    err = writeRegister(REG2E_ADC_CTRL, 0x80);
    if (err != ESP_OK) { // ADC_EN = 1, 15-bit, Continuous
      ESP_LOGE(TAG, "Failed to reset WD_RST by write to REG2E_ADC_CTRL");
    }
    // writeRegister(0x05, 0xDC);
    lastUpdateTime = MILLIS();
  }

  return err;
}

esp_err_t BQ25672::getVBAT(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVBATRaw(&result), TAG, "Failed get VBAT");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getVSYS(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVSYSRaw(&result), TAG, "Failed get VSYS");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getBatteryPercentage(float *output) {
  uint16_t vbatAdc;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3B_VBAT_ADC, 2, &vbatAdc), TAG,
                      "Failed get battery percentage");
  float vbat = static_cast<float>(vbatAdc); // Convert to mV

  // Callulate percentage
  float vbat_min = static_cast<float>(_battery_empty_mv);
  float vbat_max = static_cast<float>(_battery_full_mv);
  float denom = vbat_max - vbat_min;
  if (denom <= 0.0f) {
    ESP_LOGE(TAG, "Invalid battery range (empty=%u, full=%u)", _battery_empty_mv,
             _battery_full_mv);
    *output = 0.0f;
    return ESP_ERR_INVALID_STATE;
  }

  float batteryPercentage = ((vbat - vbat_min) / denom) * 100;
  if (batteryPercentage > 100) {
    batteryPercentage = 100;
  } else if (batteryPercentage < 0) {
    batteryPercentage = 0;
  }
  *output = batteryPercentage;

  return ESP_OK;
}

esp_err_t BQ25672::getTemperature(float *output) {
  uint16_t temp;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3F_TEMP_ADC, 2, &temp), TAG, "Failed get temperature");
  *output = (float)temp;
  return ESP_OK;
}

esp_err_t BQ25672::getVBUS(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVBUSRaw(&result), TAG, "Failed get VBUS");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getBatteryCurrent(int16_t *output) {
  uint16_t raw;
  ESP_RETURN_ON_ERROR(getVBUSRaw(&raw), TAG, "Failed get battery current");
  int16_t batteryCurrent = (int16_t)(raw * 1.0);

  // Convert 2's Complement to signed integer
  if (batteryCurrent & 0x8000) {
    batteryCurrent = batteryCurrent - 0x10000;
  }
  *output = batteryCurrent;

  return ESP_OK;
}

BQ25672::ChargingStatus BQ25672::getChargingStatus() {
  // Read Charger Status 0
  uint16_t result;
  if (writeReadRegister(REG1B_CHG_STAT, 1, &result) != ESP_OK) {
    ESP_LOGE(TAG, "Failed get charging status raw");
    return ChargingStatus::Unknown;
  }

  // Extract CHG_STAT[2:0] (Bits 2 to 0)
  uint8_t chargingStatus = (uint8_t)result;
  uint8_t status = (chargingStatus >> 5) & 0x07;

  // Check charging status
  ChargingStatus cs = ChargingStatus::Unknown;
  switch (status) {
  case 0b000:
    cs = ChargingStatus::NotCharging;
    ESP_LOGI(TAG, "Charging status: not charging");
    break;
  case 0b001:
    cs = ChargingStatus::TrickleCharge;
    ESP_LOGI(TAG, "Charging status: trickle charge");
    break;
  case 0b010:
    cs = ChargingStatus::PreCharge;
    ESP_LOGI(TAG, "Charging status: pre-charge");
    break;
  case 0b011:
    cs = ChargingStatus::FastCharge;
    ESP_LOGI(TAG, "Charging status: fast charge (CC Mode)");
    break;
  case 0b100:
    cs = ChargingStatus::TaperCharge;
    ESP_LOGI(TAG, "Charging status: taper charge (CV Mode)");
    break;
  case 0b110:
    cs = ChargingStatus::TopOffTimerActiveCharging;
    ESP_LOGI(TAG, "Charging status: Top Off Timer Active Charging");
    break;
  case 0b111:
    cs = ChargingStatus::ChargeTerminationDone;
    ESP_LOGI(TAG, "Charging status: charge termination done");
    break;
  default:
    cs = ChargingStatus::Unknown;
    ESP_LOGI(TAG, "Charging status: unknown");
    break;
  }

  return cs;
}

esp_err_t BQ25672::getVBATRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3B_VBAT_ADC, 2, output), TAG, "Failed get VBAT raw");
  return ESP_OK;
}

esp_err_t BQ25672::getVSYSRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3D_VSYS_ADC, 2, output), TAG, "Failed get VSYS raw");
  return ESP_OK;
}

esp_err_t BQ25672::getTemperatureRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3F_TEMP_ADC, 2, output), TAG,
                      "Failed get raw temperature");
  return ESP_OK;
}

esp_err_t BQ25672::getVBUSRaw(uint16_t *output) {
  // First try reading from ADC1
  uint16_t adc1_value;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG37_VBUS_ADC1, 2, &adc1_value), TAG, "Failed get VBUS ADC1");
  
  // If ADC1 value is >= 1, use it
  if (adc1_value >= 1000) {
    *output = adc1_value;
    return ESP_OK;
  }
  
  // Otherwise, fall back to ADC2
  ESP_RETURN_ON_ERROR(writeReadRegister(REG39_VBUS_ADC2, 2, output), TAG, "Failed get VBUS ADC2");
  return ESP_OK;
}

esp_err_t BQ25672::getIBATRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG39_VBUS_ADC2, 2, output), TAG, "Failed get raw IBAT");
  return ESP_OK;
}

void BQ25672::printSystemStatus() {
  ESP_LOGI(TAG, "==== SYSTEM STATUS ====");
  uint16_t value;
  if (writeReadRegister(REG_SYSTEM_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "System status (0x%.2x): 0x%.2x", REG_SYSTEM_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_FAULT_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Fault status (0x%.2x): 0x%.2x", REG_FAULT_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_CHARGE_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge status (0x%.2x): 0x%.2x", REG_CHARGE_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_INPUT_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input status (0x%.2x): 0x%.2x", REG_INPUT_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG15_MPPT, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "MPPT status (0x%.2x): 0x%.2x", REG15_MPPT, (uint8_t)value);
  }
  ESP_LOGI(TAG, "*********");
}

void BQ25672::printControlAndConfiguration() {
  ESP_LOGI(TAG, "==== CONTROL and CONFIGURATION REGISTERS ====");
  uint16_t value;
  if (writeReadRegister(REG00_MINIMAL_SYSTEM_VOLTAGE, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Minimal System Voltage Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG00_MINIMAL_SYSTEM_VOLTAGE, value, ((value * 250) + 2500));
  }
  if (writeReadRegister(REG01_CHARGE_VOLTAGE_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Voltage Limit Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG01_CHARGE_VOLTAGE_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG03_CHARGE_CURRENT_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Current Limit Raw (0x%.2X): 0x%04X | Converted: %d mA",
             REG03_CHARGE_CURRENT_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG05_INPUT_VOLTAGE_LIMIT, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input Voltage Limit Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG05_INPUT_VOLTAGE_LIMIT, value, (value * 100));
  }
  if (writeReadRegister(REG06_INPUT_CURRENT_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input Current Limit Raw (0x%.2X): 0x%04X | Converted: %d mA",
             REG06_INPUT_CURRENT_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG2E_ADC_CTRL, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "ADC Control (0x%.2X): 0x%02X", REG2E_ADC_CTRL, value);
  }
  ESP_LOGI(TAG, "*********");
}

esp_err_t BQ25672::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t txBuf[2] = {reg, value};
  ESP_RETURN_ON_ERROR(i2c_master_write_to_device(_port, BQ25672_I2C_ADDRESS, txBuf,
                                                 sizeof(txBuf), pdMS_TO_TICKS(500)),
                      TAG,
                      "i2c_master_write_to_device, write register failed (r: 0x%.2x, v:0x%.2x)",
                      reg, value);
  return ESP_OK;
}

esp_err_t BQ25672::writeRegister16(uint8_t reg, uint16_t value) {
  uint8_t txBuf[3] = {reg, static_cast<uint8_t>(value >> 8),
                      static_cast<uint8_t>(value & 0xFF)};
  ESP_RETURN_ON_ERROR(
      i2c_master_write_to_device(_port, BQ25672_I2C_ADDRESS, txBuf, sizeof(txBuf),
                                 pdMS_TO_TICKS(500)),
      TAG, "i2c_master_write_to_device, write register16 failed (r: 0x%.2x, v:0x%.4x)",
      reg, value);
  return ESP_OK;
}

esp_err_t BQ25672::writeReadRegister(uint8_t reg, int size, uint16_t *output) {
  uint8_t buffer[2] = {0};
  ESP_RETURN_ON_ERROR(
      i2c_master_write_read_device(_port, BQ25672_I2C_ADDRESS, &reg, 1, buffer, size,
                                   pdMS_TO_TICKS(500)),
      TAG,
      "i2c_master_write_read_device, write and read register failed (r: 0x%.2x)", reg);

  // Compile result
  uint16_t value = 0;
  for (int i = 0; i < size; i++) {
    value = (value << 8) | buffer[i];
  }
  *output = value;

  return ESP_OK;
}
