#include "bq76905.h"
#include "freertos/FreeRTOS.h"

BQ76905::BQ76905(i2c_port_t port, uint8_t addr) 
    : _port(port), _addr(addr), _timeout_ms(1000), _adc_offset(0), _adc_gain(0) {
}

uint32_t BQ76905::_timeoutTicks() const {
    return pdMS_TO_TICKS(_timeout_ms);
}

void BQ76905::setTimeout(uint32_t timeout_ms) {
    if (timeout_ms > 0) {
        _timeout_ms = timeout_ms;
    }
}

esp_err_t BQ76905::readReg(Reg reg, uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t reg_addr = static_cast<uint8_t>(reg);
    return i2c_master_write_read_device(_port, _addr, &reg_addr, 1, data, len, _timeoutTicks());
}

esp_err_t BQ76905::writeReg(Reg reg, const uint8_t *data, size_t len) {
    if (!data && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(reg), true);
    if (len > 0) {
        i2c_master_write(cmd, (uint8_t *)data, len, true);
    }
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, _timeoutTicks());
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t BQ76905::readU8(Reg reg, uint8_t &value) {
    return readReg(reg, &value, 1);
}

esp_err_t BQ76905::writeU8(Reg reg, uint8_t value) {
    return writeReg(reg, &value, 1);
}

esp_err_t BQ76905::readU16LE(Reg reg, uint16_t &value) {
    uint8_t data[2] = {0};
    esp_err_t err = readReg(reg, data, 2);
    if (err == ESP_OK) {
        value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    }
    return err;
}

esp_err_t BQ76905::writeU16LE(Reg reg, uint16_t value) {
    uint8_t data[2] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
    };
    return writeReg(reg, data, 2);
}

esp_err_t BQ76905::begin() {
    uint8_t gain1 = 0, gain2 = 0;
    
    esp_err_t err = readU8(Reg::ADCGAIN1, gain1);
    if (err != ESP_OK) return err;
    err = readU8(Reg::ADCGAIN2, gain2);
    if (err != ESP_OK) return err;
    uint8_t offset_u8 = 0;
    err = readU8(Reg::ADCOFFSET, offset_u8);
    if (err != ESP_OK) return err;
    
    _adc_offset = (int8_t)offset_u8;
    
    // Gain calculation from earlier
    uint8_t gain_val = ((gain1 & 0x0C) << 1) | ((gain2 & 0xE0) >> 5);
    _adc_gain = 365 + gain_val;

    return ESP_OK;
}

esp_err_t BQ76905::getCellVoltage(uint8_t cell_idx, uint16_t &voltage_mv) {
    if (cell_idx < 1 || cell_idx > 15) return ESP_ERR_INVALID_ARG;
    
    // VC1 starts at 0x0C
    uint8_t base_reg_val = static_cast<uint8_t>(Reg::VC1_HI) + (cell_idx - 1) * 2;
    Reg base_reg = static_cast<Reg>(base_reg_val);
    
    uint16_t adc_val = 0;
    esp_err_t err = readU16LE(base_reg, adc_val);
    if (err != ESP_OK) return err;

    int32_t uv = (int32_t)adc_val * _adc_gain + (int32_t)_adc_offset * 1000;
    voltage_mv = (uint16_t)(uv / 1000);
    return ESP_OK;
}

esp_err_t BQ76905::getPackVoltage(uint16_t &voltage_mv) {
    uint16_t adc_val = 0;
    esp_err_t err = readU16LE(Reg::BAT_HI, adc_val);
    if (err != ESP_OK) return err;

    int32_t uv = 4 * ((int32_t)adc_val * _adc_gain + (int32_t)_adc_offset * 1000);
    voltage_mv = (uint16_t)(uv / 1000);
    return ESP_OK;
}

esp_err_t BQ76905::getCurrent(int16_t &current_ma) {
    uint16_t adc_raw = 0;
    esp_err_t err = readU16LE(Reg::CC_HI, adc_raw);
    if (err != ESP_OK) return err;

    int16_t adc_signed = (int16_t)adc_raw;
    const int32_t SHUNT_RESISTOR_MOHM = 1; 
    
    int32_t current = (int32_t)adc_signed * 844 / (100 * SHUNT_RESISTOR_MOHM);
    current_ma = (int16_t)current;
    return ESP_OK;
}

esp_err_t BQ76905::getTemperature(float &temp_c) {
    uint16_t adc_val = 0;
    esp_err_t err = readU16LE(Reg::TS1_HI, adc_val);
    if (err != ESP_OK) return err;
    
    // Placeholder
    temp_c = 25.0f;
    return ESP_OK;
}

esp_err_t BQ76905::setProtections(uint8_t ov_trip, uint8_t uv_trip, uint8_t protect1, uint8_t protect2, uint8_t protect3) {
    esp_err_t err;
    err = writeU8(Reg::OV_TRIP, ov_trip);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::UV_TRIP, uv_trip);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::PROTECT1, protect1);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::PROTECT2, protect2);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::PROTECT3, protect3);
    return err;
}

esp_err_t BQ76905::enableBalancing(uint16_t cells_mask) {
    uint8_t cb1 = (cells_mask & 0x001F);
    uint8_t cb2 = (cells_mask & 0x03E0) >> 5;
    uint8_t cb3 = (cells_mask & 0x7C00) >> 10;
    
    esp_err_t err;
    err = writeU8(Reg::CELLBAL1, cb1);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::CELLBAL2, cb2);
    if (err != ESP_OK) return err;
    err = writeU8(Reg::CELLBAL3, cb3);
    return err;
}
