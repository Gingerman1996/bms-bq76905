#include "bq76905.h"

#include <cmath>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char *TAG_BQ = "BQ76905";

// NTC Thermistor constants for temperature calculation
static constexpr float NTC_R_PU = 20000.0f;      // Internal Pull-up 20k Ohm
static constexpr float NTC_R_25 = 10000.0f;      // NTC resistance at 25C = 10k Ohm
static constexpr float NTC_B_CONSTANT = 3950.0f; // B-constant of NTC
static constexpr float NTC_T_25_KELVIN = 298.15f; // 25 degrees Celsius in Kelvin

BQ76905::BQ76905(i2c_port_t port, uint8_t addr)
    : _port(port), _addr(addr), _timeout_ms(1000) {
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
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

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
    esp_err_t err = readReg(reg, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return ESP_OK;
}

esp_err_t BQ76905::writeU16LE(Reg reg, uint16_t value) {
    uint8_t data[2] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
    };
    return writeReg(reg, data, sizeof(data));
}

esp_err_t BQ76905::begin() {
    uint8_t status = 0;
    return readU8(Reg::ControlStatus, status);
}

esp_err_t BQ76905::getCellVoltage(uint8_t cell_idx, uint16_t &voltage_mv) {
    if (cell_idx < 1 || cell_idx > kMaxCells) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t base_reg_val = static_cast<uint8_t>(Reg::Cell1Voltage) + (cell_idx - 1) * 2;
    uint16_t raw = 0;
    esp_err_t err = readU16LE(static_cast<Reg>(base_reg_val), raw);
    if (err != ESP_OK) {
        return err;
    }

    voltage_mv = raw;
    return ESP_OK;
}

esp_err_t BQ76905::getPackVoltage(uint16_t &voltage_mv) {
    return readU16LE(Reg::StackVoltage, voltage_mv);
}

esp_err_t BQ76905::getCurrent(int16_t &current_usera) {
    uint16_t raw = 0;
    esp_err_t err = readU16LE(Reg::Current, raw);
    if (err != ESP_OK) {
        return err;
    }

    current_usera = static_cast<int16_t>(raw);
    return ESP_OK;
}

esp_err_t BQ76905::getTemperature(float &temp_c) {
    // Read internal temperature register (0x28)
    // This gives raw internal die temperature
    uint16_t raw = 0;
    esp_err_t err = readU16LE(Reg::IntTemperature, raw);
    if (err != ESP_OK) {
        return err;
    }

    // Internal temperature is in 0.1 Kelvin units
    // Convert to Celsius: (raw * 0.1) - 273.15
    temp_c = (static_cast<float>(raw) * 0.1f) - 273.15f;
    return ESP_OK;
}

esp_err_t BQ76905::getTSTemperature(float &temp_c) {
    // Read TS Measurement register (0x2A) for external NTC thermistor
    uint16_t adc_counts = 0;
    esp_err_t err = readU16LE(Reg::TsMeasurement, adc_counts);
    if (err != ESP_OK) {
        return err;
    }
    
    ESP_LOGI(TAG_BQ, "[TS] Raw ADC counts: %u", adc_counts);

    // Calculate voltage ratio
    // Full-scale (32768) = V_REF * (5/3)
    // V_ratio = (adc_counts * 5/3) / 32768
    float v_ratio = (static_cast<float>(adc_counts) * (5.0f / 3.0f)) / 32768.0f;
    
    // Prevent division by zero
    if (v_ratio >= 1.0f) {
        ESP_LOGW(TAG_BQ, "[TS] Invalid v_ratio: %.4f (NTC may be disconnected)", v_ratio);
        temp_c = -999.0f;  // Error value
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate NTC resistance using voltage divider formula
    // V_ts = V_ref * (R_ntc / (R_ntc + R_pu))
    // R_ntc = (v_ratio * R_pu) / (1 - v_ratio)
    float r_ntc = (v_ratio * NTC_R_PU) / (1.0f - v_ratio);
    
    ESP_LOGI(TAG_BQ, "[TS] V_ratio: %.4f, R_ntc: %.1f Ohm", v_ratio, r_ntc);

    // Calculate temperature using B-constant equation (Steinhart-Hart)
    // 1/T = 1/T25 + (1/B) * ln(R_ntc / R25)
    float inv_t = (1.0f / NTC_T_25_KELVIN) + (1.0f / NTC_B_CONSTANT) * logf(r_ntc / NTC_R_25);
    float t_kelvin = 1.0f / inv_t;

    // Convert from Kelvin to Celsius
    temp_c = t_kelvin - 273.15f;
    
    ESP_LOGI(TAG_BQ, "[TS] Calculated temperature: %.2f C", temp_c);
    return ESP_OK;
}

esp_err_t BQ76905::getRawCurrent(int32_t &raw_current) {
    uint8_t data[3] = {0};
    esp_err_t err = readReg(Reg::RawCurrent, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    int32_t value = (int32_t)data[0] | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16);
    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }

    raw_current = value;
    return ESP_OK;
}

esp_err_t BQ76905::clearAlarmStatus(uint8_t mask) {
    return writeU8(Reg::AlarmStatus, mask);
}

// ========== Subcommand Access Functions ==========

esp_err_t BQ76905::sendSubcommand(SubCmd cmd) {
    // Subcommand is sent by writing 16-bit value to register 0x3E
    uint16_t cmd_val = static_cast<uint16_t>(cmd);
    uint8_t data[2] = {
        (uint8_t)(cmd_val & 0xFF),         // LSB
        (uint8_t)((cmd_val >> 8) & 0xFF),  // MSB
    };
    
    ESP_LOGI(TAG_BQ, "[SUBCMD] Sending subcommand 0x%04X to register 0x3E", cmd_val);
    ESP_LOGI(TAG_BQ, "  -> Data: [0x%02X, 0x%02X]", data[0], data[1]);
    
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    if (!i2c_cmd) {
        ESP_LOGE(TAG_BQ, "  -> FAILED: Could not create I2C command");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, 0x3E, true);  // Subcommand register address
    i2c_master_write(i2c_cmd, data, 2, true);
    i2c_master_stop(i2c_cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, i2c_cmd, _timeoutTicks());
    i2c_cmd_link_delete(i2c_cmd);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG_BQ, "  -> SUCCESS");
    } else {
        ESP_LOGE(TAG_BQ, "  -> FAILED: %s (0x%x)", esp_err_to_name(err), err);
    }
    return err;
}

esp_err_t BQ76905::writeSubcommandData(uint16_t address, const uint8_t *data, size_t len) {
    // 1. Write address to 0x3E (LSB) and 0x3F (MSB)
    // 2. Write data to Transfer Buffer starting at 0x40
    // 3. Calculate and write checksum to 0x60
    // 4. Write length to 0x61
    
    ESP_LOGI(TAG_BQ, "[DATA_MEM] Writing to Data Memory address 0x%04X, len=%d", address, (int)len);
    
    if (len > 32 || !data) {
        ESP_LOGE(TAG_BQ, "  -> FAILED: Invalid argument (len=%d, data=%p)", (int)len, data);
        return ESP_ERR_INVALID_ARG;
    }

    // Build the complete write buffer
    uint8_t buffer[36] = {0};  // Address(2) + Data(max 32) + Checksum(1) + Length(1)
    buffer[0] = (uint8_t)(address & 0xFF);         // Address LSB -> 0x3E
    buffer[1] = (uint8_t)((address >> 8) & 0xFF);  // Address MSB -> 0x3F
    
    // Copy data to buffer (will go to 0x40+)
    for (size_t i = 0; i < len; i++) {
        buffer[2 + i] = data[i];
    }
    
    // Calculate checksum: sum of (address bytes + data bytes) then ~sum & 0xFF
    uint32_t sum = buffer[0] + buffer[1];
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    uint8_t checksum = ~((uint8_t)(sum & 0xFF));
    
    ESP_LOGI(TAG_BQ, "  -> Step 1: Write to 0x3E: addr=[0x%02X, 0x%02X], data=[0x%02X, 0x%02X]",
             buffer[0], buffer[1], 
             len >= 1 ? data[0] : 0, 
             len >= 2 ? data[1] : 0);
    
    // Write address + data in one transaction
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    if (!i2c_cmd) {
        ESP_LOGE(TAG_BQ, "  -> FAILED: Could not create I2C command");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, 0x3E, true);  // Start at register 0x3E
    i2c_master_write(i2c_cmd, buffer, 2 + len, true);  // Address + data
    i2c_master_stop(i2c_cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, i2c_cmd, _timeoutTicks());
    i2c_cmd_link_delete(i2c_cmd);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "  -> Step 1 FAILED: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG_BQ, "  -> Step 1 SUCCESS");
    
    // Write checksum and length
    ESP_LOGI(TAG_BQ, "  -> Step 2: Write to 0x60: checksum=0x%02X, length=0x%02X", checksum, (uint8_t)(len + 4));
    
    i2c_cmd = i2c_cmd_link_create();
    if (!i2c_cmd) {
        ESP_LOGE(TAG_BQ, "  -> FAILED: Could not create I2C command");
        return ESP_ERR_NO_MEM;
    }

    uint8_t checksum_len[2] = {checksum, (uint8_t)(len + 4)};  // Length includes address(2) + data + checksum + len bytes
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, 0x60, true);  // Checksum register
    i2c_master_write(i2c_cmd, checksum_len, 2, true);
    i2c_master_stop(i2c_cmd);

    err = i2c_master_cmd_begin(_port, i2c_cmd, _timeoutTicks());
    i2c_cmd_link_delete(i2c_cmd);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG_BQ, "  -> Step 2 SUCCESS");
    } else {
        ESP_LOGE(TAG_BQ, "  -> Step 2 FAILED: %s (0x%x)", esp_err_to_name(err), err);
    }
    return err;
}

// ========== Battery Configuration Functions ==========

esp_err_t BQ76905::setConfigUpdateMode(bool enable) {
    ESP_LOGI(TAG_BQ, "========================================");
    if (enable) {
        ESP_LOGI(TAG_BQ, "[CONFIG] Entering CONFIG_UPDATE mode...");
        // Send SET_CFGUPDATE (0x0090) command
        return sendSubcommand(SubCmd::SET_CFGUPDATE);
    } else {
        ESP_LOGI(TAG_BQ, "[CONFIG] Exiting CONFIG_UPDATE mode...");
        // Send EXIT_CFGUPDATE (0x0092) command
        return sendSubcommand(SubCmd::EXIT_CFGUPDATE);
    }
}

esp_err_t BQ76905::writeDataMemory(uint16_t address, uint16_t value) {
    ESP_LOGI(TAG_BQ, "[DATA_MEM] writeDataMemory(addr=0x%04X, value=%u / 0x%04X)", address, value, value);
    uint8_t data[2] = {
        (uint8_t)(value & 0xFF),         // LSB
        (uint8_t)((value >> 8) & 0xFF),  // MSB
    };
    return writeSubcommandData(address, data, 2);
}

esp_err_t BQ76905::setupBattery(BatteryType type) {
    esp_err_t err;
    
    ESP_LOGI(TAG_BQ, "########################################");
    ESP_LOGI(TAG_BQ, "# BATTERY SETUP START");
    ESP_LOGI(TAG_BQ, "# Type: %s", type == BatteryType::LFP ? "LFP (LiFePO4)" : "Li-ion (NMC/LCO)");
    ESP_LOGI(TAG_BQ, "########################################");
    
    // Enter CONFIG_UPDATE mode
    err = setConfigUpdateMode(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to enter CONFIG_UPDATE mode!");
        return err;
    }
    
    // Wait for mode transition (typically 2ms is enough)
    ESP_LOGI(TAG_BQ, "Waiting 10ms for mode transition...");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t cov_threshold;  // Cell Overvoltage threshold in mV
    uint16_t cuv_threshold;  // Cell Undervoltage threshold in mV

    switch (type) {
        case BatteryType::LFP:
            // LFP (LiFePO4) Model 18650Fe1600-WT
            // - Charging cut-off Voltage: 3.65V
            // - Discharging cut-off Voltage: 2.0V
            cov_threshold = 3650;  // 3650 mV
            cuv_threshold = 2000;  // 2000 mV
            ESP_LOGI(TAG_BQ, "LFP settings: COV=%u mV, CUV=%u mV", cov_threshold, cuv_threshold);
            break;
            
        case BatteryType::LiIon:
        default:
            // Li-ion (NMC/LCO)
            // - Charging cut-off Voltage: 4.2V
            // - Discharging cut-off Voltage: 3.0V
            cov_threshold = 4200;  // 4200 mV
            cuv_threshold = 3000;  // 3000 mV
            ESP_LOGI(TAG_BQ, "Li-ion settings: COV=%u mV, CUV=%u mV", cov_threshold, cuv_threshold);
            break;
    }

    // Write COV threshold
    ESP_LOGI(TAG_BQ, "----------------------------------------");
    ESP_LOGI(TAG_BQ, "Writing COV Threshold (0x%04X) = %u mV", ADDR_COV_THRESHOLD, cov_threshold);
    err = writeDataMemory(ADDR_COV_THRESHOLD, cov_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write COV threshold!");
        setConfigUpdateMode(false);  // Try to exit config mode even on error
        return err;
    }
    ESP_LOGI(TAG_BQ, "COV Threshold write: SUCCESS");
    
    vTaskDelay(pdMS_TO_TICKS(5));

    // Write CUV threshold
    ESP_LOGI(TAG_BQ, "----------------------------------------");
    ESP_LOGI(TAG_BQ, "Writing CUV Threshold (0x%04X) = %u mV", ADDR_CUV_THRESHOLD, cuv_threshold);
    err = writeDataMemory(ADDR_CUV_THRESHOLD, cuv_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write CUV threshold!");
        setConfigUpdateMode(false);  // Try to exit config mode even on error
        return err;
    }
    ESP_LOGI(TAG_BQ, "CUV Threshold write: SUCCESS");
    
    vTaskDelay(pdMS_TO_TICKS(5));

    // Exit CONFIG_UPDATE mode
    err = setConfigUpdateMode(false);
    
    // Wait for mode transition
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG_BQ, "########################################");
    if (err == ESP_OK) {
        ESP_LOGI(TAG_BQ, "# BATTERY SETUP COMPLETE: SUCCESS");
    } else {
        ESP_LOGE(TAG_BQ, "# BATTERY SETUP COMPLETE: FAILED");
    }
    ESP_LOGI(TAG_BQ, "########################################");
    
    return err;
}

esp_err_t BQ76905::writeDataMemory8(uint16_t address, uint8_t value) {
    ESP_LOGI(TAG_BQ, "[DATA_MEM] writeDataMemory8(addr=0x%04X, value=%u / 0x%02X)", address, value, value);
    return writeSubcommandData(address, &value, 1);
}

esp_err_t BQ76905::fullConfiguration(BatteryType type, uint8_t cell_count) {
    esp_err_t err;
    
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "################################################################");
    ESP_LOGI(TAG_BQ, "# FULL BMS CONFIGURATION START");
    ESP_LOGI(TAG_BQ, "# Battery Type: %s", type == BatteryType::LFP ? "LFP (LiFePO4)" : "Li-ion (NMC/LCO)");
    ESP_LOGI(TAG_BQ, "# Cell Count: %u", cell_count);
    ESP_LOGI(TAG_BQ, "################################################################");
    
    // ========================================
    // 1. Enter CONFIG_UPDATE mode
    // ========================================
    err = setConfigUpdateMode(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to enter CONFIG_UPDATE mode!");
        return err;
    }
    ESP_LOGI(TAG_BQ, "Waiting 10ms for mode transition...");
    vTaskDelay(pdMS_TO_TICKS(10));

    // ========================================
    // 2. System Configuration (DA Config)
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== SYSTEM CONFIGURATION ==========");
    
    // DA Config: TSMODE = 0 (Thermistor mode), ADC speed settings
    ESP_LOGI(TAG_BQ, "Setting DA_CONFIG (TSMODE=Thermistor)...");
    err = writeDataMemory8(ADDR_DA_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write DA_CONFIG!");
        setConfigUpdateMode(false);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Vcell Mode: Set cell count
    ESP_LOGI(TAG_BQ, "Setting VCELL_MODE (cells=%u)...", cell_count);
    uint8_t vcell_mode = (cell_count == 5) ? 0x00 : (cell_count == 4) ? 0x01 : (cell_count == 3) ? 0x02 : 0x00;
    err = writeDataMemory8(ADDR_VCELL_MODE, vcell_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write VCELL_MODE!");
        setConfigUpdateMode(false);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 3. FET Options - Enable autonomous FET control
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== FET CONFIGURATION ==========");
    // Bit0: FET_EN = 1 (Enable autonomous FET control)
    // Bit2: SFET = 1 (Series FET configuration)
    // Bit3: CHGFET = 1 (CHG FET present)
    // Value: 0x0D = 0b00001101
    ESP_LOGI(TAG_BQ, "Setting FET_OPTIONS (FET_EN=1, SFET=1, CHGFET=1)...");
    err = writeDataMemory8(ADDR_FET_OPTIONS, 0x0D);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write FET_OPTIONS!");
        setConfigUpdateMode(false);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 4. Voltage Protection (COV, CUV)
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== VOLTAGE PROTECTION ==========");
    
    uint16_t cov_threshold, cuv_threshold;
    
    if (type == BatteryType::LFP) {
        // LFP (LiFePO4) - Model 18650Fe1600-WT
        cov_threshold = 3650;  // 3.65V
        cuv_threshold = 2000;  // 2.0V
    } else {
        // Li-ion (NMC/LCO)
        cov_threshold = 4200;  // 4.2V
        cuv_threshold = 3000;  // 3.0V
    }
    
    ESP_LOGI(TAG_BQ, "COV Threshold: %u mV, CUV Threshold: %u mV", cov_threshold, cuv_threshold);
    
    err = writeDataMemory(ADDR_COV_THRESHOLD, cov_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write COV_THRESHOLD!");
        setConfigUpdateMode(false);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory(ADDR_CUV_THRESHOLD, cuv_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BQ, "FAILED to write CUV_THRESHOLD!");
        setConfigUpdateMode(false);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 5. Current Protection (OCC, OCD, SCD)
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== CURRENT PROTECTION ==========");
    
    uint8_t occ_thresh, ocd1_thresh, ocd2_thresh, scd_thresh;
    
    if (type == BatteryType::LFP) {
        // LFP: Max Charge 4.8A, Max Discharge 8.0A
        occ_thresh = 3;   // ~5mV threshold (OCC)
        ocd1_thresh = 4;  // ~8mV threshold (OCD1 - first level)
        ocd2_thresh = 6;  // ~12mV threshold (OCD2 - second level)
        scd_thresh = 10;  // ~20mV threshold (SCD - short circuit)
    } else {
        // Li-ion: More conservative
        occ_thresh = 2;   // ~4mV threshold
        ocd1_thresh = 3;  // ~6mV threshold
        ocd2_thresh = 5;  // ~10mV threshold
        scd_thresh = 8;   // ~16mV threshold
    }
    
    ESP_LOGI(TAG_BQ, "OCC=%u, OCD1=%u, OCD2=%u, SCD=%u", occ_thresh, ocd1_thresh, ocd2_thresh, scd_thresh);
    
    err = writeDataMemory8(ADDR_OCC_THRESHOLD, occ_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory8(ADDR_OCD1_THRESHOLD, ocd1_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory8(ADDR_OCD2_THRESHOLD, ocd2_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory8(ADDR_SCD_THRESHOLD, scd_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 6. Temperature Protection (OTC, OTD, UTC, UTD)
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== TEMPERATURE PROTECTION ==========");
    
    // ADC counts for temperature (approximate values for 10k NTC with 20k pull-up)
    // Higher counts = lower temperature, Lower counts = higher temperature
    uint16_t otc_thresh, otd_thresh, utc_thresh, utd_thresh;
    
    if (type == BatteryType::LFP) {
        // LFP: Charge 0-60°C, Discharge -40°C to 60°C
        otc_thresh = 2476;   // ~60°C for charge overtemp
        otd_thresh = 2476;   // ~60°C for discharge overtemp
        utc_thresh = 19284;  // ~0°C for charge undertemp
        utd_thresh = 26000;  // ~-40°C for discharge undertemp (LFP can handle extreme cold)
    } else {
        // Li-ion: More restrictive temperature range
        // Charge 0-45°C, Discharge -20°C to 60°C
        otc_thresh = 3200;   // ~45°C for charge overtemp
        otd_thresh = 2476;   // ~60°C for discharge overtemp
        utc_thresh = 19284;  // ~0°C for charge undertemp
        utd_thresh = 22000;  // ~-20°C for discharge undertemp
    }
    
    ESP_LOGI(TAG_BQ, "OTC=%u, OTD=%u, UTC=%u, UTD=%u (ADC counts)", otc_thresh, otd_thresh, utc_thresh, utd_thresh);
    
    err = writeDataMemory(ADDR_OTC_THRESHOLD, otc_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory(ADDR_OTD_THRESHOLD, otd_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory(ADDR_UTC_THRESHOLD, utc_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    err = writeDataMemory(ADDR_UTD_THRESHOLD, utd_thresh);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 7. Enable Protections
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    ESP_LOGI(TAG_BQ, "========== ENABLE PROTECTIONS ==========");
    
    // Enable Protections A: 0xFC
    // Bit7: COV_EN = 1
    // Bit6: CUV_EN = 1
    // Bit5: SCD_EN = 1
    // Bit4: OCD1_EN = 1
    // Bit3: OCD2_EN = 1
    // Bit2: OCC_EN = 1
    // = 0b11111100 = 0xFC
    ESP_LOGI(TAG_BQ, "ENABLE_PROT_A = 0xFC (COV, CUV, SCD, OCD1, OCD2, OCC)");
    err = writeDataMemory8(ADDR_ENABLE_PROT_A, 0xFC);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Enable Protections B: 0x3C
    // Bit5: OTD_EN = 1
    // Bit4: OTC_EN = 1
    // Bit3: UTD_EN = 1
    // Bit2: UTC_EN = 1
    // = 0b00111100 = 0x3C
    ESP_LOGI(TAG_BQ, "ENABLE_PROT_B = 0x3C (OTD, OTC, UTD, UTC)");
    err = writeDataMemory8(ADDR_ENABLE_PROT_B, 0x3C);
    if (err != ESP_OK) { setConfigUpdateMode(false); return err; }
    vTaskDelay(pdMS_TO_TICKS(5));

    // ========================================
    // 8. Exit CONFIG_UPDATE mode
    // ========================================
    ESP_LOGI(TAG_BQ, "");
    err = setConfigUpdateMode(false);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG_BQ, "################################################################");
    if (err == ESP_OK) {
        ESP_LOGI(TAG_BQ, "# FULL CONFIGURATION COMPLETE: SUCCESS");
    } else {
        ESP_LOGE(TAG_BQ, "# FULL CONFIGURATION COMPLETE: FAILED");
    }
    ESP_LOGI(TAG_BQ, "################################################################");
    
    return err;
}
