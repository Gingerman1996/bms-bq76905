#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c.h"
#include "esp_err.h"

class BQ76905 {
public:
    // Registers
    enum class Reg : uint8_t {
        SYS_STAT    = 0x00,
        CELLBAL1    = 0x01,
        CELLBAL2    = 0x02,
        CELLBAL3    = 0x03,
        SYS_CTRL1   = 0x04,
        SYS_CTRL2   = 0x05,
        PROTECT1    = 0x06,
        PROTECT2    = 0x07,
        PROTECT3    = 0x08,
        OV_TRIP     = 0x09,
        UV_TRIP     = 0x0A,
        CC_CFG      = 0x0B,
        VC1_HI      = 0x0C,
        VC1_LO      = 0x0D,
        VC2_HI      = 0x0E,
        VC2_LO      = 0x0F,
        VC3_HI      = 0x10,
        VC3_LO      = 0x11,
        VC4_HI      = 0x12,
        VC4_LO      = 0x13,
        VC5_HI      = 0x14,
        VC5_LO      = 0x15,
        VC6_HI      = 0x16,
        VC6_LO      = 0x17,
        VC7_HI      = 0x18,
        VC7_LO      = 0x19,
        VC8_HI      = 0x1A,
        VC8_LO      = 0x1B,
        VC9_HI      = 0x1C,
        VC9_LO      = 0x1D,
        VC10_HI     = 0x1E,
        VC10_LO     = 0x1F,
        VC11_HI     = 0x20,
        VC11_LO     = 0x21,
        VC12_HI     = 0x22,
        VC12_LO     = 0x23,
        VC13_HI     = 0x24,
        VC13_LO     = 0x25,
        VC14_HI     = 0x26,
        VC14_LO     = 0x27,
        VC15_HI     = 0x28,
        VC15_LO     = 0x29,
        BAT_HI      = 0x2A,
        BAT_LO      = 0x2B,
        TS1_HI      = 0x2C,
        TS1_LO      = 0x2D,
        TS2_HI      = 0x2E,
        TS2_LO      = 0x2F,
        TS3_HI      = 0x30,
        TS3_LO      = 0x31,
        CC_HI       = 0x32,
        CC_LO       = 0x33,
        ADCGAIN1    = 0x50,
        ADCGAIN2    = 0x59,
        ADCOFFSET   = 0x51,
    };

    BQ76905(i2c_port_t port, uint8_t addr = 0x08);

    // Initialize the device and read calibration data
    esp_err_t begin();

    void setTimeout(uint32_t timeout_ms);

    // High-level BMS functions
    /**
     * @brief Read voltage of a specific cell (1-15)
     * @param cell_idx Cell index (1-based)
     * @param voltage_mv Reference to store voltage in mV
     * @return ESP_OK on success
     */
    esp_err_t getCellVoltage(uint8_t cell_idx, uint16_t &voltage_mv);

    /**
     * @brief Read total pack voltage
     * @param voltage_mv Reference to store voltage in mV
     * @return ESP_OK on success
     */
    esp_err_t getPackVoltage(uint16_t &voltage_mv);

    /**
     * @brief Read current
     * @param current_ma Reference to store current in mA
     * @return ESP_OK on success
     */
    esp_err_t getCurrent(int16_t &current_ma);

    /**
     * @brief Read die temperature
     * @param temp_c Reference to store temperature in degrees Celsius
     * @return ESP_OK on success
     */
    esp_err_t getTemperature(float &temp_c);

    /**
     * @brief Configure protections (OV, UV, OCD, SCD)
     * @param ov_trip OV threshold register value
     * @param uv_trip UV threshold register value
     * @param protect1 PROTECT1 register value (SCD/OCD settings)
     * @param protect2 PROTECT2 register value (OCD settings)
     * @param protect3 PROTECT3 register value (UV delay)
     * @return ESP_OK on success
     */
    esp_err_t setProtections(uint8_t ov_trip, uint8_t uv_trip, uint8_t protect1, uint8_t protect2, uint8_t protect3);

    /**
     * @brief Enable cell balancing for specific cells
     * @param cells_mask Bitmask of cells to balance (Bit 0 = Cell 1, Bit 14 = Cell 15)
     * @return ESP_OK on success
     */
    esp_err_t enableBalancing(uint16_t cells_mask);

    // Low level access if needed
    esp_err_t readReg(Reg reg, uint8_t *data, size_t len);
    esp_err_t writeReg(Reg reg, const uint8_t *data, size_t len);
    esp_err_t readU8(Reg reg, uint8_t &value);
    esp_err_t writeU8(Reg reg, uint8_t value);
    esp_err_t readU16LE(Reg reg, uint16_t &value);
    esp_err_t writeU16LE(Reg reg, uint16_t value);

private:
    i2c_port_t _port;
    uint8_t _addr;
    uint32_t _timeout_ms;
    
    // Calibration data
    int8_t _adc_offset;
    uint16_t _adc_gain;

    uint32_t _timeoutTicks() const;
};
