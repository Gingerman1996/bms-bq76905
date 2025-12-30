#include <stdio.h>
#include "bq76905.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMS_APP";

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0,
    };

    i2c_param_config((i2c_port_t)i2c_master_port, &conf);

    return i2c_driver_install((i2c_port_t)i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void setup_li_ion(BQ76905 &bms) {
    ESP_LOGI(TAG, "Configuring for Li-Ion (3.7V nominal, 4.2V max)");
    // Li-Ion: OV=4.25V, UV=2.8V
    // Register values depend on datasheet scaling. 
    // Assuming OV_TRIP step is approx 50mV/LSB and UV is similar (check datasheet for exact scale)
    // Placeholder values: 
    // OV 4.25V -> 0xAC (example)
    // UV 2.8V -> 0x97 (example)
    // PROTECT1: SCD/OCD (Short Circuit / Over Current)
    
    bms.setProtections(0xAC, 0x97, 0x05, 0x05, 0x00);
}

void setup_lfp(BQ76905 &bms) {
    ESP_LOGI(TAG, "Configuring for LiFePO4 (LFP) (3.2V nominal, 3.65V max)");
    // LFP: OV=3.7V, UV=2.5V
    // Placeholder values:
    // OV 3.7V -> 0x8C (example)
    // UV 2.5V -> 0x80 (example)
    
    bms.setProtections(0x8C, 0x80, 0x05, 0x05, 0x00);
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    BQ76905 bms((i2c_port_t)I2C_MASTER_NUM);

    if (bms.begin() == ESP_OK) {
        ESP_LOGI(TAG, "BMS initialized successfully");
        
        // Example usage:
        // Uncomment the chemistry you want to test
        setup_li_ion(bms);
        // setup_lfp(bms);
        
        uint16_t pack_voltage = 0;
        if (bms.getPackVoltage(pack_voltage) == ESP_OK) {
            ESP_LOGI(TAG, "Pack Voltage: %d mV", pack_voltage);
        } else {
            ESP_LOGE(TAG, "Failed to read pack voltage");
        }

        int16_t current_ma = 0;
        if (bms.getCurrent(current_ma) == ESP_OK) {
            ESP_LOGI(TAG, "Current: %d mA", current_ma);
        } else {
            ESP_LOGE(TAG, "Failed to read current");
        }

        float temp_c = 0.0f;
        if (bms.getTemperature(temp_c) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C", temp_c);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature");
        }

        // Read first 5 cells (assuming 5S for this test, though chip supports up to 15)
        for (int i = 1; i <= 5; i++) {
            uint16_t cell_vol = 0;
            if (bms.getCellVoltage(i, cell_vol) == ESP_OK) {
                ESP_LOGI(TAG, "Cell %d Voltage: %d mV", i, cell_vol);
            }
        }

        // Enable balancing for Cell 1 and 2
        // bms.enableBalancing(0x0003); 

    } else {
        ESP_LOGE(TAG, "BMS initialization failed");
    }

    while (1) {
        // Repeatedly read status every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        uint16_t pack_voltage = 0;
        bms.getPackVoltage(pack_voltage);
        ESP_LOGI(TAG, "Pack: %d mV", pack_voltage);
    }
}
