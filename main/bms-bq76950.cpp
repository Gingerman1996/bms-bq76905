#include "BQ25672.h"
#include "bq76905.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "driver/gpio.h"

static const char *TAG = "BMS_APP";

// ============================================================
// BATTERY TYPE CONFIGURATION
// ============================================================
// Change this value to select battery type:
//   0 = LFP (LiFePO4)  - COV: 3.65V, CUV: 2.0V
//   1 = Li-ion (NMC/LCO) - COV: 4.2V, CUV: 3.0V
// ============================================================
#define BATTERY_TYPE 0 // <-- Change here: 0=LFP, 1=Li-ion
// ============================================================

// ============================================================
// BATTERY CELL COUNT CONFIGURATION
// ============================================================
// Number of battery cells in series (valid: 3, 4, or 5)
// ============================================================
#define CELL_COUNT 3 // <-- Change here: 3, 4, or 5 cells
// ============================================================

// ============================================================
// BQ25672 CHARGE CURRENT CONFIGURATION
// ============================================================
// Set to 0 to keep the charger default.
// ============================================================
#define BQ25672_CHARGE_CURRENT_MA 500 // <-- Change here if needed
// ============================================================

#define I2C_MASTER_SCL_IO GPIO_NUM_6 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_7 /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM 0    /*!< I2C master i2c port number */

#define GPIO_PIN_CHECK GPIO_NUM_18
#define GPIO_PIN_ALERT GPIO_NUM_19

static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
  i2c_master_bus_config_t conf = {};
  conf.i2c_port = I2C_MASTER_NUM;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.clk_source = I2C_CLK_SRC_DEFAULT;
  conf.glitch_ignore_cnt = 7;
  conf.flags.enable_internal_pullup = true;

  return i2c_new_master_bus(&conf, bus_handle);
}

extern "C" void app_main(void) {
  // Set log levels at startup
  esp_log_level_set("*", ESP_LOG_INFO);        // Default all components to INFO
  esp_log_level_set("BQ76905", ESP_LOG_DEBUG); // BQ76905 driver to DEBUG
  esp_log_level_set("BQ25672", ESP_LOG_INFO);  // BQ25672 charger to INFO
  esp_log_level_set("BMS_APP", ESP_LOG_DEBUG); // Main app to DEBUG

  // Initialize GPIOs
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_PIN_CHECK) | (1ULL << GPIO_PIN_ALERT);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Assume pull-up needed for checking "LOW"
  gpio_config(&io_conf);

  // Check if GPIO 18 is LOW
  if (gpio_get_level(GPIO_PIN_CHECK) == 0) {
    ESP_LOGI(TAG, "GPIO 18 is LOW, proceeding with BMS configuration...");

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_master_init(&i2c_bus));
    ESP_LOGI(TAG, "I2C initialized successfully");

    BQ25672 charger;
    bool charger_ready = false;
    if (charger.begin(i2c_bus) == ESP_OK) {
      ESP_LOGI(TAG, "BQ25672 initialized successfully");

#if BATTERY_TYPE == 0
      esp_err_t charger_result = charger.configureBattery(BQ25672::BatteryType::LFP, CELL_COUNT,
                                                          BQ25672_CHARGE_CURRENT_MA);
#else
      esp_err_t charger_result = charger.configureBattery(BQ25672::BatteryType::LiIon, CELL_COUNT,
                                                          BQ25672_CHARGE_CURRENT_MA);
#endif

      if (charger_result == ESP_OK) {
        charger_ready = true;
        ESP_LOGI(TAG, "BQ25672 configuration: SUCCESS");
        charger.printControlAndConfiguration();
      } else {
        ESP_LOGE(TAG, "BQ25672 configuration: FAILED (0x%x)", charger_result);
      }
    } else {
      ESP_LOGW(TAG, "BQ25672 initialization failed");
    }

    BQ76905 bms(i2c_bus);

    if (bms.begin() == ESP_OK) {
      ESP_LOGI(TAG, "BMS initialized successfully");

      // ============================================================
      // Full BMS Configuration (based on BATTERY_TYPE define)
      // ============================================================
      // This configures:
      // - System (DA Config, Cell Count, FET Options)
      // - Voltage Protection (COV, CUV)
      // - Current Protection (OCC, OCD1, OCD2, SCD)
      // - Temperature Protection (OTC, OTD, UTC, UTD)
      // - Protection Enable Registers
      // ============================================================

#if BATTERY_TYPE == 0
      // LFP (LiFePO4) battery - 18650Fe1600-WT
      esp_err_t setup_result = bms.fullConfiguration(BQ76905::BatteryType::LFP, CELL_COUNT);
#else
      // Li-ion (NMC/LCO) battery
      esp_err_t setup_result = bms.fullConfiguration(BQ76905::BatteryType::LiIon, CELL_COUNT);
#endif

      if (setup_result == ESP_OK) {
        ESP_LOGI(TAG, ">>> Full Configuration Result: SUCCESS <<<");
      } else {
        ESP_LOGE(TAG, ">>> Full Configuration Result: FAILED (0x%x) <<<", setup_result);
      }

      ESP_LOGI(TAG, "");
      ESP_LOGI(TAG, "==============================================");
      ESP_LOGI(TAG, "Full configuration complete. Starting monitoring loop...");
      ESP_LOGI(TAG, "==============================================");

      while (1) {
        // Repeatedly read status every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "--- Reading BMS Data ---");

        if (charger_ready) {
          charger.update();

          uint16_t vbat_mv = 0;
          uint16_t vbus_mv = 0;
          float soc_pct = 0.0f;
          if (charger.getVBAT(&vbat_mv) == ESP_OK && charger.getVBUS(&vbus_mv) == ESP_OK &&
              charger.getBatteryPercentage(&soc_pct) == ESP_OK) {
            ESP_LOGI(TAG, "Charger: VBAT=%u mV, VBUS=%u mV, SOC=%.1f%%", vbat_mv, vbus_mv, soc_pct);
          } else {
            ESP_LOGW(TAG, "Charger read failed");
          }
        }

        int16_t current_ma = 0;
        if (bms.getCurrent(current_ma) == ESP_OK) {
          ESP_LOGI(TAG, "Current: %d mA", current_ma);
        } else {
          ESP_LOGW(TAG, "Current read failed");
        }

        float temp_c = 0.0f;
        if (bms.getTemperature(temp_c) == ESP_OK) {
          ESP_LOGI(TAG, "Internal Temperature: %.2f C", temp_c);
        } else {
          ESP_LOGW(TAG, "Internal Temperature read failed");
        }

        float ts_temp_c = 0.0f;
        if (bms.getTSTemperature(ts_temp_c) == ESP_OK) {
          ESP_LOGI(TAG, "TS (NTC) Temperature: %.2f C", ts_temp_c);
        } else {
          ESP_LOGW(TAG, "TS Temperature read failed");
        }

        uint32_t cell_sum_mv = 0;
        ESP_LOGI(TAG, "Scanning all cell voltages (1-%u):", BQ76905::kMaxCells);
        for (uint8_t i = 1; i <= BQ76905::kMaxCells; i++) {
          uint16_t cell_vol = 0;
          esp_err_t err = bms.getCellVoltage(i, cell_vol);
          if (err == ESP_OK && cell_vol > 0) {
            ESP_LOGI(TAG, "  Cell %u: %u mV", i, cell_vol);
            if (cell_vol > 1000 && cell_vol < 5000) {
              cell_sum_mv += cell_vol;
            }
          }
        }

        ESP_LOGI(TAG, "Sum of Cells: %lu mV", cell_sum_mv);

        uint16_t pack_voltage = 0;
        if (bms.getPackVoltage(pack_voltage) == ESP_OK) {
          ESP_LOGI(TAG, "Pack Register: %u mV", pack_voltage);
        } else {
          ESP_LOGW(TAG, "Pack voltage read failed");
        }

        ESP_LOGI(TAG, "Alert Pin: %d", gpio_get_level(GPIO_PIN_ALERT));

        // Check BMS safety status and faults
        bms.checkStatus();

        ESP_LOGI(TAG, "------------------------");
      }
    } else {
      ESP_LOGE(TAG, "BMS initialization failed");
    }
  } else {
    ESP_LOGW(TAG, "GPIO 18 is HIGH. BMS configuration skipped.");
  }

  // If skipped or failed, just loop doing nothing or maybe check again?
  // Usually app_main shouldn't return.
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
