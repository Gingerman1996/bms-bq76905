# BMS BQ76905

A Battery Management System (BMS) driver for the Texas Instruments BQ76905 analog front-end, built on ESP-IDF.

## Features

- **I2C Communication**: Direct register access and subcommand interface
- **Cell Monitoring**: Read individual cell voltages (up to 5 cells) and pack voltage
- **Current Measurement**: Read charge/discharge current
- **Temperature Monitoring**: Internal die temperature and external NTC thermistor support
- **Battery Protection Configuration**:
  - **Voltage Protection**: Cell Overvoltage (COV), Cell Undervoltage (CUV)
  - **Current Protection**: Overcurrent Charge (OCC), Overcurrent Discharge (OCD1/OCD2), Short Circuit (SCD)
  - **Temperature Protection**: Overtemperature (OTC/OTD), Undertemperature (UTC/UTD)
- **Preset Battery Profiles**: LiFePO4 (LFP) and Li-ion (NMC/LCO)

## Hardware Requirements

- ESP32-based development board
- BQ76905 IC connected via I2C
- Default I2C pins: **SCL = GPIO 6**, **SDA = GPIO 7**
- Optional: External NTC thermistor (10kΩ @ 25°C, β = 3950)
- Optional: Alert pin on **GPIO 19**, Configuration check pin on **GPIO 18**

## Project Structure

```
bms-bq76950/
├── main/
│   ├── bms-bq76950.cpp      # Main application
│   └── CMakeLists.txt
├── components/
│   └── bq76905/
│       ├── bq76905.cpp      # BQ76905 driver implementation
│       └── include/
│           └── bq76905.h    # Driver header file
├── CMakeLists.txt
├── partitions_8mb.csv
└── sdkconfig
```

## Configuration

### Battery Type Selection

In `main/bms-bq76950.cpp`, set the `BATTERY_TYPE` define:

```cpp
#define BATTERY_TYPE 1  // 0 = LFP (LiFePO4), 1 = Li-ion (NMC/LCO)
```

| Parameter | LFP (LiFePO4) | Li-ion (NMC/LCO) |
|-----------|---------------|------------------|
| COV (Overvoltage) | 3.65V | 4.2V |
| CUV (Undervoltage) | 2.0V | 3.0V |
| Charge Temp Range | 0°C to 60°C | 0°C to 45°C |
| Discharge Temp Range | -40°C to 60°C | -20°C to 60°C |

### Cell Count

The default cell count can be configured via the `fullConfiguration()` function:

```cpp
bms.fullConfiguration(BQ76905::BatteryType::LiIon, 3);  // 3-cell Li-ion
bms.fullConfiguration(BQ76905::BatteryType::LFP, 5);    // 5-cell LFP
```

## Building and Flashing

```bash
# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Build the project
idf.py build

# Flash to the device
idf.py -p /dev/ttyUSB0 flash monitor
```

## API Reference

### Initialization

```cpp
BQ76905 bms((i2c_port_t)I2C_MASTER_NUM);
esp_err_t result = bms.begin();
```

### Reading Measurements

```cpp
// Cell voltage (1-5)
uint16_t cell_mv;
bms.getCellVoltage(1, cell_mv);

// Pack voltage
uint16_t pack_mv;
bms.getPackVoltage(pack_mv);

// Current
int16_t current;
bms.getCurrent(current);

// Internal temperature
float internal_temp;
bms.getTemperature(internal_temp);

// External NTC temperature
float ntc_temp;
bms.getTSTemperature(ntc_temp);
```

### Full Configuration

```cpp
// Configure all protections for Li-ion 3-cell battery
bms.fullConfiguration(BQ76905::BatteryType::LiIon, 3);
```

## License

This project is provided as-is for educational and development purposes.

## References

- [BQ76905 Datasheet (Texas Instruments)](https://www.ti.com/product/BQ76905)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
