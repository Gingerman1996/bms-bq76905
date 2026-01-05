idf_component_register(SRCS "bms-bq76950.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c driver BQ25672 bq76905)
