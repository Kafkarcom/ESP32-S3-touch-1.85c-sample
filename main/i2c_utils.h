#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "driver/i2c_master.h"

// I2C Configuration
#define I2C_SCL_IO              10
#define I2C_SDA_IO              11
#define I2C_MASTER_NUM          0
#define I2C_MASTER_FREQ_HZ      400000

// Struct to hold I2C handles
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t tca_handle;
} i2c_system_handles_t;

// Function declarations
esp_err_t i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *tca_handle);
esp_err_t initialize_i2c(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *tca_handle);
esp_err_t i2c_system_init(i2c_system_handles_t *handles);

#endif // I2C_UTILS_H