#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "driver/i2c_master.h"
#include "esp_lcd_touch.h"

// I2C Configuration
#define I2C_SCL_IO              10
#define I2C_SDA_IO              11
#define I2C_MASTER_NUM          0
#define I2C_MASTER_FREQ_HZ      400000
#define TCA9554_ADDRESS         0x20
#define TCA9554_OUTPUT_REG      0x01
#define TCA9554_CONFIG_REG      0x03
#define TOUCH_INT_IO            4

// TCA9554 Pin Definitions
#define TCA_PIN_TOUCH_RST  (1 << 0) // EXIO1
#define TCA_PIN_LCD_RST    (1 << 1) // EXIO2

// Function declarations
esp_err_t i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *tca_handle);
void tca9554_write_reg(i2c_master_dev_handle_t tca_handle, uint8_t reg, uint8_t data);
esp_err_t touch_init(i2c_master_bus_handle_t bus_handle, esp_lcd_touch_handle_t *touch_handle);

#endif // I2C_UTILS_H