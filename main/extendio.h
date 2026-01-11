#ifndef EXTENDIO_H
#define EXTENDIO_H

#include "driver/i2c_master.h"

// TCA9554 Configuration
#define TCA9554_ADDRESS         0x20
#define TCA9554_OUTPUT_REG      0x01
#define TCA9554_CONFIG_REG      0x03

// TCA9554 Pin Definitions
#define TCA_PIN_TOUCH_RST  (1 << 0) // EXIO1
#define TCA_PIN_LCD_RST    (1 << 1) // EXIO2

// Function declarations
void tca9554_write_reg(i2c_master_dev_handle_t tca_handle, uint8_t reg, uint8_t data);
void reset_lcd_via_tca9554(i2c_master_dev_handle_t tca_handle);
void reset_touch_via_tca9554(i2c_master_dev_handle_t tca_handle);

#endif // EXTENDIO_H