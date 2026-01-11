#ifndef TOUCH_H
#define TOUCH_H

#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"

// Touch interrupt pin
#define TOUCH_INT_IO 4

// Function declarations
esp_err_t touch_init(i2c_master_bus_handle_t bus_handle, esp_lcd_touch_handle_t *touch_handle);
void touch_start(esp_lcd_touch_handle_t touch_handle);

#endif // TOUCH_H