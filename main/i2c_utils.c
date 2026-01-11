#include "i2c_utils.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_panel_io.h"

static const char *TAG = "I2C_UTILS";

/**
 * @brief Initialize I2C bus and TCA9554 device
 * @param bus_handle Pointer to store the I2C bus handle
 * @param tca_handle Pointer to store the TCA9554 device handle
 * @return ESP_OK on success
 */
esp_err_t i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *tca_handle) {
    // Initialize I2C bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_bus_config, bus_handle), TAG, "Failed to create I2C bus");

    // Add TCA9554 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9554_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(*bus_handle, &dev_cfg, tca_handle), TAG, "Failed to add TCA9554 device");

    return ESP_OK;
}

/**
 * @brief Write to TCA9554 register
 * @param tca_handle TCA9554 device handle
 * @param reg Register address
 * @param data Data to write
 */
void tca9554_write_reg(i2c_master_dev_handle_t tca_handle, uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    i2c_master_transmit(tca_handle, write_buf, 2, -1);
}

/**
 * @brief Initialize touch sensor
 * @param bus_handle I2C bus handle
 * @param touch_handle Pointer to store the touch handle
 * @return ESP_OK on success
 */
esp_err_t touch_init(i2c_master_bus_handle_t bus_handle, esp_lcd_touch_handle_t *touch_handle) {
    // Configure touch I2C IO
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,
        .scl_speed_hz = 100000,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .flags.disable_control_phase = 1,
    };

    esp_lcd_panel_io_handle_t tp_io_handle;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus_handle, &tp_io_cfg, &tp_io_handle), TAG, "Failed to create touch IO");

    // Configure touch
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 360, .y_max = 360, .rst_gpio_num = -1, .int_gpio_num = TOUCH_INT_IO,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, touch_handle), TAG, "Failed to create touch handle");

    return ESP_OK;
}