#include "i2c_utils.h"
#include "extendio.h"
#include "esp_log.h"
#include "esp_check.h"

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
 * @brief Initialize I2C system
 * @param handles Pointer to struct to store the I2C handles
 * @return ESP_OK on success
 */
esp_err_t i2c_system_init(i2c_system_handles_t *handles) {
    return i2c_init(&handles->bus_handle, &handles->tca_handle);
}