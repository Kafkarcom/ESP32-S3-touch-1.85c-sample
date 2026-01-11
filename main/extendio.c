#include "extendio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "esp_system.h"

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
 * @brief Reset LCD via TCA9554
 * @param tca_handle TCA9554 device handle
 */
void reset_lcd_via_tca9554(i2c_master_dev_handle_t tca_handle) {
    // Configuration Register: 0 = Output, 1 = Input.
    // We set EXIO2 (bit 1) to 0.
    tca9554_write_reg(tca_handle, TCA9554_CONFIG_REG, 0xFD); // 1111 1101

    // Resetting ST77916 Display...
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFD); // Pull EXIO2 LOW
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFF); // Pull EXIO2 HIGH
    vTaskDelay(pdMS_TO_TICKS(120));
}

/**
 * @brief Reset Touch via TCA9554
 * @param tca_handle TCA9554 device handle
 */
void reset_touch_via_tca9554(i2c_master_dev_handle_t tca_handle) {
    // 1. Set EXIO1 (bit 0) to Output.
    // We keep EXIO2 (bit 1) as output too if it was already set.
    // 0xFC = 1111 1100 (Bits 0 and 1 are outputs)
    tca9554_write_reg(tca_handle, TCA9554_CONFIG_REG, 0xFC);

    // 2. Pull EXIO1 LOW (Reset Touch), keep EXIO2 HIGH (LCD Active)
    // 1111 1110 = 0xFE
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFE);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Release Reset: Pull EXIO1 HIGH
    // 1111 1111 = 0xFF
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFF);

    // 4. CRITICAL: Wait for CST816S firmware to boot before I2C calls
    vTaskDelay(pdMS_TO_TICKS(200));
}