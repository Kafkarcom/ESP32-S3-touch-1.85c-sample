#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st77916.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "esp_lcd_touch.h"          // Defines esp_lcd_touch_point_t
#include "i2c_utils.h"              // I2C utilities
#include "display.h"                // Display functions
#include "touch.h"                  // Touch functions
#include "extendio.h"               // Extended IO functions

static const char *TAG = "Touch 1.85 sampole";

void app_main(void)
{
    // --- 1. INITIALIZE I2C ---
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t tca_handle;
    ESP_ERROR_CHECK(initialize_i2c(&bus_handle, &tca_handle));

    // --- 2. HARDWARE RESET VIA TCA9554 ---
    reset_lcd_via_tca9554(tca_handle);

    // --- 3. INITIALIZE QSPI BUS ---
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCK,
        .data0_io_num = LCD_SDA0,
        .data1_io_num = LCD_SDA1,
        .data2_io_num = LCD_SDA2,
        .data3_io_num = LCD_SDA3,
        // .max_transfer_sz = LCD_H_RES * 40 * sizeof(uint16_t),
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2, // Allocate enough for full buffer
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // --- 4. CONFIGURE LCD PANEL ---
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(display_init(&panel_handle));

    ESP_LOGI(TAG, "Heartbeat: ST77916 360x360 is active!");


    // --- 2. SELECTIVE TOUCH RESET VIA TCA9554 ---
    reset_touch_via_tca9554(tca_handle);
    esp_lcd_touch_handle_t touch_handle;

    // Initialize touch sensor
    ESP_ERROR_CHECK(touch_init(bus_handle, &touch_handle));


    // --- After LCD/Touch Initialization is complete ---
    touch_start(touch_handle);

    ESP_LOGI(TAG, "System Ready. Touch screen to cycle colors!");

    while (1) {
        ESP_LOGI(TAG, "ST77916 is still alive");

        ESP_LOGI(TAG, "Filling Red");
        set_backlight_brightness(0);
        lcd_fill_screen(panel_handle, 0xF800);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling Red");

        for (int i = 0; i < 101; i++) {
            set_backlight_brightness(i);
            vTaskDelay(pdMS_TO_TICKS(20));  
        }

        for (int i = 101; i > -1; i--) {
            set_backlight_brightness(i);
            vTaskDelay(pdMS_TO_TICKS(20));  
        }

        ESP_LOGI(TAG, "Filling Green");
        set_backlight_brightness(60);
        lcd_fill_screen(panel_handle, 0x07E0);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling Blue");
        set_backlight_brightness(20);
        lcd_fill_screen(panel_handle, 0x001F);
        vTaskDelay(pdMS_TO_TICKS(2000));        
    }
}