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

static const char *TAG = "Touch 1.85 sampole";

void app_main(void)
{
    // --- 1. INITIALIZE I2C ---
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t tca_handle;
    ESP_ERROR_CHECK(i2c_init(&bus_handle, &tca_handle));

    // --- 2. HARDWARE RESET VIA TCA9554 ---
    ESP_LOGI(TAG, "Configuring TCA9554 EXIO2 as Output...");
    // Configuration Register: 0 = Output, 1 = Input. 
    // We set EXIO2 (bit 1) to 0. 
    tca9554_write_reg(tca_handle, TCA9554_CONFIG_REG, 0xFD); // 1111 1101

    ESP_LOGI(TAG, "Resetting ST77916 Display...");
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFD); // Pull EXIO2 LOW
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFF); // Pull EXIO2 HIGH
    vTaskDelay(pdMS_TO_TICKS(120));

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
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = -1,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .flags.quad_mode = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    st77916_vendor_config_t vendor_config = {
        .init_cmds = vendor_specific_init_new,
        .init_cmds_size = vendor_init_cmds_size,
        .flags.use_qspi_interface = 1,
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1, // We handled reset via I2C manually
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // --- 5. BACKLIGHT ---
    gpio_set_direction(LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BL, 1);

    backlight_init();

    ESP_LOGI(TAG, "Heartbeat: ST77916 360x360 is active!");


    // --- 2. SELECTIVE TOUCH RESET VIA TCA9554 ---
    ESP_LOGI(TAG, "Configuring TCA9554 EXIO1 (Touch Reset) as Output...");

    // 1. Set EXIO1 (bit 0) to Output. 
    // We keep EXIO2 (bit 1) as output too if it was already set.
    // 0xFC = 1111 1100 (Bits 0 and 1 are outputs)
    tca9554_write_reg(tca_handle, TCA9554_CONFIG_REG, 0xFC); 

    ESP_LOGI(TAG, "Performing Touch-Only Reset...");

    // 2. Pull EXIO1 LOW (Reset Touch), keep EXIO2 HIGH (LCD Active)
    // 1111 1110 = 0xFE
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFE); 
    vTaskDelay(pdMS_TO_TICKS(50)); 

    // 3. Release Reset: Pull EXIO1 HIGH
    // 1111 1111 = 0xFF
    tca9554_write_reg(tca_handle, TCA9554_OUTPUT_REG, 0xFF); 

    // 4. CRITICAL: Wait for CST816S firmware to boot before I2C calls
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_lcd_touch_handle_t touch_handle;

    // Initialize touch sensor
    ESP_ERROR_CHECK(touch_init(bus_handle, &touch_handle));
   
    // Official touch data structure
    esp_lcd_touch_point_data_t tp_point; 
    uint8_t touch_points;


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