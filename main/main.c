#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st77916.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "esp_lcd_touch.h"          // Defines esp_lcd_touch_point_t
#include "esp_lcd_touch_cst816s.h"  // Your specific driver

static const char *TAG = "S3_ST77916_LCD";

/** * User Provided I2C & TCA9554 Settings 
 */
#define I2C_SCL_IO              10
#define I2C_SDA_IO              11
#define I2C_MASTER_NUM          0
#define I2C_MASTER_FREQ_HZ      400000
#define TCA9554_ADDRESS         0x20
#define TCA9554_OUTPUT_REG      0x01
#define TCA9554_CONFIG_REG      0x03
#define TOUCH_INT_IO            4


#define TCA9554_EXIO1 0x01
#define TCA9554_EXIO2 0x02
#define TCA9554_EXIO3 0x03
#define TCA9554_EXIO4 0x04
#define TCA9554_EXIO5 0x05
#define TCA9554_EXIO6 0x06
#define TCA9554_EXIO7 0x07
#define TCA9554_EXIO8 0x08

#define TCA_PIN_TOUCH_RST  (1 << 0) // EXIO1
#define TCA_PIN_LCD_RST    (1 << 1) // EXIO2

// Constants from the supplier
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_RESOLUTION        LEDC_TIMER_13_BIT
#define LEDC_MAX_DUTY          ((1 << 13) - 1) // 8191

/**
 * Display Settings
 */
#define LCD_H_RES              360
#define LCD_V_RES              360
#define LCD_SDA0               46
#define LCD_SDA1               45
#define LCD_SDA2               42
#define LCD_SDA3               41
#define LCD_SCK                40
#define LCD_CS                 21
#define LCD_BL                 5


static const st77916_lcd_init_cmd_t vendor_specific_init_new[] = {
  {0xF0, (uint8_t []){0x28}, 1, 0},
  {0xF2, (uint8_t []){0x28}, 1, 0},
  {0x73, (uint8_t []){0xF0}, 1, 0},
  {0x7C, (uint8_t []){0xD1}, 1, 0},
  {0x83, (uint8_t []){0xE0}, 1, 0},
  {0x84, (uint8_t []){0x61}, 1, 0},
  {0xF2, (uint8_t []){0x82}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x01}, 1, 0},
  {0xF1, (uint8_t []){0x01}, 1, 0},
  {0xB0, (uint8_t []){0x56}, 1, 0},
  {0xB1, (uint8_t []){0x4D}, 1, 0},
  {0xB2, (uint8_t []){0x24}, 1, 0},
  {0xB4, (uint8_t []){0x87}, 1, 0},
  {0xB5, (uint8_t []){0x44}, 1, 0},
  {0xB6, (uint8_t []){0x8B}, 1, 0},
  {0xB7, (uint8_t []){0x40}, 1, 0},
  {0xB8, (uint8_t []){0x86}, 1, 0},
  {0xBA, (uint8_t []){0x00}, 1, 0},
  {0xBB, (uint8_t []){0x08}, 1, 0},
  {0xBC, (uint8_t []){0x08}, 1, 0},
  {0xBD, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x80}, 1, 0},
  {0xC1, (uint8_t []){0x10}, 1, 0},
  {0xC2, (uint8_t []){0x37}, 1, 0},
  {0xC3, (uint8_t []){0x80}, 1, 0},
  {0xC4, (uint8_t []){0x10}, 1, 0},
  {0xC5, (uint8_t []){0x37}, 1, 0},
  {0xC6, (uint8_t []){0xA9}, 1, 0},
  {0xC7, (uint8_t []){0x41}, 1, 0},
  {0xC8, (uint8_t []){0x01}, 1, 0},
  {0xC9, (uint8_t []){0xA9}, 1, 0},
  {0xCA, (uint8_t []){0x41}, 1, 0},
  {0xCB, (uint8_t []){0x01}, 1, 0},
  {0xD0, (uint8_t []){0x91}, 1, 0},
  {0xD1, (uint8_t []){0x68}, 1, 0},
  {0xD2, (uint8_t []){0x68}, 1, 0},
  {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
  {0xDD, (uint8_t []){0x4F}, 1, 0},
  {0xDE, (uint8_t []){0x4F}, 1, 0},
  {0xF1, (uint8_t []){0x10}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x02}, 1, 0},
  {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
  {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
  {0xF0, (uint8_t []){0x10}, 1, 0},
  {0xF3, (uint8_t []){0x10}, 1, 0},
  {0xE0, (uint8_t []){0x07}, 1, 0},
  {0xE1, (uint8_t []){0x00}, 1, 0},
  {0xE2, (uint8_t []){0x00}, 1, 0},
  {0xE3, (uint8_t []){0x00}, 1, 0},
  {0xE4, (uint8_t []){0xE0}, 1, 0},
  {0xE5, (uint8_t []){0x06}, 1, 0},
  {0xE6, (uint8_t []){0x21}, 1, 0},
  {0xE7, (uint8_t []){0x01}, 1, 0},
  {0xE8, (uint8_t []){0x05}, 1, 0},
  {0xE9, (uint8_t []){0x02}, 1, 0},
  {0xEA, (uint8_t []){0xDA}, 1, 0},
  {0xEB, (uint8_t []){0x00}, 1, 0},
  {0xEC, (uint8_t []){0x00}, 1, 0},
  {0xED, (uint8_t []){0x0F}, 1, 0},
  {0xEE, (uint8_t []){0x00}, 1, 0},
  {0xEF, (uint8_t []){0x00}, 1, 0},
  {0xF8, (uint8_t []){0x00}, 1, 0},
  {0xF9, (uint8_t []){0x00}, 1, 0},
  {0xFA, (uint8_t []){0x00}, 1, 0},
  {0xFB, (uint8_t []){0x00}, 1, 0},
  {0xFC, (uint8_t []){0x00}, 1, 0},
  {0xFD, (uint8_t []){0x00}, 1, 0},
  {0xFE, (uint8_t []){0x00}, 1, 0},
  {0xFF, (uint8_t []){0x00}, 1, 0},
  {0x60, (uint8_t []){0x40}, 1, 0},
  {0x61, (uint8_t []){0x04}, 1, 0},
  {0x62, (uint8_t []){0x00}, 1, 0},
  {0x63, (uint8_t []){0x42}, 1, 0},
  {0x64, (uint8_t []){0xD9}, 1, 0},
  {0x65, (uint8_t []){0x00}, 1, 0},
  {0x66, (uint8_t []){0x00}, 1, 0},
  {0x67, (uint8_t []){0x00}, 1, 0},
  {0x68, (uint8_t []){0x00}, 1, 0},
  {0x69, (uint8_t []){0x00}, 1, 0},
  {0x6A, (uint8_t []){0x00}, 1, 0},
  {0x6B, (uint8_t []){0x00}, 1, 0},
  {0x70, (uint8_t []){0x40}, 1, 0},
  {0x71, (uint8_t []){0x03}, 1, 0},
  {0x72, (uint8_t []){0x00}, 1, 0},
  {0x73, (uint8_t []){0x42}, 1, 0},
  {0x74, (uint8_t []){0xD8}, 1, 0},
  {0x75, (uint8_t []){0x00}, 1, 0},
  {0x76, (uint8_t []){0x00}, 1, 0},
  {0x77, (uint8_t []){0x00}, 1, 0},
  {0x78, (uint8_t []){0x00}, 1, 0},
  {0x79, (uint8_t []){0x00}, 1, 0},
  {0x7A, (uint8_t []){0x00}, 1, 0},
  {0x7B, (uint8_t []){0x00}, 1, 0},
  {0x80, (uint8_t []){0x48}, 1, 0},
  {0x81, (uint8_t []){0x00}, 1, 0},
  {0x82, (uint8_t []){0x06}, 1, 0},
  {0x83, (uint8_t []){0x02}, 1, 0},
  {0x84, (uint8_t []){0xD6}, 1, 0},
  {0x85, (uint8_t []){0x04}, 1, 0},
  {0x86, (uint8_t []){0x00}, 1, 0},
  {0x87, (uint8_t []){0x00}, 1, 0},
  {0x88, (uint8_t []){0x48}, 1, 0},
  {0x89, (uint8_t []){0x00}, 1, 0},
  {0x8A, (uint8_t []){0x08}, 1, 0},
  {0x8B, (uint8_t []){0x02}, 1, 0},
  {0x8C, (uint8_t []){0xD8}, 1, 0},
  {0x8D, (uint8_t []){0x04}, 1, 0},
  {0x8E, (uint8_t []){0x00}, 1, 0},
  {0x8F, (uint8_t []){0x00}, 1, 0},
  {0x90, (uint8_t []){0x48}, 1, 0},
  {0x91, (uint8_t []){0x00}, 1, 0},
  {0x92, (uint8_t []){0x0A}, 1, 0},
  {0x93, (uint8_t []){0x02}, 1, 0},
  {0x94, (uint8_t []){0xDA}, 1, 0},
  {0x95, (uint8_t []){0x04}, 1, 0},
  {0x96, (uint8_t []){0x00}, 1, 0},
  {0x97, (uint8_t []){0x00}, 1, 0},
  {0x98, (uint8_t []){0x48}, 1, 0},
  {0x99, (uint8_t []){0x00}, 1, 0},
  {0x9A, (uint8_t []){0x0C}, 1, 0},
  {0x9B, (uint8_t []){0x02}, 1, 0},
  {0x9C, (uint8_t []){0xDC}, 1, 0},
  {0x9D, (uint8_t []){0x04}, 1, 0},
  {0x9E, (uint8_t []){0x00}, 1, 0},
  {0x9F, (uint8_t []){0x00}, 1, 0},
  {0xA0, (uint8_t []){0x48}, 1, 0},
  {0xA1, (uint8_t []){0x00}, 1, 0},
  {0xA2, (uint8_t []){0x05}, 1, 0},
  {0xA3, (uint8_t []){0x02}, 1, 0},
  {0xA4, (uint8_t []){0xD5}, 1, 0},
  {0xA5, (uint8_t []){0x04}, 1, 0},
  {0xA6, (uint8_t []){0x00}, 1, 0},
  {0xA7, (uint8_t []){0x00}, 1, 0},
  {0xA8, (uint8_t []){0x48}, 1, 0},
  {0xA9, (uint8_t []){0x00}, 1, 0},
  {0xAA, (uint8_t []){0x07}, 1, 0},
  {0xAB, (uint8_t []){0x02}, 1, 0},
  {0xAC, (uint8_t []){0xD7}, 1, 0},
  {0xAD, (uint8_t []){0x04}, 1, 0},
  {0xAE, (uint8_t []){0x00}, 1, 0},
  {0xAF, (uint8_t []){0x00}, 1, 0},
  {0xB0, (uint8_t []){0x48}, 1, 0},
  {0xB1, (uint8_t []){0x00}, 1, 0},
  {0xB2, (uint8_t []){0x09}, 1, 0},
  {0xB3, (uint8_t []){0x02}, 1, 0},
  {0xB4, (uint8_t []){0xD9}, 1, 0},
  {0xB5, (uint8_t []){0x04}, 1, 0},
  {0xB6, (uint8_t []){0x00}, 1, 0},
  {0xB7, (uint8_t []){0x00}, 1, 0},
  
  {0xB8, (uint8_t []){0x48}, 1, 0},
  {0xB9, (uint8_t []){0x00}, 1, 0},
  {0xBA, (uint8_t []){0x0B}, 1, 0},
  {0xBB, (uint8_t []){0x02}, 1, 0},
  {0xBC, (uint8_t []){0xDB}, 1, 0},
  {0xBD, (uint8_t []){0x04}, 1, 0},
  {0xBE, (uint8_t []){0x00}, 1, 0},
  {0xBF, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x10}, 1, 0},
  {0xC1, (uint8_t []){0x47}, 1, 0},
  {0xC2, (uint8_t []){0x56}, 1, 0},
  {0xC3, (uint8_t []){0x65}, 1, 0},
  {0xC4, (uint8_t []){0x74}, 1, 0},
  {0xC5, (uint8_t []){0x88}, 1, 0},
  {0xC6, (uint8_t []){0x99}, 1, 0},
  {0xC7, (uint8_t []){0x01}, 1, 0},
  {0xC8, (uint8_t []){0xBB}, 1, 0},
  {0xC9, (uint8_t []){0xAA}, 1, 0},
  {0xD0, (uint8_t []){0x10}, 1, 0},
  {0xD1, (uint8_t []){0x47}, 1, 0},
  {0xD2, (uint8_t []){0x56}, 1, 0},
  {0xD3, (uint8_t []){0x65}, 1, 0},
  {0xD4, (uint8_t []){0x74}, 1, 0},
  {0xD5, (uint8_t []){0x88}, 1, 0},
  {0xD6, (uint8_t []){0x99}, 1, 0},
  {0xD7, (uint8_t []){0x01}, 1, 0},
  {0xD8, (uint8_t []){0xBB}, 1, 0},
  {0xD9, (uint8_t []){0xAA}, 1, 0},
  {0xF3, (uint8_t []){0x01}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0x21, (uint8_t []){0x00}, 1, 0},
  {0x11, (uint8_t []){0x00}, 1, 120},
  {0x29, (uint8_t []){0x00}, 1, 0},  
};

i2c_master_dev_handle_t tca_handle;

// Helper to write to TCA9554 registers
void tca9554_write_reg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    i2c_master_transmit(tca_handle, write_buf, 2, -1);
}

/**
 * @brief Initialize the LEDC peripheral for Backlight PWM
 */
void backlight_init(void) {
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = 5000,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = 5, // LCD_BL from your setup
        .speed_mode = LEDC_LS_MODE,
        .timer_sel  = LEDC_HS_TIMER,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Sets backlight brightness
 * @param percentage 0 to 100
 */
void set_backlight_brightness(uint8_t percentage) {
    if (percentage > 100) percentage = 100;
    
    // Calculate duty: (percentage / 100) * LEDC_MAX_DUTY
    uint32_t duty = (percentage * LEDC_MAX_DUTY) / 100;
    
    ledc_set_duty(LEDC_LS_MODE, LEDC_HS_CH0_CHANNEL, duty);
    ledc_update_duty(LEDC_LS_MODE, LEDC_HS_CH0_CHANNEL);
}

/**
 * @brief Fills the screen with a single color
 * @param panel_handle The handle for the initialized LCD
 * @param color RGB565 color value (e.g., Red = 0xF800)
 */
void lcd_fill_screen(esp_lcd_panel_handle_t panel_handle, uint16_t color)
{
    // We will draw the screen in chunks of 20 lines to save RAM
    const int lines_per_chunk = 20;
    uint16_t *buffer = heap_caps_malloc(LCD_H_RES * lines_per_chunk * sizeof(uint16_t), MALLOC_CAP_DMA);
    
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for fill buffer!");
        return;
    }

    // Fill the chunk buffer with the chosen color
    for (int i = 0; i < LCD_H_RES * lines_per_chunk; i++) {
        // Swap bytes if colors look wrong (SPI usually expects Big Endian)
        buffer[i] = (color << 8) | (color >> 8); 
    }

    // Push the chunks to the LCD
    for (int y = 0; y < LCD_V_RES; y += lines_per_chunk) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + lines_per_chunk, buffer);
    }

    free(buffer);
}

/**
 * @brief Draws a simple filled square/circle at coordinates
 * @param x_center Touch X
 * @param y_center Touch Y
 * @param r Radius
 * @param color RGB565 color
 */
void draw_circle(esp_lcd_panel_handle_t panel_handle, uint16_t x_center, uint16_t y_center, uint16_t r, uint16_t color) {
    int side = r * 2;
    uint16_t swapped_color = (color << 8) | (color >> 8);
    
    // Allocate a buffer for the circle's bounding box
    uint16_t *buf = heap_caps_malloc(side * side * 2, MALLOC_CAP_DMA);
    if (!buf) return;

    for (int y = 0; y < side; y++) {
        for (int x = 0; x < side; x++) {
            int dx = x - r;
            int dy = y - r;
            // Mathematical check: is the pixel inside the circle radius?
            if ((dx * dx + dy * dy) <= (r * r)) {
                buf[y * side + x] = swapped_color;
            } else {
                // Transparent/Background (optional: read back buffer or just use black)
                buf[y * side + x] = 0x0000; 
            }
        }
    }

    // Ensure we don't draw off-screen
    uint16_t draw_x = (x_center > r) ? (x_center - r) : 0;
    uint16_t draw_y = (y_center > r) ? (y_center - r) : 0;

    esp_lcd_panel_draw_bitmap(panel_handle, draw_x, draw_y, draw_x + side, draw_y + side, buf);
    free(buf);
}



static TaskHandle_t touch_task_handle = NULL;

// This task sleeps until the ISR wakes it up
void touch_process_task(void *pvParameters) {
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)pvParameters;
    esp_lcd_touch_point_data_t point;
    uint8_t points;

    while (1) {
        // Wait for notification from ISR (indefinite timeout)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Read data from the CST816S via I2C
        if (esp_lcd_touch_read_data(tp) == ESP_OK) {
            if (esp_lcd_touch_get_data(tp, &point, &points, 1) == ESP_OK) {
                if (points > 0) {
                    ESP_LOGI("TOUCH", "X:%d Y:%d", point.x, point.y);
                    // You can trigger events here, like updating a UI
                }
            }
        }
        // The CST816S releases the INT line after the I2C read is successful
    }
}

static void IRAM_ATTR touch_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(touch_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void)
{
    // --- 1. INITIALIZE I2C ---
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9554_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &tca_handle));

    // --- 2. HARDWARE RESET VIA TCA9554 ---
    ESP_LOGI(TAG, "Configuring TCA9554 EXIO2 as Output...");
    // Configuration Register: 0 = Output, 1 = Input. 
    // We set EXIO2 (bit 1) to 0. 
    tca9554_write_reg(TCA9554_CONFIG_REG, 0xFD); // 1111 1101

    ESP_LOGI(TAG, "Resetting ST77916 Display...");
    tca9554_write_reg(TCA9554_OUTPUT_REG, 0xFD); // Pull EXIO2 LOW
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_write_reg(TCA9554_OUTPUT_REG, 0xFF); // Pull EXIO2 HIGH
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
        .init_cmds_size = sizeof(vendor_specific_init_new) / sizeof(st77916_lcd_init_cmd_t),
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
    tca9554_write_reg(TCA9554_CONFIG_REG, 0xFC); 

    ESP_LOGI(TAG, "Performing Touch-Only Reset...");

    // 2. Pull EXIO1 LOW (Reset Touch), keep EXIO2 HIGH (LCD Active)
    // 1111 1110 = 0xFE
    tca9554_write_reg(TCA9554_OUTPUT_REG, 0xFE); 
    vTaskDelay(pdMS_TO_TICKS(50)); 

    // 3. Release Reset: Pull EXIO1 HIGH
    // 1111 1111 = 0xFF
    tca9554_write_reg(TCA9554_OUTPUT_REG, 0xFF); 

    // 4. CRITICAL: Wait for CST816S firmware to boot before I2C calls
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_lcd_touch_handle_t touch_handle;

   // D. Official CST816S Touch Driver Setup
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,
        .scl_speed_hz = 100000,                      // Try 100kHz for better stability
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .flags.disable_control_phase = 1,
    };
    
    esp_lcd_panel_io_handle_t tp_io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &tp_io_cfg, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES, .y_max = LCD_V_RES, .rst_gpio_num = -1, .int_gpio_num = TOUCH_INT_IO,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle));
    
    // Variables for the demo
    uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF}; // Red, Green, Blue, White
    int color_idx = 0;
    
    // Official touch data structure
    esp_lcd_touch_point_data_t tp_point; 
    uint8_t touch_points;


    // --- After LCD/Touch Initialization is complete ---

    // 1. Create the dedicated Touch Task (High Priority)
    xTaskCreate(touch_process_task, "touch_task", 4096, touch_handle, 10, &touch_task_handle);

    // 2. Configure the Interrupt Pin (GPIO 4)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // CST816S pulls LOW on touch
        .pin_bit_mask = (1ULL << TOUCH_INT_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Essential for I2C stability
    };
    gpio_config(&io_conf);

    // 3. Install ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TOUCH_INT_IO, touch_isr_handler, NULL);



    ESP_LOGI(TAG, "System Ready. Touch screen to cycle colors!");

    while (1) {
        ESP_LOGI(TAG, "ST77916 is still alive");

// 1. Read data from hardware into internal buffer
        // if (gpio_get_level(TOUCH_INT_IO) == 0) {
        //     if (esp_lcd_touch_read_data(touch_handle) == ESP_OK) {
        //         ESP_LOGI(TAG, "esp_lcd_touch_read_data = ok");
        //         // 2. Extract the first point into our structure
        //         // Note: We use &tp_point (esp_lcd_touch_point_t)
        //         if (esp_lcd_touch_get_data(touch_handle, &tp_point, &touch_points, 1) == ESP_OK) {
                    
        //             if (touch_points > 0) {
        //                 // 3. Access coordinates via .x and .y (Now valid)
        //                 ESP_LOGI(TAG, "Touch Detected at [%d, %d]", tp_point.x, tp_point.y);
                        
        //                 // 4. Use the 'colors' variables (Fixes "unused variable" warnings)
        //                 // lcd_fill(colors[color_idx]);
        //                 // color_idx = (color_idx + 1) % 4;
                        
        //                 //vTaskDelay(pdMS_TO_TICKS(400)); // Debounce
        //             }
        //         }
        //     }
        //     else
        //     {
        //         ESP_LOGI(TAG, "esp_lcd_touch_read_data = not ok");
        //     }
        // }
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