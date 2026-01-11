#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st77916.h"

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

// Vendor specific init commands
extern const st77916_lcd_init_cmd_t vendor_specific_init_new[];
extern const size_t vendor_init_cmds_size;

// Function declarations
void backlight_init(void);
void set_backlight_brightness(uint8_t percentage);
void lcd_fill_screen(esp_lcd_panel_handle_t panel_handle, uint16_t color);
void draw_circle(esp_lcd_panel_handle_t panel_handle, uint16_t x_center, uint16_t y_center, uint16_t r, uint16_t color);
esp_err_t display_init(esp_lcd_panel_handle_t *panel_handle);

#endif // DISPLAY_H