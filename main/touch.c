#include "touch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_panel_io.h"
#include "esp_check.h"

static const char *TAG = "TOUCH";

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

void touch_start(esp_lcd_touch_handle_t touch_handle) {
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
}