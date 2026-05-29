#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

// 确认你的板子是 10 号口
#define BLINK_GPIO 10

static const char *TAG = "C3_Zero_LED";
static led_strip_handle_t led_strip;

void app_main(void)
{
    ESP_LOGI(TAG, "正在启动，尝试点亮 GPIO %d...", BLINK_GPIO);

    /* 1. 基础配置：只保留旧版本支持的成员 */
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };

    /* 2. RMT 驱动配置 */
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };

    /* 3. 创建驱动实例 */
    // 如果这里还报错，说明你可能没安装 led_strip 组件
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "初始化失败: %s", esp_err_to_name(err));
        return;
    }

    bool led_state = false;
    while (1) {
        if (led_state) {
            // 设置颜色：(灯带实例, 像素索引, 红, 绿, 蓝)
            // 稍微调亮一点，方便你观察
            led_strip_set_pixel(led_strip, 0, 50, 0, 50); 
            led_strip_refresh(led_strip);
            ESP_LOGI(TAG, "灯亮 (紫色)");
        } else {
            led_strip_clear(led_strip);
            ESP_LOGI(TAG, "灯灭");
        }

        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}