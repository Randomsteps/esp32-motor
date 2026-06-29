#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

// =====================================================
// 用户配置 - TTL 单线半双工总线 (带 DIR 方向控制)
// =====================================================
//
// 你的转接板: TX+RX 经三态缓冲器汇成一根 DATA 线
// DIR=HIGH → 发送, DIR=LOW → 接收
// ESP32 RTS 引脚硬件自动控制 DIR, 无需软件干预
//
// 接线:
//   ESP32 GPIO4 (TX)  ──→ 转接板 DI
//   ESP32 GPIO5 (RX)  ──→ 转接板 RO
//   ESP32 GPIO6 (RTS) ──→ 转接板 DIR
//   转接板 DATA        ──→ 舵机 DATA
//   GND ─── 全部共地

#define DXL_UART_NUM        UART_NUM_1
#define DXL_TX_GPIO         4
#define DXL_RX_GPIO         5
#define DXL_DIR_GPIO        6       // RTS 引脚, 接转接板方向控制
#define DXL_BAUDRATE        57600
#define DXL_ID              1
#define DXL_RX_BUF_SIZE     1024

static const char *TAG = "XL330_TTL";

// =====================================================
// DYNAMIXEL Protocol 2.0
// =====================================================

#define DXL_INST_PING       0x01
#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03
#define DXL_STATUS_PACKET   0x55

#define ADDR_MODEL_NUMBER        0
#define ADDR_FIRMWARE_VERSION    6
#define ADDR_OPERATING_MODE      11
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132

#define MODE_POSITION_CONTROL    3

// =====================================================
// CRC 表
// =====================================================

static const uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
};

static uint16_t dxl_update_crc(uint16_t crc_accum,
                               const uint8_t *data_blk_ptr,
                               uint16_t data_blk_size)
{
    for (uint16_t j = 0; j < data_blk_size; j++) {
        uint16_t i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

// =====================================================
// UART 初始化 (TTL 单线半双工, RTS 硬件控制 DIR)
// =====================================================

static void dxl_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = DXL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(DXL_UART_NUM, DXL_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DXL_UART_NUM, &uart_config));

    /*
     * TTL 单线半双工: RTS 硬件自动控制 DIR 引脚
     * DIR=HIGH → 发送使能, DIR=LOW → 接收使能
     * uart_write_bytes 会硬件自动拉高 DIR, 发完后自动拉低
     */
    ESP_ERROR_CHECK(uart_set_mode(DXL_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));

    /*
     * uart_set_pin 参数: TX, RX, RTS(方向控制), CTS
     */
    ESP_ERROR_CHECK(uart_set_pin(
        DXL_UART_NUM,
        DXL_TX_GPIO,
        DXL_RX_GPIO,
        DXL_DIR_GPIO,        // RTS → 转接板 DIR
        UART_PIN_NO_CHANGE
    ));

    uart_flush_input(DXL_UART_NUM);

    ESP_LOGI(TAG, "TTL 半双工初始化: TX=GPIO%d RX=GPIO%d DIR=GPIO%d Baud=%d",
             DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO, DXL_BAUDRATE);
}

// =====================================================
// 发包
// =====================================================

static esp_err_t dxl_send_packet(uint8_t id,
                                 uint8_t instruction,
                                 const uint8_t *params,
                                 uint16_t param_len)
{
    uint8_t packet[256];
    uint16_t idx = 0;

    uint16_t length = param_len + 3;

    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;
    packet[idx++] = id;
    packet[idx++] = length & 0xFF;
    packet[idx++] = (length >> 8) & 0xFF;
    packet[idx++] = instruction;

    for (uint16_t i = 0; i < param_len; i++) {
        packet[idx++] = params[i];
    }

    uint16_t crc = dxl_update_crc(0, packet, idx);
    packet[idx++] = crc & 0xFF;
    packet[idx++] = (crc >> 8) & 0xFF;

    int written = uart_write_bytes(DXL_UART_NUM, (const char *)packet, idx);

    /*
     * UART_MODE_RS485_HALF_DUPLEX 模式下:
     * 1. 硬件自动拉高 DIR → 2. 发数据 → 3. 发完 Txfifo 空 → 4. 硬件自动拉低 DIR
     */
    uart_wait_tx_done(DXL_UART_NUM, pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "TX: ID=%d INST=0x%02X LEN=%d", id, instruction, idx);

    if (written != idx) {
        ESP_LOGE(TAG, "UART write failed: %d/%d", written, idx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// =====================================================
// 收状态包
// =====================================================

static esp_err_t dxl_read_status(uint8_t expected_id,
                                 uint8_t *params,
                                 uint16_t *param_len,
                                 uint8_t *error_code,
                                 int timeout_ms)
{
    uint8_t rx[512];
    int total = 0;
    TickType_t start_tick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        int len = uart_read_bytes(
            DXL_UART_NUM,
            rx + total,
            sizeof(rx) - total,
            pdMS_TO_TICKS(20)
        );

        if (len > 0) {
            total += len;

            for (int i = 0; i <= total - 10; i++) {
                if (rx[i]     != 0xFF ||
                    rx[i + 1] != 0xFF ||
                    rx[i + 2] != 0xFD ||
                    rx[i + 3] != 0x00) {
                    continue;
                }

                uint8_t id = rx[i + 4];
                uint16_t length = rx[i + 5] | (rx[i + 6] << 8);
                int packet_total_len = 7 + length;

                if (packet_total_len < 10 || i + packet_total_len > total) {
                    continue;
                }

                uint8_t status = rx[i + 7];

                // 只接受 0x55 状态包 (过滤掉板子回显的 TX 数据)
                if (status != DXL_STATUS_PACKET || id != expected_id) {
                    continue;
                }

                uint16_t crc_recv =
                    rx[i + packet_total_len - 2] |
                    (rx[i + packet_total_len - 1] << 8);

                uint16_t crc_calc =
                    dxl_update_crc(0, &rx[i], packet_total_len - 2);

                if (crc_recv != crc_calc) {
                    ESP_LOGE(TAG, "CRC err: recv=0x%04X calc=0x%04X",
                             crc_recv, crc_calc);
                    return ESP_FAIL;
                }

                uint8_t err = rx[i + 8];

                if (error_code) *error_code = err;

                uint16_t real_param_len = length - 4;
                if (param_len) *param_len = real_param_len;
                if (params && real_param_len > 0) {
                    memcpy(params, &rx[i + 9], real_param_len);
                }

                ESP_LOGI(TAG, "RX: ID=%d ERR=0x%02X LEN=%d",
                         id, err, real_param_len);

                return ESP_OK;
            }

            if (total > 450) total = 0;
        }
    }

    ESP_LOGW(TAG, "读状态超时 (已收 %d 字节)", total);
    return ESP_ERR_TIMEOUT;
}

// =====================================================
// 高层命令
// =====================================================

static esp_err_t dxl_ping(uint8_t id)
{
    ESP_LOGI(TAG, "Ping ID=%d", id);

    ESP_ERROR_CHECK(dxl_send_packet(id, DXL_INST_PING, NULL, 0));

    uint8_t params[16], err = 0;
    uint16_t pl = 0;
    esp_err_t ret = dxl_read_status(id, params, &pl, &err, 200);

    if (ret != ESP_OK) return ret;
    if (err != 0) return ESP_FAIL;

    if (pl >= 3) {
        uint16_t model = params[0] | (params[1] << 8);
        uint8_t fw = params[2];
        ESP_LOGI(TAG, "型号=%u 固件=%u", model, fw);
    }

    return ESP_OK;
}

static esp_err_t dxl_write(uint8_t id, uint16_t addr,
                           const uint8_t *data, uint16_t dlen)
{
    uint8_t params[128];
    params[0] = addr & 0xFF;
    params[1] = (addr >> 8) & 0xFF;
    memcpy(&params[2], data, dlen);

    ESP_ERROR_CHECK(dxl_send_packet(id, DXL_INST_WRITE, params, dlen + 2));

    uint16_t pl = 0;
    uint8_t err = 0;
    esp_err_t ret = dxl_read_status(id, NULL, &pl, &err, 200);

    if (ret != ESP_OK) return ret;
    return err == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t dxl_read(uint8_t id, uint16_t addr,
                          uint16_t rlen, uint8_t *out)
{
    uint8_t params[4];
    params[0] = addr & 0xFF;  params[1] = (addr >> 8) & 0xFF;
    params[2] = rlen & 0xFF;  params[3] = (rlen >> 8) & 0xFF;

    ESP_ERROR_CHECK(dxl_send_packet(id, DXL_INST_READ, params, 4));

    uint16_t pl = 0;
    uint8_t err = 0;
    esp_err_t ret = dxl_read_status(id, out, &pl, &err, 200);

    if (ret != ESP_OK) return ret;
    if (pl != rlen) return ESP_FAIL;
    return err == 0 ? ESP_OK : ESP_FAIL;
}

// =====================================================
// 主程序
// =====================================================

void app_main(void)
{
    dxl_uart_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "===== XL330 RS485 半双工验证 =====");

    // 1. Ping
    ESP_LOGI(TAG, "--- 第1步: Ping ---");
    if (dxl_ping(DXL_ID) != ESP_OK) {
        ESP_LOGE(TAG, "Ping 失败! 检查接线/电源/波特率");
        return;
    }

    // 2. 关扭矩
    ESP_LOGI(TAG, "--- 第2步: Torque OFF ---");
    uint8_t off = 0;
    dxl_write(DXL_ID, ADDR_TORQUE_ENABLE, &off, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. 设位置模式
    ESP_LOGI(TAG, "--- 第3步: 位置模式 ---");
    uint8_t mode = MODE_POSITION_CONTROL;
    dxl_write(DXL_ID, ADDR_OPERATING_MODE, &mode, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 4. 开扭矩
    ESP_LOGI(TAG, "--- 第4步: Torque ON ---");
    uint8_t on = 1;
    dxl_write(DXL_ID, ADDR_TORQUE_ENABLE, &on, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 5. 循环转动
    ESP_LOGI(TAG, "--- 第5步: 循环控制 ---");
    while (1) {
        uint32_t pos;
        uint8_t data[4];

        pos = 1024;
        data[0]=pos&0xFF; data[1]=(pos>>8)&0xFF; data[2]=(pos>>16)&0xFF; data[3]=(pos>>24)&0xFF;
        dxl_write(DXL_ID, ADDR_GOAL_POSITION, data, 4);
        vTaskDelay(pdMS_TO_TICKS(1500));

        uint8_t rd[4];
        if (dxl_read(DXL_ID, ADDR_PRESENT_POSITION, 4, rd) == ESP_OK) {
            int32_t p = rd[0]|(rd[1]<<8)|(rd[2]<<16)|(rd[3]<<24);
            ESP_LOGI(TAG, "当前位置: %ld (%.1f°)", (long)p, p/4096.0*360);
        }

        pos = 2048;
        data[0]=pos&0xFF; data[1]=(pos>>8)&0xFF; data[2]=(pos>>16)&0xFF; data[3]=(pos>>24)&0xFF;
        dxl_write(DXL_ID, ADDR_GOAL_POSITION, data, 4);
        vTaskDelay(pdMS_TO_TICKS(1500));

        if (dxl_read(DXL_ID, ADDR_PRESENT_POSITION, 4, rd) == ESP_OK) {
            int32_t p = rd[0]|(rd[1]<<8)|(rd[2]<<16)|(rd[3]<<24);
            ESP_LOGI(TAG, "当前位置: %ld (%.1f°)", (long)p, p/4096.0*360);
        }

        pos = 3072;
        data[0]=pos&0xFF; data[1]=(pos>>8)&0xFF; data[2]=(pos>>16)&0xFF; data[3]=(pos>>24)&0xFF;
        dxl_write(DXL_ID, ADDR_GOAL_POSITION, data, 4);
        vTaskDelay(pdMS_TO_TICKS(1500));

        if (dxl_read(DXL_ID, ADDR_PRESENT_POSITION, 4, rd) == ESP_OK) {
            int32_t p = rd[0]|(rd[1]<<8)|(rd[2]<<16)|(rd[3]<<24);
            ESP_LOGI(TAG, "当前位置: %ld (%.1f°)", (long)p, p/4096.0*360);
        }
    }
}
