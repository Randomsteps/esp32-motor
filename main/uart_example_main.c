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
// 用户配置
// =====================================================

#define DXL_UART_NUM        UART_NUM_1

#define DXL_TX_GPIO         4
#define DXL_RX_GPIO         5

#define DXL_BAUDRATE        57600
#define DXL_ID              1

#define DXL_RX_BUF_SIZE     1024

static const char *TAG = "XL330";

// =====================================================
// DYNAMIXEL Protocol 2.0
// =====================================================

#define DXL_INST_PING       0x01
#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03

#define DXL_STATUS_PACKET   0x55

// =====================================================
// XL330 控制表地址
// =====================================================

#define ADDR_OPERATING_MODE       11
#define ADDR_TORQUE_ENABLE        64
#define ADDR_GOAL_POSITION        116
#define ADDR_PRESENT_POSITION     132

#define MODE_POSITION_CONTROL     3

#define TORQUE_OFF                0
#define TORQUE_ON                 1

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
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
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
// 十六进制打印
// =====================================================

static void print_hex(const char *prefix, const uint8_t *data, int len)
{
    printf("%s", prefix);

    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }

    printf("\n");
}

// =====================================================
// UART 初始化
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

    ESP_ERROR_CHECK(uart_driver_install(
        DXL_UART_NUM,
        DXL_RX_BUF_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(DXL_UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        DXL_UART_NUM,
        DXL_TX_GPIO,
        DXL_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    /*
     * 你的转接板是自动半双工、无方向控制脚。
     * 所以这里不能用 UART_MODE_RS485_HALF_DUPLEX。
     * 直接普通 UART 模式即可。
     */
    ESP_ERROR_CHECK(uart_set_mode(DXL_UART_NUM, UART_MODE_UART));

    uart_flush_input(DXL_UART_NUM);

    ESP_LOGI(TAG, "UART init done. TX=%d RX=%d Baud=%d",
             DXL_TX_GPIO,
             DXL_RX_GPIO,
             DXL_BAUDRATE);
}

// =====================================================
// 发送 DYNAMIXEL 包
// =====================================================

static esp_err_t dxl_send_packet(uint8_t id,
                                 uint8_t instruction,
                                 const uint8_t *params,
                                 uint16_t param_len)
{
    uint8_t packet[256];
    uint16_t idx = 0;

    /*
     * DYNAMIXEL Protocol 2.0:
     * Length = Instruction 1 byte + Parameters N bytes + CRC 2 bytes
     */
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
    uart_wait_tx_done(DXL_UART_NUM, pdMS_TO_TICKS(30));

    print_hex("TX: ", packet, idx);

    if (written != idx) {
        ESP_LOGE(TAG, "UART write failed. written=%d expected=%d",
                 written,
                 idx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// =====================================================
// 读取状态包
//
// 自动半双工板可能会回显自己发出去的 TX 包。
// 所以这里只接受第 8 字节为 0x55 的状态包。
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
    TickType_t timeout_tick = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_tick) {
        int len = uart_read_bytes(
            DXL_UART_NUM,
            rx + total,
            sizeof(rx) - total,
            pdMS_TO_TICKS(10)
        );

        if (len <= 0) {
            continue;
        }

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

            if (packet_total_len < 10) {
                continue;
            }

            if (i + packet_total_len > total) {
                continue;
            }

            uint8_t status = rx[i + 7];

            /*
             * 关键：
             * 自己发出的包这里是 0x01、0x02、0x03。
             * XL330 返回的状态包这里是 0x55。
             */
            if (status != DXL_STATUS_PACKET) {
                continue;
            }

            if (id != expected_id) {
                continue;
            }

            uint16_t crc_recv =
                rx[i + packet_total_len - 2] |
                (rx[i + packet_total_len - 1] << 8);

            uint16_t crc_calc =
                dxl_update_crc(0, &rx[i], packet_total_len - 2);

            if (crc_recv != crc_calc) {
                print_hex("RX CRC ERROR: ", &rx[i], packet_total_len);

                ESP_LOGE(TAG,
                         "CRC error. recv=0x%04X calc=0x%04X",
                         crc_recv,
                         crc_calc);

                return ESP_FAIL;
            }

            uint8_t err = rx[i + 8];

            if (error_code) {
                *error_code = err;
            }

            uint16_t real_param_len = length - 4;

            if (param_len) {
                *param_len = real_param_len;
            }

            if (params && real_param_len > 0) {
                memcpy(params, &rx[i + 9], real_param_len);
            }

            print_hex("RX: ", &rx[i], packet_total_len);

            ESP_LOGI(TAG,
                     "Status OK. ID=%d Error=0x%02X ParamLen=%d",
                     id,
                     err,
                     real_param_len);

            return ESP_OK;
        }

        if (total > 450) {
            print_hex("RX BUFFER CLEAR: ", rx, total);
            total = 0;
        }
    }

    if (total > 0) {
        print_hex("RX TIMEOUT BUFFER: ", rx, total);
    }

    ESP_LOGW(TAG, "Read status timeout");
    return ESP_ERR_TIMEOUT;
}

// =====================================================
// Ping
// =====================================================

static esp_err_t dxl_ping(uint8_t id)
{
    uint8_t params[16] = {0};
    uint16_t param_len = 0;
    uint8_t error = 0;

    ESP_LOGI(TAG, "Ping ID=%d", id);

    esp_err_t ret = dxl_send_packet(id, DXL_INST_PING, NULL, 0);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = dxl_read_status(id, params, &param_len, &error, 200);

    if (ret != ESP_OK) {
        return ret;
    }

    if (error != 0) {
        ESP_LOGE(TAG, "Ping returned error=0x%02X", error);
        return ESP_FAIL;
    }

    if (param_len >= 3) {
        uint16_t model_number = params[0] | (params[1] << 8);
        uint8_t firmware = params[2];

        ESP_LOGI(TAG,
                 "Model Number=%u, Firmware=%u",
                 model_number,
                 firmware);
    }

    return ESP_OK;
}

// =====================================================
// Write
// =====================================================

static esp_err_t dxl_write(uint8_t id,
                           uint16_t address,
                           const uint8_t *data,
                           uint16_t data_len)
{
    uint8_t params[128];

    if (data_len + 2 > sizeof(params)) {
        return ESP_ERR_INVALID_SIZE;
    }

    params[0] = address & 0xFF;
    params[1] = (address >> 8) & 0xFF;

    memcpy(&params[2], data, data_len);

    esp_err_t ret = dxl_send_packet(
        id,
        DXL_INST_WRITE,
        params,
        data_len + 2
    );

    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t status_params[16];
    uint16_t status_param_len = 0;
    uint8_t error = 0;

    ret = dxl_read_status(
        id,
        status_params,
        &status_param_len,
        &error,
        200
    );

    if (ret != ESP_OK) {
        ESP_LOGW(TAG,
                 "Write address %u sent, but no status packet received.",
                 address);

        return ret;
    }

    if (error != 0) {
        ESP_LOGE(TAG,
                 "Write address %u failed. DXL error=0x%02X",
                 address,
                 error);

        return ESP_FAIL;
    }

    return ESP_OK;
}

// =====================================================
// Read
// =====================================================

static esp_err_t dxl_read(uint8_t id,
                          uint16_t address,
                          uint16_t read_len,
                          uint8_t *out)
{
    uint8_t params[4];

    params[0] = address & 0xFF;
    params[1] = (address >> 8) & 0xFF;
    params[2] = read_len & 0xFF;
    params[3] = (read_len >> 8) & 0xFF;

    esp_err_t ret = dxl_send_packet(
        id,
        DXL_INST_READ,
        params,
        4
    );

    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t param_len = 0;
    uint8_t error = 0;

    ret = dxl_read_status(
        id,
        out,
        &param_len,
        &error,
        200
    );

    if (ret != ESP_OK) {
        return ret;
    }

    if (error != 0) {
        ESP_LOGE(TAG,
                 "Read address %u failed. DXL error=0x%02X",
                 address,
                 error);

        return ESP_FAIL;
    }

    if (param_len != read_len) {
        ESP_LOGE(TAG,
                 "Read length mismatch. expected=%u actual=%u",
                 read_len,
                 param_len);

        return ESP_FAIL;
    }

    return ESP_OK;
}

// =====================================================
// XL330 高层控制
// =====================================================

static esp_err_t xl330_set_torque(uint8_t id, bool enable)
{
    uint8_t value = enable ? TORQUE_ON : TORQUE_OFF;

    ESP_LOGI(TAG, "Set Torque = %s", enable ? "ON" : "OFF");

    return dxl_write(
        id,
        ADDR_TORQUE_ENABLE,
        &value,
        1
    );
}

static esp_err_t xl330_set_position_mode(uint8_t id)
{
    uint8_t mode = MODE_POSITION_CONTROL;

    ESP_LOGI(TAG, "Set Operating Mode = Position Control");

    return dxl_write(
        id,
        ADDR_OPERATING_MODE,
        &mode,
        1
    );
}

static esp_err_t xl330_set_goal_position(uint8_t id, uint32_t position)
{
    if (position > 4095) {
        position = 4095;
    }

    uint8_t data[4];

    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    data[2] = (position >> 16) & 0xFF;
    data[3] = (position >> 24) & 0xFF;

    ESP_LOGI(TAG,
             "Set Goal Position = %lu",
             (unsigned long)position);

    return dxl_write(
        id,
        ADDR_GOAL_POSITION,
        data,
        4
    );
}

static esp_err_t xl330_get_present_position(uint8_t id, int32_t *position)
{
    uint8_t data[4];

    esp_err_t ret = dxl_read(
        id,
        ADDR_PRESENT_POSITION,
        4,
        data
    );

    if (ret != ESP_OK) {
        return ret;
    }

    uint32_t raw =
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    *position = (int32_t)raw;

    ESP_LOGI(TAG,
             "Present Position = %ld",
             (long)*position);

    return ESP_OK;
}

// =====================================================
// 主程序
// =====================================================

void app_main(void)
{
    dxl_uart_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "XL330 force send test start");

    while (1) {
        int32_t present_position = 0;

        ESP_LOGI(TAG, "==============================");

        ESP_LOGI(TAG, "STEP 1: Ping");
        dxl_ping(DXL_ID);
        vTaskDelay(pdMS_TO_TICKS(800));

        /*
         * 改 Operating Mode 之前，先 Torque OFF。
         */
        ESP_LOGI(TAG, "STEP 2: Torque OFF");
        xl330_set_torque(DXL_ID, false);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "STEP 3: Position Mode");
        xl330_set_position_mode(DXL_ID);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "STEP 4: Torque ON");
        xl330_set_torque(DXL_ID, true);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "STEP 5: Goal Position 1024");
        xl330_set_goal_position(DXL_ID, 1024);
        vTaskDelay(pdMS_TO_TICKS(1500));

        xl330_get_present_position(DXL_ID, &present_position);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "STEP 6: Goal Position 2048");
        xl330_set_goal_position(DXL_ID, 2048);
        vTaskDelay(pdMS_TO_TICKS(1500));

        xl330_get_present_position(DXL_ID, &present_position);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "STEP 7: Goal Position 3072");
        xl330_set_goal_position(DXL_ID, 3072);
        vTaskDelay(pdMS_TO_TICKS(1500));

        xl330_get_present_position(DXL_ID, &present_position);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}