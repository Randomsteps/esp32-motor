#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "led_strip.h"

// =====================================================
// 用户配置
// =====================================================

#define DXL_UART_NUM        UART_NUM_1
#define DXL_TX_GPIO         4
#define DXL_RX_GPIO         5
#define DXL_BAUDRATE        57600
#define DXL_ID              1
#define DXL_RX_BUF_SIZE     1024

#define WIFI_SSID           "XL330-Wizard"
#define WIFI_PASS           "12345678"
#define WS2812_GPIO         38   // ESP32-S3-DevKitC v1.1

static const char *TAG = "XL330";
static led_strip_handle_t ws2812;
static SemaphoreHandle_t uart_mutex;

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
#define ADDR_LED                 65
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_PRESENT_VELOCITY    128
#define ADDR_PRESENT_CURRENT     126
#define ADDR_HW_ERROR_STATUS     70

#define MODE_CURRENT             0
#define MODE_VELOCITY            1
#define MODE_POSITION            3
#define MODE_EXT_POSITION        4

// =====================================================
// CRC 表
// =====================================================

static const uint16_t crc_table[256] = {
    0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
    0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
    0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
    0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
    0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
    0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
    0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
    0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
    0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
    0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
    0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
    0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
    0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
    0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
    0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
    0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
    0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
    0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
    0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
    0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
    0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
    0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
    0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
    0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
    0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
    0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
    0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
    0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
    0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
    0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
    0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
    0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
};

static uint16_t dxl_update_crc(uint16_t crc_accum, const uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    for (uint16_t j = 0; j < data_blk_size; j++) {
        uint16_t i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

// =====================================================
// UART
// =====================================================

static void dxl_uart_init(void) {
    uart_config_t c = {.baud_rate=DXL_BAUDRATE,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,
                       .stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(DXL_UART_NUM,DXL_RX_BUF_SIZE,0,0,NULL,0));
    ESP_ERROR_CHECK(uart_param_config(DXL_UART_NUM,&c));
    ESP_ERROR_CHECK(uart_set_pin(DXL_UART_NUM,DXL_TX_GPIO,DXL_RX_GPIO,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(DXL_UART_NUM,UART_MODE_UART));
    uart_flush_input(DXL_UART_NUM);
}

static esp_err_t dxl_send(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t plen) {
    uint8_t p[256]; uint16_t idx=0, len=plen+3;
    p[idx++]=0xFF;p[idx++]=0xFF;p[idx++]=0xFD;p[idx++]=0x00;p[idx++]=id;
    p[idx++]=len&0xFF;p[idx++]=len>>8;p[idx++]=inst;
    for(int i=0;i<plen;i++)p[idx++]=params[i];
    uint16_t crc=0;for(int i=0;i<idx;i++)crc=dxl_update_crc(crc,&p[i],1);
    p[idx++]=crc&0xFF;p[idx++]=crc>>8;
    int w=uart_write_bytes(DXL_UART_NUM,(const char*)p,idx);
    uart_wait_tx_done(DXL_UART_NUM,pdMS_TO_TICKS(30));
    return w==idx?ESP_OK:ESP_FAIL;
}

static esp_err_t dxl_recv(uint8_t *id, uint8_t *err, uint8_t *params, uint16_t *plen, int to) {
    uint8_t rx[512]; int total=0;
    TickType_t s=xTaskGetTickCount(), d=pdMS_TO_TICKS(to);
    while((xTaskGetTickCount()-s)<d){
        int n=uart_read_bytes(DXL_UART_NUM,rx+total,sizeof(rx)-total,pdMS_TO_TICKS(10));
        if(n>0){total+=n;
            for(int i=0;i<=total-10;i++){
                if(rx[i]!=0xFF||rx[i+1]!=0xFF||rx[i+2]!=0xFD||rx[i+3]!=0x00)continue;
                uint8_t rid=rx[i+4];uint16_t len=rx[i+5]|(rx[i+6]<<8);int pl=7+len;
                if(pl<10||i+pl>total)continue;
                if(rx[i+7]!=DXL_STATUS_PACKET||rid!=*id)continue;
                uint16_t rc=rx[i+pl-2]|(rx[i+pl-1]<<8),cc=0;
                for(int k=0;k<pl-2;k++)cc=dxl_update_crc(cc,&rx[i+k],1);
                if(rc!=cc)return ESP_FAIL;
                *err=rx[i+8];uint16_t rl=len-4;
                if(plen)*plen=rl;
                if(params&&rl)memcpy(params,rx+i+9,rl);
                *id=rid;return ESP_OK;
            }
            if(total>450)total=0;
        }
    }
    return ESP_ERR_TIMEOUT;
}

// =====================================================
// 高层 API (给 HTTP 用, 已加 UART 锁)
// =====================================================

static bool dl_ping(uint8_t id) {
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return false;
    dxl_send(id,DXL_INST_PING,NULL,0);
    uint8_t rid=id,err=0;uint16_t pl=0;uint8_t p[16];
    bool ok=dxl_recv(&rid,&err,p,&pl,80)==ESP_OK&&err==0;
    xSemaphoreGive(uart_mutex);
    return ok;
}

static esp_err_t dl_write1(uint8_t id,uint16_t addr,uint8_t v) {
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return ESP_ERR_TIMEOUT;
    uint8_t p[]={addr&0xFF,addr>>8,v};
    esp_err_t r=dxl_send(id,DXL_INST_WRITE,p,3);
    if(r==ESP_OK){uint8_t rid=id,err=0;uint16_t pl=0;r=dxl_recv(&rid,&err,NULL,&pl,150);}
    xSemaphoreGive(uart_mutex);
    return r;
}

static esp_err_t dl_write4(uint8_t id,uint16_t addr,uint32_t v) {
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return ESP_ERR_TIMEOUT;
    uint8_t p[]={addr&0xFF,addr>>8,v&0xFF,(v>>8)&0xFF,(v>>16)&0xFF,(v>>24)&0xFF};
    esp_err_t r=dxl_send(id,DXL_INST_WRITE,p,6);
    if(r==ESP_OK){uint8_t rid=id,err=0;uint16_t pl=0;r=dxl_recv(&rid,&err,NULL,&pl,150);}
    xSemaphoreGive(uart_mutex);
    return r;
}

// =====================================================
// 后台高速轮询 (50ms 间隔, 一次批量读)
// =====================================================

static volatile int32_t g_pos = 0;   // Present Position
static volatile int32_t g_vel = 0;   // Present Velocity
static volatile int16_t g_cur = 0;   // Present Current
static volatile int32_t g_goal = 0;  // 上一次发送的目标角度 (deg)

// 一次批量读 10 字节: Current(2) + Velocity(4) + Position(4) = addr 126
static void dl_read_bulk(uint8_t id, int32_t *pos, int32_t *vel, int16_t *cur) {
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(200))!=pdTRUE)return;
    uint8_t p[]={126&0xFF,126>>8,10,0};
    if(dxl_send(id,DXL_INST_READ,p,4)==ESP_OK){
        uint8_t rid=id,err=0,data[12]={0};uint16_t pl=0;
        if(dxl_recv(&rid,&err,data,&pl,80)==ESP_OK&&pl>=10){
            *cur=(int16_t)(data[0]|(data[1]<<8));
            *vel=data[2]|(data[3]<<8)|(data[4]<<16)|(data[5]<<24);
            *pos=data[6]|(data[7]<<8)|(data[8]<<16)|(data[9]<<24);
        }
    }
    xSemaphoreGive(uart_mutex);
}

static void poll_task(void *arg) {
    while(1) {
        int32_t p=0,v=0;int16_t c=0;
        dl_read_bulk(DXL_ID,&p,&v,&c);
        g_pos=p;g_vel=v;g_cur=c;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// 独立恢复任务: 每 3 秒检查一次, 只占锁一瞬间
static void recover_task(void *arg) {
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(500))!=pdTRUE)continue;
        // 读扭矩
        uint8_t tq=0, p1[]={ADDR_TORQUE_ENABLE&0xFF,0,1,0};
        if(dxl_send(DXL_ID,DXL_INST_READ,p1,4)==ESP_OK){
            uint8_t rid=DXL_ID,er=0,d=0;uint16_t pl=0;
            if(dxl_recv(&rid,&er,&d,&pl,80)==ESP_OK)tq=d;
        }
        xSemaphoreGive(uart_mutex);
        // 如果扭矩掉了, 在锁外面恢复 (dl_write1 自己会拿锁)
        if(tq==0){
            ESP_LOGW(TAG,"扭矩被关断, 自动恢复");
            dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,0);
            dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,1);
        }
    }
}

// =====================================================
// WS2812
// =====================================================

static void ws2812_init(void) {
    led_strip_config_t sc={.strip_gpio_num=WS2812_GPIO,.max_leds=1};
    led_strip_rmt_config_t rc={.resolution_hz=10*1000*1000};
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc,&rc,&ws2812));
    led_strip_clear(ws2812);
}
static void ws2812_set(bool on) {
    if(on){led_strip_set_pixel(ws2812,0,30,0,30);}
    else{led_strip_clear(ws2812);}
    led_strip_refresh(ws2812);
}

// =====================================================
// HTTP Server
// =====================================================

static uint8_t get_id(httpd_req_t *r) {
    char b[32],v[8]; httpd_req_get_url_query_str(r,b,sizeof(b));
    if(httpd_query_key_value(b,"id",v,sizeof(v))==ESP_OK)return atoi(v);
    return DXL_ID;
}

static esp_err_t h_ping(httpd_req_t *r) {
    uint8_t id=get_id(r); bool ok=dl_ping(id);
    char m[32]; snprintf(m,sizeof(m),ok?"ID%d OK":"ID%d 无响应",id);
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,m,strlen(m));
    return ESP_OK;
}

static esp_err_t h_scan(httpd_req_t *r) {
    char m[256]="";
    for(int id=0;id<=6;id++) {
        if(dl_ping(id)){char t[32];snprintf(t,sizeof(t),"ID%d ",id);strcat(m,t);}
    }
    if(strlen(m)==0)strcpy(m,"未找到舵机");
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,m,strlen(m));
    return ESP_OK;
}

// 写操作封装: 失败重试一次
static esp_err_t dl_write1_retry(uint8_t id,uint16_t addr,uint8_t v){
    if(dl_write1(id,addr,v)==ESP_OK)return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(30));
    return dl_write1(id,addr,v);
}
static esp_err_t dl_write4_retry(uint8_t id,uint16_t addr,uint32_t v){
    if(dl_write4(id,addr,v)==ESP_OK)return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(30));
    return dl_write4(id,addr,v);
}

static esp_err_t h_torque(httpd_req_t *r) {
    char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int on=0;if(httpd_query_key_value(b,"on",v,sizeof(v))==ESP_OK)on=atoi(v);
    esp_err_t ret=dl_write1_retry(get_id(r),ADDR_TORQUE_ENABLE,on?1:0);
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,ret==ESP_OK?(on?"ON":"OFF"):"FAIL",ret==ESP_OK?2:4);
    return ESP_OK;
}

static esp_err_t h_mode(httpd_req_t *r) {
    char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int m=3;if(httpd_query_key_value(b,"v",v,sizeof(v))==ESP_OK)m=atoi(v);
    uint8_t id=get_id(r);
    dl_write1_retry(id,ADDR_TORQUE_ENABLE,0);
    dl_write1_retry(id,ADDR_OPERATING_MODE,m);
    dl_write1_retry(id,ADDR_TORQUE_ENABLE,1);
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,"OK",2);
    return ESP_OK;
}

static esp_err_t h_pos(httpd_req_t *r) {
    char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int deg=0;if(httpd_query_key_value(b,"deg",v,sizeof(v))==ESP_OK)deg=atoi(v);
    if(deg<0||deg>360){httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"0-360");return ESP_FAIL;}
    g_goal=deg;
    esp_err_t ret=dl_write4_retry(get_id(r),ADDR_GOAL_POSITION,(uint32_t)(deg/360.0*4096));
    char m[32];snprintf(m,sizeof(m),ret==ESP_OK?"%d°":"%d° 重试失败",deg);
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,m,strlen(m));
    return ESP_OK;
}

static esp_err_t h_led(httpd_req_t *r) {
    char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int on=0;if(httpd_query_key_value(b,"on",v,sizeof(v))==ESP_OK)on=atoi(v);
    esp_err_t ret=dl_write1_retry(get_id(r),ADDR_LED,on?1:0);
    httpd_resp_set_type(r,"text/plain"); httpd_resp_send(r,ret==ESP_OK?(on?"ON":"OFF"):"FAIL",ret==ESP_OK?2:4);
    return ESP_OK;
}

// 瞬返 /read (数据由后台 poll_task 持续更新)
static esp_err_t h_read(httpd_req_t *r) {
    int32_t p=g_pos, v=g_vel; int16_t c=g_cur;
    char j[160];snprintf(j,sizeof(j),"{\"pos\":%ld,\"vel\":%ld,\"cur\":%d,\"goal\":%ld}",
                         (long)p,(long)v,(int)(c*269/100),(long)g_goal);
    httpd_resp_set_type(r,"application/json");httpd_resp_send(r,j,strlen(j));
    return ESP_OK;
}

// =====================================================
// HTML 页面
// =====================================================

static const char *HTML =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>XL330</title><style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{background:#1a1a2e;color:#cdd;font:14px/1.5 'Segoe UI',Arial,sans-serif;padding:12px;max-width:420px;margin:auto}"
".hdr{display:flex;align-items:center;gap:10px;margin-bottom:10px}"
".hdr h1{font-size:18px;color:#a78bfa;flex:1}"
".hdr button{padding:6px 12px;border:none;border-radius:4px;font-size:11px;font-weight:bold;cursor:pointer;color:#fff;background:#3b82f6}"
".hdr button:active{opacity:.8}"
".card{background:#16213e;border-radius:8px;padding:12px;margin-bottom:8px}"
".card h3{font-size:13px;color:#7c3aed;margin-bottom:8px;border-bottom:1px solid #2a2a3e;padding-bottom:5px}"
".row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-bottom:6px}"
"button{color:#fff;border:none;border-radius:4px;padding:8px 12px;font-size:13px;font-weight:bold;cursor:pointer}"
"button:active{opacity:.8}"
"#cur-pos{font-size:36px;font-weight:bold;color:#a78bfa;text-align:center;padding:8px 0}"
"#cur-deg{font-size:16px;color:#888;text-align:center}"
".big{font-size:16px;padding:10px 20px}"
".b-go{background:#7c3aed;flex:1}.b-go2{background:#7c3aed;width:100%}"
".b-step{background:#2a2a3e;min-width:40px;font-size:18px}"
".b-num{background:#3b82f6}"
".b-red{background:#ef4444}.b-grn{background:#10b981}.b-yel{background:#f59e0b}"
".tgl{width:40px;height:22px;background:#374151;border-radius:11px;position:relative;cursor:pointer;display:inline-block;vertical-align:middle;transition:.2s}"
".tgl.on{background:#10b981}"
".tgl::after{content:'';width:18px;height:18px;background:#fff;border-radius:50%;position:absolute;top:2px;left:2px;transition:.2s}"
".tgl.on::after{left:20px}"
"select,input[type=number]{background:#2a2a3e;color:#fff;border:1px solid #444;padding:6px 8px;border-radius:4px;font-size:13px}"
"input[type=number]{width:70px;text-align:center}"
".d-row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #1e2d4a;font-size:13px}"
".d-row .l{color:#888}.d-row .v{color:#a78bfa;font-weight:bold}"
".ft{margin-top:10px;color:#666;font-size:11px;text-align:center}"
"</style></head><body>"
//顶栏
"<div class='hdr'><h1>XL330 舵机控制</h1>"
"<button onclick='api(\"/scan\")'>扫描</button></div>"
//当前位置
"<div class='card'><h3>当前位置</h3>"
"<div id='cur-pos'>--</div><div id='cur-deg'>等待数据...</div></div>"
//目标控制
"<div class='card'><h3>目标角度</h3>"
"<div class='row'>"
"<button class='b-step' onclick='adj(-10)'>-10°</button>"
"<button class='b-step' onclick='adj(-1)'>-1°</button>"
"<input type='number' id='goal' value='180' min='0' max='360' style='font-size:22px;width:80px;flex:1'>"
"<button class='b-step' onclick='adj(1)'>+1°</button>"
"<button class='b-step' onclick='adj(10)'>+10°</button></div>"
"<div class='row' style='margin-top:6px'>"
"<button class='b-num' onclick='setGoal(0)'>0°</button>"
"<button class='b-num' onclick='setGoal(90)'>90°</button>"
"<button class='b-num' onclick='setGoal(180)'>180°</button>"
"<button class='b-num' onclick='setGoal(270)'>270°</button></div>"
"<button class='b-go2' onclick='setGoal(parseInt(document.getElementById(\"goal\").value))'>转到目标角度</button></div>"
//扭矩+模式+LED
"<div class='card'><h3>开关</h3><div class='row'>"
"<label>扭矩</label><span class='tgl' id='t-torque' onclick='toggleT(this)'></span>"
"<label style='margin-left:10px'>模式</label><select id='mode' onchange='api(\"/mode?v=\"+this.value)'>"
"<option value='3'>位置</option><option value='4'>扩展位置</option><option value='1'>速度</option><option value='0'>电流</option></select>"
"<button class='b-yel' style='margin-left:auto' onclick='api(\"/sled?on=\"+(this.textContent==\"LED开\"?0:1));this.textContent=this.textContent==\"LED开\"?\"LED关\":\"LED开\"'>LED关</button>"
"</div></div>"
//实时数据
"<div class='card'><h3>实时数据</h3>"
"<div class='d-row'><span class='l'>位置</span><span class='v' id='v-pos'>--</span></div>"
"<div class='d-row'><span class='l'>速度</span><span class='v' id='v-vel'>--</span></div>"
"<div class='d-row'><span class='l'>电流</span><span class='v' id='v-cur'>--</span></div></div>"
"<div class='ft'>WiFi: " WIFI_SSID " | ID:1 | 57600</div>"
"<script>"
"var CID=1,curDeg=0;"
"function api(u){"
"u+=(u.includes('?')?'&':'?')+'id='+CID;"
"fetch(u).then(r=>r.text()).then(t=>{document.querySelector('.ft').textContent=t.substring(0,40);}).catch(()=>{});}"
"function toggleT(el){var on=el.classList.contains('on');el.classList.toggle('on');api('/torque?on='+(on?0:1));}"
"function setGoal(deg){deg=Math.max(0,Math.min(360,deg));document.getElementById('goal').value=deg;api('/pos?deg='+deg);}"
"function adj(d){setGoal(parseInt(document.getElementById('goal').value)+d);}"
"function update(){fetch('/read?id='+CID).then(r=>r.json()).then(j=>{"
"if(j.pos>=0){curDeg=j.pos/4096*360;"
"document.getElementById('cur-pos').textContent=j.pos;"
"document.getElementById('cur-deg').textContent=curDeg.toFixed(1)+'° / '+(360-curDeg).toFixed(1)+'°';"
"document.getElementById('v-pos').textContent=j.pos+' ('+curDeg.toFixed(1)+'°)';"
"document.getElementById('v-vel').textContent=j.vel;"
"document.getElementById('v-cur').textContent=j.cur+' mA';"
"}}).catch(()=>{});}"
"setInterval(update,300);update();"
"</script></body></html>";

static esp_err_t h_root(httpd_req_t *r) {
    httpd_resp_set_type(r,"text/html; charset=utf-8");
    httpd_resp_send(r,HTML,strlen(HTML));
    return ESP_OK;
}

// =====================================================
// 初始化
// =====================================================

static void wifi_init(void) {
    nvs_flash_init();esp_netif_init();esp_event_loop_create_default();esp_netif_create_default_wifi_ap();
    wifi_init_config_t wc=WIFI_INIT_CONFIG_DEFAULT();esp_wifi_init(&wc);
    wifi_config_t ap={.ap={.ssid=WIFI_SSID,.ssid_len=strlen(WIFI_SSID),.password=WIFI_PASS,
                     .channel=1,.authmode=WIFI_AUTH_WPA2_PSK,.max_connection=4}};
    esp_wifi_set_mode(WIFI_MODE_AP);esp_wifi_set_config(WIFI_IF_AP,&ap);esp_wifi_start();
    ESP_LOGI(TAG,"WiFi: %s -> http://192.168.4.1",WIFI_SSID);
}

static void http_init(void) {
    httpd_handle_t s=NULL;httpd_config_t c=HTTPD_DEFAULT_CONFIG();httpd_start(&s,&c);
    httpd_uri_t u[]={
        {.uri="/",.method=HTTP_GET,.handler=h_root},{.uri="/ping",.method=HTTP_GET,.handler=h_ping},
        {.uri="/scan",.method=HTTP_GET,.handler=h_scan},{.uri="/torque",.method=HTTP_GET,.handler=h_torque},
        {.uri="/mode",.method=HTTP_GET,.handler=h_mode},{.uri="/pos",.method=HTTP_GET,.handler=h_pos},
        {.uri="/sled",.method=HTTP_GET,.handler=h_led},{.uri="/read",.method=HTTP_GET,.handler=h_read},
    };
    for(int i=0;i<sizeof(u)/sizeof(u[0]);i++)httpd_register_uri_handler(s,&u[i]);
}

// =====================================================
// 主程序
// =====================================================

void app_main(void) {
    uart_mutex = xSemaphoreCreateMutex();
    ws2812_init();
    for(int i=0;i<3;i++){ws2812_set(1);vTaskDelay(pdMS_TO_TICKS(80));ws2812_set(0);vTaskDelay(pdMS_TO_TICKS(80));}
    dxl_uart_init();
    ESP_LOGI(TAG,"UART ready. TX=GPIO%d RX=GPIO%d",DXL_TX_GPIO,DXL_RX_GPIO);

    // 初始化舵机
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,0);
    dl_write1(DXL_ID,ADDR_OPERATING_MODE,MODE_POSITION);
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,1);
    ESP_LOGI(TAG,"Servo ID%d initialized",DXL_ID);

    xTaskCreate(poll_task,"poll",4096,NULL,5,NULL);
    xTaskCreate(recover_task,"recov",3072,NULL,1,NULL);

    wifi_init();http_init();
    ws2812_set(1);
    ESP_LOGI(TAG,"WiFi: %s 密码: %s",WIFI_SSID,WIFI_PASS);
    ESP_LOGI(TAG,"浏览器打开 http://192.168.4.1");
}
