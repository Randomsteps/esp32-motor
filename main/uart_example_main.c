#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

#include "driver/uart.h"

#include "led_strip.h"

// =====================================================
// 用户配置
// =====================================================

#define WIFI_SSID           "Redmi K70"
#define WIFI_PASS           "aaaaaaaa"

#define DXL_UART_NUM        UART_NUM_1
#define DXL_TX_GPIO         4
#define DXL_RX_GPIO         5
#define DXL_BAUDRATE        57600
#define DXL_ID              1
#define DXL_RX_BUF_SIZE     1024

#define WS2812_GPIO         38

static const char *TAG = "XL330";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static led_strip_handle_t ws2812 = NULL;
static char g_ip_str[32] = "0.0.0.0";
static SemaphoreHandle_t uart_mutex;

// =====================================================
// WS2812
// =====================================================

static void ws2812_init(void) {
    led_strip_config_t sc = {.strip_gpio_num=WS2812_GPIO,.max_leds=1};
    led_strip_rmt_config_t rc = {.resolution_hz=10*1000*1000};
    esp_err_t ret = led_strip_new_rmt_device(&sc,&rc,&ws2812);
    if(ret==ESP_OK){led_strip_clear(ws2812);ESP_LOGI(TAG,"WS2812 OK");}
    else{ESP_LOGW(TAG,"WS2812 fail");ws2812=NULL;}
}
static void ws2812_rgb(uint8_t r,uint8_t g,uint8_t b) {
    if(!ws2812)return;
    led_strip_set_pixel(ws2812,0,r,g,b);led_strip_refresh(ws2812);
}
// Dynamixel Protocol 2.0
// =====================================================

#define DXL_INST_PING       0x01
#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03
#define DXL_STATUS           0x55

#define ADDR_OPERATING_MODE      11
#define ADDR_TORQUE_ENABLE       64
#define ADDR_LED                 65
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_PRESENT_VELOCITY    128
#define ADDR_PRESENT_CURRENT     126

#define MODE_POSITION            3

static const uint16_t crc16[256]={
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
    0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202};

static uint16_t crc_up(uint16_t c,const uint8_t*d,uint16_t n){
    for(uint16_t j=0;j<n;j++){uint16_t i=((c>>8)^d[j])&0xFF;c=(c<<8)^crc16[i];}
    return c;
}

// =====================================================
// UART
// =====================================================

static void dxl_uart_init(void) {
    uart_config_t c={.baud_rate=DXL_BAUDRATE,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,
                     .stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(DXL_UART_NUM,DXL_RX_BUF_SIZE,0,0,NULL,0));
    ESP_ERROR_CHECK(uart_param_config(DXL_UART_NUM,&c));
    ESP_ERROR_CHECK(uart_set_pin(DXL_UART_NUM,DXL_TX_GPIO,DXL_RX_GPIO,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(DXL_UART_NUM,UART_MODE_UART));
    uart_flush_input(DXL_UART_NUM);
}

static esp_err_t dxl_send(uint8_t id,uint8_t inst,const uint8_t*params,uint16_t plen){
    uint8_t p[256];uint16_t idx=0,len=plen+3;
    p[idx++]=0xFF;p[idx++]=0xFF;p[idx++]=0xFD;p[idx++]=0;p[idx++]=id;
    p[idx++]=len&0xFF;p[idx++]=len>>8;p[idx++]=inst;
    for(int i=0;i<plen;i++)p[idx++]=params[i];
    uint16_t crc=0;for(int i=0;i<idx;i++)crc=crc_up(crc,&p[i],1);
    p[idx++]=crc&0xFF;p[idx++]=crc>>8;
    int w=uart_write_bytes(DXL_UART_NUM,(const char*)p,idx);
    uart_wait_tx_done(DXL_UART_NUM,pdMS_TO_TICKS(30));
    return w==idx?ESP_OK:ESP_FAIL;
}

static esp_err_t dxl_recv(uint8_t*id,uint8_t*err,uint8_t*params,uint16_t*plen,int to){
    uint8_t rx[512];int t=0;TickType_t s=xTaskGetTickCount(),d=pdMS_TO_TICKS(to);
    while((xTaskGetTickCount()-s)<d){
        int n=uart_read_bytes(DXL_UART_NUM,rx+t,sizeof(rx)-t,pdMS_TO_TICKS(10));
        if(n>0){t+=n;
            for(int i=0;i<=t-10;i++){
                if(rx[i]!=0xFF||rx[i+1]!=0xFF||rx[i+2]!=0xFD||rx[i+3]!=0)continue;
                uint8_t rid=rx[i+4];uint16_t len=rx[i+5]|(rx[i+6]<<8);int pl=7+len;
                if(pl<10||i+pl>t)continue;
                if(rx[i+7]!=DXL_STATUS||rid!=*id)continue;
                uint16_t rc=rx[i+pl-2]|(rx[i+pl-1]<<8),cc=0;
                for(int k=0;k<pl-2;k++)cc=crc_up(cc,&rx[i+k],1);
                if(rc!=cc)return ESP_FAIL;
                *err=rx[i+8];
                uint16_t rl=len-4;
                if(plen) *plen=rl;
                if(params&&rl) memcpy(params,rx+i+9,rl);
                *id=rid;
                return ESP_OK;
            }
            if(t>450)t=0;
        }
    }
    return ESP_ERR_TIMEOUT;
}

// =====================================================
// 高层 API (带锁)
// =====================================================

static bool dl_ping(uint8_t id){
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return false;
    dxl_send(id,DXL_INST_PING,NULL,0);
    uint8_t rid=id,err=0;uint16_t pl=0;uint8_t p[16];
    bool ok=dxl_recv(&rid,&err,p,&pl,80)==ESP_OK&&err==0;
    xSemaphoreGive(uart_mutex);return ok;
}

static esp_err_t dl_write1(uint8_t id,uint16_t addr,uint8_t v){
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return ESP_ERR_TIMEOUT;
    uint8_t p[]={addr&0xFF,addr>>8,v};
    esp_err_t r=dxl_send(id,DXL_INST_WRITE,p,3);
    if(r==ESP_OK){uint8_t rid=id,err=0;uint16_t pl=0;r=dxl_recv(&rid,&err,NULL,&pl,150);}
    xSemaphoreGive(uart_mutex);return r;
}

static esp_err_t dl_write4(uint8_t id,uint16_t addr,uint32_t v){
    if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(800))!=pdTRUE)return ESP_ERR_TIMEOUT;
    uint8_t p[]={addr&0xFF,addr>>8,v&0xFF,(v>>8)&0xFF,(v>>16)&0xFF,(v>>24)&0xFF};
    esp_err_t r=dxl_send(id,DXL_INST_WRITE,p,6);
    if(r==ESP_OK){uint8_t rid=id,err=0;uint16_t pl=0;r=dxl_recv(&rid,&err,NULL,&pl,150);}
    xSemaphoreGive(uart_mutex);return r;
}

// 批量读: Current(2)+Vel(4)+Pos(4) = 10 bytes from addr 126
static void dl_read_bulk(uint8_t id,int32_t*pos,int32_t*vel,int16_t*cur){
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

// =====================================================
// 后台任务
// =====================================================

static volatile int32_t g_pos=0,g_vel=0,g_goal=0;
static volatile int16_t g_cur=0;

static void poll_task(void*arg){
    while(1){int32_t p=0,v=0;int16_t c=0;dl_read_bulk(DXL_ID,&p,&v,&c);g_pos=p;g_vel=v;g_cur=c;vTaskDelay(pdMS_TO_TICKS(50));}
}

static void recover_task(void*arg){
    while(1){
        vTaskDelay(pdMS_TO_TICKS(3000));
        if(xSemaphoreTake(uart_mutex,pdMS_TO_TICKS(500))!=pdTRUE)continue;
        uint8_t tq=0;bool ok=false;
        uint8_t p1[]={ADDR_TORQUE_ENABLE&0xFF,0,1,0};
        if(dxl_send(DXL_ID,DXL_INST_READ,p1,4)==ESP_OK){
            uint8_t rid=DXL_ID,er=0,d=0;uint16_t pl=0;
            if(dxl_recv(&rid,&er,&d,&pl,80)==ESP_OK){tq=d;ok=true;}
        }
        xSemaphoreGive(uart_mutex);
        if(ok&&tq==0){ESP_LOGW(TAG,"扭矩关断,自动恢复");dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,0);dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,1);}
    }
}

// =====================================================
// HTTP 页面
// =====================================================

static const char *HTML=
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>XL330</title><style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{background:#111827;color:#e5e7eb;font:14px/1.5 'Segoe UI',Arial,sans-serif;padding:12px;max-width:420px;margin:auto}"
".hdr{display:flex;align-items:center;gap:8px;margin-bottom:8px}"
".hdr h1{font-size:18px;color:#a78bfa;flex:1}"
".hdr span{font-size:11px;color:#666}.hdr button{padding:5px 10px;border:none;border-radius:4px;font-size:11px;font-weight:bold;cursor:pointer;color:#fff;background:#3b82f6}"
".card{background:#1f2937;border-radius:8px;padding:12px;margin-bottom:8px}"
".card h3{font-size:12px;color:#7c3aed;margin-bottom:6px;border-bottom:1px solid #374151;padding-bottom:4px}"
".row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-bottom:4px}"
"button{color:#fff;border:none;border-radius:4px;padding:8px 12px;font-size:13px;font-weight:bold;cursor:pointer}"
"button:active{opacity:.8}"
"#pos{font-size:38px;font-weight:bold;color:#a78bfa;text-align:center;padding:6px 0}"
"#deg{font-size:16px;color:#888;text-align:center}"
".b-go{background:#7c3aed;width:100%}.b-num{background:#3b82f6}.b-step{background:#374151;min-width:40px;font-size:16px}"
".b-yel{background:#f59e0b}"
".tgl{width:40px;height:22px;background:#374151;border-radius:11px;position:relative;cursor:pointer;display:inline-block;vertical-align:middle;transition:.2s}"
".tgl.on{background:#10b981}"
".tgl::after{content:'';width:18px;height:18px;background:#fff;border-radius:50%;position:absolute;top:2px;left:2px;transition:.2s}"
".tgl.on::after{left:20px}"
"input[type=number]{background:#374151;color:#fff;border:1px solid #444;padding:6px;border-radius:4px;font-size:13px;width:70px;text-align:center}"
"label{font-size:12px;color:#9ca3af}"
".dr{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #1e293b;font-size:13px}"
".dr .l{color:#9ca3af}.dr .v{color:#a78bfa;font-weight:bold}"
".ft{margin-top:8px;font-size:11px;color:#555;text-align:center}"
"</style></head><body>"
"<div class='hdr'><h1>XL330 舵机</h1><span id='wifi-st'>--</span><button onclick='api(\"/scan\")'>扫描</button></div>"
"<div class='card'><h3>当前位置</h3><div id='pos'>--</div><div id='deg'>等待...</div></div>"
"<div class='card'><h3>目标角度</h3><div class='row'>"
"<button class='b-step' onclick='adj(-10)'>-10°</button><button class='b-step' onclick='adj(-1)'>-1°</button>"
"<input type='number' id='goal' value='180' min='0' max='360'>"
"<button class='b-step' onclick='adj(1)'>+1°</button><button class='b-step' onclick='adj(10)'>+10°</button></div>"
"<div class='row' style='margin-top:4px'><button class='b-num' onclick='setG(0)'>0°</button><button class='b-num' onclick='setG(90)'>90°</button>"
"<button class='b-num' onclick='setG(180)'>180°</button><button class='b-num' onclick='setG(270)'>270°</button></div>"
"<button class='b-go' onclick='setG(parseInt(document.getElementById(\"goal\").value))'>转到目标角度</button></div>"
"<div class='card'><h3>开关</h3><div class='row'>"
"<label>扭矩</label><span class='tgl' onclick='toggleT(this)'></span>"
"<label style='margin-left:8px'>模式</label><select id='mode' onchange='api(\"/mode?v=\"+this.value)'><option value='3'>位置</option><option value='4'>扩展位置</option><option value='1'>速度</option></select>"
"<button class='b-yel' style='margin-left:auto' onclick='var b=this;api(\"/sled?on=\"+(b.textContent==\"LED开\"?0:1));b.textContent=b.textContent==\"LED开\"?\"LED关\":\"LED开\"'>LED关</button>"
"</div></div>"
"<div class='card'><h3>实时数据</h3>"
"<div class='dr'><span class='l'>位置</span><span class='v' id='v-pos'>--</span></div>"
"<div class='dr'><span class='l'>速度</span><span class='v' id='v-vel'>--</span></div>"
"<div class='dr'><span class='l'>电流</span><span class='v' id='v-cur'>--</span></div></div>"
"<div class='ft' id='status'>就绪</div>"
"<script>"
"var CID=1;"
"function api(u){u+=(u.includes('?')?'&':'?')+'id='+CID;return fetch(u).then(r=>r.text());}"
"function toggleT(el){var on=el.classList.contains('on');el.classList.toggle('on');api('/torque?on='+(on?0:1));}"
"function setG(d){d=Math.max(0,Math.min(360,d));document.getElementById('goal').value=d;api('/pos?deg='+d).then(t=>{document.getElementById('status').textContent=t;})}"
"function adj(d){setG(parseInt(document.getElementById('goal').value)+d);}"
"function up(){fetch('/read?id='+CID).then(r=>r.json()).then(j=>{if(j.pos>=0){var d=j.pos/4096*360;"
"document.getElementById('pos').textContent=j.pos;document.getElementById('deg').textContent=d.toFixed(1)+'° / '+(360-d).toFixed(1)+'°';"
"document.getElementById('v-pos').textContent=j.pos+' ('+d.toFixed(1)+'°)';"
"document.getElementById('v-vel').textContent=j.vel;document.getElementById('v-cur').textContent=j.cur+' mA';"
"document.getElementById('wifi-st').textContent='在线';}}).catch(()=>{document.getElementById('wifi-st').textContent='离线';});}"
"setInterval(up,300);up();"
"</script></body></html>";

// =====================================================
// HTTP 端点
// =====================================================

static void cors(httpd_req_t*r){
    httpd_resp_set_hdr(r,"Access-Control-Allow-Origin","*");
}

static esp_err_t h_root(httpd_req_t*r){cors(r);httpd_resp_set_type(r,"text/html; charset=utf-8");httpd_resp_send(r,HTML,strlen(HTML));return ESP_OK;}

static esp_err_t h_info(httpd_req_t*r){
    cors(r);char j[200];
    snprintf(j,sizeof(j),"{\"name\":\"XL330-%d\",\"ip\":\"%s\",\"pos\":%ld,\"vel\":%ld,\"cur\":%d}",
             DXL_ID,g_ip_str,(long)g_pos,(long)g_vel,(int)(g_cur*269/100));
    httpd_resp_set_type(r,"application/json");httpd_resp_send(r,j,strlen(j));return ESP_OK;
}

static esp_err_t h_ping(httpd_req_t*r){cors(r);httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,"OK",2);return ESP_OK;}

static esp_err_t h_scan(httpd_req_t*r){
    cors(r);char m[256]="";for(int id=0;id<=6;id++)if(dl_ping(id)){char t[32];snprintf(t,sizeof(t),"ID%d ",id);strcat(m,t);}
    if(strlen(m)==0)strcpy(m,"未找到舵机");
    httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,m,strlen(m));return ESP_OK;
}

static esp_err_t h_torque(httpd_req_t*r){
    cors(r);char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int on=0;if(httpd_query_key_value(b,"on",v,sizeof(v))==ESP_OK)on=atoi(v);
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,on?1:0);
    httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,on?"ON":"OFF",2);return ESP_OK;
}

static esp_err_t h_mode(httpd_req_t*r){
    cors(r);char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int m=3;if(httpd_query_key_value(b,"v",v,sizeof(v))==ESP_OK)m=atoi(v);
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,0);dl_write1(DXL_ID,ADDR_OPERATING_MODE,m);dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,1);
    httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,"OK",2);return ESP_OK;
}

static esp_err_t h_pos(httpd_req_t*r){
    cors(r);char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int deg=0;if(httpd_query_key_value(b,"deg",v,sizeof(v))==ESP_OK)deg=atoi(v);
    if(deg<0||deg>360){httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"0-360");return ESP_FAIL;}
    g_goal=deg;dl_write4(DXL_ID,ADDR_GOAL_POSITION,(uint32_t)(deg/360.0*4096));
    char m[32];snprintf(m,sizeof(m),"%d°",deg);
    httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,m,strlen(m));return ESP_OK;
}

static esp_err_t h_led(httpd_req_t*r){
    cors(r);char b[32],v[8];httpd_req_get_url_query_str(r,b,sizeof(b));
    int on=0;if(httpd_query_key_value(b,"on",v,sizeof(v))==ESP_OK)on=atoi(v);
    dl_write1(DXL_ID,ADDR_LED,on?1:0);
    httpd_resp_set_type(r,"text/plain");httpd_resp_send(r,on?"ON":"OFF",2);return ESP_OK;
}

static esp_err_t h_read(httpd_req_t*r){
    cors(r);int32_t p=g_pos,v=g_vel;int16_t c=g_cur;
    char j[160];snprintf(j,sizeof(j),"{\"pos\":%ld,\"vel\":%ld,\"cur\":%d,\"goal\":%ld}",(long)p,(long)v,(int)(c*269/100),(long)g_goal);
    httpd_resp_set_type(r,"application/json");httpd_resp_send(r,j,strlen(j));return ESP_OK;
}

// =====================================================
// HTTP 初始化
// =====================================================

static void http_init(void){
    httpd_handle_t s=NULL;httpd_config_t c=HTTPD_DEFAULT_CONFIG();
    c.lru_purge_enable=true;c.max_uri_handlers=16;httpd_start(&s,&c);
    httpd_uri_t u[]={
        {.uri="/",.method=HTTP_GET,.handler=h_root},{.uri="/info",.method=HTTP_GET,.handler=h_info},
        {.uri="/ping",.method=HTTP_GET,.handler=h_ping},{.uri="/scan",.method=HTTP_GET,.handler=h_scan},
        {.uri="/torque",.method=HTTP_GET,.handler=h_torque},{.uri="/mode",.method=HTTP_GET,.handler=h_mode},
        {.uri="/pos",.method=HTTP_GET,.handler=h_pos},{.uri="/sled",.method=HTTP_GET,.handler=h_led},
        {.uri="/read",.method=HTTP_GET,.handler=h_read},
    };
    for(int i=0;i<sizeof(u)/sizeof(u[0]);i++)httpd_register_uri_handler(s,&u[i]);
    ESP_LOGI(TAG,"HTTP server started");
}

// =====================================================
// WiFi STA
// =====================================================

static void wifi_event_handler(void*arg,esp_event_base_t eb,int32_t id,void*data){
    if(eb==WIFI_EVENT&&id==WIFI_EVENT_STA_START){esp_wifi_connect();}
    else if(eb==WIFI_EVENT&&id==WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGW(TAG,"WiFi断开,重连中...");snprintf(g_ip_str,sizeof(g_ip_str),"0.0.0.0");
        xEventGroupClearBits(wifi_event_group,WIFI_CONNECTED_BIT);
        ws2812_rgb(30,0,0);vTaskDelay(pdMS_TO_TICKS(1000));esp_wifi_connect();
    }else if(eb==IP_EVENT&&id==IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t*ev=(ip_event_got_ip_t*)data;
        snprintf(g_ip_str,sizeof(g_ip_str),IPSTR,IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG,"Got IP: %s",g_ip_str);
        xEventGroupSetBits(wifi_event_group,WIFI_CONNECTED_BIT);
        ws2812_rgb(0,30,0);
    }
}

static void wifi_init(void){
    esp_err_t r=nvs_flash_init();
    if(r==ESP_ERR_NVS_NO_FREE_PAGES||r==ESP_ERR_NVS_NEW_VERSION_FOUND){nvs_flash_erase();nvs_flash_init();}
    esp_netif_init();esp_event_loop_create_default();
    wifi_event_group=xEventGroupCreate();esp_netif_create_default_wifi_sta();
    wifi_init_config_t wc=WIFI_INIT_CONFIG_DEFAULT();esp_wifi_init(&wc);
    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_handler,NULL,NULL);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event_handler,NULL,NULL);
    wifi_config_t sc={.sta={.ssid=WIFI_SSID,.password=WIFI_PASS,.threshold.authmode=WIFI_AUTH_OPEN}};
    esp_wifi_set_mode(WIFI_MODE_STA);esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_config(WIFI_IF_STA,&sc);esp_wifi_start();
    ESP_LOGI(TAG,"连接WiFi: %s",WIFI_SSID);
    xEventGroupWaitBits(wifi_event_group,WIFI_CONNECTED_BIT,pdFALSE,pdTRUE,pdMS_TO_TICKS(20000));
}

// =====================================================
// 主程序
// =====================================================

void app_main(void){
    ESP_LOGI(TAG,"App start");
    uart_mutex=xSemaphoreCreateMutex();
    ws2812_init();ws2812_rgb(0,0,30);

    dxl_uart_init();
    ESP_LOGI(TAG,"UART: TX=GPIO%d RX=GPIO%d",DXL_TX_GPIO,DXL_RX_GPIO);

    // 初始化舵机
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,0);
    dl_write1(DXL_ID,ADDR_OPERATING_MODE,MODE_POSITION);
    dl_write1(DXL_ID,ADDR_TORQUE_ENABLE,1);
    ESP_LOGI(TAG,"舵机 ID%d 初始化完成",DXL_ID);

    xTaskCreate(poll_task,"poll",4096,NULL,5,NULL);
    xTaskCreate(recover_task,"recov",3072,NULL,1,NULL);

    wifi_init();http_init();

    ESP_LOGI(TAG,"就绪! http://%s/",g_ip_str);
}
