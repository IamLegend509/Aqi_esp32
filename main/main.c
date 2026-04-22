/*
 * main.c — AQI ESP32 with WiFi + WebSocket server
 *
 * - Serves a single-page dashboard over HTTP GET /
 * - Streams sensor JSON over WebSocket at ws://<IP>/ws
 * - All sensor tasks write to g_aqi (mutex-guarded)
 * - Serial output is UNCHANGED — sensors still printf as before
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "adc_shared.h"
#include "dht22.h"
#include "bmp280.h"
#include "mq7.h"
#include "mq135.h"

/* ── WiFi credentials ──────────────────────────────────── */
#define WIFI_SSID   "delta_virus_2.4G"
#define WIFI_PASS   "66380115"
#define MAX_RETRY   5

/* ── Tags ───────────────────────────────────────────────── */
static const char *TAG_WIFI = "WIFI";
static const char *TAG_WS   = "WS";

/* ── WiFi event group ───────────────────────────────────── */
static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry = 0;

/* ══════════════════════════════════════════════════════════
 *  SHARED SENSOR DATA
 * ══════════════════════════════════════════════════════════ */
typedef struct {
    /* DHT22 */
    float   dht_temp;
    float   dht_hum;
    float   dht_hi;
    char    dht_comfort[16];

    /* BMP280 */
    float   bmp_temp;
    float   bmp_pres;
    float   bmp_alt;

    /* MQ7 */
    int     mq7_raw;
    float   mq7_gpio_v;
    float   mq7_aout_v;
    float   mq7_rs;
    float   mq7_ratio;
    int     mq7_warmup_s;

    /* MQ135 */
    int     mq135_raw;
    float   mq135_gpio_v;
    float   mq135_aout_v;
    float   mq135_rs;
    float   mq135_ratio;
    float   mq135_co2;
    float   mq135_co;
    float   mq135_nh3;
    float   mq135_alc;
    float   mq135_benz;
    float   mq135_smoke;
    int     mq135_warmup_s;
} aqi_data_t;

static aqi_data_t    g_aqi   = {0};
static SemaphoreHandle_t g_mutex = NULL;

/* ── Convenience macro for sensor tasks ─────────────────── */
#define AQI_LOCK()   xSemaphoreTake(g_mutex, portMAX_DELAY)
#define AQI_UNLOCK() xSemaphoreGive(g_mutex)

/* ══════════════════════════════════════════════════════════
 *  SENSOR TASK WRAPPERS
 *  These replace the sensor module's own tasks.
 *  They call the same read logic but also update g_aqi.
 *  Serial output is produced by the sensor modules — untouched.
 * ══════════════════════════════════════════════════════════ */

/* Forward-declare the per-sensor read helpers that already
 * exist inside each component (they are called by each
 * component's own _task). We shadow those tasks here so we
 * can intercept the data.
 *
 * If your component exposes a read function, call it here.
 * If it only has a task, duplicate the read block below.
 * Adjust field names to match your actual component API.   */

/* ── DHT22 wrapper ─────────────────────────────────────── */
extern float dht22_last_temp;
extern float dht22_last_hum;
extern float dht22_last_hi;
extern char  dht22_last_comfort[16];

void dht22_wrapper_task(void *arg)
{
    /* Let the real task run — we just mirror its globals */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3500));
        AQI_LOCK();
        g_aqi.dht_temp = dht22_last_temp;
        g_aqi.dht_hum  = dht22_last_hum;
        g_aqi.dht_hi   = dht22_last_hi;
        strncpy(g_aqi.dht_comfort, dht22_last_comfort, sizeof(g_aqi.dht_comfort) - 1);
        AQI_UNLOCK();
    }
}

/* ── BMP280 wrapper ─────────────────────────────────────── */
extern float bmp280_last_temp;
extern float bmp280_last_pres;
extern float bmp280_last_alt;

void bmp280_wrapper_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        AQI_LOCK();
        g_aqi.bmp_temp = bmp280_last_temp;
        g_aqi.bmp_pres = bmp280_last_pres;
        g_aqi.bmp_alt  = bmp280_last_alt;
        AQI_UNLOCK();
    }
}

/* ── MQ7 wrapper ────────────────────────────────────────── */
extern int   mq7_last_raw;
extern float mq7_last_gpio_v;
extern float mq7_last_aout_v;
extern float mq7_last_rs;
extern float mq7_last_ratio;
extern int   mq7_warmup_s;

void mq7_wrapper_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        AQI_LOCK();
        g_aqi.mq7_raw      = mq7_last_raw;
        g_aqi.mq7_gpio_v   = mq7_last_gpio_v;
        g_aqi.mq7_aout_v   = mq7_last_aout_v;
        g_aqi.mq7_rs       = mq7_last_rs;
        g_aqi.mq7_ratio    = mq7_last_ratio;
        g_aqi.mq7_warmup_s = mq7_warmup_s;
        AQI_UNLOCK();
    }
}

/* ── MQ135 wrapper ──────────────────────────────────────── */
extern int   mq135_last_raw;
extern float mq135_last_gpio_v;
extern float mq135_last_aout_v;
extern float mq135_last_rs;
extern float mq135_last_ratio;
extern float mq135_last_co2;
extern float mq135_last_co;
extern float mq135_last_nh3;
extern float mq135_last_alc;
extern float mq135_last_benz;
extern float mq135_last_smoke;
extern int   mq135_warmup_s;

void mq135_wrapper_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        AQI_LOCK();
        g_aqi.mq135_raw      = mq135_last_raw;
        g_aqi.mq135_gpio_v   = mq135_last_gpio_v;
        g_aqi.mq135_aout_v   = mq135_last_aout_v;
        g_aqi.mq135_rs       = mq135_last_rs;
        g_aqi.mq135_ratio    = mq135_last_ratio;
        g_aqi.mq135_co2      = mq135_last_co2;
        g_aqi.mq135_co       = mq135_last_co;
        g_aqi.mq135_nh3      = mq135_last_nh3;
        g_aqi.mq135_alc      = mq135_last_alc;
        g_aqi.mq135_benz     = mq135_last_benz;
        g_aqi.mq135_smoke    = mq135_last_smoke;
        g_aqi.mq135_warmup_s = mq135_warmup_s;
        AQI_UNLOCK();
    }
}

/* ══════════════════════════════════════════════════════════
 *  HTML PAGE  (served on GET /)
 *  Kept in a raw string literal — no SPIFFS needed.
 * ══════════════════════════════════════════════════════════ */
static const char HTML_PAGE[] =
"<!DOCTYPE html><html lang='en'><head>"
"<meta charset='UTF-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>AQI Monitor</title>"
"<link href='https://fonts.googleapis.com/css2?family=DM+Mono:wght@300;400;500&family=DM+Sans:wght@300;400;500&display=swap' rel='stylesheet'/>"
"<style>"
":root{"
"--bg:#f7f6f3;--surface:#ffffff;--border:#e8e5de;--text:#1a1a1a;"
"--muted:#8a8680;--dim:#c5c1ba;--accent:#2d6a4f;--warn:#b5651d;"
"--danger:#c0392b;--mono:'DM Mono',monospace;--sans:'DM Sans',sans-serif;"
"}"
"*{box-sizing:border-box;margin:0;padding:0;}"
"body{background:var(--bg);color:var(--text);font-family:var(--sans);"
"font-size:14px;min-height:100vh;}"
"header{background:var(--surface);border-bottom:1px solid var(--border);"
"padding:16px 28px;display:flex;align-items:center;justify-content:space-between;"
"position:sticky;top:0;z-index:10;}"
".logo{font-family:var(--mono);font-size:1rem;font-weight:500;letter-spacing:0.04em;color:var(--text);}"
".logo span{color:var(--accent);}"
".hdr-right{display:flex;align-items:center;gap:16px;}"
"#clock{font-family:var(--mono);font-size:0.78rem;color:var(--muted);}"
".pill{display:flex;align-items:center;gap:6px;font-family:var(--mono);"
"font-size:0.7rem;padding:4px 10px;border-radius:20px;"
"border:1px solid var(--border);color:var(--muted);transition:all .3s;}"
".pill.live{border-color:var(--accent);color:var(--accent);}"
".dot{width:6px;height:6px;border-radius:50%;background:currentColor;}"
".pill.live .dot{animation:pulse 1.4s infinite;}"
"@keyframes pulse{0%,100%{opacity:1}50%{opacity:.2}}"
"main{max-width:900px;margin:28px auto;padding:0 20px;display:grid;"
"grid-template-columns:1fr 1fr;gap:16px;}"
".card{background:var(--surface);border:1px solid var(--border);"
"border-radius:8px;overflow:hidden;}"
".card.wide{grid-column:1/-1;}"
".card-head{padding:12px 18px;border-bottom:1px solid var(--border);"
"display:flex;align-items:center;justify-content:space-between;}"
".card-head .name{font-family:var(--mono);font-size:0.72rem;font-weight:500;"
"letter-spacing:0.08em;text-transform:uppercase;color:var(--text);}"
".card-head .chip{font-size:0.65rem;color:var(--muted);}"
".tag{font-size:0.62rem;padding:2px 8px;border-radius:12px;"
"font-family:var(--mono);letter-spacing:0.06em;}"
".tag-ok  {background:#eaf4ee;color:var(--accent);}"
".tag-warm{background:#fef3e8;color:var(--warn);}"
".rows{padding:4px 0;}"
".row{display:grid;grid-template-columns:130px 1fr;align-items:baseline;"
"padding:7px 18px;gap:10px;border-bottom:1px solid #f2f0eb;}"
".row:last-child{border-bottom:none;}"
".rk{font-size:0.7rem;color:var(--muted);}"
".rv{font-family:var(--mono);font-size:0.92rem;color:var(--text);text-align:right;}"
".rv .u{font-size:0.68rem;color:var(--muted);margin-left:3px;}"
".rv .comfort{font-size:0.72rem;padding:1px 8px;border-radius:10px;"
"background:#eaf4ee;color:var(--accent);}"
".rv .comfort.dry    {background:#e8f4fd;color:#1a6ea8;}"
".rv .comfort.humid  {background:#fef3e8;color:var(--warn);}"
".rv .comfort.vhumid {background:#fdecea;color:var(--danger);}"
/* gas sub-grid inside MQ135 */
".gas-grid{display:grid;grid-template-columns:repeat(3,1fr);"
"gap:1px;background:var(--border);border-top:1px solid var(--border);}"
".gc{background:var(--surface);padding:10px 16px;}"
".gc .gn{font-size:0.62rem;color:var(--muted);margin-bottom:3px;}"
".gc .gv{font-family:var(--mono);font-size:1rem;color:var(--text);}"
".gc .gu{font-size:0.62rem;color:var(--muted);margin-left:2px;}"
/* warmup bar */
".wrow{padding:8px 18px;border-top:1px solid var(--border);}"
".wlabel{display:flex;justify-content:space-between;font-size:0.62rem;"
"color:var(--muted);margin-bottom:5px;}"
".wbar{height:3px;background:var(--border);border-radius:2px;overflow:hidden;}"
".wfill{height:100%;background:var(--warn);border-radius:2px;"
"transition:width .8s ease;width:0%;}"
".wfill.done{background:var(--accent);}"
"footer{text-align:center;padding:20px;font-size:0.65rem;"
"color:var(--dim);font-family:var(--mono);}"
"@media(max-width:600px){main{grid-template-columns:1fr;}.card.wide{grid-column:auto;}"
".gas-grid{grid-template-columns:repeat(2,1fr);}}"
"</style></head><body>"
"<header>"
"<div class='logo'>AQI<span>.</span>monitor</div>"
"<div class='hdr-right'>"
"<span id='clk'></span>"
"<div class='pill' id='pill'><div class='dot'></div><span id='st'>offline</span></div>"
"</div>"
"</header>"
"<main>"
/* DHT22 */
"<div class='card'>"
"<div class='card-head'><span class='name'>DHT22</span><span class='chip'>Temp &amp; Humidity</span></div>"
"<div class='rows'>"
"<div class='row'><span class='rk'>Temperature</span><span class='rv' id='dt'>—<span class='u'>°C</span></span></div>"
"<div class='row'><span class='rk'>Humidity</span><span class='rv' id='dh'>—<span class='u'>%</span></span></div>"
"<div class='row'><span class='rk'>Heat Index</span><span class='rv' id='dhi'>—<span class='u'>°C</span></span></div>"
"<div class='row'><span class='rk'>Comfort</span><span class='rv'><span class='comfort' id='dc'>—</span></span></div>"
"</div></div>"
/* BMP280 */
"<div class='card'>"
"<div class='card-head'><span class='name'>BMP280</span><span class='chip'>Pressure &amp; Altitude</span></div>"
"<div class='rows'>"
"<div class='row'><span class='rk'>Temperature</span><span class='rv' id='bt'>—<span class='u'>°C</span></span></div>"
"<div class='row'><span class='rk'>Pressure</span><span class='rv' id='bp'>—<span class='u'>hPa</span></span></div>"
"<div class='row'><span class='rk'>Altitude</span><span class='rv' id='ba'>—<span class='u'>m</span></span></div>"
"</div></div>"
/* MQ7 */
"<div class='card'>"
"<div class='card-head'><span class='name'>MQ-7</span><span class='chip'>Carbon Monoxide</span>"
"<span class='tag' id='m7t'>warmup</span></div>"
"<div class='rows'>"
"<div class='row'><span class='rk'>Raw ADC</span><span class='rv' id='m7r'>—<span class='u'>/ 4095</span></span></div>"
"<div class='row'><span class='rk'>GPIO V</span><span class='rv' id='m7g'>—<span class='u'>V</span></span></div>"
"<div class='row'><span class='rk'>AOUT V</span><span class='rv' id='m7a'>—<span class='u'>V</span></span></div>"
"<div class='row'><span class='rk'>Rs</span><span class='rv' id='m7s'>—<span class='u'>kΩ</span></span></div>"
"<div class='row'><span class='rk'>Rs / R0</span><span class='rv' id='m7o'>—</span></div>"
"</div>"
"<div class='wrow'><div class='wlabel'><span>Warmup</span><span id='m7wp'>0 / 300s</span></div>"
"<div class='wbar'><div class='wfill' id='m7wf'></div></div></div>"
"</div>"
/* MQ135 wide */
"<div class='card wide'>"
"<div class='card-head'><span class='name'>MQ-135</span><span class='chip'>Air Quality</span>"
"<span class='tag' id='m5t'>warmup</span></div>"
"<div class='rows' style='display:grid;grid-template-columns:1fr 1fr;'>"
"<div class='row'><span class='rk'>Raw ADC</span><span class='rv' id='m5r'>—<span class='u'>/ 4095</span></span></div>"
"<div class='row'><span class='rk'>GPIO V</span><span class='rv' id='m5g'>—<span class='u'>V</span></span></div>"
"<div class='row'><span class='rk'>AOUT V</span><span class='rv' id='m5a'>—<span class='u'>V</span></span></div>"
"<div class='row'><span class='rk'>Rs</span><span class='rv' id='m5s'>—<span class='u'>kΩ</span></span></div>"
"<div class='row' style='grid-column:1/-1'><span class='rk'>Rs / R0</span><span class='rv' id='m5o'>—</span></div>"
"</div>"
"<div class='gas-grid'>"
"<div class='gc'><div class='gn'>CO₂</div><div class='gv'><span id='g1'>—</span><span class='gu'>ppm</span></div></div>"
"<div class='gc'><div class='gn'>CO</div><div class='gv'><span id='g2'>—</span><span class='gu'>ppm</span></div></div>"
"<div class='gc'><div class='gn'>NH₃</div><div class='gv'><span id='g3'>—</span><span class='gu'>ppm</span></div></div>"
"<div class='gc'><div class='gn'>Alcohol</div><div class='gv'><span id='g4'>—</span><span class='gu'>ppm</span></div></div>"
"<div class='gc'><div class='gn'>Benzene</div><div class='gv'><span id='g5'>—</span><span class='gu'>ppm</span></div></div>"
"<div class='gc'><div class='gn'>Smoke</div><div class='gv'><span id='g6'>—</span><span class='gu'>ppm</span></div></div>"
"</div>"
"<div class='wrow'><div class='wlabel'><span>Warmup</span><span id='m5wp'>0 / 300s</span></div>"
"<div class='wbar'><div class='wfill' id='m5wf'></div></div></div>"
"</div>"
"</main>"
"<footer>aqi.monitor · esp32 · ws live</footer>"
"<script>"
"const $=id=>document.getElementById(id);"
"function set(id,v){const e=$(id);if(e&&v!=null)e.firstChild.nodeValue=v;}"
"function num(v,d){return v==null?'—':(+v).toFixed(d??1);}"
"setInterval(()=>{$('clk').textContent=new Date().toTimeString().slice(0,8);},1000);"
"const comfortCls={'Dry':'dry','Comfortable':'','Humid':'humid','Very Humid':'vhumid'};"
"function upd(d){"
"  set('dt',num(d.dht_temp,1));set('dh',num(d.dht_hum,1));"
"  set('dhi',num(d.dht_hi,1));"
"  const c=$('dc');c.textContent=d.dht_comfort||'—';"
"  c.className='comfort '+(comfortCls[d.dht_comfort]||'');"
"  set('bt',num(d.bmp_temp,1));set('bp',num(d.bmp_pres,1));set('ba',num(d.bmp_alt,0));"
"  set('m7r',d.mq7_raw);set('m7g',num(d.mq7_gpio_v,3));"
"  set('m7a',num(d.mq7_aout_v,3));set('m7s',num(d.mq7_rs,2));set('m7o',num(d.mq7_ratio,4));"
"  const w7=d.mq7_warmup_s??0;$('m7wp').textContent=w7+' / 300s';"
"  $('m7wf').style.width=Math.min(w7/300*100,100)+'%';"
"  if(w7>=300){$('m7wf').classList.add('done');$('m7t').textContent='ready';$('m7t').className='tag tag-ok';}"
"  else{$('m7t').className='tag tag-warm';}"
"  set('m5r',d.mq135_raw);set('m5g',num(d.mq135_gpio_v,3));"
"  set('m5a',num(d.mq135_aout_v,3));set('m5s',num(d.mq135_rs,2));set('m5o',num(d.mq135_ratio,4));"
"  set('g1',num(d.mq135_co2,1));set('g2',num(d.mq135_co,1));"
"  set('g3',num(d.mq135_nh3,1));set('g4',num(d.mq135_alc,1));"
"  set('g5',num(d.mq135_benz,1));set('g6',num(d.mq135_smoke,1));"
"  const w5=d.mq135_warmup_s??0;$('m5wp').textContent=w5+' / 300s';"
"  $('m5wf').style.width=Math.min(w5/300*100,100)+'%';"
"  if(w5>=300){$('m5wf').classList.add('done');$('m5t').textContent='ready';$('m5t').className='tag tag-ok';}"
"  else{$('m5t').className='tag tag-warm';}"
"}"
"let ws,rd=2000,rt;"
"function live(on){$('pill').classList.toggle('live',on);$('st').textContent=on?'live':'offline';}"
"function conn(){"
"  ws=new WebSocket('ws://'+location.host+'/ws');"
"  ws.onopen=()=>{live(true);rd=2000;};"
"  ws.onmessage=e=>{try{upd(JSON.parse(e.data));}catch{}};"
"  ws.onclose=()=>{live(false);clearTimeout(rt);rt=setTimeout(()=>{rd=Math.min(rd*1.5,30000);conn();},rd);};"
"}"
"conn();"
"</script></body></html>";

/* ══════════════════════════════════════════════════════════
 *  JSON BUILDER
 * ══════════════════════════════════════════════════════════ */
static int build_json(char *buf, size_t len)
{
    aqi_data_t s;
    AQI_LOCK();
    s = g_aqi;
    AQI_UNLOCK();

    return snprintf(buf, len,
        "{"
        "\"dht_temp\":%.1f,\"dht_hum\":%.1f,\"dht_hi\":%.1f,\"dht_comfort\":\"%s\","
        "\"bmp_temp\":%.1f,\"bmp_pres\":%.1f,\"bmp_alt\":%.1f,"
        "\"mq7_raw\":%d,\"mq7_gpio_v\":%.3f,\"mq7_aout_v\":%.3f,"
        "\"mq7_rs\":%.2f,\"mq7_ratio\":%.4f,\"mq7_warmup_s\":%d,"
        "\"mq135_raw\":%d,\"mq135_gpio_v\":%.3f,\"mq135_aout_v\":%.3f,"
        "\"mq135_rs\":%.2f,\"mq135_ratio\":%.4f,"
        "\"mq135_co2\":%.1f,\"mq135_co\":%.1f,\"mq135_nh3\":%.1f,"
        "\"mq135_alc\":%.1f,\"mq135_benz\":%.1f,\"mq135_smoke\":%.1f,"
        "\"mq135_warmup_s\":%d"
        "}",
        s.dht_temp, s.dht_hum, s.dht_hi, s.dht_comfort,
        s.bmp_temp, s.bmp_pres, s.bmp_alt,
        s.mq7_raw, s.mq7_gpio_v, s.mq7_aout_v,
        s.mq7_rs, s.mq7_ratio, s.mq7_warmup_s,
        s.mq135_raw, s.mq135_gpio_v, s.mq135_aout_v,
        s.mq135_rs, s.mq135_ratio,
        s.mq135_co2, s.mq135_co, s.mq135_nh3,
        s.mq135_alc, s.mq135_benz, s.mq135_smoke,
        s.mq135_warmup_s
    );
}

/* ══════════════════════════════════════════════════════════
 *  WEBSOCKET SERVER
 * ══════════════════════════════════════════════════════════ */
#define MAX_WS_CLIENTS 4
static httpd_handle_t s_server = NULL;
static int  s_ws_fds[MAX_WS_CLIENTS];
static int  s_ws_count = 0;

/* WebSocket handshake + incoming frame handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        if (s_ws_count < MAX_WS_CLIENTS) {
            s_ws_fds[s_ws_count++] = fd;
            ESP_LOGI(TAG_WS, "Client connected fd=%d (%d total)", fd, s_ws_count);
        }
        return ESP_OK;
    }
    /* Drain any incoming frame (we don't act on client messages) */
    httpd_ws_frame_t f = {.type = HTTPD_WS_TYPE_TEXT};
    uint8_t tmp[64] = {0};
    f.payload = tmp;
    httpd_ws_recv_frame(req, &f, sizeof(tmp) - 1);
    return ESP_OK;
}

static const httpd_uri_t uri_ws = {
    .uri          = "/ws",
    .method       = HTTP_GET,
    .handler      = ws_handler,
    .is_websocket = true,
};

/* HTTP GET / — serve the dashboard HTML */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t uri_root = {
    .uri     = "/",
    .method  = HTTP_GET,
    .handler = root_handler,
};

/* Broadcast task — sends JSON to every connected client every 2s */
static void broadcast_task(void *arg)
{
    char buf[640];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int n = build_json(buf, sizeof(buf));
        if (n <= 0) continue;

        httpd_ws_frame_t frame = {
            .type    = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)buf,
            .len     = (size_t)n,
        };

        for (int i = 0; i < s_ws_count; ) {
            esp_err_t err = httpd_ws_send_frame_async(s_server, s_ws_fds[i], &frame);
            if (err != ESP_OK) {
                ESP_LOGW(TAG_WS, "Client fd=%d gone, removing", s_ws_fds[i]);
                s_ws_fds[i] = s_ws_fds[--s_ws_count];
            } else {
                i++;
            }
        }
    }
}

static void start_webserver(void)
{
    memset(s_ws_fds, -1, sizeof(s_ws_fds));
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    ESP_ERROR_CHECK(httpd_start(&s_server, &cfg));
    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_ws);
    xTaskCreate(broadcast_task, "ws_broadcast", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG_WS, "Server started — open http://<ESP32-IP> in browser");
}

/* ══════════════════════════════════════════════════════════
 *  WIFI
 * ══════════════════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < MAX_RETRY) {
            esp_wifi_connect();
            s_retry++;
            ESP_LOGI(TAG_WIFI, "Retrying WiFi (%d/%d)...", s_retry, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
            ESP_LOGE(TAG_WIFI, "WiFi connection failed.");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG_WIFI, "Connected! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h1, h2;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,  &wifi_event_handler, NULL, &h1));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &h2));

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_eg,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));

    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* ══════════════════════════════════════════════════════════
 *  app_main
 * ══════════════════════════════════════════════════════════ */
void app_main(void)
{
    printf("\n=== AQI System Starting ===\n\n");

    /* NVS — required by WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Shared data mutex */
    g_mutex = xSemaphoreCreateMutex();

    /* Sensor init — serial output unchanged */
    adc_shared_init();
    bmp280_start();
    mq7_start();
    mq135_start();
    dht22_start();

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* WiFi */
    if (wifi_init_sta()) {
        start_webserver();
    } else {
        ESP_LOGW(TAG_WIFI, "No WiFi — running sensors only, no web dashboard.");
    }

    /* Sensor tasks (original — keep serial output) */
    xTaskCreate(dht22_task,  "dht22",  4096, NULL, 5, NULL);
    xTaskCreate(bmp280_task, "bmp280", 4096, NULL, 5, NULL);
    xTaskCreate(mq7_task,    "mq7",    4096, NULL, 5, NULL);
    xTaskCreate(mq135_task,  "mq135",  4096, NULL, 5, NULL);

    /* Wrapper tasks — mirror sensor globals into g_aqi */
    xTaskCreate(dht22_wrapper_task,  "dht22_ws",  2048, NULL, 4, NULL);
    xTaskCreate(bmp280_wrapper_task, "bmp280_ws", 2048, NULL, 4, NULL);
    xTaskCreate(mq7_wrapper_task,    "mq7_ws",    2048, NULL, 4, NULL);
    xTaskCreate(mq135_wrapper_task,  "mq135_ws",  2048, NULL, 4, NULL);
}