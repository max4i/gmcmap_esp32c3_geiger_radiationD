#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <time.h>
#include <math.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// ==================== DEKLARACJE FUNKCJI ====================
void handle_root();
void handle_save();
void handle_restart();
void handle_info();
void handle_tube_info();
void handle_reset_tube();
void handle_reset_graph();
String generateGraphHTML();
String generateEnvGraphHTML();
void graphLoad();
void graphSave();
void envGraphLoad();
void envGraphSave();
void graphAddPoint(float cpm, float acpm, float usv_h);
void envGraphAddPoint(float temp, float hum, float pres);
void graphInitialize();
void envGraphInitialize();
void update_uptime();
void setup_time();
uint32_t get_current_unix_time();
void save_all_data();
void load_tube_state();
void reset_tube_counter();
uint32_t calculate_tube_uptime();
String format_years_days(uint32_t seconds);
void update_measurements();
void update_environment();
void calculate_derived_env();
void calculate_iaq();
void init_mdns();
void wifi_event_handler(WiFiEvent_t event);
void connect_wifi();
void start_ap_mode();
void check_wifi_reconnect();
void send_to_gmcmap();
void send_to_custom_server();
void send_to_custom_server2();
void IRAM_ATTR geiger_isr();
void log_nvs_status();
void save_all_data_combined();
bool time_elapsed(uint32_t &last_time, uint32_t interval);
bool init_bme680();
String validate_input(const String &input, const String &type, float min_val = 0, float max_val = 0);
void calculate_additional_env_metrics();

// ==================== KONFIGURACJA SPRZĘTOWA ====================
#define GEIGER_PIN              4       
#define LED_PIN                 10       
#define DEBOUNCE_US             80
#define BME_SDA                 8       // ESP32-C3: GPIO8 = SDA
#define BME_SCL                 9       // ESP32-C3: GPIO9 = SCL

// ==================== PARAMETRY TUBY ====================
#define TUBE_LIFETIME           100000000
#define TUBE_SAVE_INTERVAL      600000    

// ==================== PARAMETRY SYSTEMOWE ====================
#define CPM_WINDOW_SIZE         120
#define ACPM_WINDOW_SIZE        600
#define DOSE_SAVE_INTERVAL      900000  
#define MEASUREMENT_LOG_INTERVAL 60000  
#define STABILITY_CALC_INTERVAL 60000
#define WIFI_RECONNECT_INTERVAL 30000
#define WIFI_CONNECT_TIMEOUT    30000
#define WIFI_AP_TIMEOUT         60000
#define ALL_DATA_SAVE_INTERVAL  600000
#define ENV_READ_INTERVAL       30000

#define DEFAULT_WIFI_SSID       "Geiger_DIY_Setup"
#define DEFAULT_WIFI_PASS       "ChangeMe123!"
#define DEFAULT_FACTOR          0.00297
#define DEFAULT_UPDATE_INTERVAL 10

#define MDNS_PRIMARY_NAME       "geiger"
#define MDNS_SECONDARY_NAME     "geigercnt"

// ==================== WYKRES HISTORYCZNY ====================
#define GRAPH_POINTS            144
struct GraphPoint {
  float cpm;
  float acpm;
  float usv_h;
  uint32_t timestamp;
};

struct EnvGraphPoint {
  float temperature;
  float humidity;
  float pressure;
  float iaq_score;
  uint32_t timestamp;
};

GraphPoint graphData[GRAPH_POINTS];
EnvGraphPoint envGraphData[GRAPH_POINTS];
uint16_t graphIndex = 0;
uint16_t envGraphIndex = 0;
bool graphFilled = false;
bool envGraphFilled = false;
uint32_t last_graph_update = 0;
uint32_t last_env_update = 0;
bool graphInitialized = false;
bool envGraphInitialized = false;

// ==================== DANE CZUJNIKA ====================
TwoWire I2CBME = TwoWire(0);
Adafruit_BME680 bme(&I2CBME);
bool bme_initialized = false;
bool bme_sensor_found = false;

struct BME680_Data {
  float temperature;
  float humidity;
  float pressure;
  float gas_resistance;
  float dew_point;
  float heat_index;
  float absolute_humidity;
  float vapor_pressure;
  float air_density;
  float iaq_score;
  String iaq_level;
  float co2_equivalent;
  float voc_equivalent;
  float sound_speed;
  float humidex;
  float discomfort_index;
  float summer_simmer_index;
  float apparent_temperature;
  float wind_chill;
  float thom_index;
  float effective_temperature;
  float temperature_humidity_wind_index;
  float wet_bulb_temperature;
  float relative_strain_index;
  float thermal_comfort_index;
  unsigned long timestamp;
};

BME680_Data env_data;

// ==================== MONITORING PROCESORA ====================
#define CPU_AVG_WINDOW          10
uint32_t cpu_loop_times[CPU_AVG_WINDOW] = {0};
uint8_t cpu_loop_index = 0;
float cpu_usage = 0.0;
uint32_t last_cpu_calc = 0;
uint32_t loop_start_time = 0;

// ==================== NVS STATISTICS ====================
uint32_t nvs_write_count = 0;
uint32_t last_nvs_log = 0;
#define NVS_MAX_WRITES          100000
#define NVS_LOG_INTERVAL        3600000

// ==================== ZMIENNE ====================
volatile uint32_t pulse_count = 0;
volatile uint32_t last_pulse_micros = 0;

uint32_t second_buffer[ACPM_WINDOW_SIZE] = {0};
uint16_t buffer_index = 0;
uint32_t last_second_update = 0;
uint32_t last_buffer_shift = 0;

uint32_t total_pulses = 0;
uint32_t total_lifetime_pulses = 0;
uint32_t tube_start_time = 0;
uint32_t last_tube_save = 0;
float tube_lifetime_percent = 0.0;

uint32_t last_gmc_send = 0;
uint32_t last_custom_send = 0;
uint32_t last_custom2_send = 0;
uint32_t last_dose_save = 0;
uint32_t last_measurement_log = 0;
uint32_t last_stability_calc = 0;
uint32_t last_wifi_reconnect = 0;
uint32_t wifi_disconnect_time = 0;
uint32_t wifi_connect_start = 0;
uint32_t last_env_read = 0;
bool wifi_connected = false;
bool ap_mode = false;
bool wifi_reconnecting = false;
String device_ip = "";

float current_cpm = 0.0;
float current_acpm = 0.0;
float current_usv_h = 0.0;
float daily_dose = 0.0;
float conversion_factor = DEFAULT_FACTOR;
float background_stability = 0.0;
float background_history[60] = {0};
uint8_t history_index = 0;

char uptime_str[32] = "0d 00:00:00";

String custom_server_url = "";
String custom_server_token = "";
bool custom_server_enabled = false;
bool custom_server_last_status = false;
String custom_server_last_response = "";
uint32_t custom_server_last_attempt = 0;

String custom_server2_url = "";
String custom_server2_token = "";
bool custom_server2_enabled = false;
bool custom_server2_last_status = false;
String custom_server2_last_response = "";
uint32_t custom_server2_last_attempt = 0;

uint32_t last_all_data_save = 0;
uint32_t update_interval_minutes = DEFAULT_UPDATE_INTERVAL;
uint32_t update_interval_ms = DEFAULT_UPDATE_INTERVAL * 60000UL;

WebServer server(80);
Preferences prefs;

// ==================== HTML HEADER ====================
const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Geiger DIY Monitor v6/env</title>
<style>
:root { 
    --bg: #0d1117; 
    --card: #161b22; 
    --primary: #238636; 
    --sec: #58a6ff; 
    --text: #c9d1d9; 
    --border: #30363d; 
    --success: #2ea043;
    --warning: #d29922;
    --danger: #f85149;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { 
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; 
    background: var(--bg); 
    color: var(--text); 
    line-height: 1.6;
    padding: 20px;
    min-height: 100vh;
}
.container { 
    max-width: 1000px; 
    margin: 0 auto;
}
.header {
    text-align: center;
    margin-bottom: 30px;
    padding-bottom: 20px;
    border-bottom: 2px solid var(--border);
}
h1 { 
    color: var(--sec); 
    font-size: 2.2rem;
    margin-bottom: 10px;
}
h2 {
    color: var(--sec);
    font-size: 1.5rem;
    margin-bottom: 20px;
    padding-bottom: 10px;
    border-bottom: 1px solid var(--border);
}
h3 {
    color: var(--text);
    font-size: 1.2rem;
    margin: 25px 0 15px 0;
}
.subtitle {
    color: #8b949e;
    font-size: 1rem;
    margin-bottom: 20px;
}
.status-bar {
    display: flex;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 15px;
    background: var(--card);
    padding: 15px;
    border-radius: 8px;
    border: 1px solid var(--border);
    margin-bottom: 30px;
    font-size: 0.9rem;
}
.status-item {
    display: flex;
    align-items: center;
    gap: 8px;
}
.status-icon {
    font-size: 1.1rem;
}
.status-wifi-good { color: var(--success); }
.status-wifi-weak { color: var(--warning); }
.status-wifi-bad { color: var(--danger); }
.card { 
    background: var(--card); 
    border: 1px solid var(--border); 
    border-radius: 10px; 
    padding: 25px; 
    margin-bottom: 25px;
    box-shadow: 0 4px 15px rgba(0, 0, 0, 0.15);
}
.stat-grid { 
    display: grid; 
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); 
    gap: 20px;
    margin-top: 20px;
}
.stat-box { 
    background: linear-gradient(145deg, #0d1117, #161b22);
    padding: 20px;
    border: 1px solid var(--border);
    border-radius: 8px;
    text-align: center;
    transition: transform 0.2s, box-shadow 0.2s;
}
.stat-box:hover {
    transform: translateY(-3px);
    box-shadow: 0 6px 20px rgba(0, 0, 0, 0.2);
}
.stat-value { 
    font-size: 2.2rem; 
    font-weight: bold; 
    color: white; 
    display: block;
    margin: 10px 0;
    text-shadow: 0 2px 4px rgba(0,0,0,0.3);
}
.unit { 
    font-size: 0.9rem; 
    color: var(--primary); 
    font-weight: bold;
    letter-spacing: 0.5px;
}
.stat-label { 
    font-size: 0.85rem; 
    color: #8b949e; 
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-top: 5px;
}
.environment-box {
    background: linear-gradient(145deg, #0d1117, #161b22);
    padding: 15px;
    border: 1px solid var(--border);
    border-radius: 8px;
    text-align: center;
    transition: transform 0.2s;
}
.environment-box:hover {
    transform: translateY(-2px);
}
.env-value {
    font-size: 1.6rem; 
    font-weight: bold; 
    color: white; 
    display: block;
    margin: 5px 0;
}
.env-label {
    font-size: 0.8rem; 
    color: #8b949e; 
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin-top: 3px;
}
.access-methods {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
    gap: 20px;
    margin-top: 20px;
}
.access-card {
    background: rgba(13, 17, 23, 0.5);
    padding: 20px;
    border-radius: 8px;
    border: 1px solid var(--border);
    text-align: center;
}
.access-icon {
    font-size: 2.5rem;
    margin-bottom: 15px;
}
.access-title {
    font-size: 1.1rem;
    font-weight: bold;
    margin-bottom: 10px;
    color: var(--sec);
}
.access-link {
    display: block;
    background: var(--primary);
    color: white;
    padding: 10px 15px;
    border-radius: 5px;
    text-decoration: none;
    margin-top: 10px;
    transition: background 0.3s;
}
.access-link:hover {
    background: #2ea043;
}
.form-group {
    margin-bottom: 20px;
    position: relative;
}
.form-label {
    display: block;
    margin-bottom: 8px;
    font-weight: 500;
    color: var(--text);
}
.form-input {
    width: 100%;
    padding: 12px 15px;
    background: #0d1117;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: white;
    font-size: 1rem;
    transition: border 0.3s;
}
.form-input:focus {
    outline: none;
    border-color: var(--primary);
    box-shadow: 0 0 0 2px rgba(35, 134, 54, 0.2);
}
.form-help {
    font-size: 0.85rem;
    color: #8b949e;
    margin-top: 5px;
    font-style: italic;
}
.error-message {
    color: var(--danger);
    font-size: 0.85rem;
    margin-top: 5px;
    display: none;
}
.btn {
    background: linear-gradient(135deg, var(--primary), #2ea043);
    color: white;
    border: none;
    padding: 14px 25px;
    border-radius: 6px;
    font-size: 1rem;
    font-weight: bold;
    cursor: pointer;
    transition: all 0.3s;
    width: 100%;
    text-align: center;
    display: block;
    text-decoration: none;
}
.btn:hover {
    background: linear-gradient(135deg, #2ea043, #238636);
    box-shadow: 0 4px 15px rgba(35, 134, 54, 0.3);
}
.btn:active {
    transform: translateY(1px);
}
.btn-warning {
    background: linear-gradient(135deg, var(--warning), #bb8009);
}
.btn-warning:hover {
    background: linear-gradient(135deg, #bb8009, var(--warning));
}
.legend-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 20px;
    margin-top: 20px;
}
.legend-item {
    background: rgba(13, 17, 23, 0.5);
    padding: 20px;
    border-radius: 8px;
    border-left: 4px solid var(--sec);
}
.legend-title {
    color: var(--sec);
    font-weight: bold;
    margin-bottom: 10px;
    font-size: 1.1rem;
}
.legend-desc {
    color: #c9d1d9;
    font-size: 0.95rem;
    line-height: 1.5;
}
.calibration-box {
    background: rgba(88, 166, 255, 0.1);
    border: 1px solid var(--sec);
    padding: 25px;
    border-radius: 8px;
    margin-top: 20px;
}
.calibration-step {
    margin-bottom: 15px;
    padding-left: 20px;
    position: relative;
}
.calibration-step:before {
    content: "▶";
    position: absolute;
    left: 0;
    color: var(--primary);
}
.code {
    background: #0d1117;
    padding: 15px;
    border-radius: 6px;
    border: 1px solid var(--border);
    font-family: 'Courier New', monospace;
    margin: 15px 0;
    color: var(--sec);
    overflow-x: auto;
}
.warning-box {
    background: rgba(248, 81, 73, 0.1);
    border: 1px solid var(--danger);
    padding: 20px;
    border-radius: 8px;
    margin: 25px 0;
}
.warning-title {
    color: var(--danger);
    font-weight: bold;
    margin-bottom: 10px;
    display: flex;
    align-items: center;
    gap: 10px;
}
.success-box {
    background: rgba(46, 160, 67, 0.1);
    border: 1px solid var(--success);
    padding: 20px;
    border-radius: 8px;
    margin: 25px 0;
}
.footer {
    text-align: center;
    margin-top: 40px;
    padding-top: 20px;
    border-top: 1px solid var(--border);
    color: #8b949e;
    font-size: 0.9rem;
}
.qr-code {
    background: white;
    padding: 10px;
    border-radius: 10px;
    display: inline-block;
    margin: 15px 0;
}
.gmc-status {
    display: inline-block;
    padding: 5px 12px;
    border-radius: 20px;
    font-size: 0.85rem;
    font-weight: bold;
    margin-left: 10px;
}
.gmc-online {
    background: rgba(46, 160, 67, 0.2);
    color: var(--success);
}
.gmc-offline {
    background: rgba(248, 81, 73, 0.2);
    color: var(--danger);
}
.info-section {
    margin: 30px 0;
    padding: 25px;
    background: rgba(13, 17, 23, 0.3);
    border-radius: 10px;
    border: 1px solid var(--border);
}
.info-title {
    color: var(--sec);
    font-size: 1.3rem;
    margin-bottom: 15px;
    padding-bottom: 10px;
    border-bottom: 1px solid var(--border);
}
.note-box {
    background: rgba(210, 153, 34, 0.1);
    border: 1px solid var(--warning);
    padding: 15px;
    border-radius: 8px;
    margin: 15px 0;
}
.note-title {
    color: var(--warning);
    font-weight: bold;
    margin-bottom: 8px;
}
.url-box {
    display: flex;
    flex-direction: column;
    gap: 10px;
    margin-top: 10px;
}
.url-code {
    width: 100%;
    background: var(--bg);
    padding: 10px 15px;
    border-radius: 5px;
    border: 1px solid var(--border);
    font-family: monospace;
    overflow-x: auto;
    word-break: break-all;
    font-size: 0.9rem;
}
.success-message {
    position: fixed;
    top: 20px;
    left: 50%;
    transform: translateX(-50%);
    background: linear-gradient(135deg, var(--primary), #2ea043);
    color: white;
    padding: 20px 40px;
    border-radius: 10px;
    z-index: 1000;
    box-shadow: 0 5px 20px rgba(0,0,0,0.3);
    animation: slideIn 0.5s ease-out;
}
@keyframes slideIn {
    from { top: -100px; }
    to { top: 20px; }
}
@media (max-width: 768px) {
    body { padding: 15px; }
    .card { padding: 20px; }
    .stat-grid { grid-template-columns: 1fr; }
    .status-bar { flex-direction: column; gap: 10px; }
    .access-methods { grid-template-columns: 1fr; }
    .legend-grid { grid-template-columns: 1fr; }
    h1 { font-size: 1.8rem; }
}
</style>
</head>
<body>
<div class="container">
)rawliteral";

// ==================== PRZERWANIE ====================
void IRAM_ATTR geiger_isr() {
    uint32_t now_micros = micros();
    uint32_t delta;
    
    if (now_micros >= last_pulse_micros) {
        delta = now_micros - last_pulse_micros;
    } else {
        delta = (UINT32_MAX - last_pulse_micros) + now_micros + 1;
    }
    
    if (delta >= DEBOUNCE_US) {
        pulse_count++;
        last_pulse_micros = now_micros;
    }
}

// ==================== FUNKCJE CZASU ====================
bool time_elapsed(uint32_t &last_time, uint32_t interval) {
    uint32_t current = millis();
    uint32_t elapsed;
    
    if (current >= last_time) {
        elapsed = current - last_time;
    } else {
        elapsed = (UINT32_MAX - last_time) + current + 1;
    }
    
    if (elapsed >= interval) {
        last_time = current;
        return true;
    }
    return false;
}

// ==================== WALIDACJA DANYCH ====================
String validate_input(const String &input, const String &type, float min_val, float max_val) {
    if (type == "float") {
        String temp = input;
        temp.replace(',', '.');
        float val = temp.toFloat();
        
        if (temp.length() == 0) return "Pole nie może być puste";
        if (isnan(val)) return "Nieprawidłowa wartość liczbowa";
        if (min_val != max_val) {
            if (val < min_val) return "Wartość zbyt mała (min: " + String(min_val, 3) + ")";
            if (val > max_val) return "Wartość zbyt duża (max: " + String(max_val, 3) + ")";
        }
        return "";
    }
    else if (type == "int") {
        int val = input.toInt();
        if (input.length() == 0) return "Pole nie może być puste";
        if (val == 0 && input != "0") return "Nieprawidłowa wartość całkowita";
        if (min_val != max_val) {
            if (val < min_val) return "Wartość zbyt mała (min: " + String((int)min_val) + ")";
            if (val > max_val) return "Wartość zbyt duża (max: " + String((int)max_val) + ")";
        }
        return "";
    }
    else if (type == "ssid") {
        if (input.length() == 0) return "SSID nie może być pusty";
        if (input.length() > 32) return "SSID zbyt długi (max 32 znaki)";
        return "";
    }
    else if (type == "password") {
        if (input.length() < 8) return "Hasło zbyt krótkie (min 8 znaków)";
        if (input.length() > 63) return "Hasło zbyt długie (max 63 znaki)";
        return "";
    }
    else if (type == "url") {
        if (input.length() == 0) return "";
        if (!input.startsWith("http://") && !input.startsWith("https://")) {
            return "URL musi zaczynać się od http:// lub https://";
        }
        return "";
    }
    return "";
}

// ==================== CZUJNIK BME680 ====================
bool init_bme680() {
    Serial.println("[BME] Inicjalizacja BME680...");
    
    I2CBME.begin(BME_SDA, BME_SCL, 400000);
    delay(100);
    
    // Próba znalezienia czujnika pod różnymi adresami
    bme_sensor_found = false;
    
    if (bme.begin(0x76)) {
        Serial.println("[BME] Znaleziono BME680 pod adresem 0x76");
        bme_sensor_found = true;
    } else if (bme.begin(0x77)) {
        Serial.println("[BME] Znaleziono BME680 pod adresem 0x77");
        bme_sensor_found = true;
    } else {
        Serial.println("[BME] BŁĄD: Nie znaleziono BME680!");
        return false;
    }
    
    if (bme_sensor_found) {
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
        
        // Wstępny odczyt
        if (bme.performReading()) {
            env_data.temperature = bme.temperature;
            env_data.humidity = bme.humidity;
            env_data.pressure = bme.pressure / 100.0;
            env_data.gas_resistance = bme.gas_resistance;
            env_data.timestamp = millis();
            calculate_derived_env();
            calculate_additional_env_metrics();
            
            Serial.printf("[BME] BME680 zainicjalizowany: Temp=%.1f°C, Hum=%.1f%%, Pres=%.1fhPa\n",
                         env_data.temperature, env_data.humidity, env_data.pressure);
        }
    }
    
    return bme_sensor_found;
}

void calculate_derived_env() {
    const float R = 287.05;
    const float T = env_data.temperature + 273.15;
    const float RH = env_data.humidity;
    const float P = env_data.pressure * 100.0;
    
    // Punkt rosy (Magnus)
    float a = 17.62;
    float b = 243.12;
    float gamma = log(RH / 100.0) + (a * env_data.temperature) / (b + env_data.temperature);
    env_data.dew_point = (b * gamma) / (a - gamma);
    
    // Ciśnienie pary nasyconej (Buck)
    env_data.vapor_pressure = 6.1121 * exp((18.678 - env_data.temperature / 234.5) *
                   (env_data.temperature / (257.14 + env_data.temperature)));
    
    float e = env_data.vapor_pressure * RH / 100.0;
    env_data.absolute_humidity = (216.7 * e) / T;
    env_data.air_density = P / (R * T);
    
    // Heat Index
    if (env_data.temperature >= 26.0 && RH >= 40.0) {
        float T2 = env_data.temperature * env_data.temperature;
        float RH2 = RH * RH;
        env_data.heat_index = -8.784695 + 1.61139411 * env_data.temperature +
                             2.338549 * RH - 0.14611605 * env_data.temperature * RH -
                             0.012308094 * T2 - 0.016424828 * RH2 +
                             0.002211732 * T2 * RH + 0.00072546 * env_data.temperature * RH2 -
                             0.000003582 * T2 * RH2;
    } else {
        env_data.heat_index = env_data.temperature;
    }
    
    calculate_iaq();
}

void calculate_additional_env_metrics() {
    float T = env_data.temperature;
    float RH = env_data.humidity;
    float P = env_data.pressure;
    
    // Prędkość dźwięku w powietrzu (m/s)
    env_data.sound_speed = 331.3 * sqrt(1 + (T / 273.15));
    
    // Humidex (Kanadyjski wskaźnik odczuwalnej temperatury)
    float e = 6.11 * exp(5417.753 * ((1/273.16) - (1/(env_data.dew_point + 273.15))));
    env_data.humidex = T + 0.5555 * (e - 10.0);
    
    // Discomfort Index (DI) - wskaźnik dyskomfortu
    env_data.discomfort_index = 0.5 * (T + 61.0 + ((T - 68.0) * 1.2) + (RH * 0.094));
    
    // Summer Simmer Index (SSI)
    env_data.summer_simmer_index = 1.98 * (T - (0.55 - 0.0055 * RH) * (T - 58.0)) - 56.83;
    
    // Apparent Temperature (odczuwalna temperatura)
    env_data.apparent_temperature = T + 0.33 * env_data.vapor_pressure - 0.7 * 0.0 - 4.0;
    
    // Wind Chill (temperatura odczuwalna przy wietrze) - zakładamy wiatr 0 m/s
    env_data.wind_chill = T;
    if (T <= 10.0) {
        float wind_speed_kmh = 0.0; // brak danych o wietrze
        env_data.wind_chill = 13.12 + 0.6215 * T - 11.37 * pow(wind_speed_kmh, 0.16) + 
                              0.3965 * T * pow(wind_speed_kmh, 0.16);
    }
    
    // Thom Index (discomfort index)
    env_data.thom_index = T - (0.55 - 0.0055 * RH) * (T - 14.5);
    
    // Effective Temperature
    env_data.effective_temperature = 37.0 - ((37.0 - T) / (0.68 - 0.0014 * RH + 1/(1.76 + 1.4 * pow(0.0, 0.75)))) - 0.29 * T * (1 - RH/100.0);
    
    // Temperature Humidity Wind Index (THW Index)
    env_data.temperature_humidity_wind_index = T + 0.33 * env_data.vapor_pressure - 0.7 * 0.0 - 4.0;
    
    // Wet Bulb Temperature (temperatura termometru zwilżonego)
    env_data.wet_bulb_temperature = T * atan(0.151977 * sqrt(RH + 8.313659)) + 
                                    atan(T + RH) - atan(RH - 1.676331) + 
                                    0.00391838 * pow(RH, 1.5) * atan(0.023101 * RH) - 4.686035;
    
    // Relative Strain Index (RSI)
    env_data.relative_strain_index = 0.5 * (T + 61.0 + ((T - 68.0) * 1.2) + (RH * 0.094));
    
    // Thermal Comfort Index
    env_data.thermal_comfort_index = 0.5 * (T + env_data.wet_bulb_temperature);
}

void calculate_iaq() {
    if (!bme_sensor_found) return;
    
    float hum_score = 50.0 - abs(env_data.humidity - 45.0);
    hum_score = constrain(hum_score, 0, 50);
    
    float gas_score = 0;
    if (env_data.gas_resistance > 0) {
        gas_score = log10(env_data.gas_resistance) * 15.0;
        gas_score = constrain(gas_score, 0, 50);
    }
    
    env_data.iaq_score = hum_score + gas_score;
    
    if (env_data.iaq_score >= 90) env_data.iaq_level = "DOSKONAŁA";
    else if (env_data.iaq_score >= 70) env_data.iaq_level = "DOBRA";
    else if (env_data.iaq_score >= 50) env_data.iaq_level = "ŚREDNIA";
    else if (env_data.iaq_score >= 25) env_data.iaq_level = "SŁABA";
    else env_data.iaq_level = "BARDZO SŁABA";
    
    if (env_data.gas_resistance > 0) {
        env_data.co2_equivalent = constrain(400 + (100000.0 - env_data.gas_resistance) / 150.0, 400, 5000);
        env_data.voc_equivalent = env_data.gas_resistance / 12000.0;
    } else {
        env_data.co2_equivalent = 0;
        env_data.voc_equivalent = 0;
    }
}

void update_environment() {
    if (!bme_sensor_found) return;
    
    if (time_elapsed(last_env_read, ENV_READ_INTERVAL)) {
        if (bme.performReading()) {
            env_data.temperature = bme.temperature;
            env_data.humidity = bme.humidity;
            env_data.pressure = bme.pressure / 100.0;
            env_data.gas_resistance = bme.gas_resistance;
            env_data.timestamp = millis();
            
            calculate_derived_env();
            calculate_additional_env_metrics();
            
            envGraphAddPoint(env_data.temperature, env_data.humidity, env_data.pressure);
        } else {
            Serial.println("[ENV] Błąd odczytu BME680");
        }
    }
}

// ==================== WYSYŁKA DO GMCMap ====================
void send_to_gmcmap() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    String aid = prefs.getString("aid", "");
    String gid = prefs.getString("gid", "");

    if (aid == "" || gid == "" || aid == "null") {
        return;
    }

    HTTPClient http;
    http.setTimeout(10000);
    String url = "http://www.gmcmap.com/log2.asp?AID=" + aid + 
                 "&GID=" + gid + 
                 "&CPM=" + String((int)round(current_cpm)) + 
                 "&ACPM=" + String((int)round(current_acpm)) + 
                 "&uSV=" + String(current_usv_h, 4);
    
    bool success = http.begin(url);
    if (!success) {
        return;
    }
    
    int httpCode = http.GET();

    if (httpCode > 0) {
        digitalWrite(LED_PIN, LOW); 
        delay(100); 
        digitalWrite(LED_PIN, HIGH);
    }
    http.end();
}

// ==================== WYSYŁKA DO WŁASNEGO SERWERA ====================
String generate_json_payload() {
    String jsonPayload = "{";
    
    jsonPayload += "\"timestamp\":" + String(get_current_unix_time()) + ",";
    jsonPayload += "\"measurement_time_ms\":" + String(millis()) + ",";
    jsonPayload += "\"device_id\":\"geiger_esp32c3_" + String((uint32_t)ESP.getEfuseMac(), HEX) + "\",";
    jsonPayload += "\"update_interval_minutes\":" + String(update_interval_minutes) + ",";
    
    jsonPayload += "\"radiation\":{";
    jsonPayload += "\"cpm\":" + String(current_cpm, 1) + ",";
    jsonPayload += "\"acpm\":" + String(current_acpm, 1) + ",";
    jsonPayload += "\"usv_h\":" + String(current_usv_h, 4) + ",";
    jsonPayload += "\"daily_dose\":" + String(daily_dose, 6) + ",";
    jsonPayload += "\"stability\":" + String(background_stability, 1);
    jsonPayload += "},";
    
    uint32_t tube_uptime = calculate_tube_uptime();
    float tube_uptime_days = tube_uptime / 86400.0;
    float daily_avg = (tube_uptime_days > 0.1) ? (total_lifetime_pulses / tube_uptime_days) : 0;
    
    jsonPayload += "\"tube\":{";
    jsonPayload += "\"total_pulses\":" + String(total_lifetime_pulses) + ",";
    jsonPayload += "\"lifetime_percent\":" + String(tube_lifetime_percent, 2) + ",";
    jsonPayload += "\"uptime_seconds\":" + String(tube_uptime) + ",";
    jsonPayload += "\"uptime_days\":" + String(tube_uptime_days, 1) + ",";
    jsonPayload += "\"remaining_pulses\":" + String(TUBE_LIFETIME - total_lifetime_pulses) + ",";
    jsonPayload += "\"daily_average\":" + String(daily_avg, 1) + ",";
    
    String tube_status = "good";
    if (tube_lifetime_percent > 80) tube_status = "warning";
    if (tube_lifetime_percent > 95) tube_status = "critical";
    jsonPayload += "\"status\":\"" + tube_status + "\"";
    jsonPayload += "},";
    
    jsonPayload += "\"system\":{";
    jsonPayload += "\"uptime_seconds\":" + String(millis() / 1000) + ",";
    jsonPayload += "\"uptime_str\":\"" + String(uptime_str) + "\",";
    jsonPayload += "\"cpu_usage\":" + String(cpu_usage, 1) + ",";
    jsonPayload += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
    jsonPayload += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
    jsonPayload += "\"ip_address\":\"" + device_ip + "\",";
    jsonPayload += "\"wifi_ssid\":\"" + WiFi.SSID() + "\",";
    jsonPayload += "\"ap_mode\":" + String(ap_mode ? "true" : "false") + ",";
    jsonPayload += "\"update_interval_minutes\":" + String(update_interval_minutes);
    jsonPayload += "},";
    
    jsonPayload += "\"environment\":{";
    jsonPayload += "\"temperature\":" + String(env_data.temperature, 2) + ",";
    jsonPayload += "\"humidity\":" + String(env_data.humidity, 1) + ",";
    jsonPayload += "\"pressure\":" + String(env_data.pressure, 1) + ",";
    jsonPayload += "\"gas_resistance\":" + String(env_data.gas_resistance, 0) + ",";
    jsonPayload += "\"dew_point\":" + String(env_data.dew_point, 2) + ",";
    jsonPayload += "\"heat_index\":" + String(env_data.heat_index, 2) + ",";
    jsonPayload += "\"absolute_humidity\":" + String(env_data.absolute_humidity, 2) + ",";
    jsonPayload += "\"air_density\":" + String(env_data.air_density, 3) + ",";
    jsonPayload += "\"iaq_score\":" + String(env_data.iaq_score, 1) + ",";
    jsonPayload += "\"iaq_level\":\"" + env_data.iaq_level + "\",";
    jsonPayload += "\"co2_equivalent\":" + String(env_data.co2_equivalent, 0) + ",";
    jsonPayload += "\"voc_equivalent\":" + String(env_data.voc_equivalent, 2) + ",";
    jsonPayload += "\"sound_speed\":" + String(env_data.sound_speed, 1) + ",";
    jsonPayload += "\"humidex\":" + String(env_data.humidex, 1) + ",";
    jsonPayload += "\"discomfort_index\":" + String(env_data.discomfort_index, 1) + ",";
    jsonPayload += "\"summer_simmer_index\":" + String(env_data.summer_simmer_index, 1) + ",";
    jsonPayload += "\"apparent_temperature\":" + String(env_data.apparent_temperature, 1) + ",";
    jsonPayload += "\"wind_chill\":" + String(env_data.wind_chill, 1) + ",";
    jsonPayload += "\"thom_index\":" + String(env_data.thom_index, 1) + ",";
    jsonPayload += "\"effective_temperature\":" + String(env_data.effective_temperature, 1) + ",";
    jsonPayload += "\"temperature_humidity_wind_index\":" + String(env_data.temperature_humidity_wind_index, 1) + ",";
    jsonPayload += "\"wet_bulb_temperature\":" + String(env_data.wet_bulb_temperature, 1) + ",";
    jsonPayload += "\"relative_strain_index\":" + String(env_data.relative_strain_index, 1) + ",";
    jsonPayload += "\"thermal_comfort_index\":" + String(env_data.thermal_comfort_index, 1);
    jsonPayload += "},";
    
    jsonPayload += "\"metadata\":{";
    jsonPayload += "\"version\":\"0.86/env\",";
    jsonPayload += "\"model\":\"RadiationD v1.1\",";
    jsonPayload += "\"location\":null,";
    jsonPayload += "\"gmcmap_active\":" + String(prefs.getString("aid", "") != "" ? "true" : "false") + ",";
    jsonPayload += "\"bme680_active\":" + String(bme_sensor_found ? "true" : "false") + ",";
    jsonPayload += "\"custom_server1_active\":" + String(custom_server_enabled ? "true" : "false") + ",";
    jsonPayload += "\"custom_server2_active\":" + String(custom_server2_enabled ? "true" : "false") + ",";
    jsonPayload += "\"update_interval_minutes\":" + String(update_interval_minutes);
    jsonPayload += "}";
    
    jsonPayload += "}";
    
    return jsonPayload;
}

void send_to_custom_server() {
    if (!custom_server_enabled || custom_server_url == "") {
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        custom_server_last_status = false;
        return;
    }

    HTTPClient http;
    http.setTimeout(10000);
    
    String jsonPayload = generate_json_payload();
    
    bool success = http.begin(custom_server_url);
    if (!success) {
        custom_server_last_status = false;
        return;
    }
    
    http.addHeader("Content-Type", "application/json");
    if (custom_server_token != "") {
        http.addHeader("Authorization", "Bearer " + custom_server_token);
    }
    
    int httpCode = http.POST(jsonPayload);
    
    if (httpCode > 0) {
        if (httpCode == 200) {
            custom_server_last_status = true;
            custom_server_last_response = http.getString();
        } else {
            custom_server_last_status = false;
            custom_server_last_response = http.getString();
        }
    } else {
        custom_server_last_status = false;
    }
    
    http.end();
    custom_server_last_attempt = millis();
}

void send_to_custom_server2() {
    if (!custom_server2_enabled || custom_server2_url == "") {
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        custom_server2_last_status = false;
        return;
    }

    HTTPClient http;
    http.setTimeout(10000);
    
    String jsonPayload = generate_json_payload();
    
    bool success = http.begin(custom_server2_url);
    if (!success) {
        custom_server2_last_status = false;
        return;
    }
    
    http.addHeader("Content-Type", "application/json");
    if (custom_server2_token != "") {
        http.addHeader("Authorization", "Bearer " + custom_server2_token);
    }
    
    int httpCode = http.POST(jsonPayload);
    
    if (httpCode > 0) {
        if (httpCode == 200) {
            custom_server2_last_status = true;
            custom_server2_last_response = http.getString();
        } else {
            custom_server2_last_status = false;
            custom_server2_last_response = http.getString();
        }
    } else {
        custom_server2_last_status = false;
    }
    
    http.end();
    custom_server2_last_attempt = millis();
}

// ==================== FUNKCJE WYKRESU ====================
void graphInitialize() {
    for(int i = 0; i < GRAPH_POINTS; i++) {
        graphData[i].cpm = 0.0;
        graphData[i].acpm = 0.0;
        graphData[i].usv_h = 0.0;
        graphData[i].timestamp = 0;
    }
    graphIndex = 0;
    graphFilled = false;
    graphInitialized = true;
    Serial.println("[GRAPH] Wykres promieniowania zainicjalizowany");
}

void envGraphInitialize() {
    for(int i = 0; i < GRAPH_POINTS; i++) {
        envGraphData[i].temperature = 0.0;
        envGraphData[i].humidity = 0.0;
        envGraphData[i].pressure = 0.0;
        envGraphData[i].iaq_score = 0.0;
        envGraphData[i].timestamp = 0;
    }
    envGraphIndex = 0;
    envGraphFilled = false;
    envGraphInitialized = true;
    Serial.println("[GRAPH] Wykres środowiska zainicjalizowany");
}

void graphLoad() {
    Serial.println("[GRAPH] Ładowanie wykresu promieniowania z NVS...");
    
    graphInitialized = prefs.getBool("graph_init", false);
    
    if (!graphInitialized) {
        Serial.println("[GRAPH] Wykres promieniowania nie był zainicjalizowany");
        graphInitialize();
        return;
    }
    
    graphIndex = prefs.getUShort("graph_idx", 0);
    graphFilled = prefs.getBool("graph_full", false);
    
    if (graphIndex >= GRAPH_POINTS) {
        Serial.printf("[GRAPH] Nieprawidłowy index: %d - reset\n", graphIndex);
        graphIndex = 0;
        graphFilled = false;
    }
    
    bool loadSuccess = true;
    const int CHUNK_SIZE = 10;
    int chunks = prefs.getUInt("graph_chunks", 0);
    
    if (chunks > 0) {
        Serial.printf("[GRAPH] Ładowanie %d fragmentów promieniowania...\n", chunks);
        
        for (uint32_t chunk = 0; chunk < chunks; chunk++) {
            char key[20];
            snprintf(key, sizeof(key), "graph_%d", chunk);
            
            size_t bytesRead = prefs.getBytesLength(key);
            if (bytesRead > 0 && bytesRead <= sizeof(GraphPoint) * CHUNK_SIZE) {
                int start_idx = chunk * CHUNK_SIZE;
                int points_in_chunk = min(CHUNK_SIZE, GRAPH_POINTS - start_idx);
                
                if (!prefs.getBytes(key, (uint8_t*)&graphData[start_idx], bytesRead)) {
                    Serial.printf("[GRAPH] Błąd odczytu fragmentu %d promieniowania\n", chunk);
                    loadSuccess = false;
                    break;
                }
            } else {
                loadSuccess = false;
                break;
            }
        }
    } else {
        // Stara metoda - pojedynczy plik
        size_t bytesRead = prefs.getBytesLength("graph_data");
        size_t expectedSize = sizeof(GraphPoint) * GRAPH_POINTS;
        
        if (bytesRead == expectedSize) {
            loadSuccess = prefs.getBytes("graph_data", (uint8_t*)graphData, expectedSize);
        } else {
            loadSuccess = false;
        }
    }
    
    if (!loadSuccess) {
        Serial.println("[GRAPH] Błąd ładowania danych promieniowania - reset");
        graphInitialize();
    } else {
        Serial.printf("[GRAPH] Załadowano wykres promieniowania: index=%d, filled=%d\n", graphIndex, graphFilled);
    }
}

void envGraphLoad() {
    Serial.println("[GRAPH] Ładowanie wykresu środowiska z NVS...");
    
    envGraphInitialized = prefs.getBool("env_graph_init", false);
    
    if (!envGraphInitialized) {
        Serial.println("[GRAPH] Wykres środowiska nie był zainicjalizowany");
        envGraphInitialize();
        return;
    }
    
    envGraphIndex = prefs.getUShort("env_graph_idx", 0);
    envGraphFilled = prefs.getBool("env_graph_full", false);
    
    if (envGraphIndex >= GRAPH_POINTS) {
        Serial.printf("[GRAPH] Nieprawidłowy index środowiska: %d - reset\n", envGraphIndex);
        envGraphIndex = 0;
        envGraphFilled = false;
    }
    
    bool loadSuccess = true;
    const int CHUNK_SIZE = 10;
    int chunks = prefs.getUInt("env_graph_chunks", 0);
    
    if (chunks > 0) {
        Serial.printf("[GRAPH] Ładowanie %d fragmentów środowiska...\n", chunks);
        
        for (uint32_t chunk = 0; chunk < chunks; chunk++) {
            char key[20];
            snprintf(key, sizeof(key), "env_graph_%d", chunk);
            
            size_t bytesRead = prefs.getBytesLength(key);
            if (bytesRead > 0 && bytesRead <= sizeof(EnvGraphPoint) * CHUNK_SIZE) {
                int start_idx = chunk * CHUNK_SIZE;
                int points_in_chunk = min(CHUNK_SIZE, GRAPH_POINTS - start_idx);
                
                if (!prefs.getBytes(key, (uint8_t*)&envGraphData[start_idx], bytesRead)) {
                    Serial.printf("[GRAPH] Błąd odczytu fragmentu %d środowiska\n", chunk);
                    loadSuccess = false;
                    break;
                }
            } else {
                loadSuccess = false;
                break;
            }
        }
    } else {
        // Stara metoda - pojedynczy plik
        size_t bytesRead = prefs.getBytesLength("env_graph_data");
        size_t expectedSize = sizeof(EnvGraphPoint) * GRAPH_POINTS;
        
        if (bytesRead == expectedSize) {
            loadSuccess = prefs.getBytes("env_graph_data", (uint8_t*)envGraphData, expectedSize);
        } else {
            loadSuccess = false;
        }
    }
    
    if (!loadSuccess) {
        Serial.println("[GRAPH] Błąd ładowania danych środowiska - reset");
        envGraphInitialize();
    } else {
        Serial.printf("[GRAPH] Załadowano wykres środowiska: index=%d, filled=%d\n", envGraphIndex, envGraphFilled);
    }
}

void graphSave() {
    static GraphPoint last_saved[GRAPH_POINTS];
    static bool first_save = true;
    
    if (!first_save && memcmp(graphData, last_saved, sizeof(graphData)) == 0) {
        return;
    }
    first_save = false;
    
    bool save1 = prefs.putUShort("graph_idx", graphIndex);
    bool save2 = prefs.putBool("graph_full", graphFilled);
    bool save3 = prefs.putBool("graph_init", true);
    
    const int CHUNK_SIZE = 10;
    int chunks = (GRAPH_POINTS + CHUNK_SIZE - 1) / CHUNK_SIZE;
    
    bool saveSuccess = true;
    for (int chunk = 0; chunk < chunks; chunk++) {
        char key[20];
        snprintf(key, sizeof(key), "graph_%d", chunk);
        
        int start_idx = chunk * CHUNK_SIZE;
        int points_in_chunk = min(CHUNK_SIZE, GRAPH_POINTS - start_idx);
        size_t chunkSize = sizeof(GraphPoint) * points_in_chunk;
        
        if (!prefs.putBytes(key, (uint8_t*)&graphData[start_idx], chunkSize)) {
            Serial.printf("[GRAPH] Błąd zapisu fragmentu %d promieniowania\n", chunk);
            saveSuccess = false;
            break;
        }
    }
    
    if (saveSuccess) {
        prefs.putUInt("graph_chunks", chunks);
        memcpy(last_saved, graphData, sizeof(graphData));
        Serial.printf("[GRAPH] Zapisano wykres promieniowania: index=%d, filled=%d\n", graphIndex, graphFilled);
    }
    
    nvs_write_count++;
}

void envGraphSave() {
    static EnvGraphPoint last_saved[GRAPH_POINTS];
    static bool first_save = true;
    
    if (!first_save && memcmp(envGraphData, last_saved, sizeof(envGraphData)) == 0) {
        return;
    }
    first_save = false;
    
    bool save1 = prefs.putUShort("env_graph_idx", envGraphIndex);
    bool save2 = prefs.putBool("env_graph_full", envGraphFilled);
    bool save3 = prefs.putBool("env_graph_init", true);
    
    const int CHUNK_SIZE = 10;
    int chunks = (GRAPH_POINTS + CHUNK_SIZE - 1) / CHUNK_SIZE;
    
    bool saveSuccess = true;
    for (int chunk = 0; chunk < chunks; chunk++) {
        char key[20];
        snprintf(key, sizeof(key), "env_graph_%d", chunk);
        
        int start_idx = chunk * CHUNK_SIZE;
        int points_in_chunk = min(CHUNK_SIZE, GRAPH_POINTS - start_idx);
        size_t chunkSize = sizeof(EnvGraphPoint) * points_in_chunk;
        
        if (!prefs.putBytes(key, (uint8_t*)&envGraphData[start_idx], chunkSize)) {
            Serial.printf("[GRAPH] Błąd zapisu fragmentu %d środowiska\n", chunk);
            saveSuccess = false;
            break;
        }
    }
    
    if (saveSuccess) {
        prefs.putUInt("env_graph_chunks", chunks);
        memcpy(last_saved, envGraphData, sizeof(envGraphData));
        Serial.printf("[GRAPH] Zapisano wykres środowiska: index=%d, filled=%d\n", envGraphIndex, envGraphFilled);
    }
    
    nvs_write_count++;
}

void graphAddPoint(float cpm, float acpm, float usv_h) {
    uint32_t now_seconds = get_current_unix_time();
    
    if (graphIndex > 0 || graphFilled) {
        int prevIndex;
        if (graphIndex == 0 && graphFilled) {
            prevIndex = GRAPH_POINTS - 1;
        } else if (graphIndex > 0) {
            prevIndex = graphIndex - 1;
        } else {
            prevIndex = 0;
        }
        
        if (graphData[prevIndex].timestamp >= now_seconds - (update_interval_minutes * 60 / 2)) {
            return;
        }
    }
    
    graphData[graphIndex].cpm = cpm;
    graphData[graphIndex].acpm = acpm;
    graphData[graphIndex].usv_h = usv_h;
    graphData[graphIndex].timestamp = now_seconds;
    
    graphIndex++;
    if (graphIndex >= GRAPH_POINTS) {
        graphIndex = 0;
        graphFilled = true;
    }
}

void envGraphAddPoint(float temp, float hum, float pres) {
    uint32_t now_seconds = get_current_unix_time();
    
    if (envGraphIndex > 0 || envGraphFilled) {
        int prevIndex;
        if (envGraphIndex == 0 && envGraphFilled) {
            prevIndex = GRAPH_POINTS - 1;
        } else if (envGraphIndex > 0) {
            prevIndex = envGraphIndex - 1;
        } else {
            prevIndex = 0;
        }
        
        if (envGraphData[prevIndex].timestamp >= now_seconds - (update_interval_minutes * 60 / 2)) {
            return;
        }
    }
    
    envGraphData[envGraphIndex].temperature = temp;
    envGraphData[envGraphIndex].humidity = hum;
    envGraphData[envGraphIndex].pressure = pres;
    envGraphData[envGraphIndex].iaq_score = env_data.iaq_score;
    envGraphData[envGraphIndex].timestamp = now_seconds;
    
    envGraphIndex++;
    if (envGraphIndex >= GRAPH_POINTS) {
        envGraphIndex = 0;
        envGraphFilled = true;
    }
}

uint32_t get_current_unix_time() {
    if (WiFi.status() == WL_CONNECTED) {
        time_t now;
        time(&now);
        return (uint32_t)now;
    } else {
        static uint32_t startup_unix = prefs.getUInt("last_known_unix", 0);
        static uint32_t startup_millis = 0;
        
        if (startup_millis == 0) {
            startup_millis = millis();
            if (startup_unix == 0) {
                startup_unix = 1704067200;
            }
        }
        
        return startup_unix + ((millis() - startup_millis) / 1000);
    }
}

// ==================== GENEROWANIE HTML DLA WYKRESÓW ====================
String generateGraphHTML() {
    String html = "";
    html += "<div class='card'>";
    html += "<h2>📈 Historia promieniowania</h2>";
    
    html += "<div style='margin: 20px 0;'>";
    html += "<canvas id='historyChart' width='100%' height='175'></canvas>";
    html += "</div>";
    
    html += "<div style='display: flex; justify-content: center; gap: 30px; margin-top: 15px; flex-wrap: wrap;'>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #4caf50;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>CPM</span>";
    html += "</div>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #2196f3;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>ACPM</span>";
    html += "</div>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #ff9800;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>µSv/h</span>";
    html += "</div>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<script>";
    html += "const graphData = [];";
    html += "const graphTimestamps = [];";
    
    uint32_t current_time = get_current_unix_time();
    
    int points_added = 0;
    int count = graphFilled ? GRAPH_POINTS : graphIndex;
    int startIdx = graphFilled ? graphIndex : 0;
    
    for (int i = 0; i < count; i++) {
        int idx = (startIdx + i) % GRAPH_POINTS;
        
        if (graphData[idx].timestamp > 0) {
            html += "graphData.push([" + String(graphData[idx].cpm, 1) + "," +
                    String(graphData[idx].acpm, 1) + "," +
                    String(graphData[idx].usv_h, 4) + "]);";
            
            int hours_ago = (current_time - graphData[idx].timestamp) / 3600;
            html += "graphTimestamps.push(" + String(hours_ago) + ");";
            points_added++;
        }
    }
    
    if (points_added == 0) {
        html += "graphData.push([0,0,0]);";
        html += "graphTimestamps.push(0);";
    }
    
    html += R"=====(
    function drawHistoryChart() {
        const canvas = document.getElementById('historyChart');
        if (!canvas) return;
        
        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.parentElement.clientWidth;
        const height = canvas.height;
        
        ctx.clearRect(0, 0, width, height);
        
        if (graphData.length < 2 || (graphData.length === 1 && graphData[0][0] === 0)) {
            ctx.font = '14px sans-serif';
            ctx.fillStyle = '#8b949e';
            ctx.textAlign = 'center';
            ctx.fillText('Zbieranie danych...', width/2, height/2);
            return;
        }
        
        const margin = {top: 20, right: 40, bottom: 40, left: 60};
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        let maxCPM_ACPM = 0.1;
        let maxUSV = 0.0001;
        
        graphData.forEach(point => {
            maxCPM_ACPM = Math.max(maxCPM_ACPM, point[0], point[1]);
            maxUSV = Math.max(maxUSV, point[2]);
        });
        
        maxCPM_ACPM *= 1.1;
        maxUSV *= 1.1;
        
        maxCPM_ACPM = Math.max(maxCPM_ACPM, 10);
        maxUSV = Math.max(maxUSV, 0.05);
        
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 0.5;
        ctx.fillStyle = '#8b949e';
        ctx.font = '10px sans-serif';
        
        for (let i = 0; i <= 5; i++) {
            const y = margin.top + (chartHeight * (5-i) / 5);
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(width - margin.right, y);
            ctx.stroke();
            
            ctx.textAlign = 'right';
            const cpm_acpmValue = (maxCPM_ACPM * i / 5).toFixed(0);
            ctx.fillText(cpm_acpmValue, margin.left - 5, y + 3);
        }
        
        for (let i = 0; i <= 5; i++) {
            const y = margin.top + (chartHeight * (5-i) / 5);
            ctx.textAlign = 'left';
            const usvValue = (maxUSV * i / 5).toFixed(4);
            ctx.fillText(usvValue, width - margin.right + 5, y + 3);
        }
        
        const timeLabels = [];
        if (graphTimestamps.length > 1) {
            const oldest = graphTimestamps[0];
            const newest = graphTimestamps[graphTimestamps.length - 1];
            
            for (let i = 0; i < 5; i++) {
                const hoursAgo = Math.round(oldest + (newest - oldest) * i / 4);
                if (hoursAgo === 0) {
                    timeLabels.push('Teraz');
                } else if (hoursAgo >= 24) {
                    timeLabels.push(Math.round(hoursAgo/24) + 'd');
                } else {
                    timeLabels.push(hoursAgo + 'h');
                }
            }
        } else {
            timeLabels = ['24h', '18h', '12h', '6h', 'Teraz'];
        }
        
        for (let i = 0; i < 5; i++) {
            const x = margin.left + (chartWidth * i / 4);
            ctx.beginPath();
            ctx.moveTo(x, height - margin.bottom);
            ctx.lineTo(x, height - margin.bottom + 5);
            ctx.stroke();
            
            ctx.textAlign = 'center';
            ctx.fillText(timeLabels[i], x, height - margin.bottom + 18);
        }
        
        function drawLine(dataIndex, color, scaleFactor = 1) {
            if (graphData.length < 2) return;
            
            ctx.beginPath();
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            
            const maxValue = (dataIndex === 0 || dataIndex === 1) ? maxCPM_ACPM : maxUSV;
            const actualMax = maxValue * scaleFactor;
            
            for (let i = 0; i < graphData.length; i++) {
                const x = margin.left + (chartWidth * i / (graphData.length - 1));
                const value = graphData[i][dataIndex] * scaleFactor;
                const y = margin.top + chartHeight - ((value / actualMax) * chartHeight);
                
                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            
            ctx.stroke();
        }
        
        drawLine(0, '#4caf50', 1);
        const usvScale = maxCPM_ACPM / maxUSV;
        drawLine(2, '#ff9800', usvScale);
        drawLine(1, '#2196f3', 1);
        
        ctx.fillStyle = '#58a6ff';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        
        ctx.save();
        ctx.translate(margin.left - 40, margin.top + chartHeight/2);
        ctx.rotate(-Math.PI/2);
        ctx.fillText('CPM / ACPM', 0, 0);
        ctx.restore();
        
        ctx.save();
        ctx.translate(width - margin.right + 30, margin.top + chartHeight/2);
        ctx.rotate(Math.PI/2);
        ctx.fillText('µSv/h', 0, 0);
        ctx.restore();
        
        ctx.fillStyle = '#c9d1d9';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'left';
        
        const legendY = margin.top - 10;
        ctx.fillStyle = '#4caf50';
        ctx.fillRect(margin.left, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('CPM', margin.left + 20, legendY + 4);
        
        ctx.fillStyle = '#2196f3';
        ctx.fillRect(margin.left + 60, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('ACPM', margin.left + 80, legendY + 4);
        
        ctx.fillStyle = '#ff9800';
        ctx.fillRect(margin.left + 130, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('µSv/h', margin.left + 150, legendY + 4);
        
        if (graphData.length > 0) {
            const lastPoint = graphData[graphData.length - 1];
            ctx.fillStyle = '#8b949e';
            ctx.font = '10px sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(`Ostatni: CPM=${lastPoint[0].toFixed(1)}, ACPM=${lastPoint[1].toFixed(1)}, µSv/h=${lastPoint[2].toFixed(4)}`, 
                        width - margin.right, margin.top - 20);
        }
    }
    
    drawHistoryChart();
    window.addEventListener('resize', drawHistoryChart);
    )=====";
    
    html += "</script>";
    
    return html;
}

String generateEnvGraphHTML() {
    String html = "";
    html += "<div class='card'>";
    html += "<h2>🌡️ Historia środowiska</h2>";
    
    html += "<div style='margin: 20px 0;'>";
    html += "<canvas id='envHistoryChart' width='100%' height='175'></canvas>";
    html += "</div>";
    
    html += "<div style='display: flex; justify-content: center; gap: 30px; margin-top: 15px; flex-wrap: wrap;'>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #ff4444;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>Temperatura (°C)</span>";
    html += "</div>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #44aaff;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>Wilgotność (%)</span>";
    html += "</div>";
    html += "<div style='display: flex; align-items: center; gap: 8px;'>";
    html += "<div style='width: 15px; height: 3px; background: #aa44ff;'></div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>Ciśnienie (hPa)</span>";
    html += "</div>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<script>";
    html += "const envGraphData = [];";
    html += "const envGraphTimestamps = [];";
    
    uint32_t current_time = get_current_unix_time();
    
    int points_added = 0;
    int count = envGraphFilled ? GRAPH_POINTS : envGraphIndex;
    int startIdx = envGraphFilled ? envGraphIndex : 0;
    
    for (int i = 0; i < count; i++) {
        int idx = (startIdx + i) % GRAPH_POINTS;
        
        if (envGraphData[idx].timestamp > 0) {
            html += "envGraphData.push([" + String(envGraphData[idx].temperature, 1) + "," +
                    String(envGraphData[idx].humidity, 1) + "," +
                    String(envGraphData[idx].pressure, 1) + "]);";
            
            int hours_ago = (current_time - envGraphData[idx].timestamp) / 3600;
            html += "envGraphTimestamps.push(" + String(hours_ago) + ");";
            points_added++;
        }
    }
    
    if (points_added == 0) {
        html += "envGraphData.push([0,0,0]);";
        html += "envGraphTimestamps.push(0);";
    }
    
    html += R"=====(
    function drawEnvHistoryChart() {
        const canvas = document.getElementById('envHistoryChart');
        if (!canvas) return;
        
        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.parentElement.clientWidth;
        const height = canvas.height;
        
        ctx.clearRect(0, 0, width, height);
        
        if (envGraphData.length < 2 || (envGraphData.length === 1 && envGraphData[0][0] === 0)) {
            ctx.font = '14px sans-serif';
            ctx.fillStyle = '#8b949e';
            ctx.textAlign = 'center';
            ctx.fillText('Zbieranie danych...', width/2, height/2);
            return;
        }
        
        const margin = {top: 20, right: 40, bottom: 40, left: 60};
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        // STAŁE SKALE
        const tempMin = -15;
        const tempMax = 40;
        const humMin = 0;
        const humMax = 100;
        const presMin = 940;
        const presMax = 1050;
        
        // Ustaw linie siatki
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 0.5;
        ctx.fillStyle = '#8b949e';
        ctx.font = '10px sans-serif';
        
        // Lewa oś - temperatura
        for (let i = 0; i <= 5; i++) {
            const y = margin.top + (chartHeight * (5-i) / 5);
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(width - margin.right, y);
            ctx.stroke();
            
            ctx.textAlign = 'right';
            const tempValue = tempMin + ((tempMax - tempMin) * i / 5);
            ctx.fillText(tempValue.toFixed(0), margin.left - 5, y + 3);
        }
        
        // Prawa oś - ciśnienie
        for (let i = 0; i <= 5; i++) {
            const y = margin.top + (chartHeight * (5-i) / 5);
            ctx.textAlign = 'left';
            const presValue = presMin + ((presMax - presMin) * i / 5);
            ctx.fillText(presValue.toFixed(0), width - margin.right + 5, y + 3);
        }
        
        // Oś czasu
        const timeLabels = [];
        if (envGraphTimestamps.length > 1) {
            const oldest = envGraphTimestamps[0];
            const newest = envGraphTimestamps[envGraphTimestamps.length - 1];
            
            for (let i = 0; i < 5; i++) {
                const hoursAgo = Math.round(oldest + (newest - oldest) * i / 4);
                if (hoursAgo === 0) {
                    timeLabels.push('Teraz');
                } else if (hoursAgo >= 24) {
                    timeLabels.push(Math.round(hoursAgo/24) + 'd');
                } else {
                    timeLabels.push(hoursAgo + 'h');
                }
            }
        } else {
            timeLabels = ['24h', '18h', '12h', '6h', 'Teraz'];
        }
        
        for (let i = 0; i < 5; i++) {
            const x = margin.left + (chartWidth * i / 4);
            ctx.beginPath();
            ctx.moveTo(x, height - margin.bottom);
            ctx.lineTo(x, height - margin.bottom + 5);
            ctx.stroke();
            
            ctx.textAlign = 'center';
            ctx.fillText(timeLabels[i], x, height - margin.bottom + 18);
        }
        
        // Rysowanie linii
        function drawLine(dataIndex, color, minVal, maxVal) {
            if (envGraphData.length < 2) return;
            
            ctx.beginPath();
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            
            const scale = maxVal - minVal;
            
            for (let i = 0; i < envGraphData.length; i++) {
                const x = margin.left + (chartWidth * i / (envGraphData.length - 1));
                let value = envGraphData[i][dataIndex];
                
                // Ogranicz wartości do zakresu wykresu
                value = Math.max(minVal, Math.min(maxVal, value));
                
                const normalizedValue = (value - minVal) / scale;
                const y = margin.top + chartHeight - (normalizedValue * chartHeight);
                
                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            
            ctx.stroke();
        }
        
        drawLine(0, '#ff4444', tempMin, tempMax); // temperatura
        drawLine(1, '#44aaff', humMin, humMax);  // wilgotność
        drawLine(2, '#aa44ff', presMin, presMax); // ciśnienie
        
        // Opisy osi
        ctx.fillStyle = '#58a6ff';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        
        ctx.save();
        ctx.translate(margin.left - 40, margin.top + chartHeight/2);
        ctx.rotate(-Math.PI/2);
        ctx.fillText('Temperatura (°C)', 0, 0);
        ctx.restore();
        
        ctx.save();
        ctx.translate(width - margin.right + 30, margin.top + chartHeight/2);
        ctx.rotate(Math.PI/2);
        ctx.fillText('Ciśnienie (hPa)', 0, 0);
        ctx.restore();
        
        // Legenda
        ctx.fillStyle = '#c9d1d9';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'left';
        
        const legendY = margin.top - 10;
        
        // Temperatura
        ctx.fillStyle = '#ff4444';
        ctx.fillRect(margin.left, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('Temperatura', margin.left + 20, legendY + 4);
        
        // Wilgotność
        ctx.fillStyle = '#44aaff';
        ctx.fillRect(margin.left + 120, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('Wilgotność', margin.left + 140, legendY + 4);
        
        // Ciśnienie
        ctx.fillStyle = '#aa44ff';
        ctx.fillRect(margin.left + 220, legendY, 15, 2);
        ctx.fillStyle = '#c9d1d9';
        ctx.fillText('Ciśnienie', margin.left + 240, legendY + 4);
        
        // Dodatkowa oś dla wilgotności (środkowa)
        ctx.save();
        ctx.translate(width/2, margin.top + chartHeight + 20);
        ctx.fillText('Wilgotność (%)', 0, 0);
        ctx.restore();
        
        // Ostatni punkt
        if (envGraphData.length > 0) {
            const lastPoint = envGraphData[envGraphData.length - 1];
            ctx.fillStyle = '#8b949e';
            ctx.font = '10px sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(`Ostatni: Temp=${lastPoint[0].toFixed(1)}°C, Hum=${lastPoint[1].toFixed(1)}%, Pres=${lastPoint[2].toFixed(1)}hPa`, 
                        width - margin.right, margin.top - 20);
        }
    }
    
    drawEnvHistoryChart();
    window.addEventListener('resize', drawEnvHistoryChart);
    )=====";
    
    html += "</script>";
    
    return html;
}

// ==================== LOGIKA SYSTEMOWA ====================
void update_uptime() {
    uint32_t up = millis() / 1000;
    snprintf(uptime_str, sizeof(uptime_str), "%ludd %02lu:%02lu:%02lu", 
             up/86400, (up/3600)%24, (up/60)%60, up%60);
}

void setup_time() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
}

void save_all_data() {
    // Zapis wszystkich kluczowych danych
    prefs.putUInt("tube_pulses", total_lifetime_pulses);
    prefs.putUInt("tube_start_time", tube_start_time);
    prefs.putFloat("daily_dose", daily_dose);
    prefs.putFloat("factor", conversion_factor); // DODANO: zapis współczynnika kalibracji
    
    uint32_t current_unix = get_current_unix_time();
    prefs.putUInt("last_known_unix", current_unix);
    
    graphSave();
    envGraphSave();
    
    prefs.putUInt("nvs_write_count", nvs_write_count);
    
    Serial.println("[NVS] Zapisano wszystkie dane systemowe");
}

void save_all_data_combined() {
    uint32_t now = millis();
    if (now - last_all_data_save >= ALL_DATA_SAVE_INTERVAL) {
        save_all_data();
        last_all_data_save = now;
    }
}

void load_tube_state() {
    total_lifetime_pulses = prefs.getUInt("tube_pulses", 0);
    tube_start_time = prefs.getUInt("tube_start_time", 0);
    
    uint32_t current_unix = get_current_unix_time();
    
    if (tube_start_time == 0) {
        tube_start_time = current_unix;
        prefs.putUInt("tube_start_time", tube_start_time);
    } else {
        if (tube_start_time > current_unix) {
            tube_start_time = current_unix;
            prefs.putUInt("tube_start_time", tube_start_time);
        } else if (current_unix - tube_start_time > 31536000 * 5) {
            tube_start_time = current_unix;
            total_lifetime_pulses = 0;
            prefs.putUInt("tube_start_time", tube_start_time);
            prefs.putUInt("tube_pulses", 0);
        }
    }
}

void reset_tube_counter() {
    total_lifetime_pulses = 0;
    tube_start_time = get_current_unix_time();
    tube_lifetime_percent = 0.0;
    save_all_data();
}

uint32_t calculate_tube_uptime() {
    uint32_t current_unix = get_current_unix_time();
    if (tube_start_time > 0 && current_unix > tube_start_time) {
        return current_unix - tube_start_time;
    }
    return 0;
}

String format_years_days(uint32_t seconds) {
    if (seconds == 0) return "0 dni";
    
    uint32_t years = seconds / 31536000;
    uint32_t days = (seconds % 31536000) / 86400;
    uint32_t hours = (seconds % 86400) / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    
    char buffer[64];
    if (years > 0) {
        snprintf(buffer, sizeof(buffer), "%u lat, %u dni, %02u:%02u", years, days, hours, minutes);
    } else if (days > 0) {
        snprintf(buffer, sizeof(buffer), "%u dni, %02u:%02u", days, hours, minutes);
    } else if (hours > 0) {
        snprintf(buffer, sizeof(buffer), "%u godzin, %u minut", hours, minutes);
    } else {
        snprintf(buffer, sizeof(buffer), "%u minut", minutes);
    }
    return String(buffer);
}

void update_measurements() {
    uint32_t now = millis();

    if (time_elapsed(last_second_update, 1000)) {
        uint32_t pulses;
        noInterrupts();
        pulses = pulse_count;
        pulse_count = 0;
        interrupts();

        second_buffer[buffer_index] = pulses;
        buffer_index = (buffer_index + 1) % ACPM_WINDOW_SIZE;
        total_pulses += pulses;
        total_lifetime_pulses += pulses;
    }

    if (time_elapsed(last_buffer_shift, 1000)) {
        uint32_t sum_120 = 0;
        for(int i=0; i<CPM_WINDOW_SIZE; i++) {
            int idx = (buffer_index - 1 - i + ACPM_WINDOW_SIZE) % ACPM_WINDOW_SIZE;
            sum_120 += second_buffer[idx];
        }
        current_cpm = (float)sum_120 * (60.0 / CPM_WINDOW_SIZE);

        uint32_t sum_600 = 0;
        for(int i=0; i<ACPM_WINDOW_SIZE; i++) {
            sum_600 += second_buffer[i];
        }
        current_acpm = (float)sum_600 * (60.0 / ACPM_WINDOW_SIZE);

        current_usv_h = current_acpm * conversion_factor;

        static uint32_t last_calc = 0;
        if (last_calc > 0) {
            float h = (now - last_calc) / 3600000.0;
            daily_dose += current_usv_h * h;
        }
        last_calc = now;
    }

    if (time_elapsed(last_stability_calc, STABILITY_CALC_INTERVAL)) {
        background_history[history_index] = current_acpm;
        history_index = (history_index + 1) % 60;
        float sum = 0, mean = 0, sq_diff = 0;
        int count = 0;
        for(int i=0; i<60; i++) { 
            if(background_history[i] > 0) { 
                sum += background_history[i]; 
                count++; 
            } 
        }
        if(count > 10) {
            mean = sum / count;
            for(int i=0; i<60; i++) { 
                if(background_history[i] > 0) {
                    sq_diff += pow(background_history[i] - mean, 2); 
                }
            }
            float std_dev = sqrt(sq_diff / count);
            background_stability = (mean > 0) ? (1.0 - (std_dev/mean)) * 100.0 : 0;
            if(background_stability < 0) background_stability = 0;
        }
    }
    
    tube_lifetime_percent = (float)total_lifetime_pulses / TUBE_LIFETIME * 100.0;
    if (tube_lifetime_percent > 100.0) tube_lifetime_percent = 100.0;
    
    if (time_elapsed(last_graph_update, update_interval_ms)) {
        graphAddPoint(current_cpm, current_acpm, current_usv_h);
    }
    
    if (time_elapsed(last_cpu_calc, 1000)) {
        uint32_t total_time = 0;
        for(int i = 0; i < CPU_AVG_WINDOW; i++) {
            total_time += cpu_loop_times[i];
        }
        
        if (total_time > 0) {
            float avg_loop_time = total_time / (float)CPU_AVG_WINDOW;
            cpu_usage = min(100.0, (avg_loop_time / 1000.0) * 100.0);
        }
    }
    
    save_all_data_combined();
}

// ==================== NVS STATISTICS ====================
void log_nvs_status() {
    if (time_elapsed(last_nvs_log, NVS_LOG_INTERVAL)) {
        float nvs_usage_percent = (float)nvs_write_count / NVS_MAX_WRITES * 100.0;
        
        if (nvs_write_count > 0 && nvs_write_count < NVS_MAX_WRITES) {
            uint32_t system_uptime_days = millis() / 86400000;
            if (system_uptime_days > 0) {
                float writes_per_day = (float)nvs_write_count / system_uptime_days;
                if (writes_per_day > 0) {
                    float days_remaining = (NVS_MAX_WRITES - nvs_write_count) / writes_per_day;
                    float years_remaining = days_remaining / 365.0;
                    
                    Serial.printf("[NVS] Zapisów: %u (%.1f%%) | Szacowany czas: %.1f dni (%.1f lat)\n",
                                 nvs_write_count, nvs_usage_percent, days_remaining, years_remaining);
                } else {
                    Serial.printf("[NVS] Zapisów: %u (%.1f%%)\n", nvs_write_count, nvs_usage_percent);
                }
            } else {
                Serial.printf("[NVS] Zapisów: %u (%.1f%%)\n", nvs_write_count, nvs_usage_percent);
            }
        } else if (nvs_write_count >= NVS_MAX_WRITES) {
            Serial.printf("[NVS] OSTRZEŻENIE: Osiągnięto limit %u zapisów!\n", NVS_MAX_WRITES);
        }
    }
}

// ==================== mDNS & WiFi ====================
void init_mdns() {
    delay(100);
    
    if (!MDNS.begin(MDNS_PRIMARY_NAME)) {
        if (!MDNS.begin(MDNS_SECONDARY_NAME)) {
            return;
        }
    }
    
    static bool servicesAdded = false;
    if (!servicesAdded) {
        MDNS.addService("http", "tcp", 80);
        
        MDNS.addServiceTxt("http", "tcp", "device", "GeigerCounter");
        MDNS.addServiceTxt("http", "tcp", "version", "0.86/env");
        MDNS.addServiceTxt("http", "tcp", "model", "RadiationD v1.1");
        servicesAdded = true;
    }
}

void wifi_event_handler(WiFiEvent_t event) {
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            wifi_connected = true;
            wifi_reconnecting = false;
            ap_mode = false;
            break;
            
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            device_ip = WiFi.localIP().toString();
            digitalWrite(LED_PIN, LOW);
            
            init_mdns();
            setup_time();
            break;
            
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            wifi_connected = false;
            wifi_reconnecting = true;
            digitalWrite(LED_PIN, HIGH);
            wifi_disconnect_time = millis();
            break;
            
        default:
            break;
    }
}

void connect_wifi() {
    String ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
    String pass = prefs.getString("pass", DEFAULT_WIFI_PASS);
    
    if (pass == DEFAULT_WIFI_PASS) {
        Serial.println("[SECURITY] Używane domyślne hasło! Zmień w konfiguracji.");
    }
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
WiFi.setTxPower(WIFI_POWER_19_5dBm);
    
    WiFi.setHostname(MDNS_PRIMARY_NAME);
    
    WiFi.onEvent(wifi_event_handler);
    
    WiFi.begin(ssid.c_str(), pass.c_str());
    wifi_connect_start = millis();
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        wifi_reconnecting = false;
    } else {
        start_ap_mode();
    }
}

void start_ap_mode() {
    if (ap_mode) return;
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Geiger-AP", "1234567890");
    device_ip = WiFi.softAPIP().toString();
    ap_mode = true;
    wifi_connected = false;
    wifi_reconnecting = false;
    
    digitalWrite(LED_PIN, HIGH);
}

void check_wifi_reconnect() {
    if (!wifi_connected && !ap_mode) {
        if (time_elapsed(wifi_disconnect_time, WIFI_AP_TIMEOUT)) {
            start_ap_mode();
        }
        else if (time_elapsed(last_wifi_reconnect, WIFI_RECONNECT_INTERVAL)) {
            WiFi.reconnect();
        }
    }
    else if (ap_mode && !wifi_reconnecting) {
        if (WiFi.scanNetworks() > 0) {
            String saved_ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
            for (int i = 0; i < WiFi.scanComplete(); i++) {
                if (WiFi.SSID(i) == saved_ssid) {
                    wifi_reconnecting = true;
                    WiFi.mode(WIFI_STA);
                    WiFi.begin(saved_ssid.c_str(), prefs.getString("pass", DEFAULT_WIFI_PASS).c_str());
                    wifi_connect_start = millis();
                    break;
                }
            }
        }
    }
}

// ==================== STRONY WWW ====================
void handle_root() {
    String html = String(HTML_HEAD);
    
    html += "<div class='header'>";
    html += "<h1>☢ Geiger DIY Monitor v0.86/env</h1>";
    html += "<div class='subtitle'>System monitorowania promieniowania i środowiska</div>";
    html += "</div>";
    
    html += "<div class='status-bar'>";
    html += "<div class='status-item'><span class='status-icon'>⏱</span> Czas pracy: " + String(uptime_str) + "</div>";
    
    if (wifi_connected) {
        int rssi = WiFi.RSSI();
        String wifi_class = (rssi > -60) ? "status-wifi-good" : 
                           (rssi > -70) ? "status-wifi-weak" : "status-wifi-bad";
        
        html += "<div class='status-item'><span class='status-icon " + wifi_class + "'>📡</span> WiFi: " + String(rssi) + " dBm</div>";
        html += "<div class='status-item'><span class='status-icon'>🔌</span> IP: " + device_ip + "</div>";
    } else if (ap_mode) {
        html += "<div class='status-item'><span class='status-icon status-wifi-weak'>📡</span> Tryb: Konfiguracja AP</div>";
        html += "<div class='status-item'><span class='status-icon'>🔌</span> IP: " + device_ip + "</div>";
    } else {
        html += "<div class='status-item'><span class='status-icon status-wifi-bad'>📡</span> WiFi: Rozłączono</div>";
        html += "<div class='status-item'><span class='status-icon'>🔄</span> Próba ponownego połączenia...</div>";
    }
    
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📊 Aktualne pomiary promieniowania</h2>";
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Moc dawki</div>";
    html += "<span class='stat-value'>" + String(current_usv_h, 3) + "</span>";
    html += "<span class='unit'>µSv/h</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Średnia 10-minutowa</div>";
    html += "<span class='stat-value'>" + String(current_acpm, 1) + "</span>";
    html += "<span class='unit'>ACPM</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Chwilowa 2-minutowa</div>";
    html += "<span class='stat-value'>" + String(current_cpm, 1) + "</span>";
    html += "<span class='unit'>CPM</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Stabilność pomiaru</div>";
    html += "<span class='stat-value'>" + String(background_stability, 1) + "</span>";
    html += "<span class='unit'>%</span>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🌡️ Aktualne warunki środowiskowe</h2>";
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Temperatura</div>";
    html += "<span class='stat-value'>" + String(env_data.temperature, 1) + "</span>";
    html += "<span class='unit'>°C</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Wilgotność</div>";
    html += "<span class='stat-value'>" + String(env_data.humidity, 1) + "</span>";
    html += "<span class='unit'>%</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Ciśnienie</div>";
    html += "<span class='stat-value'>" + String(env_data.pressure, 1) + "</span>";
    html += "<span class='unit'>hPa</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Jakość powietrza</div>";
    html += "<span class='stat-value'>" + String(env_data.iaq_score, 0) + "</span>";
    html += "<span class='unit'>" + env_data.iaq_level + "</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div style='margin-top: 20px; padding: 15px; background: rgba(13, 17, 23, 0.5); border-radius: 8px; border: 1px solid var(--border);'>";
    html += "<div style='display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px;'>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Punkt rosy</div>";
    html += "<div class='env-value'>" + String(env_data.dew_point, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Wskaźnik ciepła</div>";
    html += "<div class='env-value'>" + String(env_data.heat_index, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Wilgotność bezwzgl.</div>";
    html += "<div class='env-value'>" + String(env_data.absolute_humidity, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>g/m³</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Gęstość powietrza</div>";
    html += "<div class='env-value'>" + String(env_data.air_density, 2) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>kg/m³</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Prędkość dźwięku</div>";
    html += "<div class='env-value'>" + String(env_data.sound_speed, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>m/s</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Humidex</div>";
    html += "<div class='env-value'>" + String(env_data.humidex, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Wskaźnik dyskomfortu</div>";
    html += "<div class='env-value'>" + String(env_data.discomfort_index, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>DI</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Summer Simmer Index</div>";
    html += "<div class='env-value'>" + String(env_data.summer_simmer_index, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>SSI</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Temperatura odczuwalna</div>";
    html += "<div class='env-value'>" + String(env_data.apparent_temperature, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Wind Chill</div>";
    html += "<div class='env-value'>" + String(env_data.wind_chill, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Thom Index</div>";
    html += "<div class='env-value'>" + String(env_data.thom_index, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>TI</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>Wet Bulb Temp</div>";
    html += "<div class='env-value'>" + String(env_data.wet_bulb_temperature, 1) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>°C</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>CO₂ ekwiwalent</div>";
    html += "<div class='env-value'>" + String(env_data.co2_equivalent, 0) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>ppm</span>";
    html += "</div>";
    
    html += "<div class='environment-box'>";
    html += "<div class='env-label'>VOC ekwiwalent</div>";
    html += "<div class='env-value'>" + String(env_data.voc_equivalent, 2) + "</div>";
    html += "<span style='font-size: 0.8rem; color: #8b949e;'>VOC</span>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    html += "</div>";
    
    html += generateGraphHTML();
    html += generateEnvGraphHTML();
    
    html += "<div class='card'>";
    html += "<h2>📈 Statystyki systemu</h2>";
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Dawka dzienna</div>";
    html += "<span class='stat-value'>" + String(daily_dose, 4) + "</span>";
    html += "<span class='unit'>µSv</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Licznik całkowity</div>";
    html += "<span class='stat-value'>" + String(total_pulses) + "</span>";
    html += "<span class='unit'>impulsów</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Pamięć dostępna</div>";
    html += "<span class='stat-value'>" + String(ESP.getFreeHeap() / 1024) + "</span>";
    html += "<span class='unit'>KB</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Obciążenie CPU</div>";
    html += "<span class='stat-value'>" + String(cpu_usage, 1) + "</span>";
    html += "<span class='unit'>%</span>";
    html += "</div>";
    
    html += "</div>";
    
    uint32_t tube_total_uptime = calculate_tube_uptime();
    float days = tube_total_uptime / 86400.0;
    float daily_avg = (days > 0.1) ? (total_lifetime_pulses / days) : 0;
    
    html += "<div style='margin-top: 15px; padding: 15px; background: rgba(13, 17, 23, 0.5); border-radius: 8px; border: 1px solid var(--border);'>";
    html += "<div style='display: flex; align-items: center; justify-content: space-between;'>";
    html += "<div>";
    html += "<div style='font-weight: bold;'>Informacje o tubie Geigera:</div>";
    html += "<span style='font-size: 0.9rem; color: #8b949e;'>Zużycie: " + String(tube_lifetime_percent, 1) + "%, Średnio: " + String(daily_avg, 1) + " imp/dzień</span>";
    html += "</div>";
    html += "<a href='/tube_info' class='btn' style='width: auto; padding: 8px 15px; font-size: 0.9rem; background: linear-gradient(135deg, #d29922, #bb8009);'>🔬 Szczegóły</a>";
    html += "</div>";
    html += "</div>";
    
    html += "<div style='margin-top: 20px; display: flex; gap: 15px; flex-wrap: wrap;'>";
    html += "<a href='/restart' class='btn btn-warning' style='width: auto; flex: 1; min-width: 200px;' onclick=\"return confirm('Czy na pewno chcesz zrestartować urządzenie?');\">🔄 Restartuj system</a>";
    html += "<a href='/reset_graph' class='btn btn-warning' style='width: auto; flex: 1; min-width: 200px;' onclick=\"return confirm('Czy na pewno chcesz usunąć całą historię wykresów?\\n\\nWszystkie zebrane dane wykresów zostaną trwale usunięte.\\nNowe wykresy rozpoczną się od zera.');\">🗑️ Resetuj wykresy</a>";
    html += "<a href='/info' class='btn' style='width: auto; flex: 1; min-width: 200px; background: linear-gradient(135deg, var(--sec), #1f6feb);'>ℹ️ Szczegółowe informacje</a>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>⚙ Konfiguracja systemu</h2>";
    html += "<form method='POST' action='/save' id='configForm' onsubmit='return validateForm()'>";
    
    html += "<h3>📶 Konfiguracja WiFi</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>WiFi SSID:</label>";
    html += "<input type='text' class='form-input' name='ssid' value='" + prefs.getString("ssid", DEFAULT_WIFI_SSID) + "' required>";
    html += "<div class='form-help'>Nazwa sieci WiFi do połączenia</div>";
    html += "<div class='error-message' id='error-ssid'></div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>WiFi Hasło:</label>";
    html += "<input type='password' class='form-input' id='wifiPass' name='pass' value='" + prefs.getString("pass", DEFAULT_WIFI_PASS) + "' required>";
    html += "<div class='form-help'>Hasło do sieci WiFi (min. 8 znaków)</div>";
    html += "<div class='error-message' id='error-pass'></div>";
    html += "</div>";
    
    html += "<h3>⏰ Ustawienia czasowe</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>Interwał aktualizacji (minuty):</label>";
    html += "<input type='number' class='form-input' name='update_interval' value='" + String(update_interval_minutes) + "' min='1' max='60' required>";
    html += "<div class='form-help'>Częstotliwość wysyłki danych i aktualizacji wykresu (1-60 minut)</div>";
    html += "<div class='error-message' id='error-interval'></div>";
    html += "</div>";
    
    html += "<h3>📏 Kalibracja licznika</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>Współczynnik kalibracji (Factor):</label>";
    html += "<input type='text' class='form-input' name='factor' value='" + String(conversion_factor, 6) + "' required>";
    html += "<div class='form-help'>Przelicznik CPM → µSv/h (0.0001 - 0.01)</div>";
    html += "<div class='error-message' id='error-factor'></div>";
    html += "</div>";
    
    html += "<h3>🌐 GMCMap.com</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>GMCMap Account ID (AID):</label>";
    html += "<input type='text' class='form-input' name='aid' value='" + prefs.getString("aid", "") + "'>";
    html += "<div class='form-help'>ID konta z gmcmap.com (opcjonalne)</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>GMCMap Geiger ID (GID):</label>";
    html += "<input type='text' class='form-input' name='gid' value='" + prefs.getString("gid", "") + "'>";
    html += "<div class='form-help'>Nazwa licznika na gmcmap.com (opcjonalne)</div>";
    html += "</div>";
    
    html += "<h3>🌐 Serwer własny 1 (JSON)</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>URL serwera 1:</label>";
    html += "<input type='text' class='form-input' name='custom_url' value='" + custom_server_url + "' placeholder='http://serwer1.pl/api/geiger-data'>";
    html += "<div class='form-help'>Pełny adres URL do wysyłki danych JSON</div>";
    html += "<div class='error-message' id='error-url'></div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>Token API (opcjonalny):</label>";
    html += "<input type='password' class='form-input' name='custom_token' value='" + custom_server_token + "' placeholder='Token autoryzacyjny'>";
    html += "<div class='form-help'>Token do autoryzacji na serwerze 1</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label' style='display: flex; align-items: center; gap: 10px;'>";
    html += "<input type='checkbox' name='custom_enabled' value='1' " + String(custom_server_enabled ? "checked" : "") + " style='width: auto;'>";
    html += "Aktywuj wysyłkę do serwera 1";
    html += "</label>";
    html += "</div>";
    
    html += "<h3>🌐 Serwer własny 2 (JSON)</h3>";
    html += "<div class='form-group'>";
    html += "<label class='form-label'>URL serwera 2:</label>";
    html += "<input type='text' class='form-input' name='custom2_url' value='" + custom_server2_url + "' placeholder='http://serwer2.pl/api/geiger-data'>";
    html += "<div class='form-help'>Drugi serwer do wysyłki danych JSON</div>";
    html += "<div class='error-message' id='error-url2'></div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>Token API (opcjonalny):</label>";
    html += "<input type='password' class='form-input' name='custom2_token' value='" + custom_server2_token + "' placeholder='Token autoryzacyjny'>";
    html += "<div class='form-help'>Token do autoryzacji na serwerze 2</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label' style='display: flex; align-items: center; gap: 10px;'>";
    html += "<input type='checkbox' name='custom2_enabled' value='1' " + String(custom_server2_enabled ? "checked" : "") + " style='width: auto;'>";
    html += "Aktywuj wysyłkę do serwera 2";
    html += "</label>";
    html += "</div>";
    
    html += "<div class='warning-box'>";
    html += "<div class='warning-title'>⚠️ UWAGA</div>";
    html += "Po zapisaniu konfiguracji urządzenie automatycznie się zrestartuje. Upewnij się, że wprowadzone dane są poprawne.";
    html += "</div>";
    
    html += "<button type='submit' class='btn'>💾 Zapisz konfigurację i zrestartuj urządzenie</button>";
    
    html += "</form>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🛠 Kalibracja licznika Geigera</h2>";
    
    html += "<div class='calibration-box'>";
    html += "<div class='info-title'>📐 Automatyczna kalibracja</div>";
    
    html += "<div class='calibration-step'>1. Pozostaw licznik na 15-20 minut</div>";
    html += "<p>Upewnij się, że licznik stoi w miejscu z naturalnym tłem (z dala od źródeł promieniowania).</p>";
    
    html += "<div class='calibration-step'>2. Odczytaj stabilną wartość ACPM</div>";
    html += "<p>Poczekaj aż ACPM się ustabilizuje (powinno być >80% stabilności).</p>";
    
    html += "<div class='calibration-step'>3. Wprowadź referencyjną wartość tła</div>";
    html += "<div class='code'>// Dla Polski użyj mapy Państwowej Agencji Atomistyki<br>// https://monitoring.paa.gov.pl/maps-portal/<br>// Znajdź swoją lokalizację i odczytaj wartość (np. 0.09 µSv/h)</div>";
    
    html += "<div class='calibration-step'>4. Oblicz nowy współczynnik</div>";
    html += "<div class='code'>NOWY_FACTOR = (Wartość referencyjna w µSv/h) ÷ (ACPM)</div>";
    
    html += "<div class='success-box'>";
    html += "<div style='font-weight: bold; color: var(--success); margin-bottom: 10px;'>PRZYKŁAD:</div>";
    html += "<p><strong>Mapa PAA:</strong> 0.09 µSv/h</p>";
    html += "<p><strong>Odczyt ACPM:</strong> 30.3 impulsów/minutę</p>";
    html += "<p><strong>Obliczenia:</strong> Factor = 0.09 ÷ 30.3 = 0.00297</p>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🎓 Objaśnienia pomiarów</h2>";
    
    html += "<div class='legend-grid'>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>µSv/h (mikrosiwert na godzinę)</div>";
    html += "<div class='legend-desc'>Jednostka równoważnika dawki pochłoniętej promieniowania jonizującego. Określa biologiczny efekt promieniowania na organizmy żywe. Typowe tło naturalne w Polsce: 0.08-0.12 µSv/h.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>CPM (Counts Per Minute)</div>";
    html += "<div class='legend-desc'>Liczba impulsów na minutę. Bezpośredni pomiar z tuby Geigera-Müllera. Średnia krocząca z 2 minut. Wartość chwilowa, podatna na fluktuacje statystyczne.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>ACPM (Average CPM)</div>";
    html += "<div class='legend-desc'>Średnia krocząca z 10 minut. Najbardziej wiarygodna wartość do określania tła naturalnego. Wykorzystywana do obliczeń dawki. Mniej podatna na fluktuacje statystyczne.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>IAQ (Indeks jakości powietrza)</div>";
    html += "<div class='legend-desc'>Wskaźnik jakości powietrza obliczany na podstawie wilgotności i oporu gazu. 90-100: doskonała, 70-89: dobra, 50-69: średnia, 25-49: słaba, 0-24: bardzo słaba.</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🔗 Metody dostępu do systemu</h2>";
    html += "<div class='access-methods'>";
    
    html += "<div class='access-card'>";
    html += "<div class='access-icon'>🔢</div>";
    html += "<div class='access-title'>Adres IP (zalecane)</div>";
    html += "<p>Zawsze działa na każdym systemie</p>";
    html += "<div class='url-box'>";
    html += "<code class='url-code'>http://" + device_ip + "</code>";
    html += "</div>";
    html += "</div>";
    
    if (wifi_connected) {
        html += "<div class='access-card'>";
        html += "<div class='access-icon'>🔍</div>";
        html += "<div class='access-title'>Nazwa mDNS (geiger.local)</div>";
        html += "<p>Działa na macOS, Linux, iOS, Android</p>";
        html += "<div class='url-box'>";
        html += "<code class='url-code'>http://geiger.local</code>";
        html += "</div>";
        html += "</div>";
        
        html += "<div class='access-card'>";
        html += "<div class='access-icon'>📱</div>";
        html += "<div class='access-title'>Alternatywna nazwa</div>";
        html += "<p>Dla systemów z problemami</p>";
        html += "<div class='url-box'>";
        html += "<code class='url-code'>http://geigercnt.local</code>";
        html += "</div>";
        html += "</div>";
    }
    
    html += "</div>";
    
    html += "<div style='text-align: center; margin-top: 30px;'>";
    html += "<div class='qr-code'>";
    html += "<img src='https://api.qrserver.com/v1/create-qr-code/?size=150x150&data=http://" + device_ip + "' alt='QR Code'>";
    html += "</div>";
    html += "<p style='margin-top: 10px; font-size: 0.9rem; color: #8b949e;'>Zeskanuj kod QR aby szybko przejść do panelu</p>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='footer'>";
    html += "<p>Geiger DIY Monitor v0.86/env | Autor: MaxGyver | Hardware: RadiationD v1.1 + ESP32-C3 mini + BME680</p>";
    html += "<p>WiFi: " + String(wifi_connected ? "Połączono" : ap_mode ? "Tryb AP" : "Rozłączono") + " | Ostatni pomiar: " + String(millis() / 1000) + "s</p>";
    html += "</div>";
    
    html += "</div>";
    
    html += R"=====(
    <script>
    function showError(id, message) {
        const element = document.getElementById('error-' + id);
        element.textContent = message;
        element.style.display = 'block';
        return false;
    }
    
    function hideError(id) {
        const element = document.getElementById('error-' + id);
        element.style.display = 'none';
        return true;
    }
    
    function validateForm() {
        let isValid = true;
        
        const ssid = document.querySelector('input[name="ssid"]').value;
        if (ssid.length === 0) {
            isValid = showError('ssid', 'SSID nie może być pusty');
        } else if (ssid.length > 32) {
            isValid = showError('ssid', 'SSID zbyt długi (max 32 znaki)');
        } else {
            hideError('ssid');
        }
        
        const password = document.querySelector('input[name="pass"]').value;
        if (password.length < 8) {
            isValid = showError('pass', 'Hasło zbyt krótkie (min 8 znaków)');
        } else if (password.length > 63) {
            isValid = showError('pass', 'Hasło zbyt długie (max 63 znaki)');
        } else {
            hideError('pass');
        }
        
        const interval = document.querySelector('input[name="update_interval"]').value;
        const intervalNum = parseInt(interval);
        if (isNaN(intervalNum) || intervalNum < 1 || intervalNum > 60) {
            isValid = showError('interval', 'Interwał musi być między 1 a 60 minut');
        } else {
            hideError('interval');
        }
        
        const factor = document.querySelector('input[name="factor"]').value;
        const factorNum = parseFloat(factor.replace(',', '.'));
        if (isNaN(factorNum) || factorNum < 0.0001 || factorNum > 0.01) {
            isValid = showError('factor', 'Współczynnik musi być między 0.0001 a 0.01');
        } else {
            hideError('factor');
        }
        
        const url = document.querySelector('input[name="custom_url"]').value;
        if (url.length > 0 && !url.startsWith('http://') && !url.startsWith('https://')) {
            isValid = showError('url', 'URL musi zaczynać się od http:// lub https://');
        } else {
            hideError('url');
        }
        
        const url2 = document.querySelector('input[name="custom2_url"]').value;
        if (url2.length > 0 && !url2.startsWith('http://') && !url2.startsWith('https://')) {
            isValid = showError('url2', 'URL serwera 2 musi zaczynać się od http:// lub https://');
        } else {
            hideError('url2');
        }
        
        if (!isValid) {
            return false;
        }
        
        if (!confirm('Czy na pewno chcesz zapisać konfigurację i zrestartować urządzenie?')) {
            return false;
        }
        
        return true;
    }
    
    document.getElementById('configForm').addEventListener('submit', function(e) {
        if (!validateForm()) {
            e.preventDefault();
        }
    });
    </script>
    )=====";
    
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_save() {
    String errors = "";
    
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    String interval_str = server.arg("update_interval");
    String factor_str = server.arg("factor");
    String custom_url = server.arg("custom_url");
    String custom2_url = server.arg("custom2_url");
    
    String ssid_error = validate_input(ssid, "ssid");
    String pass_error = validate_input(pass, "password");
    String interval_error = validate_input(interval_str, "int", 1, 60);
    String factor_error = validate_input(factor_str, "float", 0.0001, 0.01);
    String url_error = validate_input(custom_url, "url");
    String url2_error = validate_input(custom2_url, "url");
    
    if (ssid_error != "") errors += "SSID: " + ssid_error + "<br>";
    if (pass_error != "") errors += "Hasło: " + pass_error + "<br>";
    if (interval_error != "") errors += "Interwał: " + interval_error + "<br>";
    if (factor_error != "") errors += "Współczynnik: " + factor_error + "<br>";
    if (url_error != "") errors += "URL serwer 1: " + url_error + "<br>";
    if (url2_error != "") errors += "URL serwer 2: " + url2_error + "<br>";
    
    if (errors != "") {
        String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<title>Błąd walidacji</title><style>";
        html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
        html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
        html += ".container { background: #161b22; border: 1px solid #f85149; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
        html += "h1 { color: #f85149; margin-bottom: 20px; }";
        html += ".error-box { background: rgba(248, 81, 73, 0.1); border: 1px solid #f85149; padding: 15px; border-radius: 8px; margin: 20px 0; text-align: left; }";
        html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; cursor: pointer; text-decoration: none; display: inline-block; margin-top: 20px; }";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h1>❌ Błąd walidacji</h1>";
        html += "<div class='error-box'>";
        html += errors;
        html += "</div>";
        html += "<a href='javascript:history.back()' class='btn'>← Wróć i popraw dane</a>";
        html += "</div>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
        return;
    }
    
    // Zapis wszystkich danych do NVS
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    
    uint32_t interval = interval_str.toInt();
    prefs.putUInt("update_interval", interval);
    
    // Poprawne zapisanie współczynnika kalibracji
    String f_str = factor_str; 
    f_str.replace(',', '.'); 
    float f = f_str.toFloat(); 
    prefs.putFloat("factor", f);
    conversion_factor = f; // Aktualizacja zmiennej w pamięci RAM
    
    prefs.putString("aid", server.arg("aid"));
    prefs.putString("gid", server.arg("gid"));
    
    // Serwer 1
    prefs.putString("custom_url", custom_url);
    prefs.putString("custom_token", server.arg("custom_token"));
    if (server.hasArg("custom_enabled")) {
        prefs.putBool("custom_enabled", true);
        custom_server_enabled = true;
    } else {
        prefs.putBool("custom_enabled", false);
        custom_server_enabled = false;
    }
    
    // Serwer 2
    prefs.putString("custom2_url", custom2_url);
    prefs.putString("custom2_token", server.arg("custom2_token"));
    if (server.hasArg("custom2_enabled")) {
        prefs.putBool("custom2_enabled", true);
        custom_server2_enabled = true;
    } else {
        prefs.putBool("custom2_enabled", false);
        custom_server2_enabled = false;
    }
    
    // Aktualizacja zmiennych w pamięci RAM
    custom_server_url = custom_url;
    custom_server_token = server.arg("custom_token");
    
    custom_server2_url = custom2_url;
    custom_server2_token = server.arg("custom2_token");
    
    update_interval_minutes = interval;
    update_interval_ms = update_interval_minutes * 60000UL;
    
    Serial.println("[CONFIG] Konfiguracja zapisana do NVS");
    Serial.printf("[CONFIG] Współczynnik: %.6f\n", conversion_factor);
    Serial.printf("[CONFIG] Interwał: %u minut\n", update_interval_minutes);
    Serial.printf("[CONFIG] Serwer 1: %s (%s)\n", custom_server_url.c_str(), custom_server_enabled ? "aktywny" : "nieaktywny");
    Serial.printf("[CONFIG] Serwer 2: %s (%s)\n", custom_server2_url.c_str(), custom_server2_enabled ? "aktywny" : "nieaktywny");
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Konfiguracja zapisana</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #30363d; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #2ea043; margin-bottom: 20px; }";
    html += ".loader { border: 4px solid #30363d; border-top: 4px solid #2ea043; border-radius: 50%; width: 40px; height: 40px; ";
    html += "animation: spin 1s linear infinite; margin: 0 auto 20px; }";
    html += "@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }";
    html += ".success { background: rgba(46, 160, 67, 0.1); border: 1px solid #2ea043; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<div class='loader'></div>";
    html += "<h1>✅ Konfiguracja zapisana</h1>";
    html += "<div class='success'>";
    html += "<strong>Sukces!</strong> Konfiguracja została zapisana.";
    html += "<br><br>Urządzenie zostanie zrestartowane za 3 sekundy...";
    html += "<br>Po restarcie połączy się z nową siecią WiFi.";
    html += "</div>";
    html += "</div>";
    html += "<script>setTimeout(function() { window.location.href = '/'; }, 3000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
    delay(3000);
    ESP.restart();
}

void handle_restart() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Restartowanie urządzenia</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #d29922; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #d29922; margin-bottom: 20px; }";
    html += ".loader { border: 4px solid #30363d; border-top: 4px solid #d29922; border-radius: 50%; width: 40px; height: 40px; ";
    html += "animation: spin 1s linear infinite; margin: 0 auto 20px; }";
    html += "@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }";
    html += ".info { background: rgba(210, 153, 34, 0.1); border: 1px solid #d29922; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<div class='loader'></div>";
    html += "<h1>🔄 Restartowanie systemu</h1>";
    html += "<div class='info'>";
    html += "<strong>System zostanie zrestartowany za 3 sekundy...</strong>";
    html += "<br><br>Po restarcie strona automatycznie się odświeży.";
    html += "<br>Proces restartu trwa około 10-15 sekund.";
    html += "</div>";
    html += "</div>";
    html += "<script>setTimeout(function() { window.location.href = '/'; }, 3000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
    delay(3000);
    ESP.restart();
}

void handle_info() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Informacje o systemie</title>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; padding: 20px; }";
    html += ".container { max-width: 1000px; margin: 0 auto; }";
    html += ".header { text-align: center; margin-bottom: 30px; padding-bottom: 20px; border-bottom: 2px solid #30363d; }";
    html += "h1 { color: #58a6ff; margin-bottom: 10px; }";
    html += "h2 { color: #58a6ff; margin: 25px 0 15px 0; }";
    html += ".card { background: #161b22; border: 1px solid #30363d; border-radius: 10px; padding: 25px; margin-bottom: 25px; }";
    html += ".info-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-top: 20px; }";
    html += ".info-item { background: rgba(13, 17, 23, 0.5); padding: 15px; border-radius: 8px; border: 1px solid #30363d; }";
    html += ".info-label { color: #8b949e; font-size: 0.9rem; margin-bottom: 5px; }";
    html += ".info-value { font-size: 1.1rem; font-weight: bold; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; text-decoration: none; display: inline-block; margin-top: 20px; }";
    html += ".btn:hover { background: #2ea043; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<div class='header'>";
    html += "<h1>ℹ️ Informacje o systemie</h1>";
    html += "<p>Szczegółowe informacje o konfiguracji i stanie systemu</p>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📋 Informacje ogólne</h2>";
    html += "<div class='info-grid'>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Wersja oprogramowania</div>";
    html += "<div class='info-value'>v0.86/env</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Model sprzętu</div>";
    html += "<div class='info-value'>RadiationD v1.1</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Procesor</div>";
    html += "<div class='info-value'>ESP32-C3 mini</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Czujnik środowiska</div>";
    html += "<div class='info-value'>" + String(bme_sensor_found ? "BME680 (aktywny)" : "Brak") + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>ID urządzenia</div>";
    html += "<div class='info-value'>geiger_esp32c3_" + String((uint32_t)ESP.getEfuseMac(), HEX) + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Czas pracy systemu</div>";
    html += "<div class='info-value'>" + String(uptime_str) + "</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📊 Statystyki promieniowania</h2>";
    html += "<div class='info-grid'>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Aktualna moc dawki</div>";
    html += "<div class='info-value'>" + String(current_usv_h, 4) + " µSv/h</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Średnia 10-minutowa (ACPM)</div>";
    html += "<div class='info-value'>" + String(current_acpm, 1) + " CPM</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Chwilowa 2-minutowa (CPM)</div>";
    html += "<div class='info-value'>" + String(current_cpm, 1) + " CPM</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Dawka dzienna</div>";
    html += "<div class='info-value'>" + String(daily_dose, 6) + " µSv</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Stabilność pomiaru</div>";
    html += "<div class='info-value'>" + String(background_stability, 1) + " %</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Współczynnik kalibracji</div>";
    html += "<div class='info-value'>" + String(conversion_factor, 6) + "</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📈 Dane wykresów</h2>";
    html += "<div class='info-grid'>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Wykres promieniowania</div>";
    html += "<div class='info-value'>" + String(graphFilled ? GRAPH_POINTS : graphIndex) + "/" + String(GRAPH_POINTS) + " punktów</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Wykres środowiska</div>";
    html += "<div class='info-value'>" + String(envGraphFilled ? GRAPH_POINTS : envGraphIndex) + "/" + String(GRAPH_POINTS) + " punktów</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Interwał aktualizacji wykresu</div>";
    html += "<div class='info-value'>" + String(update_interval_minutes) + " minut</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Maksymalna liczba punktów</div>";
    html += "<div class='info-value'>" + String(GRAPH_POINTS) + " (24h przy 10-min interwale)</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>💾 Stan pamięci i NVS</h2>";
    html += "<div class='info-grid'>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Wolna pamięć RAM</div>";
    html += "<div class='info-value'>" + String(ESP.getFreeHeap()) + " bajtów</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Obciążenie CPU</div>";
    html += "<div class='info-value'>" + String(cpu_usage, 1) + " %</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Zapisy NVS</div>";
    html += "<div class='info-value'>" + String(nvs_write_count) + " (" + String((float)nvs_write_count / NVS_MAX_WRITES * 100.0, 1) + "%)</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Limit zapisów NVS</div>";
    html += "<div class='info-value'>" + String(NVS_MAX_WRITES) + " zapisów</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Stan WiFi</div>";
    html += "<div class='info-value'>" + String(wifi_connected ? "Połączono" : ap_mode ? "Tryb AP" : "Rozłączono") + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Siła sygnału WiFi</div>";
    html += "<div class='info-value'>" + String(WiFi.RSSI()) + " dBm</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🌐 Konfiguracja sieci i serwerów</h2>";
    html += "<div class='info-grid'>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Połączona sieć WiFi</div>";
    html += "<div class='info-value'>" + WiFi.SSID() + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Adres IP</div>";
    html += "<div class='info-value'>" + device_ip + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>GMCMap.com</div>";
    html += "<div class='info-value'>" + String(prefs.getString("aid", "") != "" ? "Aktywny" : "Nieaktywny") + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Serwer własny 1</div>";
    html += "<div class='info-value'>" + String(custom_server_enabled ? "Aktywny" : "Nieaktywny") + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Serwer własny 2</div>";
    html += "<div class='info-value'>" + String(custom_server2_enabled ? "Aktywny" : "Nieaktywny") + "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<div class='info-label'>Interwał wysyłki</div>";
    html += "<div class='info-value'>" + String(update_interval_minutes) + " minut</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    html += "<a href='/' class='btn'>← Powrót do panelu głównego</a>";
    
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_tube_info() {
    uint32_t tube_uptime = calculate_tube_uptime();
    String tube_uptime_formatted = format_years_days(tube_uptime);
    float days = tube_uptime / 86400.0;
    float daily_avg = (days > 0.1) ? (total_lifetime_pulses / days) : 0;
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Informacje o tubie Geigera</title>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; padding: 20px; }";
    html += ".container { max-width: 800px; margin: 0 auto; }";
    html += ".header { text-align: center; margin-bottom: 30px; padding-bottom: 20px; border-bottom: 2px solid #30363d; }";
    html += "h1 { color: #58a6ff; margin-bottom: 10px; }";
    html += ".card { background: #161b22; border: 1px solid #30363d; border-radius: 10px; padding: 25px; margin-bottom: 25px; }";
    html += ".stat-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-top: 20px; }";
    html += ".stat-box { background: rgba(13, 17, 23, 0.5); padding: 20px; border-radius: 8px; border: 1px solid #30363d; text-align: center; }";
    html += ".stat-value { font-size: 2rem; font-weight: bold; color: white; display: block; margin: 10px 0; }";
    html += ".stat-label { font-size: 0.85rem; color: #8b949e; text-transform: uppercase; letter-spacing: 1px; }";
    html += ".progress-bar { height: 20px; background: #30363d; border-radius: 10px; overflow: hidden; margin: 20px 0; }";
    html += ".progress-fill { height: 100%; background: linear-gradient(90deg, #238636, #2ea043); transition: width 0.5s; }";
    html += ".warning-box { background: rgba(210, 153, 34, 0.1); border: 1px solid #d29922; padding: 20px; border-radius: 8px; margin: 20px 0; }";
    html += ".warning-title { color: #d29922; font-weight: bold; margin-bottom: 10px; }";
    html += ".danger-box { background: rgba(248, 81, 73, 0.1); border: 1px solid #f85149; padding: 20px; border-radius: 8px; margin: 20px 0; }";
    html += ".danger-title { color: #f85149; font-weight: bold; margin-bottom: 10px; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; text-decoration: none; display: inline-block; margin: 10px 5px; }";
    html += ".btn:hover { background: #2ea043; }";
    html += ".btn-warning { background: #d29922; }";
    html += ".btn-warning:hover { background: #bb8009; }";
    html += ".btn-danger { background: #f85149; }";
    html += ".btn-danger:hover { background: #da3633; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<div class='header'>";
    html += "<h1>🔬 Informacje o tubie Geigera</h1>";
    html += "<p>Stan żywotności i statystyki pracy tuby Geigera-Müllera</p>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📊 Stan żywotności tuby</h2>";
    
    String tube_status = "DOBRA";
    String status_color = "#238636";
    if (tube_lifetime_percent > 80) {
        tube_status = "OSTRZEŻENIE";
        status_color = "#d29922";
    }
    if (tube_lifetime_percent > 95) {
        tube_status = "KRYTYCZNY";
        status_color = "#f85149";
    }
    
    html += "<div style='text-align: center; margin-bottom: 20px;'>";
    html += "<div style='font-size: 1.2rem; font-weight: bold; color: " + String(status_color) + ";'>Stan: " + tube_status + "</div>";
    html += "</div>";
    
    html += "<div class='progress-bar'>";
    html += "<div class='progress-fill' style='width: " + String(tube_lifetime_percent) + "%;'></div>";
    html += "</div>";
    
    html += "<div style='text-align: center; font-size: 1.1rem; margin-bottom: 20px;'>";
    html += "<span style='color: #c9d1d9;'>" + String(tube_lifetime_percent, 1) + "%</span>";
    html += "<span style='color: #8b949e; margin: 0 10px;'>•</span>";
    html += "<span style='color: #c9d1d9;'>" + String(total_lifetime_pulses) + " / " + String(TUBE_LIFETIME) + " impulsów</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📈 Statystyki pracy</h2>";
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Całkowita liczba impulsów</div>";
    html += "<div class='stat-value'>" + String(total_lifetime_pulses) + "</div>";
    html += "<div style='font-size: 0.9rem; color: #8b949e;'>impulsów</div>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Pozostało impulsów</div>";
    html += "<div class='stat-value'>" + String(TUBE_LIFETIME - total_lifetime_pulses) + "</div>";
    html += "<div style='font-size: 0.9rem; color: #8b949e;'>impulsów</div>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Czas pracy tuby</div>";
    html += "<div class='stat-value' style='font-size: 1.5rem;'>" + tube_uptime_formatted + "</div>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Średnia dzienna</div>";
    html += "<div class='stat-value'>" + String(daily_avg, 1) + "</div>";
    html += "<div style='font-size: 0.9rem; color: #8b949e;'>imp/dzień</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    if (tube_lifetime_percent > 80 && tube_lifetime_percent <= 95) {
        html += "<div class='warning-box'>";
        html += "<div class='warning-title'>⚠️ OSTRZEŻENIE</div>";
        html += "<p>Tuba Geigera zużyła się w ponad 80%. Rozważ przygotowanie zapasowej tuby.</p>";
        html += "<p>Przewidywany czas pracy: " + String((100.0 - tube_lifetime_percent) * (days / tube_lifetime_percent), 0) + " dni</p>";
        html += "</div>";
    } else if (tube_lifetime_percent > 95) {
        html += "<div class='danger-box'>";
        html += "<div class='danger-title'>🚨 STAN KRYTYCZNY</div>";
        html += "<p>Tuba Geigera zużyła się w ponad 95%. Należy ją jak najszybciej wymienić!</p>";
        html += "<p>Dalsza praca może prowadzić do nieprawidłowych odczytów.</p>";
        html += "</div>";
    }
    
    html += "<div class='card'>";
    html += "<h2>🔄 Zarządzanie tubą</h2>";
    html += "<p>W przypadku wymiany tuby na nową, zresetuj licznik impulsów.</p>";
    
    html += "<div style='text-align: center; margin-top: 20px;'>";
    html += "<a href='javascript:history.back()' class='btn'>← Powrót</a>";
    html += "<a href='/reset_tube' class='btn btn-warning' onclick=\"return confirm('UWAGA: Czy na pewno chcesz zresetować licznik tuby?\\\\n\\\\nWszystkie dane dotyczące żywotności tuby zostaną wyzerowane.\\\\nTa operacja jest nieodwracalna i powinna być wykonana tylko po wymianie tuby na nową.');\">🔄 Resetuj licznik tuby</a>";
    html += "</div>";
    
    html += "</div>";
    
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_reset_tube() {
    reset_tube_counter();
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Licznik tuby zresetowany</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #2ea043; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #2ea043; margin-bottom: 20px; }";
    html += ".success-box { background: rgba(46, 160, 67, 0.1); border: 1px solid #2ea043; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; text-decoration: none; display: inline-block; margin-top: 20px; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>✅ Licznik tuby zresetowany</h1>";
    html += "<div class='success-box'>";
    html += "<strong>Sukces!</strong> Licznik impulsów tuby Geigera został wyzerowany.";
    html += "<br><br>Nowa tuba została zarejestrowana z datą: ";
    html += "<br><strong>" + String(get_current_unix_time()) + " (unix timestamp)</strong>";
    html += "</div>";
    html += "<a href='/' class='btn'>← Powrót do panelu głównego</a>";
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_reset_graph() {
    graphInitialize();
    envGraphInitialize();
    save_all_data();
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Wykresy zresetowane</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #2ea043; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #2ea043; margin-bottom: 20px; }";
    html += ".success-box { background: rgba(46, 160, 67, 0.1); border: 1px solid #2ea043; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; text-decoration: none; display: inline-block; margin-top: 20px; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>✅ Wykresy zresetowane</h1>";
    html += "<div class='success-box'>";
    html += "<strong>Sukces!</strong> Wszystkie wykresy zostały zresetowane.";
    html += "<br><br>Historia promieniowania i środowiska została wyczyszczona.";
    html += "<br>Nowe wykresy rozpoczną zbieranie danych od teraz.";
    html += "</div>";
    html += "<a href='/' class='btn'>← Powrót do panelu głównego</a>";
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n" + String(80, '='));
    Serial.println("     GEIGER DIY MONITOR v0.86/env - DUAL CUSTOM SERVERS");
    Serial.println("     ESP32-C3 + RadiationD v1.1 + BME680");
    Serial.println("     Autor: MaxGyver");
    Serial.println(String(80, '=') + "\n");
    
    pinMode(LED_PIN, OUTPUT); 
    digitalWrite(LED_PIN, HIGH);
    pinMode(GEIGER_PIN, INPUT);

    // Inicjalizacja Preferences z poprawionym namespace
    if (!prefs.begin("geiger")) {
        Serial.println("[NVS] BŁĄD: Nie można otworzyć pamięci NVS!");
    } else {
        Serial.println("[NVS] Pamięć NVS otwarta pomyślnie");
    }
    
    // Ładowanie konfiguracji - NAPRAWIONE
    update_interval_minutes = prefs.getUInt("update_interval", DEFAULT_UPDATE_INTERVAL);
    if (update_interval_minutes < 1 || update_interval_minutes > 60) {
        update_interval_minutes = DEFAULT_UPDATE_INTERVAL;
        prefs.putUInt("update_interval", update_interval_minutes);
    }
    update_interval_ms = update_interval_minutes * 60000UL;
    
    conversion_factor = prefs.getFloat("factor", DEFAULT_FACTOR);
    daily_dose = prefs.getFloat("daily_dose", 0.0);
    nvs_write_count = prefs.getUInt("nvs_write_count", 0);
    
    // Ładowanie konfiguracji serwerów - NAPRAWIONE
    custom_server_url = prefs.getString("custom_url", "");
    custom_server_token = prefs.getString("custom_token", "");
    custom_server_enabled = prefs.getBool("custom_enabled", false);
    
    custom_server2_url = prefs.getString("custom2_url", "");
    custom_server2_token = prefs.getString("custom2_token", "");
    custom_server2_enabled = prefs.getBool("custom2_enabled", false);
    
    // WAŻNE: Ładowanie obu wykresów PRZED innymi danymi
    graphLoad();
    envGraphLoad();
    
    load_tube_state();
    
    Serial.printf("[SYSTEM] Współczynnik: %.6f\n", conversion_factor);
    Serial.printf("[SYSTEM] Tuba: %u impulsów (%.1f%%)\n", total_lifetime_pulses, tube_lifetime_percent);
    Serial.printf("[SYSTEM] Wykres promieniowania: %d punktów (filled: %d)\n", 
                 graphFilled ? GRAPH_POINTS : graphIndex, graphFilled);
    Serial.printf("[SYSTEM] Wykres środowiska: %d punktów (filled: %d)\n", 
                 envGraphFilled ? GRAPH_POINTS : envGraphIndex, envGraphFilled);
    Serial.printf("[SYSTEM] NVS zapisów: %u\n", nvs_write_count);
    Serial.printf("[SYSTEM] Interwał aktualizacji: %u minut (%lu ms)\n", update_interval_minutes, update_interval_ms);
    
    if (custom_server_enabled && custom_server_url != "") {
        Serial.printf("[SYSTEM] Serwer 1: %s\n", custom_server_url.c_str());
    }
    if (custom_server2_enabled && custom_server2_url != "") {
        Serial.printf("[SYSTEM] Serwer 2: %s\n", custom_server2_url.c_str());
    }
    
    uint32_t tube_uptime = calculate_tube_uptime();
    String tube_uptime_formatted = format_years_days(tube_uptime);
    Serial.printf("[SYSTEM] Czas pracy tuby: %s\n", tube_uptime_formatted.c_str());
    
    bme_sensor_found = init_bme680();
    bme_initialized = bme_sensor_found;
    
    attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), geiger_isr, FALLING);
    connect_wifi();
    
    server.on("/", handle_root);
    server.on("/save", HTTP_POST, handle_save);
    server.on("/restart", handle_restart);
    server.on("/info", handle_info);
    server.on("/tube_info", handle_tube_info);
    server.on("/reset_tube", handle_reset_tube);
    server.on("/reset_graph", handle_reset_graph);
    
    server.begin();
    
    Serial.println("\n[SYSTEM] Serwer HTTP uruchomiony na porcie 80");
    Serial.println("[SYSTEM] System gotowy do pracy");
    
    last_gmc_send = millis(); 
    last_custom_send = millis();
    last_custom2_send = millis();
    last_tube_save = millis();
    last_graph_update = millis();
    last_all_data_save = millis();
    loop_start_time = millis();
}

void loop() {
    uint32_t current_loop_start = millis();
    uint32_t loop_duration;
    
    if (current_loop_start >= loop_start_time) {
        loop_duration = current_loop_start - loop_start_time;
    } else {
        loop_duration = (UINT32_MAX - loop_start_time) + current_loop_start + 1;
    }
    
    loop_start_time = current_loop_start;
    
    cpu_loop_times[cpu_loop_index] = loop_duration;
    cpu_loop_index = (cpu_loop_index + 1) % CPU_AVG_WINDOW;
    
    server.handleClient();
    update_measurements();
    update_environment();
    update_uptime();
    check_wifi_reconnect();
    log_nvs_status();
    
    if (time_elapsed(last_gmc_send, update_interval_ms) && wifi_connected) {
        send_to_gmcmap();
    }
    
    if (time_elapsed(last_custom_send, update_interval_ms) && wifi_connected) {
        send_to_custom_server();
    }
    
    if (time_elapsed(last_custom2_send, update_interval_ms) && wifi_connected) {
        send_to_custom_server2();
    }

    if (time_elapsed(last_measurement_log, MEASUREMENT_LOG_INTERVAL)) {
        uint32_t tube_uptime = calculate_tube_uptime();
        String tube_uptime_formatted = format_years_days(tube_uptime);
        
        String custom_status = "";
        if (custom_server_enabled && custom_server_url != "") {
            custom_status = custom_server_last_status ? "CUSTOM1: OK" : "CUSTOM1: ERROR";
        } else {
            custom_status = "CUSTOM1: OFF";
        }
        
        String custom2_status = "";
        if (custom_server2_enabled && custom_server2_url != "") {
            custom2_status = custom_server2_last_status ? "CUSTOM2: OK" : "CUSTOM2: ERROR";
        } else {
            custom2_status = "CUSTOM2: OFF";
        }
        
        float nvs_usage_percent = (float)nvs_write_count / NVS_MAX_WRITES * 100.0;
        
        Serial.printf("[STAT] %s | CPM: %.1f | ACPM: %.1f | uSv/h: %.4f | Temp: %.1f°C | Hum: %.1f%% | Pres: %.1fhPa | IAQ: %.1f | CPU: %.1f%% | Tuba: %u imp (%.1f%%) | Wykres rad: %d/%d | Wykres env: %d/%d | %s | %s\n", 
                      uptime_str, current_cpm, current_acpm, current_usv_h,
                      env_data.temperature, env_data.humidity, env_data.pressure, env_data.iaq_score,
                      cpu_usage, total_lifetime_pulses, tube_lifetime_percent,
                      graphFilled ? GRAPH_POINTS : graphIndex, GRAPH_POINTS,
                      envGraphFilled ? GRAPH_POINTS : envGraphIndex, GRAPH_POINTS,
                      custom_status.c_str(), custom2_status.c_str());
    }
    
    if (time_elapsed(last_dose_save, DOSE_SAVE_INTERVAL)) {
        prefs.putFloat("daily_dose", daily_dose);
    }

    static uint32_t last_led_blink = 0;
    if (!wifi_connected && !ap_mode) {
        if (time_elapsed(last_led_blink, 500)) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    } else if (ap_mode) {
        if (time_elapsed(last_led_blink, 1000)) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }
    
    delay(1);
}