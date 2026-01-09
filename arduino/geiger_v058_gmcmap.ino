#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h> 
#include <time.h>
#include <math.h>
#include <ESPmDNS.h>

// ==================== KONFIGURACJA SPRZĘTOWA ====================
#define GEIGER_PIN              4       
#define CONFIG_RESET_PIN        9       
#define LED_PIN                 8       
#define DEBOUNCE_US             80      

// ==================== PARAMETRY TUBY ====================
#define TUBE_LIFETIME           100000000  // 100 milionów impulsów (typowa tuba J305/315)
#define TUBE_SAVE_INTERVAL      300000    // 5 minut

// ==================== PARAMETRY SYSTEMOWE ====================
#define CPM_WINDOW_SIZE         120
#define ACPM_WINDOW_SIZE        600
#define GMC_SEND_INTERVAL       360000  // 6 minut
#define DOSE_SAVE_INTERVAL      900000  
#define MEASUREMENT_LOG_INTERVAL 60000  
#define STABILITY_CALC_INTERVAL 60000
#define WIFI_RECONNECT_INTERVAL 30000
#define WIFI_CONNECT_TIMEOUT    30000   // 30 sekund timeout

#define DEFAULT_WIFI_SSID       "Geiger_DIY_Setup"
#define DEFAULT_WIFI_PASS       "1234567890"
#define DEFAULT_FACTOR          0.00297

// Proste nazwy mDNS dla lepszej kompatybilności
#define MDNS_PRIMARY_NAME       "geiger"     // Najlepsza kompatybilność
#define MDNS_SECONDARY_NAME     "geigercnt"  // Alternatywa

// ==================== ZMIENNE ====================
volatile uint32_t pulse_count = 0;
volatile uint32_t last_pulse_micros = 0;

uint32_t second_buffer[ACPM_WINDOW_SIZE] = {0};
uint16_t buffer_index = 0;
uint32_t last_second_update = 0;
uint32_t last_buffer_shift = 0;

uint32_t total_pulses = 0;
uint32_t total_lifetime_pulses = 0;        // Całkowita liczba impulsów od początku
uint32_t tube_start_time = 0;              // Czas rozpoczęcia pomiarów tuby (timestamp UNIX)
uint32_t last_tube_save = 0;               // Ostatni zapis stanu tuby
float tube_lifetime_percent = 0.0;         // Procent zużycia tuby

uint32_t last_gmc_send = 0;
uint32_t last_dose_save = 0;
uint32_t last_measurement_log = 0;
uint32_t last_stability_calc = 0;
uint32_t last_wifi_reconnect = 0;
uint32_t wifi_disconnect_time = 0;
uint32_t wifi_connect_start = 0;
bool wifi_connected = false;
bool ap_mode = false;
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

WebServer server(80);
Preferences prefs;  // JEDEN namespace dla WSZYSTKICH danych

// ==================== PRZERWANIE ====================
void IRAM_ATTR geiger_isr() {
    uint32_t now_micros = micros();
    if (now_micros - last_pulse_micros >= DEBOUNCE_US) {
        pulse_count++;
        last_pulse_micros = now_micros;
    }
}

// ==================== WYSYŁKA DO GMCMap ====================
void send_to_gmcmap() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[GMC] Brak WiFi - wysyłka przerwana.");
        return;
    }

    String aid = prefs.getString("aid", "");
    String gid = prefs.getString("gid", "");

    if (aid == "" || gid == "" || aid == "null") {
        Serial.println("[GMC] Brak konfiguracji AID/GID.");
        return;
    }

    HTTPClient http;
    http.setTimeout(10000);
    String url = "http://www.gmcmap.com/log2.asp?AID=" + aid + 
                 "&GID=" + gid + 
                 "&CPM=" + String((int)round(current_cpm)) + 
                 "&ACPM=" + String((int)round(current_acpm)) + 
                 "&uSV=" + String(current_usv_h, 4);

    Serial.print("[GMC] URL: "); Serial.println(url);
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
        Serial.printf("[GMC] Sukces (Kod: %d)\n", httpCode);
        digitalWrite(LED_PIN, LOW); 
        delay(100); 
        digitalWrite(LED_PIN, HIGH);
    } else {
        Serial.printf("[GMC] Błąd: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
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

// Funkcja do uzyskiwania aktualnego czasu UNIX (bez WiFi używa przybliżenia)
uint32_t get_current_unix_time() {
    if (WiFi.status() == WL_CONNECTED) {
        time_t now;
        time(&now);
        return (uint32_t)now;
    } else {
        // Jeśli brak WiFi, używamy przybliżenia na podstawie czasu pracy
        // (dla prawidłowego obliczenia czasu pracy tuby)
        static uint32_t startup_unix = prefs.getUInt("last_known_unix", 0);
        static uint32_t startup_millis = 0;
        
        if (startup_millis == 0) {
            startup_millis = millis();
            if (startup_unix == 0) {
                // Jeśli nie mamy zapisanego czasu, używamy czasu kompilacji jako przybliżenie
                startup_unix = 1704067200; // 1 stycznia 2024 jako fallback
            }
        }
        
        return startup_unix + ((millis() - startup_millis) / 1000);
    }
}

// Funkcja do zapisu stanu tuby
void save_tube_state() {
    prefs.putUInt("tube_pulses", total_lifetime_pulses);
    prefs.putUInt("tube_start_time", tube_start_time);
    
    // Zapisujemy aktualny czas UNIX dla przyszłych obliczeń
    uint32_t current_unix = get_current_unix_time();
    prefs.putUInt("last_known_unix", current_unix);
    
    Serial.printf("[TUBE] Zapisywanie stanu: %u impulsów, start: %u (Unix)\n", 
                  total_lifetime_pulses, tube_start_time);
}

// Funkcja do ładowania stanu tuby
void load_tube_state() {
    total_lifetime_pulses = prefs.getUInt("tube_pulses", 0);
    tube_start_time = prefs.getUInt("tube_start_time", 0);
    
    // Jeśli to pierwsze uruchomienie, zainicjuj czas startu
    if (tube_start_time == 0) {
        tube_start_time = get_current_unix_time();
        prefs.putUInt("tube_start_time", tube_start_time);
        Serial.printf("[TUBE] Pierwsze uruchomienie, ustawiam czas startu: %u\n", tube_start_time);
    } else {
        // Sprawdź czy czas startu jest realistyczny (nie z przyszłości i nie za stary)
        uint32_t current_unix = get_current_unix_time();
        
        if (tube_start_time > current_unix) {
            // Czas startu jest z przyszłości - to błąd, resetujemy
            Serial.printf("[TUBE] BŁĄD: tube_start_time (%u) > current (%u) - reset\n", 
                          tube_start_time, current_unix);
            tube_start_time = current_unix;
            prefs.putUInt("tube_start_time", tube_start_time);
        }
        
        uint32_t tube_uptime = current_unix - tube_start_time;
        Serial.printf("[TUBE] Załadowano: %u impulsów, start: %u (Unix), czas pracy: %u dni\n", 
                      total_lifetime_pulses, tube_start_time, tube_uptime / 86400);
    }
}

// Funkcja do resetowania licznika tuby (np. po wymianie)
void reset_tube_counter() {
    total_lifetime_pulses = 0;
    tube_start_time = get_current_unix_time();
    tube_lifetime_percent = 0.0;
    save_tube_state();
    Serial.println("[TUBE] Licznik tuby zresetowany (wymiana tuby)");
}

// Oblicz całkowity czas pracy tuby w sekundach
uint32_t calculate_tube_uptime() {
    uint32_t current_unix = get_current_unix_time();
    if (tube_start_time > 0 && current_unix > tube_start_time) {
        return current_unix - tube_start_time;
    }
    return 0;
}

// Formatuj czas w latach, dniach, godzinach
String format_years_days(uint32_t seconds) {
    uint32_t years = seconds / 31536000;  // 365 dni
    uint32_t days = (seconds % 31536000) / 86400;
    uint32_t hours = (seconds % 86400) / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    
    char buffer[64];
    if (years > 0) {
        snprintf(buffer, sizeof(buffer), "%u lat, %u dni, %02u:%02u", years, days, hours, minutes);
    } else {
        snprintf(buffer, sizeof(buffer), "%u dni, %02u:%02u", days, hours, minutes);
    }
    return String(buffer);
}

void update_measurements() {
    uint32_t now = millis();

    if (now - last_second_update >= 1000) {
        uint32_t pulses;
        noInterrupts();
        pulses = pulse_count;
        pulse_count = 0;
        interrupts();

        second_buffer[buffer_index] = pulses;
        buffer_index = (buffer_index + 1) % ACPM_WINDOW_SIZE;
        total_pulses += pulses;
        total_lifetime_pulses += pulses;  // Dodaj do całkowitej liczby
        
        last_second_update = now;
    }

    if (now - last_buffer_shift >= 1000) {
        last_buffer_shift = now;
        
        uint32_t sum_120 = 0;
        for(int i=0; i<CPM_WINDOW_SIZE; i++) {
            int idx = (buffer_index - 1 - i + ACPM_WINDOW_SIZE) % ACPM_WINDOW_SIZE;
            sum_120 += second_buffer[idx];
        }
        current_cpm = (float)sum_120 * (60.0 / CPM_WINDOW_SIZE);

        uint32_t sum_600 = 0;
        for(int i=0; i<ACPM_WINDOW_SIZE; i++) sum_600 += second_buffer[i];
        current_acpm = (float)sum_600 * (60.0 / ACPM_WINDOW_SIZE);

        current_usv_h = current_acpm * conversion_factor;

        static uint32_t last_calc = 0;
        if (last_calc > 0) {
            float h = (now - last_calc) / 3600000.0;
            daily_dose += current_usv_h * h;
        }
        last_calc = now;
    }

    if (now - last_stability_calc >= STABILITY_CALC_INTERVAL) {
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
        last_stability_calc = now;
    }
    
    // Aktualizuj procent zużycia tuby
    tube_lifetime_percent = (float)total_lifetime_pulses / TUBE_LIFETIME * 100.0;
    if (tube_lifetime_percent > 100.0) tube_lifetime_percent = 100.0;
    
    // Okresowy zapis stanu tuby
    if (now - last_tube_save >= TUBE_SAVE_INTERVAL) {
        save_tube_state();
        last_tube_save = now;
    }
}

// ==================== mDNS & WiFi ====================
void init_mdns() {
    delay(100);
    
    Serial.print("[mDNS] Inicjalizacja... ");
    
    if (!MDNS.begin(MDNS_PRIMARY_NAME)) {
        Serial.printf("Błąd z '%s', próba '%s'... ", MDNS_PRIMARY_NAME, MDNS_SECONDARY_NAME);
        
        if (!MDNS.begin(MDNS_SECONDARY_NAME)) {
            Serial.println("FAIL");
            Serial.println("[mDNS] Uwaga: mDNS nieaktywny. Użyj adresu IP.");
            return;
        } else {
            Serial.println("OK (secondary)");
        }
    } else {
        Serial.println("OK (primary)");
    }
    
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("arduino", "tcp", 3232);
    
    MDNS.addServiceTxt("http", "tcp", "device", "GeigerCounter");
    MDNS.addServiceTxt("http", "tcp", "version", "0.58");
    MDNS.addServiceTxt("http", "tcp", "model", "RadiationD v1.1");
    
    Serial.println("[mDNS] Adresy dostępu:");
    Serial.printf("  • http://%s.local\n", MDNS_PRIMARY_NAME);
    Serial.printf("  • http://%s.local\n", MDNS_SECONDARY_NAME);
    Serial.printf("  • http://%s\n", device_ip.c_str());
}

void wifi_event_handler(WiFiEvent_t event) {
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("[WiFi] Połączono z punktem dostępowym");
            wifi_connected = true;
            ap_mode = false;
            break;
            
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            device_ip = WiFi.localIP().toString();
            Serial.printf("[WiFi] Adres IP: %s\n", device_ip.c_str());
            Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
            digitalWrite(LED_PIN, LOW);
            
            init_mdns();
            setup_time();
            
            ArduinoOTA.setHostname(MDNS_PRIMARY_NAME);
            ArduinoOTA.setPassword("geiger123");
            ArduinoOTA.begin();
            break;
            
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[WiFi] Rozłączono");
            wifi_connected = false;
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
    
    Serial.printf("[WiFi] Łączenie z: %s\n", ssid.c_str());
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    
    WiFi.setHostname(MDNS_PRIMARY_NAME);
    
    WiFi.onEvent(wifi_event_handler);
    
    WiFi.begin(ssid.c_str(), pass.c_str());
    wifi_connect_start = millis();
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\n[WiFi] Połączono pomyślnie!");
    } else {
        Serial.println("\n[WiFi] Timeout - uruchamiam tryb AP");
        start_ap_mode();
    }
}

void start_ap_mode() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Geiger-AP", "12345678");
    device_ip = WiFi.softAPIP().toString();
    Serial.println("[AP] Tryb AP uruchomiony");
    Serial.printf("[AP] SSID: Geiger-AP\n");
    Serial.printf("[AP] IP: %s\n", device_ip.c_str());
    ap_mode = true;
    digitalWrite(LED_PIN, HIGH);
}

void check_wifi_reconnect() {
    uint32_t now = millis();
    
    if (!wifi_connected && !ap_mode) {
        if (now - wifi_disconnect_time > WIFI_RECONNECT_INTERVAL) {
            Serial.println("[WiFi] Próba ponownego połączenia...");
            WiFi.reconnect();
            wifi_disconnect_time = now;
        }
    }
}

// ==================== STRONY WWW ====================
const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Geiger DIY Monitor by MaxGyver</title>
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

void handle_root() {
    String html = String(HTML_HEAD);
    
    // Header
    html += "<div class='header'>";
    html += "<h1>☢ Geiger DIY Monitor v0.58</h1>";
    html += "<div class='subtitle'>Amatorski system monitorowania promieniowania jonizującego</div>";
    html += "</div>";
    
    // Status bar
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
    }
    
    html += "</div>";
    
    // 1. KARTA Z POMIARAMI (PIERWSZA - zgodnie z życzeniem)
    html += "<div class='card'>";
    html += "<h2>📊 Aktualne pomiary</h2>";
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
    
    html += "</div>";
    
    html += "<div class='info-section'>";
    html += "<div class='info-title'>ℹ️ Objaśnienia pomiarów</div>";
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
    
    html += "</div>";
    html += "</div>";
    
    html += "</div>";
  
    // Karta z metodami dostępu
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
    
    // Karta ze statystykami
    html += "<div class='card'>";
    html += "<h2>📈 Statystyki systemu</h2>";
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Dawka dzienna</div>";
    html += "<span class='stat-value'>" + String(daily_dose, 4) + "</span>";
    html += "<span class='unit'>µSv</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Stabilność pomiaru</div>";
    html += "<span class='stat-value'>" + String(background_stability, 1) + "</span>";
    html += "<span class='unit'>%</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Licznik całkowity</div>";
    html += "<span class='stat-value'>" + String(total_pulses) + "</span>";
    html += "<span class='unit'>impulsów</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='info-section'>";
    html += "<div class='info-title'>📊 Objaśnienia statystyk</div>";
    
    html += "<div class='note-box'>";
    html += "<div class='note-title'>Dawka dzienna</div>";
    html += "<p>Całkowita dawka pochłonięta od północy (lub od ostatniego resetu). Obliczana przez całkowanie mocy dawki w czasie. Przeciętna roczna dawka w Polsce: ~2400 µSv.</p>";
    html += "</div>";
    
    html += "<div class='note-box'>";
    html += "<div class='note-title'>Stabilność pomiaru</div>";
    html += "<p>Określa, jak bardzo stabilny jest pomiar tła. Wyrażona w procentach (0-100%). Im wyższa wartość, tym mniejsze fluktuacje. >80% oznacza bardzo stabilny pomiar.</p>";
    html += "</div>";
    
    html += "<div class='note-box'>";
    html += "<div class='note-title'>Licznik całkowity</div>";
    html += "<p>Całkowita liczba zarejestrowanych impulsów od uruchomienia systemu. Każdy impuls odpowiada cząstce jonizującej (alfa, beta, gamma) która została wykryta przez tubę.</p>";
    html += "</div>";
    
    html += "</div>";
    
    html += "</div>";
    
    // Karta konfiguracyjna
    html += "<div class='card'>";
    html += "<h2>⚙ Konfiguracja systemu</h2>";
    html += "<form method='POST' action='/save' id='configForm'>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>WiFi SSID:</label>";
    html += "<input type='text' class='form-input' name='ssid' value='" + prefs.getString("ssid", DEFAULT_WIFI_SSID) + "' required>";
    html += "<div class='form-help'>Nazwa sieci WiFi do połączenia</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>WiFi Hasło:</label>";
    html += "<input type='password' class='form-input' id='wifiPass' name='pass' value='" + prefs.getString("pass", DEFAULT_WIFI_PASS) + "' required>";
    html += "<div class='form-help'>Hasło do sieci WiFi</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label class='form-label'>Współczynnik kalibracji (Factor):</label>";
    html += "<input type='text' class='form-input' name='factor' value='" + String(conversion_factor, 6) + "' required>";
    html += "<div class='form-help'>Przelicznik CPM → µSv/h (domyślnie: 0.00297 dla J305/J315)</div>";
    html += "</div>";
    
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
    
    html += "<div class='warning-box'>";
    html += "<div class='warning-title'>⚠️ UWAGA</div>";
    html += "Po zapisaniu konfiguracji urządzenie automatycznie się zrestartuje. Upewnij się, że wprowadzone dane są poprawne.";
    html += "</div>";
    
    html += "<button type='submit' class='btn'>💾 Zapisz konfigurację i zrestartuj urządzenie</button>";
    
    html += "</form>";
    html += "</div>";
    
    // Karta z instrukcją kalibracji
    html += "<div class='card'>";
    html += "<h2>🛠 Kalibracja licznika Geigera</h2>";
    
    html += "<div class='calibration-box'>";
    html += "<div class='info-title'>📐 Jak obliczyć współczynnik kalibracji?</div>";
    
    html += "<div class='calibration-step'>1. Znajdź wartość referencyjną tła naturalnego</div>";
    html += "<p>Dla Polski użyj mapy Państwowej Agencji Atomistyki: <a href='https://monitoring.paa.gov.pl/maps-portal/' target='_blank' style='color: var(--sec);'>https://monitoring.paa.gov.pl/maps-portal/</a></p>";
    html += "<p>Znajdź swoją lokalizację i odczytaj wartość mocy dawki (np. 0.09 µSv/h).</p>";
    
    html += "<div class='calibration-step'>2. Pozostaw licznik na 15-20 minut</div>";
    html += "<p>Upewnij się, że licznik stoi w miejscu z naturalnym tłem (z dala od źródeł promieniowania).</p>";
    
    html += "<div class='calibration-step'>3. Odczytaj stabilną wartość ACPM</div>";
    html += "<p>Poczekaj aż ACPM się ustabilizuje (powinno być >80% stabilności).</p>";
    
    html += "<div class='calibration-step'>4. Oblicz nowy współczynnik</div>";
    html += "<div class='code'>NOWY_FACTOR = (Wartość z mapy PAA w µSv/h) / (ACPM)</div>";
    
    html += "<div class='calibration-step'>5. Wprowadź obliczoną wartość</div>";
    html += "<p>Wpisz nowy współczynnik w polu 'Współczynnik kalibracji' powyżej.</p>";
    
    html += "<div class='success-box'>";
    html += "<div style='font-weight: bold; color: var(--success); margin-bottom: 10px;'>PRZYKŁAD KALIBRACJI:</div>";
    html += "<p><strong>Mapa PAA:</strong> 0.09 µSv/h</p>";
    html += "<p><strong>Odczyt ACPM:</strong> 30.3 impulsów/minutę</p>";
    html += "<p><strong>Obliczenia:</strong> Factor = 0.09 ÷ 30.3 = 0.00297</p>";
    html += "</div>";
    
    html += "<div class='note-box'>";
    html += "<div class='note-title'>Alternatywna kalibracja względem innego licznika</div>";
    html += "<p>Jeśli masz inny, skalibrowany licznik Geigera:</p>";
    html += "<ol style='margin-left: 20px; margin-top: 10px;'>";
    html += "<li>Umieść oba liczniki obok siebie</li>";
    html += "<li>Odczekaj 10-15 minut na stabilizację</li>";
    html += "<li>Odczytaj µSv/h z kalibrowanego licznika</li>";
    html += "<li>Odczytaj ACPM z tego licznika</li>";
    html += "<li>Oblicz: Factor = (µSv/h z kalibrowanego) ÷ (ACPM tego licznika)</li>";
    html += "</ol>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    // Karta ze słownikiem pojęć
    html += "<div class='card'>";
    html += "<h2>🎓 Słownik pojęć i teoria</h2>";
    
    html += "<div class='legend-grid'>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>Tło naturalne</div>";
    html += "<div class='legend-desc'>Promieniowanie jonizujące pochodzące z naturalnych źródeł: radon, promieniowanie kosmiczne, izotopy w skorupie ziemskiej. W Polsce wynosi zwykle 0.08-0.12 µSv/h.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>Tuba Geigera-Müllera</div>";
    html += "<div class='legend-desc'>Czujnik promieniowania jonizującego. Gdy cząstka przechodzi przez tubę wypełnioną gazem, powoduje wyładowanie elektryczne rejestrowane jako impuls.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>Statystyka pomiarów</div>";
    html += "<div class='legend-desc'>Rozpad promieniotwórczy jest procesem statystycznym. Im dłuższy czas pomiaru, tym dokładniejszy wynik. Dlatego używamy średnich kroczących.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>GMCMap.com</div>";
    html += "<div class='legend-desc'>Darmowy serwis społecznościowy do mapowania promieniowania tła na całym świecie. Dane z tego licznika wysyłane są co 6 minut (jeśli skonfigurowano AID/GID).</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>mDNS (Multicast DNS)</div>";
    html += "<div class='legend-desc'>Technologia pozwalająca na dostęp do urządzenia przez nazwę (np. geiger.local) zamiast adresu IP. Działa automatycznie w sieci lokalnej.</div>";
    html += "</div>";
    
    html += "<div class='legend-item'>";
    html += "<div class='legend-title'>RadiationD v1.1</div>";
    html += "<div class='legend-desc'>Tania gotowa płytka PCB dostępna w sieci zawierająca tubę Geigera oraz wysokonapięciowy zasilacz tuby i wzmacniacz impulsów.</div>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    
    // Karta diagnostyczna
    html += "<div class='card'>";
    html += "<h2>🔧 Diagnostyka systemu</h2>";
    
    html += "<div class='stat-grid'>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Pamięć dostępna</div>";
    html += "<span class='stat-value'>" + String(ESP.getFreeHeap() / 1024) + "</span>";
    html += "<span class='unit'>KB</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>ID chipu</div>";
    html += "<span class='stat-value'>" + String((uint32_t)ESP.getEfuseMac(), HEX) + "</span>";
    html += "<span class='unit'>HEX</span>";
    html += "</div>";
    
    html += "<div class='stat-box'>";
    html += "<div class='stat-label'>Częstotliwość CPU</div>";
    html += "<span class='stat-value'>" + String(ESP.getCpuFreqMHz()) + "</span>";
    html += "<span class='unit'>MHz</span>";
    html += "</div>";
    
    html += "</div>";
    
    // Status GMCMap
    String aid = prefs.getString("aid", "");
    html += "<div style='margin-top: 20px; padding: 15px; background: rgba(13, 17, 23, 0.5); border-radius: 8px; border: 1px solid var(--border);'>";
    html += "<div style='display: flex; align-items: center; justify-content: space-between;'>";
    html += "<div>";
    html += "<div style='font-weight: bold;'>Status GMCMap:</div>";
    if (aid != "" && aid != "null") {
        html += "<span class='gmc-status gmc-online'>AKTYWNE (AID: " + aid + ")</span>";
    } else {
        html += "<span class='gmc-status gmc-offline'>NIEAKTYWNE</span>";
    }
    html += "</div>";
    html += "<div style='font-size: 0.9rem; color: #8b949e;'>Wysyłka co 6 minut</div>";
    html += "</div>";
    html += "</div>";
    
    // Oblicz dane o tubie dla strony głównej
    uint32_t tube_total_uptime = calculate_tube_uptime();
    float days = tube_total_uptime / 86400.0;
    float daily_avg = (days > 0.1) ? (total_lifetime_pulses / days) : 0;
    
    // Link do informacji o tubie
    html += "<div style='margin-top: 20px; padding: 15px; background: rgba(13, 17, 23, 0.5); border-radius: 8px; border: 1px solid var(--border);'>";
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
    html += "<a href='/info' class='btn' style='width: auto; flex: 1; min-width: 200px; background: linear-gradient(135deg, var(--sec), #1f6feb);'>ℹ️ Szczegółowe informacje</a>";
    html += "</div>";
    
    html += "</div>";
    
    // Stopka
    html += "<div class='footer'>";
    html += "<p>Geiger DIY Monitor v0.58 | Autor: MaxGyver | Hardware: RadiationD v1.1 + ESP32-C3</p>";
    html += "<p>WiFi: " + String(wifi_connected ? "Połączono" : ap_mode ? "Tryb AP" : "Rozłączono") + " | Ostatni pomiar: " + String(millis() / 1000) + "s</p>";
    html += "</div>";
    
    html += "</div>";
    
    // JavaScript
    html += "<script>";
    html += "// Potwierdzenie zapisu konfiguracji";
    html += "document.getElementById('configForm').addEventListener('submit', function(e) {";
    html += "  if(!confirm('Czy na pewno chcesz zapisać konfigurację i zrestartować urządzenie?')) {";
    html += "    e.preventDefault();";
    html += "  }";
    html += "});";
    html += "</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_tube_info() {
    // Oblicz całkowity czas pracy tuby
    uint32_t tube_total_uptime = calculate_tube_uptime();
    
    // Formatujemy czas pracy tuby w latach i dniach
    String tube_uptime_formatted = format_years_days(tube_total_uptime);
    
    // Oblicz średnią dzienną
    float days = tube_total_uptime / 86400.0;
    float daily_avg = (days > 0.1) ? (total_lifetime_pulses / days) : 0;
    
    // Oblicz szacowany czas do wymiany w latach i dniach
    float remaining_days = 0;
    String remaining_time = "Brak danych";
    if (daily_avg > 0) {
        remaining_days = (TUBE_LIFETIME - total_lifetime_pulses) / daily_avg;
        uint32_t remaining_seconds = remaining_days * 86400;
        remaining_time = format_years_days(remaining_seconds);
    }
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Informacje o tubie Geigera</title>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; padding: 20px; }";
    html += ".container { max-width: 800px; margin: 0 auto; }";
    html += "h1 { color: #58a6ff; margin-bottom: 30px; }";
    html += "h2 { color: #58a6ff; margin-top: 40px; padding-bottom: 10px; border-bottom: 1px solid #30363d; }";
    html += ".info-section { background: #161b22; padding: 25px; border-radius: 10px; border: 1px solid #30363d; margin-bottom: 30px; }";
    html += ".info-item { margin-bottom: 15px; display: flex; justify-content: space-between; align-items: center; padding: 10px 0; border-bottom: 1px solid #30363d; }";
    html += ".info-label { font-weight: bold; color: #8b949e; }";
    html += ".info-value { font-family: monospace; color: white; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; cursor: pointer; text-decoration: none; display: inline-block; margin-top: 20px; }";
    html += ".btn:hover { background: #2ea043; }";
    html += ".btn-warning { background: #d29922; }";
    html += ".btn-warning:hover { background: #bb8009; }";
    html += ".warning { background: rgba(248, 81, 73, 0.1); border: 1px solid #f85149; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += ".progress-bar { width: 100%; height: 20px; background: #0d1117; border-radius: 10px; overflow: hidden; margin: 10px 0; }";
    html += ".progress-fill { height: 100%; background: linear-gradient(90deg, #238636, #2ea043); border-radius: 10px; }";
    html += ".progress-fill.warning { background: linear-gradient(90deg, #d29922, #bb8009); }";
    html += ".progress-fill.danger { background: linear-gradient(90deg, #f85149, #cc0000); }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🔬 Informacje o tubie Geigera</h1>";
    
    html += "<div class='info-section'>";
    html += "<h2>📊 Stan techniczny tuby</h2>";
    
    // Pasek postępu żywotności
    html += "<div style='margin-bottom: 20px;'>";
    html += "<div style='display: flex; justify-content: space-between; margin-bottom: 5px;'>";
    html += "<span class='info-label'>Zużycie tuby:</span>";
    html += "<span class='info-value'>" + String(tube_lifetime_percent, 2) + "%</span>";
    html += "</div>";
    String progress_class = "";
    if (tube_lifetime_percent > 80) progress_class = "warning";
    if (tube_lifetime_percent > 95) progress_class = "danger";
    html += "<div class='progress-bar'>";
    html += "<div class='progress-fill " + progress_class + "' style='width: " + String(tube_lifetime_percent) + "%'></div>";
    html += "</div>";
    
    // Status tekstowy
    String status_text = "";
    String status_color = "";
    if (tube_lifetime_percent < 50) {
        status_text = "DOSKONAŁY - tuba w idealnym stanie";
        status_color = "#2ea043";
    } else if (tube_lifetime_percent < 80) {
        status_text = "DOBRY - normalne zużycie";
        status_color = "#d29922";
    } else if (tube_lifetime_percent < 95) {
        status_text = "WYMAGANA UWAGA - rozważ wymianę";
        status_color = "#f85149";
    } else {
        status_text = "KRYTYCZNY - natychmiastowa wymiana wymagana";
        status_color = "#ff0000";
    }
    html += "<div style='margin-top: 10px; padding: 10px; background: rgba(13, 17, 23, 0.5); border-radius: 5px; border-left: 4px solid " + status_color + ";'>";
    html += "<strong>Status:</strong> " + status_text;
    html += "</div>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Całkowita liczba impulsów:</span>";
    html += "<span class='info-value'>" + String(total_lifetime_pulses) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Pozostało impulsów do limitu:</span>";
    html += "<span class='info-value'>" + String(TUBE_LIFETIME - total_lifetime_pulses) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Maksymalna żywotność tuby:</span>";
    html += "<span class='info-value'>" + String(TUBE_LIFETIME) + " impulsów</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Całkowity czas pracy tuby:</span>";
    html += "<span class='info-value'>" + tube_uptime_formatted + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Średnia liczba impulsów/dzień:</span>";
    html += "<span class='info-value'>" + String(daily_avg, 1) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Szacowany czas do wymiany:</span>";
    html += "<span class='info-value'>" + remaining_time + "</span>";
    html += "</div>";
    
    html += "</div>";
    
    if (tube_lifetime_percent > 80) {
        html += "<div class='warning'>";
        html += "<strong>⚠️ UWAGA:</strong> Żywotność tuby przekroczyła 80%. Zaleca się:<br>";
        html += "1. Rozważenie zakupu zapasowej tuby<br>";
        html += "2. Przeprowadzenie testu kalibracji<br>";
        html += "3. Monitorowanie stabilności pomiarów<br>";
        html += "4. Przygotowanie do wymiany tuby";
        html += "</div>";
    }
    
    html += "<div style='margin-top: 30px; display: flex; gap: 15px; flex-wrap: wrap;'>";
    html += "<a href='/' class='btn'>← Powrót do panelu głównego</a>";
    html += "<a href='/reset_tube' class='btn btn-warning' onclick='return confirm(\"UWAGA!\\\\n\\\\nCzy na pewno chcesz zresetować licznik tuby?\\\\n\\\\nUżywaj TYLKO po fizycznej wymianie tuby Geigera!\\\\n\\\\nResetowanie bez wymiany tuby spowoduje utratę danych o żywotności.\");'>🔄 Resetuj licznik (po wymianie tuby)</a>";
    html += "</div>";
    
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_reset_tube() {
    reset_tube_counter();
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Resetowanie licznika tuby</title><style>";
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
    html += "<h1>✅ Licznik tuby zresetowany</h1>";
    html += "<div class='success'>";
    html += "<strong>Sukces!</strong> Licznik żywotności tuby został zresetowany.";
    html += "<br><br>Założono nową tubę Geigera-Müllera.";
    html += "<br>Nowy licznik rozpoczął zliczanie impulsów.";
    html += "</div>";
    html += "<p>Przekierowanie za 3 sekundy...</p>";
    html += "</div>";
    html += "<script>setTimeout(function() { window.location.href = '/'; }, 3000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_info() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Informacje systemowe</title>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; padding: 20px; }";
    html += ".container { max-width: 800px; margin: 0 auto; }";
    html += "h1 { color: #58a6ff; margin-bottom: 30px; }";
    html += "h2 { color: #58a6ff; margin-top: 40px; padding-bottom: 10px; border-bottom: 1px solid #30363d; }";
    html += ".info-section { background: #161b22; padding: 25px; border-radius: 10px; border: 1px solid #30363d; margin-bottom: 30px; }";
    html += ".info-item { margin-bottom: 15px; display: flex; justify-content: space-between; align-items: center; padding: 10px 0; border-bottom: 1px solid #30363d; }";
    html += ".info-label { font-weight: bold; color: #8b949e; }";
    html += ".info-value { font-family: monospace; color: white; }";
    html += ".btn { background: #238636; color: white; padding: 12px 25px; border: none; border-radius: 6px; cursor: pointer; text-decoration: none; display: inline-block; margin-top: 20px; }";
    html += ".btn:hover { background: #2ea043; }";
    html += ".warning { background: rgba(248, 81, 73, 0.1); border: 1px solid #f85149; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>ℹ️ Szczegółowe informacje systemowe</h1>";
    
    html += "<div class='info-section'>";
    html += "<h2>📟 Informacje sprzętowe</h2>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>ID chipu:</span>";
    html += "<span class='info-value'>" + String((uint32_t)ESP.getEfuseMac(), HEX) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Procesor:</span>";
    html += "<span class='info-value'>" + String(ESP.getCpuFreqMHz()) + " MHz</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Pamięć RAM:</span>";
    html += "<span class='info-value'>" + String(ESP.getHeapSize() / 1024) + " KB (całkowita)</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Pamięć wolna:</span>";
    html += "<span class='info-value'>" + String(ESP.getFreeHeap()) + " bajtów</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Czas pracy systemu:</span>";
    html += "<span class='info-value'>" + String(uptime_str) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Wersja SDK:</span>";
    html += "<span class='info-value'>" + String(esp_get_idf_version()) + "</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='info-section'>";
    html += "<h2>🌐 Informacje sieciowe</h2>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Status WiFi:</span>";
    html += "<span class='info-value'>" + String(wifi_connected ? "✅ Połączono" : ap_mode ? "📶 Tryb AP" : "❌ Rozłączono") + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Adres IP:</span>";
    html += "<span class='info-value'>" + device_ip + "</span>";
    html += "</div>";
    
    if(wifi_connected) {
        html += "<div class='info-item'>";
        html += "<span class='info-label'>Siła sygnału (RSSI):</span>";
        html += "<span class='info-value'>" + String(WiFi.RSSI()) + " dBm</span>";
        html += "</div>";
        
        html += "<div class='info-item'>";
        html += "<span class='info-label'>SSID sieci:</span>";
        html += "<span class='info-value'>" + WiFi.SSID() + "</span>";
        html += "</div>";
        
        html += "<div class='info-item'>";
        html += "<span class='info-label'>Adres MAC:</span>";
        html += "<span class='info-value'>" + WiFi.macAddress() + "</span>";
        html += "</div>";
        
        html += "<div class='info-item'>";
        html += "<span class='info-label'>Nazwa hosta:</span>";
        html += "<span class='info-value'>" + String(WiFi.getHostname()) + "</span>";
        html += "</div>";
    }
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>mDNS primary:</span>";
    html += "<span class='info-value'>" + String(MDNS_PRIMARY_NAME) + ".local</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>mDNS secondary:</span>";
    html += "<span class='info-value'>" + String(MDNS_SECONDARY_NAME) + ".local</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='info-section'>";
    html += "<h2>📊 Informacje pomiarowe</h2>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>CPM (2-min):</span>";
    html += "<span class='info-value'>" + String(current_cpm, 1) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>ACPM (10-min):</span>";
    html += "<span class='info-value'>" + String(current_acpm, 1) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Moc dawki (µSv/h):</span>";
    html += "<span class='info-value'>" + String(current_usv_h, 4) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Dawka dzienna:</span>";
    html += "<span class='info-value'>" + String(daily_dose, 6) + " µSv</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Stabilność pomiaru:</span>";
    html += "<span class='info-value'>" + String(background_stability, 1) + "%</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Impulsy całkowite (sesja):</span>";
    html += "<span class='info-value'>" + String(total_pulses) + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Współczynnik (Factor):</span>";
    html += "<span class='info-value'>" + String(conversion_factor, 6) + "</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='info-section'>";
    html += "<h2>📡 Usługi systemowe</h2>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Serwer HTTP:</span>";
    html += "<span class='info-value'>✅ Port 80</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>OTA Update:</span>";
    html += "<span class='info-value'>✅ Aktywny</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>GMCMap:</span>";
    String aid = prefs.getString("aid", "");
    html += "<span class='info-value'>" + String((aid != "" && aid != "null") ? "✅ Aktywny" : "❌ Nieaktywny") + "</span>";
    html += "</div>";
    
    html += "<div class='info-item'>";
    html += "<span class='info-label'>Zapis danych NVS:</span>";
    html += "<span class='info-value'>✅ Aktywny</span>";
    html += "</div>";
    
    html += "</div>";
    
    html += "<div class='warning'>";
    html += "<strong>⚠️ Uwaga:</strong> Ta strona zawiera szczegółowe informacje techniczne. Większość użytkowników nie potrzebuje tych danych do normalnego korzystania z systemu.";
    html += "</div>";
    
    html += "<div style='margin-top: 20px; display: flex; gap: 15px; flex-wrap: wrap;'>";
    html += "<a href='/' class='btn'>← Powrót do panelu głównego</a>";
    html += "<a href='/tube_info' class='btn' style='background: linear-gradient(135deg, #d29922, #bb8009);'>🔬 Informacje o tubie</a>";
    html += "</div>";
    
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handle_save() {
    if (server.hasArg("ssid")) prefs.putString("ssid", server.arg("ssid"));
    if (server.hasArg("pass")) prefs.putString("pass", server.arg("pass"));
    if (server.hasArg("factor")) {
        String f_str = server.arg("factor"); 
        f_str.replace(',', '.');
        float f = f_str.toFloat(); 
        if (f > 0 && f <= 0.1) { 
            prefs.putFloat("factor", f); 
            conversion_factor = f; 
        }
    }
    if (server.hasArg("aid")) prefs.putString("aid", server.arg("aid"));
    if (server.hasArg("gid")) prefs.putString("gid", server.arg("gid"));
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Konfiguracja zapisana</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #30363d; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #2ea043; margin-bottom: 20px; }";
    html += ".loader { border: 4px solid #30363d; border-top: 4px solid #2ea043; border-radius: 50%; width: 40px; height: 40px; ";
    html += "animation: spin 1s linear infinite; margin: 0 auto 20px; }";
    html += "@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<div class='loader'></div>";
    html += "<h1>✅ Konfiguracja zapisana</h1>";
    html += "<p>Urządzenie zostanie zrestartowane za 3 sekundy...</p>";
    html += "<p>Po restarcie połączy się z nową siecią WiFi.</p>";
    html += "</div>";
    html += "<script>setTimeout(function() { window.location.href = '/'; }, 3000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
    delay(3000);
    ESP.restart();
}

void handle_restart() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Restartowanie</title><style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #0d1117; color: #c9d1d9; ";
    html += "display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 20px; }";
    html += ".container { background: #161b22; border: 1px solid #30363d; border-radius: 10px; padding: 40px; text-align: center; max-width: 500px; }";
    html += "h1 { color: #2ea043; margin-bottom: 20px; }";
    html += ".loader { border: 4px solid #30363d; border-top: 4px solid #2ea043; border-radius: 50%; width: 40px; height: 40px; ";
    html += "animation: spin 1s linear infinite; margin: 0 auto 20px; }";
    html += "@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<div class='loader'></div>";
    html += "<h1>🔄 Restartowanie urządzenia</h1>";
    html += "<p>Urządzenie zostanie zrestartowane za 3 sekundy...</p>";
    html += "<p>Po restarcie automatycznie połączy się z WiFi.</p>";
    html += "</div>";
    html += "<script>setTimeout(function() { window.location.href = '/'; }, 3000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
    delay(3000);
    ESP.restart();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n" + String(80, '='));
    Serial.println("     GEIGER DIY MONITOR v0.58 - TUBE MONITORING EDITION");
    Serial.println("     ESP32-C3 + RadiationD v1.1");
    Serial.println("     Autor: MaxGyver");
    Serial.println(String(80, '=') + "\n");
    
    pinMode(LED_PIN, OUTPUT); 
    digitalWrite(LED_PIN, HIGH);
    pinMode(CONFIG_RESET_PIN, INPUT_PULLUP);
    pinMode(GEIGER_PIN, INPUT); 

    // JEDEN namespace dla WSZYSTKICH danych
    prefs.begin("geiger", false);
    
    conversion_factor = prefs.getFloat("factor", DEFAULT_FACTOR);
    daily_dose = prefs.getFloat("daily_dose", 0.0);
    
    // Załaduj dane tuby (w tym samym namespace!)
    load_tube_state();
    
    Serial.printf("[SYSTEM] Współczynnik: %.6f\n", conversion_factor);
    Serial.printf("[SYSTEM] Tuba: %u impulsów (%.1f%%)\n", total_lifetime_pulses, (float)total_lifetime_pulses/TUBE_LIFETIME*100);
    
    // Oblicz i wyświetl czas pracy tuby
    uint32_t tube_uptime = calculate_tube_uptime();
    String tube_uptime_formatted = format_years_days(tube_uptime);
    Serial.printf("[SYSTEM] Czas pracy tuby: %s\n", tube_uptime_formatted.c_str());
    
    Serial.printf("[SYSTEM] WiFi SSID: %s\n", prefs.getString("ssid", DEFAULT_WIFI_SSID).c_str());

    attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), geiger_isr, FALLING);
    connect_wifi();
    
    server.on("/", handle_root);
    server.on("/save", HTTP_POST, handle_save);
    server.on("/restart", handle_restart);
    server.on("/info", handle_info);
    server.on("/tube_info", handle_tube_info);
    server.on("/reset_tube", handle_reset_tube);
    server.begin();
    
    Serial.println("\n[SYSTEM] Serwer HTTP uruchomiony na porcie 80");
    Serial.println("[SYSTEM] System gotowy do pracy");
    Serial.println("\n" + String(80, '-'));
    Serial.println("DOSTĘPNE ADRESY:");
    Serial.printf("  • http://%s\n", device_ip.c_str());
    Serial.printf("  • http://%s.local\n", MDNS_PRIMARY_NAME);
    Serial.printf("  • http://%s.local\n", MDNS_SECONDARY_NAME);
    Serial.println(String(80, '-') + "\n");
    
    // Konfiguracja OTA
    ArduinoOTA.onStart([]() {
        Serial.println("Rozpoczęcie aktualizacji OTA");
        digitalWrite(LED_PIN, HIGH);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nZakończenie aktualizacji OTA");
        digitalWrite(LED_PIN, LOW);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Postęp: %u%%\r", (progress / (total / 100)));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Błąd OTA[%u]: ", error);
        digitalWrite(LED_PIN, HIGH);
    });
    
    last_gmc_send = millis(); 
    last_tube_save = millis();
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    update_measurements();
    update_uptime();
    check_wifi_reconnect();
    
    uint32_t now = millis();
    
    if (now - last_gmc_send >= GMC_SEND_INTERVAL && wifi_connected) {
        send_to_gmcmap();
        last_gmc_send = now;
    }

    if (now - last_measurement_log >= MEASUREMENT_LOG_INTERVAL) {
        // Oblicz aktualny czas pracy tuby
        uint32_t tube_uptime = calculate_tube_uptime();
        String tube_uptime_formatted = format_years_days(tube_uptime);
        
        Serial.printf("[STAT] %s | CPM: %.1f | ACPM: %.1f | uSv/h: %.4f | Tuba: %u imp (%.1f%%) | Czas tuby: %s | RSSI: %d dBm\n", 
                      uptime_str, current_cpm, current_acpm, current_usv_h, 
                      total_lifetime_pulses, tube_lifetime_percent,
                      tube_uptime_formatted.c_str(), WiFi.RSSI());
        last_measurement_log = now;
    }
    
    if (now - last_dose_save >= DOSE_SAVE_INTERVAL) {
        prefs.putFloat("daily_dose", daily_dose);
        last_dose_save = now;
    }
    
    if (now - last_tube_save >= TUBE_SAVE_INTERVAL) {
        save_tube_state();
        last_tube_save = now;
    }

    if (digitalRead(CONFIG_RESET_PIN) == LOW) {
        delay(5000);
        if (digitalRead(CONFIG_RESET_PIN) == LOW) {
            Serial.println("\n[SYSTEM] RESET DO USTAWIEŃ FABRYCZNYCH!");
            // WAŻNE: ZACHOWAJ dane tuby!
            // Tylko konfiguracja systemowa jest czyszczona
            prefs.remove("ssid");
            prefs.remove("pass");
            prefs.remove("factor");
            prefs.remove("aid");
            prefs.remove("gid");
            prefs.remove("daily_dose");
            // NIE usuwamy: "tube_pulses", "tube_start_time", "last_known_unix"
            ESP.restart();
        }
    }
    
    // Mruganie LED w zależności od statusu
    static uint32_t last_led_blink = 0;
    if (!wifi_connected && !ap_mode) {
        if (now - last_led_blink > 500) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            last_led_blink = now;
        }
    } else if (ap_mode) {
        if (now - last_led_blink > 1000) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            last_led_blink = now;
        }
    }
    
    delay(1);
}
