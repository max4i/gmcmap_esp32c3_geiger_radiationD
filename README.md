Geiger DIY Monitor 
System monitorowania promieniowania jonizującego zintegrowany z serwerem GMCmap.com.


Aamatorski system do monitorowania promieniowania tła naturalnego z interfejsem webowym,
kalibracją i integracją z GMCMap.com.

📋 Funkcje
🔬 Pomiar promieniowania
CPM (Counts Per Minute) - średnia 2-minutowa

ACPM (Average CPM) - średnia 10-minutowa

µSv/h - moc dawki w mikrosiwertach na godzinę

Stabilność pomiaru - wskaźnik jakości pomiaru

Dawka dzienna - całkowita dawka od początku dnia

🌐 Sieć i dostęp
Wiele metod dostępu: IP, mDNS (geiger.local), kod QR

Tryb AP - konfiguracja przez WiFi gdy brak sieci

Interfejs webowy - responsywny, nowoczesny design

OTA updates - aktualizacje przez WiFi

⚙️ Konfiguracja i kalibracja
Konfiguracja WiFi przez interfejs webowy

Kalibracja współczynnika względem mapy PAA

Integracja z GMCMap.com - automatyczne wysyłanie danych

Reset do ustawień fabrycznych - przycisk fizyczny

📊 Edukacja i diagnostyka

Pełne objaśnienia wszystkich pomiarów

Instrukcja kalibracji krok po kroku

Słownik pojęć - teoria promieniowania

Diagnostyka systemu - stan pamięci, CPU, sieci

🛠️ Wymagania sprzętowe

Podstawowe:
ESP32-C3 supermini (lub kompatybilny)

Tuba Geigera-Müllera J305/J315
Płytka RadiationD v1.1 (lub kompatybilna)

Opcjonalne:
Obudowa ochronna druk 3D (pliki stl w katalogu STL)

Antena WiFi zewnętrzna (dla lepszego zasięgu, tylko jeżeli jest taka potrzeba)

Zasilanie bateryjne (do zastosowań mobilnych, lub jako bacup zasilania z sieci.)

🚀 Szybki start
1. Instalacja
   potrzebne biblioteki arduino
   #- WiFi
#- WebServer
#- Preferences
# - ESPmDNS
# - ArduinoOTA
# - HTTPClient

2. Podłączenie sprzętowe
   GEIGER_PIN     -> GPIO4
LED_PIN        -> GPIO8 (WS2812 lub dioda)
CONFIG_RESET_PIN -> GPIO9 (przycisk)

3. Pierwsze uruchomienie
Wgraj firmware na ESP32-C3

Połącz się z siecią WiFi Geiger-AP

Przejdź do http://192.168.4.1

Skonfiguruj swoją sieć WiFi

System zrestartuje się i połączy z Twoją siecią

📡 Dostęp do systemu
Po uruchomieniu dostępny przez:

🌍 Adres IP
http://[adres-IP-twojego-ESP32]/
🔍 mDNS (Automatyczne nazwy)
http://geiger.local/
http://geigercnt.local/
