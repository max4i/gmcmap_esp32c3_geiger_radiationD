 # *Geiger DIY Monitor dla GMCmap.com*

## **System monitorowania promieniowania jonizującego zintegrowany z serwerem GMCmap.com.**

Amatorski system do monitorowania promieniowania tła naturalnego z interfejsem webowym, kalibracją i integracją z GMCMap.com.

![Code](images/c.jpg)
![Code](images/1.jpg)
![Code](images/2.jpg)
![Code](images/esp.jpg)

![Pełny widok interfejsu](images/Snap21.jpg)
![Pełny widok interfejsu](images/Snap23.jpg)

## 📋 Dostępne Funkcje

🔬 Pomiar promieniowania

CPM (Counts Per Minute) - średnia 2-minutowa

ACPM (Average CPM) - średnia 10-minutowa

µSv/h - moc dawki w mikrosiwertach na godzinę

Stabilność pomiaru - wskaźnik jakości pomiaru

Dawka dzienna - całkowita dawka od początku dnia

## 🌐 Sieć i dostęp

Wiele metod dostępu: IP, mDNS (geiger.local), 

kod QR

![QR Code](images/qr.jpg)

Tryb AP - konfiguracja przez WiFi gdy brak sieci

Interfejs webowy - responsywny, nowoczesny design

OTA updates - aktualizacje przez WiFi

## ⚙️ Konfiguracja i kalibracja

Konfiguracja WiFi przez interfejs webowy

Kalibracja współczynnika względem mapy PAA

Integracja z GMCMap.com - automatyczne wysyłanie danych

Reset do ustawień fabrycznych - przycisk fizyczny

## 📊 Edukacja i diagnostyka

Pełne objaśnienia wszystkich pomiarów

Instrukcja kalibracji krok po kroku

Słownik pojęć - teoria promieniowania

Diagnostyka systemu - stan pamięci, CPU, sieci

## 🛠️ Wymagania sprzętowe

Podstawowe:

ESP32-C3 supermini (lub kompatybilny)

Tuba Geigera-Müllera J305/J315

Płytka RadiationD v1.1 (lub kompatybilna)

Opcjonalne:

Obudowa ochronna druk 3D (pliki stl w katalogu STL)

Antena WiFi zewnętrzna (do zwiększenia zasięgu, tylko jeżeli jest taka potrzeba)

Zasilanie bateryjne (do zastosowań mobilnych, lub jako bacup zasilania z sieci na wypadek jego zaniku.)

## 🚀 Szybki start
1. Instalacja
   potrzebne biblioteki arduino
- WiFi
- WebServer
- Preferences
- ESPmDNS
- ArduinoOTA
- HTTPClient

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

## 📡 Dostęp do systemu

Po uruchomieniu dostępny przez:

🌍 Adres IP

http://[adres-IP-twojego-ESP32]/

🔍 mDNS (Automatyczne nazwy)

http://geiger.local/
http://geigercnt.local/

📱 Telefon
Zeskanuj kod QR z interfejsu webowego

Automatyczne przekierowanie do panelu

## 🎯 Kalibracja
Krok 1: Znajdź wartość referencyjną
Odwiedź mapę PAA

Znajdź swoją lokalizację

Odczytaj wartość tła naturalnego (np. 0.09 µSv/h)

Krok 2: Odczytaj stabilny ACPM

Pozostaw licznik na 15-20 minut

Odczytaj stabilną wartość ACPM (np. 30.3)

Krok 3: Obliczanie współczynnika

NOWY_FACTOR = (Wartość z mapy PAA) / (ACPM)
Przykład: 0.09 / 30.3 = 0.00297

Krok 4: Wprowadź do systemu
Przejdź do zakładki Konfiguracja

Wprowadź obliczony współczynnik

Zapisz i zrestartuj

## 🌐 Integracja z GMCMap.com
Konfiguracja:
Zarejestruj się na gmcmap.com

Utwórz licznik (Geiger ID)

Skopiuj AID i GID do konfiguracji systemu

Dane będą automatycznie wysyłane co 6 minut bo tyle wynosi minimaly odstęp czasu na przyjmowanie danych. 

Format wysyłanych danych:

### http://www.gmcmap.com/log2.asp?AID=[Account_ID]&GID=[Geiger_ID]&CPM=[2-min_avg]&ACPM=[10-min_avg]&uSV=[µSv/h]

  🏗️ Architektura systemu
Warstwa sprzętowa:
ESP32-C3 - mikrokontroler główny

Tuba Geigera - detektor promieniowania

Układ HV - zasilanie wysokiego napięcia

Interfejsy - WiFi, GPIO, UART

Warstwa oprogramowania:
Przerwanie - liczenie impulsów z tuby

Bufor cykliczny - 600-sekundowy (10-minutowy)

Algorytmy uśredniające - CPM i ACPM

Serwer HTTP - interfejs webowy

Menadżer WiFi - połączenie/AP/ponowne łączenie

## 🔧 Zaawansowane funkcje
Bezpieczeństwo:
Reset fabryczny - przytrzymanie przycisku 5 sekund

Ochrona przed błędami - watchdog timery

Sprawdzanie poprawności danych konfiguracyjnych

Optymalizacja:
Bufor cykliczny - efektywne użycie pamięci

Sleep WiFi - oszczędzanie energii

Aktualizacje delta - minimalizacja przesyłanych danych

Rozszerzalność:
Modułowa architektura - łatwe dodawanie funkcji

API REST - możliwość integracji z innymi systemami

Plugin system - dodatkowe czujniki i funkcje

## Uwaga: Ten system jest przeznaczony do amatorskiego monitorowania tła naturalnego. Nie nadaje się do pomiarów medycznych, przemysłowych ani sytuacji awaryjnych. Zawsze weryfikuj odczyty z oficjalnymi źródłami.

## 📸 Galeria interfejsu WWW.


![Dashboard](images/Snap2.jpg)

![Pomiary w czasie rzeczywistym](images/Snap3.jpg)

![Strona konfiguracyjna WiFi](images/Snap4.jpg)

![Kalibracja współczynnika](images/Snap5.jpg)

![Statystyki systemu](images/Snap6.jpg)

![Słownik pojęć](images/Snap7.jpg)

![Diagnostyka systemu](images/Snap8.jpg)

![Instrukcja kalibracji](images/Snap9.jpg)

![Metody dostępu](images/Snap10.jpg)

![Informacje o projekcie](images/Snap11.jpg)

![Monitorowanie w trybie AP](images/Snap12.jpg)

![Pełny widok interfejsu](images/Snap13.jpg)


# ===============================
## ver. 0.54 
 pełnosprawne oprogramowanie, Błędów nie zauważono.

 # ===============================
## ver. 0.58
 dodane informacje o stanie technicznym tuby. 

![Pełny widok interfejsu](images/Snap15.jpg)
 
 # ===============================
## ver. 0.60
 dodane wykres CPPM oraz obciążenie mikrokontrolera

![Pełny widok interfejsu](images/Snap18.jpg)

  # ===============================
## ver. 0.60a
dodano automatyczne przechodzenie w tryb AP po zaniku sieci WIFI.

  # ===============================
## ver. geiger_v086env_gmcmap
dodano kilka dupereli z czujnika bme680 i wysyłanie na dwa dodatkowe własne serwery.
w katalogu dodano peły gotowy kod na stronę www. można samemu zmodyfikować nazwę miasta, herb czy też logo. 
Przykład www.skwierzyna.com.pl

![Pełny widok interfejsu](images/Snap21.jpg)
![Pełny widok interfejsu](images/Snap20.jpg)
