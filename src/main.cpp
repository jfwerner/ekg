/*
EKG Projekt WS 2023

Datum: 01.10.2023

Teammitglieder:
Tamara SUM          73319
Johannes WERNER

Beschreibung: Main

*/
#include <iostream>
#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <queue>  // Für Ringbuffer
// using namespace std;

//#include <driver/adc.h>
//#include <driver/dac.h>

SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL | 128x64 display

// ADC-Konfiguration
#define ADC_PIN 33 // Pin für EKG-Signal
#define ADC_RESOLUTION 12 // 12-Bit-Auflösung
#define ADC_SAMPLES 1 // Ein Sample pro Abtastung
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

// GPIO-Pin zum Schreiben
#define WRITE_PIN 25

// Werte festlegen
int samplingFreq = 250;       // Abtastfrequenz ADC https://pubmed.ncbi.nlm.nih.gov/30109153/
int displayTimeShown = 2;     // Auf Display werden 2s an Daten angezeigt

// Variablen Definition
std::queue<uint16_t> ringBuffer;

// Ringbuffer für EKG-Daten
#define BUFFER_SIZE 7500                    // Für 30 Sekunden EKG-Daten bei 250 Hz Abtastung (7,500 = 30s * 250Hz)
volatile int16_t ekgBuffer[BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMUX = portMUX_INITIALIZER_UNLOCKED;

//void onTimer()            
void IRAM_ATTR onTimer()                    // Interrupt Ram Attritube
{
  int16_t adcValue = analogRead(ADC_PIN);
  ekgBuffer[bufferIndex] = adcValue;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Hier können Sie Code hinzufügen, um Daten auf GPIO 25 zu schreiben (WRITE_PIN).
  // Zum Beispiel digitalWrite(WRITE_PIN, HIGH) für eine bestimmte Bedingung.
}

void setup()
{
  Serial.begin(115200);

  // Display initialisieren
  //display.init();
  //display.clear();
  //display.display(); 

  // Timer                                                        // https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_simpletimer/ESP32_simpletimer.ino
  Serial.println("Start Timer");                                  // Debug
  timer = timerBegin(0, 80, true);                                // MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true);                    // edge (not level) triggered
  timerAlarmWrite(timer, (1/samplingFreq)*1000, true);            // 1000 * 1 us = 1 ms, autoreload true
  timerAlarmEnable(timer);                                        // enable
}


void loop()
{
  Serial.println("Interrupt Loop");
  delay(500);
}