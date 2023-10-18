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
//#define ADC_RESOLUTION 12 // 12-Bit-Auflösung
//#define ADC_SAMPLES 1 // Ein Sample pro Abtastung
//#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

// GPIO-Pin zum Schreiben
#define DAC_PIN 25


// TEstfunktion
uint8_t txval = 5;                    // DAC - Simuliert EKG Signal
uint16_t rxval; 	                    // ADC

float rxvoltage;
int16_t xdisplay = 0;
int16_t  ydisplay = 32;

int delayvalue = 0;

// Werte festlegen
int samplingFreq = 250;       // Abtastfrequenz ADC https://pubmed.ncbi.nlm.nih.gov/30109153/
int displayTimeShown = 2;     // Auf Display werden 2s an Daten angezeigt

// Variablen Definition
std::queue<uint16_t> ringBuffer;


// Ringbuffer für EKG-Daten
#define BUFFER_SIZE 7500                    // Für 30 Sekunden EKG-Daten bei 250 Hz Abtastung (7,500 = 30s * 250Hz)
volatile int16_t ekgBuffer[BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;

// Interrupt
bool interruptflag = false;
hw_timer_t * timer = NULL;
//portMUX_TYPE timerMUX = portMUX_INITIALIZER_UNLOCKED;

//void onTimer()            
void IRAM_ATTR onTimer()                    // Interrupt Ram Attritube
{
  //int16_t adcValue = analogRead(ADC_PIN);
  //ekgBuffer[bufferIndex] = adcValue;
  //bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  interruptflag = true;

  // Hier können Sie Code hinzufügen, um Daten auf GPIO 25 zu schreiben (WRITE_PIN).
  // Zum Beispiel digitalWrite(WRITE_PIN, HIGH) für eine bestimmte Bedingung.
}

void setup()
{
  Serial.begin(115200);

  // Display initialisieren
  display.init();
  display.clear();
  display.display(); 

  // Timer                                                              // https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_simpletimer/ESP32_simpletimer.ino
  Serial.println("Start Timer");                                        // Debug
  //Serial.println(interruptdelay);
  timer = timerBegin(0, 80, true);                                      // MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true);                          // edge (not level) triggered
  timerAlarmWrite(timer, (uint64_t)(1000*1000 / samplingFreq), true);   // 1000 * 1 us = 1 ms, autoreload true   // 1/250=0 !!!! int division
  timerAlarmEnable(timer);                                              // Timer enable
  Serial.println("End setup");
}


//8==========D
void loop()
{
  Serial.println("Interrupt Loop");

  if (interruptflag)
  {
    ringBuffer.push(analogRead(ADC_PIN));     // Read analog signal and write to buffer
    interruptflag = false;                    // Reset interrupt flag
  }


  txval = (int) 127*(1+sin(2*PI*millis()/1000)); // sin ist im Bereich [0;2]. Mit *127 wird gesamter 8bit Bereich genutzt.
  dacWrite(DAC_PIN, txval);
  rxval = analogRead(ADC_PIN);

  rxvoltage = float(rxval)/4095*3.3; //ADC ist 12bit,

  if (xdisplay == 128)
  {
    display.clear();
    xdisplay = 0;
    ydisplay = 0;
  }
  
  ydisplay = rxvoltage/3.3 * 64;

  if (delayvalue >= 8)
  {
    display.setPixel(xdisplay, ydisplay);
    display.display();
    xdisplay++;

    delayvalue = 0;
  }
  delayvalue++;
}