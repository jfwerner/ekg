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
//#include <Wire.h>   // Wird wohl seit 1.6.5 nicht mehr benötigt
#include "SSD1306Wire.h"
#include <queue>  // Für Ringbuffer
#include "pics.h"
// using namespace std;

//#include <driver/adc.h>
//#include <driver/dac.h>

SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL | 128x64 display https://github.com/ThingPulse/esp8266-oled-ssd1306

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

struct myPixel                              // Pixels for use with Display
{
  int16_t x = 0;
  int16_t y = 0;
};

myPixel lastpx;
myPixel displaypx;

int delayvalue = 0;                          // Index wie viel Werte für die Ausgabe übersprungen werden

// Werte festlegen
int samplingFreq = 250;                      // Abtastfrequenz ADC https://pubmed.ncbi.nlm.nih.gov/30109153/
int displayTimeShown = 2;                    // Auf Display werden 2s an Daten angezeigt

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

  digitalWrite(26, HIGH);                  // sets the digital pin 26 on
  ringBuffer.push(analogRead(ADC_PIN));     // Read analog signal and write to buffer
  interruptflag = false;                    // Reset interrupt flag
  digitalWrite(26, LOW);                   // sets the digital pin 26 off

  // Hier können Sie Code hinzufügen, um Daten auf GPIO 25 zu schreiben (WRITE_PIN).
  // Zum Beispiel digitalWrite(WRITE_PIN, HIGH) für eine bestimmte Bedingung.
}

void setup()
{
  Serial.begin(115200);


  // Initialisierung PINS
  //pinMode(33, INPUT);     // sets the digital pin 33 as input
  //pinMode(25, OUTPUT);    // sets the digital pin 25 as output
  pinMode(26, OUTPUT);    // sets the digital pin 26 as output

  // Display initialisieren
  display.init();
  display.clear();
  display.display();
/*
  display.drawXbm(20, 0, 28, 64, WiFi_Logo_bits);    // 28 64
  display.display(); 
  delay(5000);
  display.clear();*/

  // TimerInterrupt                                                             // https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_simpletimer/ESP32_simpletimer.ino
  Serial.println("Start Timer");                                        // Debug
  //Serial.println(interruptdelay);
  timer = timerBegin(0, 80, true);                                      // MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true);                          // edge (not level) triggered
  timerAlarmWrite(timer, (uint64_t)(1000*1000 / samplingFreq), true);   // 1000 * 1 us = 1 ms, autoreload true   // 1/250=0 !!!! int division
  timerAlarmEnable(timer);                                              // Timer enable
  Serial.println("End setup");
}


void loop()
{
  Serial.println("Interrupt Loop");
  if (interruptflag)
  {
    
  }


  txval = (int) 127*(1+sin(2*PI*millis()/1000)); // sin ist im Bereich [0;2]. Mit *127 wird gesamter 8bit Bereich genutzt.
  dacWrite(DAC_PIN, txval);
  

  if (displaypx.x == 128)             // Reset x coordinate to start of the display
  {
    display.clear();
    lastpx.x = 0;
    displaypx.x = 0;
  }
  
  if (delayvalue >= 5)      // Display every xth value
  {
    rxval = analogRead(ADC_PIN);

    rxvoltage = float(rxval)/4095*3.3; //ADC ist 12bit,

    displaypx.y = rxvoltage/3.3 * 64;   // Convert read data into display space

    display.setPixel(displaypx.x, displaypx.y);
    display.drawLine(displaypx.x, displaypx.y, lastpx.x, lastpx.y);  // Connect last and current pixel
    display.display();
    
    lastpx = displaypx;     // Save last coordinates for line drawing
    
    displaypx.x++;
    delayvalue = 0;
  }
  delayvalue++;
}