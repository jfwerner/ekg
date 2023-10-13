/*
EKG Projekt WS 2023

Datum: 01.10.2023

Teammitglieder:
Tamara SUM
Johannes WERNER

Beschreibung: Main

*/

#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"

//#include <driver/adc.h>
//#include <driver/dac.h>

 

SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL | 128x64 display
uint8_t txval = 5;                    // DAC
uint16_t rxval; 	                    // ADC

float rxvoltage;
int16_t xdisplay = 0;
int16_t  ydisplay = 32;


void setup()
{

  Serial.begin(115200);

  // Display initialisieren
  display.init();
  display.clear();
  display.display();   

}

 

void loop()

{

  txval = (int) 127*(1+sin(2*PI*millis()/1000)); // sin ist im Bereich [0;2]. Mit *127 wird gesamter 8bit Bereich genutzt.
  dacWrite(25,txval);
  rxval = analogRead(33);


  rxvoltage = float(rxval)/4095*3.3; //ADC ist 12bit, Referenzspannung 3.3V

  const std::string displaytext = std::to_string(rxvoltage) + " V";

  // display.clear();

  if (xdisplay == 128)      // Reset wenn Display voll
  {
    display.clear();
    xdisplay = 0;
    ydisplay = 0;
  }

  //ydisplay = rxval/4095 * 64;     // Funktioniert nicht
  //ydisplay = 63 - (rxvoltage/3.3 * 64);      // [63 - ydisplay] für inverstiertes Ergebnis
  ydisplay = rxvoltage/3.3 * 64;

  //display.drawString(0,30, String(displaytext.c_str()));
  display.setPixel(xdisplay, ydisplay);
  display.display();

  Serial.printf("DAC: %d ADC: %d Voltage %6.3f Y: %d X: %d\n", txval, rxval, rxvoltage, ydisplay, xdisplay);

  //Serial.printf((String(displaytext.c_str())).c_str());

  xdisplay++;

  //Wenn 2s Anzeige: Wert skalieren und alle 2/128 s aufs display schreiben, bei pixel 128 löschen

  delay(8); // Ändern für Breite des Sinus

}