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
uint8_t txval = 5;
uint16_t rxval;
float rxvoltage;
int xdisplay;

void setup()
{
    Serial.begin(115200);
    display.init();
    display.clear();
    display.display();
    //dac_output_enable(DAC_CHANNEL_1); // Kanal 1 auf GPIO 25
    //dac_output_voltage(DAC_CHANNEL_1, 255); // 255 = 3.3V  | 1.O V

    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
    
}

void loop()
{
    txval = (int) 127*(1+sin(2*PI*millis()/1000)); // sin ist im Bereich [0;2]. Mit *127 wird gesamter 8bit Bereich genutzt. 
    dacWrite(25,txval);
    rxval = analogRead(33);

    rxvoltage = float(rxval)/4095*3.3; //ADC ist 12bit, Referenzspannung 3.3V

    const std::string displaytext = std::to_string(rxvoltage) + " V";
    display.clear();

    if (xdisplay == 128)    // Reset wenn Display voll
    {
        display.clear();
        xdisplay = 0;
    }

    display.drawString(0,30, String(displaytext.c_str()));
    display.display();
    Serial.printf("DAC: %d ADC: %d Voltage %6.3f\n", txval, rxval, rxvoltage);
    //Serial.printf((String(displaytext.c_str())).c_str());

    display.setPixel(xdisplay, rxval/4095*64);  // Sinus auf 64px skalieren und zeichnen
    xdisplay++;

    delay(15); // 2/128 = 15ms
}