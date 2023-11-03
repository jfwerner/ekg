/*
EKG Projekt WS 2023

Start: 01.10.2023
Ende: 14.11.2023

Teammitglieder:
Tamara SUM          73319
Johannes WERNER     73431

Beschreibung: Main

*/
#include <iostream>
#include <Arduino.h>
//#include <Wire.h>       // Wird wohl seit 1.6.5 nicht mehr benötigt
#include "SSD1306Wire.h"
#include <WiFiUdp.h>
#include "WiFi.h"
//#include <queue>          // Für Ringbuffer
#include "logo.h"         // HKA Logo
// using namespace std;

// Testfunktion
uint8_t txval = 5;                    // DAC - Simuliert EKG Signal
//uint16_t rxval; 	                    // ADC

int delayvalue = 0;                          // Index für wie viel Werte für die Ausgabe übersprungen werden
// Werte festlegen
int samplingFreq = 250;                      // Abtastfrequenz ADC https://pubmed.ncbi.nlm.nih.gov/30109153/
int displayTimeShown = 2;                    // Auf Display werden 2s an Daten angezeigt


WiFiServer server(80);      // HTML Webserver
#define UDP_PORT 420
WiFiUDP UDP;
char packet[255];

//const char* ssid = "Loading...";
//const char* password = "";
const char* ssid = "FRITZ!Box 7590 VL";
const char* password = "56616967766283031728";

// ADC-Konfiguration
#define ADC_PIN 33 // Pin für EKG-Signal
//#define ADC_RESOLUTION 12 // 12-Bit-Auflösung
//#define ADC_SAMPLES 1 // Ein Sample pro Abtastung
//#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

// GPIO-Pin zum Schreiben
#define DAC_PIN 25

float rxvoltage;

SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL | 128x64 display https://github.com/ThingPulse/esp8266-oled-ssd1306

struct myPixel                              // Pixels for use with Display
{
  int16_t x = 0;
  int16_t y = 0;
};

struct udpPacket
{
  uint8_t highByte;
  uint8_t lowByte;
  uint8_t dataPosition;
};

myPixel lastpx;
myPixel displaypx;

// Variablen Definition


// Ringbuffer für EKG-Daten
#define BUFFER_SIZE 7500                    // Für 30 Sekunden EKG-Daten bei 250 Hz Abtastung (7,500 = 30s * 250Hz)
uint16_t ekgBuffer[BUFFER_SIZE];            // volatile ?
//uint16_t bufferIndex = 0;

uint16_t * begin = &ekgBuffer[0];
uint16_t * middle = &ekgBuffer[BUFFER_SIZE/2];


uint16_t * writeptr = begin;
uint16_t * displayptr = begin;
uint16_t * sendptr = begin;

//bool readAwriteB;

// Interrupt
bool interruptflag = false;
hw_timer_t * timer = NULL;
bool readytosend = false;
//portMUX_TYPE timerMUX = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR readADC()                    // Interrupt Ram Attritube
{
  //int16_t adcValue = analogRead(ADC_PIN);
  //ekgBuffer[bufferIndex] = adcValue;
  //bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  interruptflag = true;
  delayvalue++;
  //digitalWrite(26, HIGH);                  // sets the digital pin 26 on

  *writeptr = analogRead(ADC_PIN); // Read analog signal and write to buffer
  writeptr++;
  //interruptflag = false;                    // Reset interrupt flag
  //digitalWrite(26, LOW);                   // sets the digital pin 26 off
}

void setup()
{
  Serial.begin(115200);
  // Initialisierung PINS
  //pinMode(33, INPUT);     // sets the digital pin 33 as input
  // GPIO 17 auf High

  // Display initialisieren
  display.init();
  display.clear();
  display.display();
/*
  display.drawXbm(20, 0, 28, 64, WiFi_Logo_bits);    // 28 64
  display.display(); 
  delay(5000);
  display.clear();*/

  ///////////////////////////////////////////////////////////////////////////
  // Netzwerkverbindung
  WiFi.mode(WIFI_STA);    // Connect to AP 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Verbindung zum WLAN-Netzwerk wird hergestellt...");
    display.drawString(2, 2, "Verbindung zum WLAN");
    display.drawString(2, 15, "wird hergestellt...");
    display.display();
    delay(1000);
    if (millis() > 30*1000) {
      Serial.println("Verbindung fehlgeschlagen! ESP wird neugestartet");
      display.drawString(0, 0, "Fehler, Neustart!");
      display.display();
      delay(3000);
      ESP.restart();
    }
  }
  Serial.println("Verbindung zum WLAN-Netzwerk hergestellt");
  display.drawString(0, 0, "Verbindung hergestellt");
  display.display();

  UDP.begin(UDP_PORT);
  //Serial.printf("UDP server IP:", String(WiFi.localIP()).c_str());
  display.clear();
  display.drawString(0,0, "IP-Address (local):");
  display.drawString(0,15, WiFi.localIP().toString());            
  //display.drawString(0,30, "Portnumber (local):" + String(std::to_string(UDP_PORT)));
  display.drawString(0,45, (String) UDP_PORT);
  display.display();
  Serial.printf("IP: %s\n", (String) WiFi.localIP().toString());
  Serial.printf("Port: %s\n", (String) UDP_PORT);
  delay(5000);
  display.clear();

  ///////////////////////////////////////////////////////////////////////////
  // TimerInterrupt                                                     // https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_simpletimer/ESP32_simpletimer.ino
  Serial.println("\nStart Timer");                                        // Debug
  //Serial.println(interruptdelay);
  timer = timerBegin(0, 80, true);                                      // MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &readADC, true);                          // edge (not level) triggered
  timerAlarmWrite(timer, (uint64_t)(1000*1000 / samplingFreq), true);   // 1000 * 1 us = 1 ms, autoreload true   // 1/250=0 !!!! int division
  timerAlarmEnable(timer);                                              // Timer enable
  Serial.println("End setup");
}

bool displaySignal()
{
  rxvoltage = float(*displayptr)/4095*3.3; //ADC ist 12bit,
  displayptr++;
  if (delayvalue >= 5)      // Display every xth value
  {
    //uint16_t rxval = analogRead(ADC_PIN);

    displaypx.y = rxvoltage/3.3 * 64;   // Convert read data into display space

    display.setPixel(displaypx.x, displaypx.y);
    display.drawLine(displaypx.x, displaypx.y, lastpx.x, lastpx.y);  // Connect last and current pixel
    display.display();
    
    lastpx = displaypx;     // Save last coordinates for line drawing
    
    displaypx.x++;
    
    if (displaypx.x == 128)             // Reset x coordinate to start of the display
    {
      display.clear();
      lastpx.x = 0;
      displaypx.x = 0;
    }
    delayvalue = 0;
    return true;
  }
  return false;
}

bool receiveUDP()
{
  int packetSize = UDP.parsePacket();
  if (packetSize > 0) {
    Serial.print("Received packet! Size: ");
    Serial.println(packetSize); 
    int len = UDP.read(packet, 255);
    if (len > 0)
    {
      packet[len] = '\0';
    }
    Serial.println(packet);
    return true;
  }
  return false;
}

bool sendAnswer()
{
  UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
  /*char reply[] = "received";
  std::string replly = "received";
  for (int i = 0; i<8; i++)
  {
    //UDP.write(char(reply[i]));
  }
  UDP.write(10);
  //UDP.write(replly.c_str(),replly.length());*/
  //UDP.write(10);
  UDP.printf("\nreceived");
  if (UDP.endPacket())
    return true;

  return false;
}

bool swapBuffers()      // Setzt pointer zurück, sobald sie das Ende erreicht haben
{
  if (writeptr == &ekgBuffer[2*BUFFER_SIZE])
  {
    Serial.printf("\nWriteptr before Reset %d", (writeptr - ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nDisplayptr: %d ", (displayptr - ekgBuffer)/sizeof(uint16_t));
    writeptr = begin;
    Serial.println("\nReset writeptr\n");
  }
  if (displayptr == &ekgBuffer[2*BUFFER_SIZE])    // Kann writeptr nicht überholen, weil er nur beim Interrupt aufgerufen wird
  {
    Serial.printf("\nDisplayptr before Reset %d", (displayptr-ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nWriteptr: %d", (writeptr-ekgBuffer)/sizeof(uint16_t));
    displayptr = begin;
    Serial.println("\nReset displayptr\n");
  }
  if (sendptr == &ekgBuffer[2*BUFFER_SIZE])   // Check wird in der Sendefunktion gemacht
  {
    
    Serial.printf("\nSendptr before Reset %d ", (sendptr-ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nWriteptr: %d", (writeptr-ekgBuffer)/sizeof(uint16_t));
    sendptr = begin;
    Serial.println("\nReset sendptr\n");
  }
  else
    return false;
  return true;
}

bool sendEKGdata()
{
  //UDP.parsePacket();
  UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
  
  if (writeptr == middle || writeptr == begin)   // Passt == begin? Oder == BUFFERSIZE?
  {
    Serial.printf("\nBuffer wird gesendet\nSendptr Stelle %d" , (sendptr-ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nWriteptr Stelle %d\n", (writeptr-ekgBuffer)/sizeof(uint16_t));

    UDP.write((uint8_t*)sendptr, BUFFER_SIZE);  // Hälfte des Buffers schicken (8 bit) -> 3750 Werte
    sendptr = sendptr + BUFFER_SIZE/2;
  }
  
  UDP.endPacket();
  //Serial.println("Sent");
  return true;
}

void loop()
{
  txval = (int) 127*(1+sin(2*PI*millis()/1000)); // sin ist im Bereich [0;2]. Mit *127 wird gesamter 8bit Bereich genutzt.
  dacWrite(DAC_PIN, txval);

  if (interruptflag)
  { 
    displaySignal();
    
    swapBuffers();

    interruptflag = false;
  }


  if (receiveUDP() || readytosend)  // Start sending data when a connection is established
  {
    readytosend = true;
    sendEKGdata();
  }
}