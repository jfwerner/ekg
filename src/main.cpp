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
#include <queue>
//#include <normal_distribution>
#include "logo.h"         // HKA Logo
// using namespace std;

// Testfunktion
uint8_t txval = 5;                    // DAC - Simuliert EKG Signal
//uint16_t rxval; 	                    // ADC

int delayvalue = 0;                          // Index für wie viel Werte für die Ausgabe übersprungen werden
// Werte festlegen
int samplingFreq = 250;                      // Abtastfrequenz ADC https://pubmed.ncbi.nlm.nih.gov/30109153/
int displayTimeShown = 2;                    // Auf Display werden 2s an Daten angezeigt
float rxvoltage;
float lastrxvoltage;
int lastHBtime = 0;
int bpm = 0;
float Rthreshold = 1.7;
std::queue<float> diffBetween3HB;
std::queue<float> diffBetweenHB;

//WiFiServer server(80);      // HTML Webserver
#define UDP_PORT 420
WiFiUDP UDP;
char packet[255];

struct udpPacket
{
  uint8_t highByte;
  uint8_t lowByte;
  uint8_t dataPosition;
};

const char* ssid = "Loading...";
const char* password = "";
//const char* ssid = "FRITZ!Box 7590 VL";
//const char* password = "56616967766283031728";

// GPIO-PIN definieren
#define ADC_PIN 33 // Pin für EKG-Signal (ADC-Konfiguration)
#define DAC_PIN 25 // GPIO-Pin zum Schreiben

SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL | 128x64 display https://github.com/ThingPulse/esp8266-oled-ssd1306

struct myPixel                              // Pixels for use with Display
{
  int16_t x = 0;
  int16_t y = 0;
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
uint16_t * lastHBptr = begin;

// Interrupt
bool interruptflag = false;
hw_timer_t * timer = NULL;
bool readytosend = false;
//portMUX_TYPE timerMUX = portMUX_INITIALIZER_UNLOCKED;

// Filterkoeffizienten
double alpha = 1 / samplingFreq;                                                 // 1 Hz = 2* 0.5Hz -> 50Hz +- 0.5Hz
double b[3] = {1.0, -2.0 * cos(2.0 * PI * 50.0 / 1000.0), 1.0};                  // Nullstellen des Filters: b = [b0 b1 b2]
double a[3] = {1.0 + alpha, -2.0 * cos(2.0 * PI * 50.0 / 1000.0), 1.0 - alpha};  // Pole des Filters: a = [a0 a1 a2]

// Zwischenspeicher des Filters
double Mem0 = 0; 
double Mem1 = 0; 
double Mem2 = 0; 

double data_flt;
 

void IRAM_ATTR readADC()                    // Interrupt Ram Attritube
{
  //int16_t adcValue = analogRead(ADC_PIN);
  //ekgBuffer[bufferIndex] = adcValue;
  //bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  interruptflag = true;
  delayvalue++;
  //digitalWrite(26, HIGH);                  // sets the digital pin 26 on

  *writeptr = analogRead(ADC_PIN);           // Read analog signal and write to buffer
  writeptr++;
  //interruptflag = false;                    // Reset interrupt flag
  //digitalWrite(26, LOW);                   // sets the digital pin 26 off
}

void setup()
{
  Serial.begin(115200);
  // Initialisierung PINS
  pinMode(17, OUTPUT);     // GPIO 17 als Output
  digitalWrite(17, true);         // High - Enable für EKG-Verstärker

  diffBetween3HB.push(0);
  diffBetween3HB.push(0);
  diffBetween3HB.push(0);
  

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

bool heartrateStuff()   // Iirgendwie 10bpm zu niedrig
{
  //Serial.printf("RXVoltage: %f\n", std::to_string(rxvoltage));
  //if (displayptr > lastHBptr)
  if (lastrxvoltage >= Rthreshold)
  {
    if (rxvoltage < Rthreshold)
    {
      // Stelle von neuem Herzschlag - letzten, * Zeitdauer von einer Messung zur nächsten in ms
      /*float timesincelastHB = (displayptr - lastHBptr)/sizeof(uint16_t)*(1000/250);
      Serial.println("TimesincelastHB");
      Serial.println(timesincelastHB);*/
      int rrInterval = (millis()-lastHBtime);
      lastHBtime = millis();
      bpm = (1000*60/rrInterval);   // 60s*Herzfrequenz = bpm
      
      /*
      Serial.println("BPM");
      Serial.println(bpm);
      display.clear();
      display.drawString(0,2, String(bpm));
      display.display();*/

      //lastHBptr = displayptr;
      diffBetweenHB.push(rrInterval);
      diffBetween3HB.push(rrInterval);
      diffBetween3HB.pop();
      //HRV = std::stddev(diffBetweenHB);
      //HRV = 0;
      lastrxvoltage = rxvoltage;
      return true;
    }
  }
  lastrxvoltage = rxvoltage;
  return false;
}

bool displaySignal()
{
  rxvoltage = float(*displayptr)/4095*3.3; //ADC ist 12bit,  
  heartrateStuff();
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
      display.drawString(0,2, String(bpm));
      //display.drawString(0,15, String(HRV));
      lastpx.x = 0;
      displaypx.x = 0;
    }
    delayvalue = 0;
    return true;
  }
  displayptr++;
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

bool swapBuffers()      // Setzt Pointer zurück, sobald sie das Ende erreicht haben
{
  if (writeptr == &ekgBuffer[BUFFER_SIZE])
  {
    Serial.printf("\nWriteptr before Reset %d", (writeptr - ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nDisplayptr: %d ", (displayptr - ekgBuffer)/sizeof(uint16_t));
    writeptr = begin;
    Serial.println("\nReset writeptr\n");
  }
  if (displayptr == &ekgBuffer[BUFFER_SIZE])    // Kann writeptr nicht überholen, weil er nur beim Interrupt aufgerufen wird
  {
    Serial.printf("\nDisplayptr before Reset %d", (displayptr-ekgBuffer)/sizeof(uint16_t));
    Serial.printf("\nWriteptr: %d", (writeptr-ekgBuffer)/sizeof(uint16_t));
    displayptr = begin;
    Serial.println("\nReset displayptr\n");
  }
  if (sendptr == &ekgBuffer[BUFFER_SIZE])   // Check wird in der Sendefunktion gemacht
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

/*
double IIR_Notch_Filter(uint16_t * adcvalue)                                             // Filter-Funktion auf Eingangsignal: Infiinite Impulse Response (unendliche Impulsantwort)
{
  double y;
    
  Mem0 = adcvalue - a[1] * Mem1 - a[2] * Mem2;                                         // Berechnung von Mem0 
  y = b[0] * Mem0 + b[1] * Mem1 + b[2] * Mem2;                                         // Ermittlung des gefilterten Werts
  // Abspeichern der berechneten Werte
  Mem2 = Mem1;
  Mem1 = Mem0; 

  return(y); 
}*/


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
    //data_flt = IIR_Filter_Calc(writeptr); // gefilterten Sensorwert berechnen (Filter 2. Ordnung)
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