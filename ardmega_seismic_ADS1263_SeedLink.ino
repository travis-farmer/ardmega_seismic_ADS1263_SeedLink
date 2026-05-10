#include <SPI.h>
#include <Ethernet.h>
#include <ADS126X.h> 

#define LED_DHCP 0
#define LED_LINK 1
#define LED_CAL 2
#define LED_SAMPLING 3

int ledGpins[] = {22, 24, 26, 28}; // Green LEDs for DHCP, Link, Calibration, Sampling
int ledRpins[] = {23, 25, 27, 29}; // Red LEDs for DHCP, Link, Calibration, Sampling

const int W5500_CS = 10;
const int ADC_CS   = 4;
const int ADC_DRDY = 9;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xDD, 0xDD };
EthernetServer server(18000);
IPAddress ip(192, 168, 1, 123);

ADS126X adc;
unsigned long nextSampleMicros = 0;
const unsigned long sampleInterval = 10000; // 10ms in microseconds
int32_t offsetZ = 0, offsetN = 0, offsetE = 0;

void calibrateOffsets() {
  //Serial.println("Calibrating True Zero (Keep Station Still)...");
  int32_t sumZ = 0, sumN = 0, sumE = 0;
  const int samples = 100;
  digitalWrite(ledRpins[LED_CAL], HIGH); // Red ON for Calibration
  for(int i = 0; i < samples; i++) {
    sumZ += adc.readADC1(ADS126X_AIN2, ADS126X_AIN3);
    sumN += adc.readADC1(ADS126X_AIN4, ADS126X_AIN5);
    sumE += adc.readADC1(ADS126X_AIN6, ADS126X_AIN7);
    delay(10);
  }
  
  offsetZ = sumZ / samples;
  offsetN = sumN / samples;
  offsetE = sumE / samples;
  digitalWrite(ledRpins[LED_CAL], LOW); // Red OFF after Calibration
  digitalWrite(ledGpins[LED_CAL], HIGH); // Green ON for Calibration Success
}

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(ledGpins[i], OUTPUT);
    pinMode(ledRpins[i], OUTPUT);
    digitalWrite(ledGpins[i], LOW);
    digitalWrite(ledRpins[i], LOW);
  }
  // test all LEDs at startup
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledGpins[i], HIGH);
    digitalWrite(ledRpins[i], HIGH);
    delay(200);
    digitalWrite(ledGpins[i], LOW);
    digitalWrite(ledRpins[i], LOW);
  }
  //Serial.begin(115200);
  pinMode(ADC_DRDY, INPUT);

  Ethernet.init(W5500_CS); 
  if (Ethernet.begin(mac) == 0) {
    digitalWrite(ledRpins[LED_DHCP], HIGH); // Red ON for DHCP failure
    if (Ethernet.linkStatus() == LinkOFF) {
      digitalWrite(ledRpins[LED_LINK], HIGH); // Red ON for Link failure
    } else {
      digitalWrite(ledGpins[LED_LINK], HIGH); // Green ON for Link success
    }
    // try to configure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  } else {
    digitalWrite(ledGpins[LED_DHCP], HIGH); // Green ON for DHCP success
    if (Ethernet.linkStatus() == LinkOFF) {
      digitalWrite(ledRpins[LED_LINK], HIGH); // Red ON for Link failure
    } else {
      digitalWrite(ledGpins[LED_LINK], HIGH); // Green ON for Link success  
    }
  }
  server.begin();

  adc.begin(ADC_CS);
  
  // The library wants (Negative Ref, Positive Ref)
  // Negative Reference on AIN1, Positive Reference on AIN0
  adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0);
  
  adc.setRate(ADS126X_RATE_1200); 
  adc.setGain(ADS126X_GAIN_32); 

  // 1. Update the local library mirror for the FILTER register (0x03)
  // 0x05 = Sinc4 + 60Hz Rejection
  adc.REGISTER_ARRAY[0x03] = 0x05; 

  // 2. Push the change from the Mega to the ADS1263
  // This writes 1 register starting at address 0x03
  adc.writeRegisters(0x03, 1);

  calibrateOffsets();

  //Serial.println("FARM Station | HHZ 100Hz | External AD580KH Ref Enabled");
  nextSampleMicros = micros();
}

void loop() {
  EthernetClient client = server.available();
  digitalWrite(ledGpins[LED_SAMPLING], client.connected() ? HIGH : LOW); // Green ON if client connected
  digitalWrite(ledRpins[LED_SAMPLING], client.connected() ? LOW : HIGH); // Red ON if no client
  unsigned long currentMicros = micros();

  // 100Hz Timer (10,000 microseconds)
  if (currentMicros - nextSampleMicros >= 10000) {
    nextSampleMicros += 10000;

    // 1. Read Raw 32-bit values from ADS1263
    int32_t rawZ = adc.readADC1(ADS126X_AIN2, ADS126X_AIN3); 
    int32_t rawN = adc.readADC1(ADS126X_AIN4, ADS126X_AIN5);
    int32_t rawE = adc.readADC1(ADS126X_AIN6, ADS126X_AIN7);

    // 2. Apply the True Zero Offsets (from our calibration)
    int32_t finalZ = rawZ - offsetZ;
    int32_t finalN = rawN - offsetN;
    int32_t finalE = rawE - offsetE;

    // 3. Send to SeedLink via TCP
    if (client.connected()) {
      // Option A: Raw Binary (Most efficient for 32-bit)
      // We send 12 bytes total (4 per axis)
      client.write((uint8_t*)&finalZ, 4);
      client.write((uint8_t*)&finalN, 4);
      client.write((uint8_t*)&finalE, 4);
    }
  }
}