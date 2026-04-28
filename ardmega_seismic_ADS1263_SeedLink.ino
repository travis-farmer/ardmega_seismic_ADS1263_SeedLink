#include <SPI.h>
#include <Ethernet.h>
#include <ADS126X.h> 

const int W5500_CS = 10;
const int ADC_CS   = 4;
const int ADC_DRDY = 9;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xDD, 0xDD };
EthernetServer server(18000);

ADS126X adc;
unsigned long nextSampleMicros = 0;
const unsigned long sampleInterval = 10000; // 10ms in microseconds
int32_t offsetZ = 0, offsetN = 0, offsetE = 0;

void calibrateOffsets() {
  Serial.println("Calibrating True Zero (Keep Station Still)...");
  int32_t sumZ = 0, sumN = 0, sumE = 0;
  const int samples = 100;

  for(int i = 0; i < samples; i++) {
    sumZ += adc.readADC1(ADS126X_AIN2, ADS126X_AIN3);
    sumN += adc.readADC1(ADS126X_AIN4, ADS126X_AIN5);
    sumE += adc.readADC1(ADS126X_AIN6, ADS126X_AIN7);
    delay(10);
  }
  
  offsetZ = sumZ / samples;
  offsetN = sumN / samples;
  offsetE = sumE / samples;
  Serial.println("Calibration Complete.");
}

void setup() {
  Serial.begin(115200);
  pinMode(ADC_DRDY, INPUT);

  Ethernet.init(W5500_CS); 
  Ethernet.begin(mac);
  server.begin();

  adc.begin(ADC_CS);
  
  // The library wants (Negative Ref, Positive Ref)
  // Negative Reference on AIN1, Positive Reference on AIN0
  adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0);
  
  adc.setRate(ADS126X_RATE_1200); 
  adc.setGain(ADS126X_GAIN_1); 

  Serial.println("FARM Station | HHZ 100Hz | External AD580KH Ref Enabled");
  nextSampleMicros = micros();

  calibrateOffsets();
}

void loop() {
  EthernetClient client = server.available();
  
  if (client) {
    while (client.connected()) {
      // Precision interval check
      if (micros() >= nextSampleMicros) {
        nextSampleMicros += sampleInterval; 

        // Rapidly poll the 3 channels
        int32_t valZ = adc.readADC1(ADS126X_AIN2, ADS126X_AIN3); 
        int32_t valN = adc.readADC1(ADS126X_AIN4, ADS126X_AIN5); 
        int32_t valE = adc.readADC1(ADS126X_AIN6, ADS126X_AIN7); 

        int32_t finalZ = valZ - offsetZ;
        int32_t finalN = valN - offsetN;
        int32_t finalE = valE - offsetE;

        // Transmit. Note: CSV is simple, but binary is faster if Proxmox lags.
        client.print(finalZ); client.print(",");
        client.print(finalN); client.print(",");
        client.println(finalE);
      }
    }
  }
}