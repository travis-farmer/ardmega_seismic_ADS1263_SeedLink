#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ADS126X.h> 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

const int W5500_CS = 10;
const int ADC_CS   = 4;
const int ADC_DRDY = 9;

LiquidCrystal_I2C lcd(0x27,20,4);

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xDD, 0xDD };
EthernetServer server(18000);
IPAddress ip(192, 168, 1, 123);

ADS126X adc;
unsigned long nextSampleMicros = 0;
const unsigned long sampleInterval = 10000; // 10ms in microseconds
int32_t offsetZ = 0, offsetN = 0, offsetE = 0;

unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 2000; // Update screen every 2000ms (0.5Hz)
bool lcdActive = true; // Start with top LCD active

void calibrateOffsets() {

  int32_t sumZ = 0, sumN = 0, sumE = 0;
  const int samples = 100;

  for(int i = 0; i < samples; i++) {
    sumZ += adc.readADC1(ADS126X_AIN0, ADS126X_AIN1);
    sumN += adc.readADC1(ADS126X_AIN2, ADS126X_AIN3);
    sumE += adc.readADC1(ADS126X_AIN4, ADS126X_AIN5);
    delay(5);
  }
  
  offsetZ = sumZ / samples;
  offsetN = sumN / samples;
  offsetE = sumE / samples;

  wdt_reset();
}

void setup() {
  wdt_disable();
  lcd.init();
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 0);
  lcd.print("SeisComP SeedLink   ");

  wdt_enable(WDTO_8S);

  pinMode(ADC_DRDY, INPUT);
  wdt_reset();
  Ethernet.init(W5500_CS); 
  if (Ethernet.begin(mac) == 0) {
    lcd.setCursor(0, 1);
    lcd.print("DHCP Failed         ");
    if (Ethernet.linkStatus() == LinkOFF) {
      lcd.setCursor(0, 2);
      lcd.print("No Link             ");
      while(1); // Halt if no DHCP and no Link, let user fix the issue
    }
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Good DHCP           ");
  }
  server.begin();
  wdt_reset();
  adc.begin(ADC_CS);
  
  // The library wants (Negative Ref, Positive Ref)
  adc.setReference(ADS126X_REF_NEG_INT, ADS126X_REF_POS_INT); // Using the internal reference for both negative and positive inputs, which is common for single-ended measurements. Adjust if using external reference.
  
  adc.setRate(ADS126X_RATE_1200); 
  adc.setGain(ADS126X_GAIN_32); 

  // 1. Update the local library mirror for the FILTER register (0x03)
  // 0x05 = Sinc4 + 60Hz Rejection
  adc.REGISTER_ARRAY[0x03] = 0x05; 

  // 2. Push the change from the Mega to the ADS1263
  // This writes 1 register starting at address 0x03
  adc.writeRegisters(0x03, 1);
  wdt_reset();
  calibrateOffsets();

  nextSampleMicros = micros();
}

void loop() {
  EthernetClient client = server.available();
  unsigned long currentMicros = micros();

  // 100Hz Timer (10,000 microseconds)
  if (currentMicros - nextSampleMicros >= 10000) {
    nextSampleMicros += 10000;

    // 1. Read Raw 32-bit values from ADS1263
    int32_t rawZ = adc.readADC1(ADS126X_AIN0, ADS126X_AIN1); 
    int32_t rawN = adc.readADC1(ADS126X_AIN2, ADS126X_AIN3);
    int32_t rawE = adc.readADC1(ADS126X_AIN4, ADS126X_AIN5);

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
  wdt_reset();

  if (millis() - lastLcdUpdate >= lcdInterval) {
    lastLcdUpdate = millis();
    if (lcdActive) {
      
      // We use setCursor and print OVER old data to avoid the blocking lcd.clear()
      lcd.setCursor(0, 0);
      lcd.print("SeisComP SeedLink   ");
      lcd.setCursor(0, 1);
      lcd.print("IP: ");
      lcd.print(Ethernet.localIP());
      
      lcd.print("      ");   // Clear trailing digits
      lcd.setCursor(0, 2);
      if (Ethernet.linkStatus() == LinkOFF) {
        lcd.print("Link: OFF           ");
      } else {
        lcd.print("Link: ON            ");
      }
      lcd.setCursor(0, 3);
      if (client.connected()) {
        lcd.print("Client              ");
      } else {
        lcd.print("No Client           ");
      }
      lcdActive = false;
    } else {
      lcd.setCursor(0, 0);
      lcd.print("ADS1263 Cal-Offsets ");
      lcd.setCursor(0, 1);
      lcd.print("Z: ");
      lcd.print(offsetZ);
      lcd.print("      "); // Clear trailing digits
      lcd.setCursor(0, 2);
      lcd.print("N: ");
      lcd.print(offsetN);
      lcd.print("      "); // Clear trailing digits
      lcd.setCursor(0, 3);
      lcd.print("E: ");
      lcd.print(offsetE);
      lcd.print("      "); // Clear trailing digits
      lcdActive = true;
    }
  }
  wdt_reset();
}
