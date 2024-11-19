#include <Wire.h>
#include "MT6701.h"

#define LedPin 13

MT6701 encoder;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  encoder.initializeI2C();

  pinMode(LedPin, OUTPUT);

  Serial.println("Wait for 1 second");
  delay(1000);

  // encoder.checkABZConfig();
  delay(3000);

  // First argument - how many steps per rotation, 1...1024
  // Second - pulse width (read datasheet), can be:
  //    MT6701_PULSE_WIDTH_1LSB
  //    MT6701_PULSE_WIDTH_2LSB
  //    MT6701_PULSE_WIDTH_4LSB
  //    MT6701_PULSE_WIDTH_8LSB
  //    MT6701_PULSE_WIDTH_12LSB
  //    MT6701_PULSE_WIDTH_16LSB
  //    MT6701_PULSE_WIDTH_180
  // Third - hysteresis, can be:
  //    MT6701_HYST_0_25
  //    MT6701_HYST_0_5
  //    MT6701_HYST_1
  //    MT6701_HYST_2
  //    MT6701_HYST_4
  //    MT6701_HYST_8
  encoder.abzModeSet(720, MT6701_PULSE_WIDTH_16LSB, MT6701_HYST_0_5);
  delay(1000);

  encoder.programmEEPROM(); // Save setting to EEPROM

  delay(1000);

  // encoder.checkABZConfig(); // Check if settings are saved

  digitalWrite(LedPin, HIGH);
  Serial.println("done");
}

void loop()
{
  // Nothing
}