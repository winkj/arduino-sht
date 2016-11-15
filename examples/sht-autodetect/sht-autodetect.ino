#include <Wire.h>

#include "SHTSensor.h"

SHTSensor sht;
// To use a specific sensor instead of probing the bus use this command:
// SHTSensor sht(SHTSensor::SHT3x);

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(9600);
  delay(1000); // let serial console settle

  sht.init();
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

}

void loop() {
  // put your main code here, to run repeatedly:

  sht.readSample();
  Serial.print("SHT:");
  Serial.print("  RH: ");
  Serial.print(sht.getHumidity(), 2);
  Serial.print("\n");
  Serial.print("  T:  ");
  Serial.print(sht.getTemperature(), 2);
  Serial.print("\n");

  delay(1000);
}
