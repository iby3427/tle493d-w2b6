
#include "hall3d.h"
#include "interface.h"
#include <Arduino.h>
#include <Wire.h>

void setup() {
  //-------------------------
  //  enable Serial
  //-------------------------
  Serial.begin(9600);
  while (!Serial);
  //-------------------------
  //  Init Sensor
  //  one bye read mode
  //  master contol mode
  //-------------------------
  TLE493D_reset();
  initSensor( );
}

void loop() {
  
  sensorUpdata();
  Serial.println(getX());
  Serial.println((String)"Bx = " + getX());
  Serial.println((String)"By = " + getY());
  Serial.println((String)"Bz = " + getZ());
  Serial.println((String)"Temp = " + getTemp());
  Serial.print((String)"Angle = ");
  Serial.println(getAngle());
  delay(500);
} 

void TLE493D_reset()
{
  Wire.requestFrom(0xFF, 0);
  Wire.requestFrom(0xFF, 0);
  Wire.beginTransmission(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x00);
  Wire.endTransmission();
  delayMicroseconds(30);
}
