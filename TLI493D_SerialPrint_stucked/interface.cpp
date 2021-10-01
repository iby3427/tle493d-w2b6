#include <Wire.h>
#include <Arduino.h>
#include "interface.h"
 



void initI2C(void) {
  Wire.begin();
  Wire.setClock(II2C_BaudRate);

}


uint8_t I2CWrite( uint8_t addr, uint8_t wData, uint8_t wLength){
  int i;
  Wire.beginTransmission(addr);
  for (i=0;i< wLength;i++){
    Wire.write((wData+i)); 
  }
  Wire.endTransmission();  
  
}


uint8_t I2cRead ( uint8_t i2cAddr, uint8_t * data, uint8_t len){

  uint8_t receivedBytes,i;
  receivedBytes= Wire.requestFrom(i2cAddr, len);    // request 2 bytes from slave device #112

  if (receivedBytes == len)
  {
    for (i = 0; i < len; i++)
    {
      *(data+i) = Wire.read();
    }
  }
  return receivedBytes;
} 


uint8_t i2cWriteArray ( uint8_t i2cAddr, uint8_t regAddr, uint8_t * data,uint8_t len ){
  uint8_t i;
  //return 
  //0:success
  //1:data too long to fit in transmit buffer
  //2:received NACK on transmit of address
  //3:received NACK on transmit of data
  //4:other error
  Wire.beginTransmission(i2cAddr);    // send sensor start bit + sensor addr                             
  Wire.write(0x20|regAddr);           // ADC trigger after Write frame is finished 
  for (i=0;i<len;i++) {
    Wire.write( *(data+i) ); 
  }
  return (Wire.endTransmission());    // stop transmitting
  
}
