#ifndef __INTERFACE_H__
#define __INTERFACE_H__
#include <Arduino.h>

#define II2C_BaudRate     400000

void initI2C (void);
uint8_t I2CWrite( uint8_t addr, uint8_t wData, uint8_t wLength);
uint8_t I2cRead ( uint8_t i2cAddr, uint8_t * data, uint8_t len);
uint8_t i2cWriteArray ( uint8_t i2cAddr, uint8_t regAddr, uint8_t * data,uint8_t len );

#endif 
