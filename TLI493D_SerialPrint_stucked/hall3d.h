#ifndef __HALL3D_H__
#define __HALL3D_H__
#include <Arduino.h>


#define SENSOR_IIC_ADDR   TLI493D_A1
    
//------------------------------------------------
// Main Function & Struct 
//--------------------------------------------------

typedef struct _sensorData {   
    int16_t BX;
    int16_t BY;
    int16_t BZ;
    int16_t TEMP;
} sensorData;


void initSensor ( void );
void sensorUpdata(void);
int16_t getX(void);
int16_t getY(void);
int16_t getZ(void);
int16_t getTemp(void);
float getAngle(void);


//------------------------------------------------
//TLI493D W2BW i2C slave device address(7bit format)
//--------------------------------------------------


#define TLI493D_A0  0x35 //7 bit address format
#define TLI493D_A1  0x22 //7 bit address format
#define TLI493D_A2  0x78 //7 bit address format
#define TLI493D_A3  0x44 //7 bit address format


//------------------------------------------------
//TLI493D W2BW Power On/Off Control pin number
//--------------------------------------------------

//
#define SensorPowerCtrlPin  5
#define INT_PIN        9

//------------------------------------------------
// Option. 몇번째 레지스터까지 읽을래?
// 6 ==> X, Y, Z, Temp           7 ==> X, Y, Z, Temp and Diag
//--------------------------------------------------

#define NumOfRdata   6 
#define numOfReg 23 //0x00~0x16

//------------------------------------------------
//TLI493D reg inital setting 
//------------------------------------------------

#define DT      0            //0 -> enable Temp(defualt), 1->  disable temp

#define AM      0            //0 -> measure Bx,y,z(Default)  , 1->  measure Bx,y only

#define TRIG    2            //0 -> no ADC trig @ one-byte read mode (default) 
                             //1 -> ADC trig before reading First MSB @ one-byte read mode 
                             //2 -> ADC trig after reading reg 0x05(temp+bz) @ one-byte read mode 

#define X2_sens    0
//#define X4_sens    0        //'00'-->[X2_sens:X4_sens]--> full range
                            //'10'-->[X2_sens:X4_sens]--> short range
                            //'11'-->[X2_sens:X4_sens]--> extra short range

#define TL_mag  1            // 0-> TC0 ( no temp compensation )
                             // 1-> TC2 ( temp compensation with TC1)
                             // 2-> TC3 ( temp compensation with TC2)
                             // 3-> TC4 ( temp compensation with TC3)


#define IICadr  1
                             // 0-> IIC addr 0x35 (equvalant  product A0 type) (default)
                             // 1-> IIC addr 0x22 (equvalant  product A1 type)
                             // 2-> IIC addr 0x78 (equvalant  product A2 type)
                             // 3-> IIC addr 0x44 (equvalant  product A3 type)
                          

#define PR  1                // 0-> 2byte read mode (default)-->  not allowed in this SW
                             // 1-> 1byte read mode 

#define CA_INT   1           // 0-> INT endable but no interrupt during I2C communication (CA active) 
                             // 1-> INT diable & Clock stetching enabled
                             // 2-> INT enable but there are possubilty inttrupt during i2c(CA not active )
                             // 3-> INT disble & Clock stetching disable (not recommended)


#define MODE   1             // 0-> LP mode         (defalut)
                             // 1-> Master mode 
                             // 2-> not allowed 
                             // 3-> fast mode 
                             
#define PRD    0            // 0->fupdate ≈ 770 Hz  (default)                          
                            // 1->fupdate ≈ 97 Hz
                            // 2->fupdate ≈ 24 Hz
                            // 3->fupdate ≈ 12 Hz
                            // 4->fupdate ≈ 6 Hz
                            // 5->fupdate ≈ 3 Hz
                            // 6->fupdate ≈ 0.4 Hz
                            // 7->fupdate ≈ 0.05 Hz



//----------------------------
// Read sensor data struct
//----------------------------


typedef struct _sensorConfigRegs {
  
    uint8_t  Dt;
    uint8_t  Am;     
    uint8_t  Trig; 
    uint8_t  x2_sens;
//    uint8_t  x4_sens;
    uint8_t  Tl_mag;   
    uint8_t  iiCadr;
    uint8_t  Pr;    
    uint8_t  Ca_int;    
    uint8_t  Mode;               
    uint8_t  Prd;  

    //============================
    uint8_t  WA;    //Read only
    uint8_t  WU;    
    uint8_t  TST;
    uint8_t  PH;    
    
    uint16_t XW;
    uint16_t YW;
    uint16_t ZW;
    
} sensorConfigRegs;



#endif 
