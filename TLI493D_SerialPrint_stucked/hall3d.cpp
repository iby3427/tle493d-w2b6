#include "hall3d.h"
#include "interface.h"
#include <Arduino.h>
//#include <Wire.h>

//---------------------------------------
//Internal function
//----------------------------------------
//extern sensorConfigRegs sensorConfig;
void  SimpleAssembleSensorData( sensorData * mag  , uint8_t * sDATA );
void EnableOneByteReadOnly (void);
void PowerOn();
void WriteMirrorToSensorReg (void);
void sensorMirrorRegInit (void);
uint8_t  CalParity( uint8_t * regData, uint8_t numByte);
void SensorRead(sensorData * mag, uint8_t numByte);

//====================
// Glrobal variable  
//=====================
sensorConfigRegs sensorConfig;
sensorData mag;
volatile uint16_t  sensorUpdated =0;
uint8_t senReg [numOfReg];
void magFieldReady_ISR (void);


void sensorMirrorRegInit (void){
      sensorConfig.Dt=DT;
      sensorConfig.Am= AM;     
      sensorConfig.Trig= TRIG; 
      sensorConfig.x2_sens= X2_sens;
//      sensorConfig.x4_sens= X4_sens;
      sensorConfig.Tl_mag= TL_mag;   
      sensorConfig.iiCadr= IICadr;
      sensorConfig.Pr= PR;    
      sensorConfig.Ca_int= CA_INT;    
      sensorConfig.Mode= MODE;               
      sensorConfig.Prd= PRD  ;
}




#define BX_R      0x00
#define BY_R      0x01
#define BZ_R      0x02

#define TEMP_R    0x03

#define BX2_R     0x04
#define TEMP2_R   0x05
#define DIAG_R    0x06      //Sensor diagnostic and status register
#define XL_R      0x07
#define YL_R      0x09
#define ZL_R      0x0B
#define XH_R      0x08
#define YH_R      0x0B
#define ZH_R      0x0C
#define WU_R      0x0D
#define TMODE_R   0x0E
#define TPHASE_R  0x0F
#define CONFIG_R  0x10      //Configuration register
#define MOD1_R    0x11      //Power mode, interrupt, address, parity
#define RSRV1_R   0x12      //reserved
#define MOD2_R    0x13      //Low Power Mode update rate
#define RSRV2_R   0x14      //reserved
#define RSRV3_R   0x15      //reserved
#define VER_R     0x16      //Version

//========================================
// define bit positon of sensor reg 
//========================================

#define DT_bs          7
#define AM_bs          6
#define TRIG_bs        4
#define X2_sens_bs     3
//#define X4_sens_bs     0
#define TL_mag_bs      1

#define IICadr_bs   5
#define PR_bs       4
#define CA_INT_bs   2
#define MODE_bs     0
#define PRD_bs      5


//
//void EnableOneByteReadOnly (void){
//    Wire.beginTransmission(SENSOR_IIC_ADDR);
//    Wire.write(0x11);             // Config, 0x10
//    Wire.write(0b10011000);       // MOD1 register,  master controlled mode, one byte read command, enabled interrupt, no collision avoidance, odd FP parity
//    Wire.endTransmission();
//}


void EnableOneByteReadOnly (void){

  //======================================
  // Set REG to Known status without Fuse parity error for basic interface
  // It is only for read interface
  // It make One byte read mode , MASTER,Disable interrupt, Enable Clock steching 
  // after this, you should reconfig sensorConfig res which will be applied to your application
  // FP should not be corrupted , no update on FP
  //=======================================
 
  //One byte, MASTER,Disable interrupt, Enable Clock steching ,PRD=0x00 with correct FP
   uint8_t MOD1_PRD[3];
    MOD1_PRD[0]=0x15;
    MOD1_PRD[1]=0x38;
    MOD1_PRD[2]=0x00;
  (void) i2cWriteArray ( SENSOR_IIC_ADDR, MOD1_R , MOD1_PRD,3 );
  
}



void  SimpleAssembleSensorData( sensorData * mag  , uint8_t * sDATA ){

  int16_t bx,by,bz,temp;
  uint8_t diag,ubyte, parity;
  sensorData HallData;
  mag->BX=(int8_t) *(sDATA+0);
  mag->BY=(int8_t) *(sDATA+1);
  mag->BZ=(int8_t) *(sDATA+2);
  mag->TEMP=(int8_t) *(sDATA+3);
  ubyte=*(sDATA+4);
  
  //assemble bx into int16
  mag->BX=(mag->BX)<<4;
  mag->BX=(mag->BX)|(ubyte>>4);

  //assemble by into int16
  mag->BY=(mag->BY)<<4;
  mag->BY=(mag->BY)|(0x0F&ubyte);

  //assemble bz into int16
  ubyte=*(sDATA+5);
  mag->BZ=(mag->BZ)<<4;
  mag->BZ=(mag->BZ)|(0x0F&ubyte);
  

  //assemble temp into int16
  mag->TEMP=(mag->TEMP)<<4;
  mag->TEMP=(mag->TEMP)|( 0x0C&(ubyte>>4) );
  
   //DIAG byte
  //mag->TEMP=*(sDATA+6);
}







//----------------------------------------------------
// if ODD parity --> return 1
//----------------------------------------------------
uint8_t  CalParity( uint8_t * regData, uint8_t numByte){

  uint8_t i,parity;

  parity=*(regData+0);
  for (i=1;i<numByte;i++)    
    parity^=*(regData+i);


    
  parity ^= parity >> 4;
  parity ^= parity >> 2;
  parity ^= parity >> 1;
  return (parity&0x01); 
  
}


void SensorRead(sensorData * mag,uint8_t numByte ){
    uint8_t reg[7];
    (void)I2cRead(SENSOR_IIC_ADDR, reg, numByte);
    (void)I2cRead(SENSOR_IIC_ADDR, reg, numByte);
    SimpleAssembleSensorData(  mag  , reg ); 
}


void sensorUpdata(void){  
  SensorRead(&mag,7);
}

int16_t getX(void){
 
  return mag.BX; 
}

int16_t getY(void){

  return mag.BY; 
}

int16_t getZ(void){

  return mag.BZ; 
}

int16_t getTemp(void){
  return mag.TEMP; 
}




float getAngle(void){
 
  float angle ; 
  angle = atan2(mag.BX, mag.BY)*180/PI;
  return angle ; 
}


void PowerOn(void){
    pinMode(SensorPowerCtrlPin,OUTPUT);
    digitalWrite(SensorPowerCtrlPin,HIGH);
    delay(100);
    digitalWrite(SensorPowerCtrlPin,LOW);
    delay(50);
}



uint8_t updateCP (void){
  uint8_t cpReg[10] ;   //0x07~0x10

  //copy 
  for (int16_t i =0 ; i< 10;i++){
    cpReg[i]=senReg[0x07+i];
  }

  cpReg[6]= cpReg[6]&(~0x80);  //bit[7] Clear   in 0x0D reg
  cpReg[7]= cpReg[7]&(~0xC0);  //bit[7:6] Clear in 0x0E reg
  cpReg[8]= cpReg[8]&(~0xC0);  //bit[7:6] Clear in 0x0F reg
  cpReg[9]= cpReg[9]&(~0x01);  //bit[1] Clear in 0x10 reg

  //return CalParity 1  if odd 
  if( CalParity( cpReg, 10) ) {
      return 0x00;
  } else{
      return 0x01;
  } 

}


uint8_t updateFP (void){
  uint8_t cpReg[2] ;   //0x07~0x10

  //copy 
  cpReg[0]=senReg[MOD1_R];
  cpReg[1]=senReg[MOD2_R];
  
  //bitclear 
  cpReg[0]= cpReg[0]&(~0x80);  //bit[7] Clear in 0x11 reg
  cpReg[1]= cpReg[1]&(~0x1F);  //bit[7:5] Clear in 0x13 reg
  
  // CalParity() return 1  if odd 
  if( CalParity( cpReg, 2) ) {
      return 0x00;
  } else{
      return 0x80;
  } 

}




void initSensor ( void ) {
  
  //---------------------------
  // Power on
  //--------------------------
//  PowerOn();

  //---------------------------
  // Init I2C
  //--------------------------  
   initI2C();
  
  //======================================
  // enable One byte read interface
  //=======================================
  EnableOneByteReadOnly();
  
  //======================================
  // update sensosrMirrorReg as default value 
  //=======================================
  sensorMirrorRegInit();
  
  //======================================
  // Write sensorMirrorReg to Sensor's real Register with correct parity bit
  //=======================================
  WriteMirrorToSensorReg();
    //======================================
  // sensor measurement done ISR attach
  //=======================================
  //pinMode(INT_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INT_PIN),magFieldReady_ISR, FALLING);  
}







void WriteMirrorToSensorReg (void){

  //=================================================================
  //  Copy sensor Register to senReg for CP calculation & block write 
  //==================================================================
  (void) I2cRead(SENSOR_IIC_ADDR,senReg,numOfReg); 


  //=================================================================
  //update  Config & Mod1 & Mod2 & RSRV2 reg from sensorConfig value
  //=================================================================
 
  senReg[CONFIG_R]=((sensorConfig.Dt)<<DT_bs)|((sensorConfig.Am)<<AM_bs)|((sensorConfig.Trig)<<TRIG_bs)|((sensorConfig.x2_sens)<<X2_sens_bs)|((sensorConfig.Tl_mag)<<TL_mag_bs);
  senReg[MOD1_R]=((sensorConfig.iiCadr)<<IICadr_bs)|((sensorConfig.Pr)<<PR_bs)|((sensorConfig.Ca_int)<<CA_INT_bs)|((sensorConfig.Mode)<<MODE_bs);
  senReg[MOD2_R]=(sensorConfig.Prd)<<PRD_bs;
  //senReg[RSRV2_R]=(sensorConfig.x4_sens)<<X4_sens_bs;


  //===================================================
  //  CP,FP update 
  //===================================================
   senReg[CONFIG_R]= (senReg[CONFIG_R]&(~0x01))|updateCP();
   senReg[MOD1_R]= (senReg[MOD1_R]&(~0x80))|updateFP();


  //========================================================================
  // Write to sensor  form config(0x10) upto Mod2(0x13) , RSRV2_R register 
  //========================================================================
  (void) i2cWriteArray ( TLI493D_A1, CONFIG_R, & (senReg[CONFIG_R]),4);
  (void) I2CWrite ( TLI493D_A1, RSRV2_R, senReg[RSRV2_R]);


}

//
//void magFieldReady_ISR (void){
//
// 
//  sensorUpdated=1;
//  if (sensorConfig.Mode!=1){  // if opreation mode is not MASTER mode)
//
//      SensorRead(&mag,NumOfRdata); 
//
//  }
// 
//
//}
