// i dont know why but the compensation formula of BMP280 in 32 bit fixed point doesnt work, 
// -> use double precision floating point instead 
/*
VCC - 3.3V
GND - GND
SDA - A4
SCL - A5

*/
#include <Wire.h>
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;
double pressure, T,ppress_msb,ppress_lsb, ppress_xlsb, padc_T,padc_P ;
void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  // get raw data from sensor
  Wire.requestFrom(0x76,6);
  // read from memory map 4.2 i n datasheet
  uint32_t press_msb = Wire.read();
  ppress_msb = press_msb;
  uint32_t press_lsb = Wire.read();
  ppress_lsb = press_lsb;
  uint32_t press_xlsb = Wire.read();
  ppress_xlsb = press_xlsb;
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  // get raw pressure and temperature 4.3.6
  uint32_t adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4) ;
  padc_P = adc_P;
  uint32_t adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  padc_T = adc_T;
  // fine resolustion temperature, copy from datasheet page 46, using 32 bit 
  // int32_t var1, var2;
  // int32_t t_fine;
  // var1 = ((((adc_T >> 3) - ((int32_t )dig_T1 <<1)))* ((int32_t )dig_T2)) >> 11;
  // var2 = (((((adc_T >> 4) - ((int32_t )dig_T1)) * ((adc_T>>4) - ((int32_t )dig_T1)))>> 12) * ((int32_t )dig_T3)) >> 14;
  // int32_t t_fine = var1 + var2;
  // tempe = ((t_fine * 5 + 128) >> 8) / 100;
  // calibrated pressure, copy from datasheet page 46 
  // uint32_t p;
  // var1 = (((int32_t )t_fine)>>1) - (int32_t )64000;
  // var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t )dig_P6);
  // var2 = var2 + ((var1*((int32_t )dig_P5)) <<1);
  // var2 = (var2>>2)+(((int32_t )dig_P4)<<16);
  // var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((int32_t )dig_P2) * var1)>>1))>>18;
  // var1 = ((((32768+var1))*((int32_t )dig_P1)) >>15);
  // if (var1 == 0) {
  //   p=0; // avoid exception caused by division by zero
  // }    
  // p = (((uint32_t )(((int32_t ))-adc_P)-(var2>>12)))*3125;
  // if(p<0x80000000){ 
  //   p = (p << 1) / ((uint32_t ) var1);
  // } else {
  //   p = (p / (uint32_t )var1) * 2;  
  // }
  // var1 = (((int32_t )dig_P9) * ((int32_t ) (((p>>3) * (p>>3))>>13)))>>12;
  // var2 = (((int32_t )(p>>2)) * ((int32_t )dig_P8))>>13;
  // p = (uint32_t)((int32_t )p + ((var1 + var2+ dig_P7) >> 4));
  //calibrate temperature using double
  double var1, var2;
  int32_t t_fine;
  var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
  var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
  (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
  t_fine = (int32_t)(var1 + var2);
  T = (var1 + var2) / 5120.0;
  //calibrate pressure
  double p;
  var1 = ((double)t_fine/2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
  var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
  if (var1 == 0.0)
  {
  return 0; // avoid exception caused by division by zero
  }
  p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
  // calculate altitude:
  pressure=(double)p/100; // presure in hPa
  AltitudeBarometer=44330*(1-pow(pressure /1013.25, 1/5.255))*100;
}
void setup() {
  Serial.begin(115200);
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);
  // Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x76);
  // config resolution for indoor navigation(x2 over sampling teperature+ x16 over sampling pressure +normal mode)
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  //config  set IIR filter coefficient to x16
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   
  uint8_t data[24], i=0; 
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);      
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  // get calibration data for sensor(manifacturer set these parameter) 3.11.2
  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22]; 
  delay(250);
  AltitudeBarometerStartUp = 0;
  for (RateCalibrationNumber=0; RateCalibrationNumber<200; RateCalibrationNumber ++) {
    barometer_signals();
    AltitudeBarometerStartUp+=AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp/=200;
}
void loop() {
  barometer_signals();
  Serial.print("initial altitude in cm: ");
  Serial.println(AltitudeBarometerStartUp);
  AltitudeBarometer-=AltitudeBarometerStartUp;
  Serial.print("changed Altitude [cm]: ");
  Serial.println(AltitudeBarometer);
  delay(200);
}