/*
  HMC5883L Triple Axis Digital Compass. Simple Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/
//I2C device found at address 0x1E  
#include <Wire.h>
#include <HMC5883L.h>
#include "GY521.h"
GY521 sensor(0x68);

HMC5883L compass;
int loopcount = 0;
void setup()
{
  // set up for magnetometer
  Serial.begin(115200);

  // Initialize HMC5883L
  // Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  //trick motioncal 
  Serial.println(F("Sensor Lab - IMU Calibration!"));

  Serial.println("Looking for a magnetometer");
  Serial.println("Looking for a gyroscope");
  Serial.println(F("Could not find a gyroscope, skipping!"));

  Serial.println("Looking for a accelerometer");
  Serial.println(F("Could not find a accelerometer, skipping!"));


  
  // Set measurement range
  // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA
  // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
  // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
  // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
  // +/- 4.00 Ga: HMC5883L_RANGE_4GA
  // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
  // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
  // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  // Idle mode:              HMC5883L_IDLE
  // Single-Measurement:     HMC5883L_SINGLE
  // Continuous-Measurement: HMC5883L_CONTINOUS (default)
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
 
  // Set data rate
  //  0.75Hz: HMC5883L_DATARATE_0_75HZ
  //  1.50Hz: HMC5883L_DATARATE_1_5HZ
  //  3.00Hz: HMC5883L_DATARATE_3HZ
  //  7.50Hz: HMC5883L_DATARATE_7_50HZ
  // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
  // 30.00Hz: HMC5883L_DATARATE_30HZ
  // 75.00Hz: HMC5883L_DATARATE_75HZ
  compass.setDataRate(HMC5883L_DATARATE_15HZ);

  // Set number of samples averaged
  // 1 sample:  HMC5883L_SAMPLES_1 (default)
  // 2 samples: HMC5883L_SAMPLES_2
  // 4 samples: HMC5883L_SAMPLES_4
  // 8 samples: HMC5883L_SAMPLES_8
  compass.setSamples(HMC5883L_SAMPLES_1);
  // set offset for HMC5883L

  //setup for MPU6050
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s
  sensor.setThrottle(false);
  //  set calibration value for MPU6050
  sensor.axe = -0.040511226;
  sensor.aye = 0.009712647;
  sensor.aze = -0.043611330;
  sensor.gxe = 3.820236682;
  sensor.gye = 0.281961870;
  sensor.gze = 1.053496241;


}



void loop()
{
  Vector raw = compass.readRaw();
  Vector norm = compass.readNormalize(); // after adding offset
  sensor.read_with_KF();
  Serial.print("Raw:");
  //Serial.print(int(sensor.getAccelX()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getAccelY()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getAccelZ()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getGyroX()*16)); Serial.print(","); // rate around axis x in [degree/second]
  //Serial.print(int(sensor.getGyroY()*16)); Serial.print(",");
  //Serial.print(int(sensor.getGyroZ()*16)); Serial.print(",");// this yaw angle only use infor from gyro, not any filter
  
  
  Serial.print(int(0)); Serial.print(",");
  Serial.print(int(0)); Serial.print(",");
  Serial.print(int(0)); Serial.print(",");
  Serial.print(int(0)); Serial.print(","); // rate around axis x in [degree/second]
  Serial.print(int(0)); Serial.print(",");
  Serial.print(int(0)); Serial.print(",");
  Serial.print(int(norm.XAxis*10)); Serial.print(",");
  Serial.print(int(norm.YAxis*10)); Serial.print(",");
  Serial.print(int(norm.ZAxis*10)); Serial.println("");
  loopcount += 1;
  if (loopcount == 50 || loopcount > 100) {
    Serial.print("Cal1:0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0");Serial.println();
    loopcount = loopcount + 1;
  }
  if (loopcount >= 100) {
    Serial.print("Cal2:0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0");Serial.println();
    loopcount = 0;
  }
  // delay(5);

}
