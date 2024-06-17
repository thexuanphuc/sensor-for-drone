/*
  HMC5883L Triple Axis Digital Compass. Simple Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/
//I2C device found at address 0x1E  
/*Arduino Uno Pin Layout
----------------------
    (Sensor) - (Arduino)
    VIN - +3V3
    GND - GND
    SCL - A5
    SDA - A4

*/
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
int loopcount = 0;

float matrix_calib[3][3] = {
    { 1.11840817, -0.04173206 , 0.01159853},
    {-0.04173206,  0.98800924, -0.09228303},
    {0.01159853, -0.09228303 , 1.27198654}
};

// float matrix_calib[3][3] = {
//     {1, 0, 0},
//     {0, 1, 0},
//     {0, 0, 1}
// };
void setup()
{
  // set up for magnetometer
  Serial.begin(115200);
  // Serial.println("BEGIN");
  // Initialize HMC5883L
  // Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    // Serial.println("BEGIN1");

    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Serial.println("BEGIN1");

   
  // Set measurement range
  // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA(MAY BE THIS IS CURRENTLY USING!!!!)
  // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
  // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
  // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
  // +/- 4.00 Ga: HMC5883L_RANGE_4GA
  // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
  // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
  // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
  compass.setRange(HMC5883L_RANGE_0_88GA);
  // Serial.println("BEGIN2");

  // Set measurement mode
  // Idle mode:              HMC5883L_IDLE
  // Single-Measurement:     HMC5883L_SINGLE
  // Continuous-Measurement: HMC5883L_CONTINOUS (default)
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Serial.println("BEGIN3");

  // Set data rate
  //  0.75Hz: HMC5883L_DATARATE_0_75HZ
  //  1.50Hz: HMC5883L_DATARATE_1_5HZ
  //  3.00Hz: HMC5883L_DATARATE_3HZ
  //  7.50Hz: HMC5883L_DATARATE_7_50HZ
  // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
  // 30.00Hz: HMC5883L_DATARATE_30HZ
  // 75.00Hz: HMC5883L_DATARATE_75HZ
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  // Serial.println("BEGIN4");

  // Set number of samples averaged
  // 1 sample:  HMC5883L_SAMPLES_1 (default)
  // 2 samples: HMC5883L_SAMPLES_2
  // 4 samples: HMC5883L_SAMPLES_4
  // 8 samples: HMC5883L_SAMPLES_8
  compass.setSamples(HMC5883L_SAMPLES_2);
  // set offset for HMC5883L
  compass.setOffset(0,0,0);
  // Serial.println("END SETUP");

  // compass.setOffset();

}

void matrix_multiplication(float matrix[3][3], float measured[3], float result[3]){
  
  for (int i = 0; i< 3; i++){
    result[i]=0;
    for(int j = 0; j< 3; j++){
      result[i] += matrix[i][j] * measured[j];
    }
  }
}

void loop()
{
  Vector raw = compass.readRaw();//after adding offset 
  Vector norm = compass.readNormalize(); // after adding offset + normalize by scale sensitivity
  float measured[3];
  measured[0] = norm.XAxis ;
  measured[1] = norm.YAxis ;
  measured[2] = norm.ZAxis ;

    //Serial.print(int(sensor.getAccelX()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getAccelY()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getAccelZ()*8192)); Serial.print(",");
  //Serial.print(int(sensor.getGyroX()*16)); Serial.print(","); // rate around axis x in [degree/second]
  //Serial.print(int(sensor.getGyroY()*16)); Serial.print(",");
  //Serial.print(int(sensor.getGyroZ()*16)); Serial.print(",");// this yaw angle only use infor from gyro, not any filter
  
  
  Serial.print(measured[0],6); Serial.print(",");
  Serial.print(measured[1],6); Serial.print(",");
  Serial.print(measured[2],6); Serial.println();  
  // Serial.println(float(atan2(measured[1],measured[0])*180/3.14 ),1); 


  // float result[3];
  // matrix_multiplication(matrix_calib,measured,result);
  // // Serial.print("X is heading ");  
  // // Serial.print(float(atan2(measured[1],measured[0])*180/3.14 ),1); 
  // // Serial.println("from north(left +; right -)");  


  // // Serial.println(float(atan2(measured[1],measured[0])*180/3.14 ),1); 
  
  delay(100);

}
