//
//    FILE: GY521_test_main.ino
// PURPOSE: output raw and compare complementary filter and Kalman filter.


#include "GY521.h"

GY521 sensor(0x68);

uint32_t counter = 0;
float last_micro;
float pre_roll_k = 0;
float pre_pitch_k = 0;
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  // Serial.print("GY521_LIB_VERSION: ");
  Serial.println(GY521_LIB_VERSION);

  Wire.begin();

  delay(100);
  while (sensor.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s

  sensor.setThrottle();
  Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = -0.040511226;
  sensor.aye = 0.009712647;
  sensor.aze = -0.043611330;
  sensor.gxe = 3.820236682;
  sensor.gye = 0.281961870;
  sensor.gze = 1.053496241;
  last_micro = 0.0;


}


void loop() {
  sensor.read_with_KF();
  float ax = sensor.getAccelX();
  float ay = sensor.getAccelY();
  float az = sensor.getAccelZ();
  float gx = sensor.getGyroX();
  float gy = sensor.getGyroY();
  float gz = sensor.getGyroZ();


  //using complementary filter
  float pitch = sensor.getPitch();
  float roll  = sensor.getRoll();

  // calculate only with accelerator
  float x = sensor.getAngleX();
  float y = sensor.getAngleY();

  // angle if use kalman filter
  float pitch_k = sensor.get_pitch_kalman();
  float roll_k = sensor.get_roll_kalman();
  // angle without any filter from accelerator
  float roll_no_filter = sensor.getAngleX();
  float pitch_no_filter = sensor.getAngleY();

  // this part to find the duration between sampling
  // float now = micros();
  // float duration = (now - last_micro) * 1e-6;  //  duration in seconds.
  // last_micro = now;
  // Serial.println();
  // Serial.println(duration, 10);

  //raw acceleration in [g]
  Serial.print(ax, 3);
  Serial.print(',');
  Serial.print(ay, 3);
  Serial.print(',');
  Serial.print(az, 3);
  Serial.print(',');
  // raw data from gyroscope [deg / s]
  Serial.print(gx, 3);
  Serial.print(',');
  Serial.print(gy, 3);
  Serial.print(',');
  Serial.print(gz, 3);
  Serial.print(',');

  // after using Kalman filter
  Serial.print(roll_k, 3); //roll Kalman filter
  Serial.print(',');  
  // Serial.print(roll, 3); //roll complementary filter
  // Serial.print(',');  
  Serial.print(roll_no_filter);  // no filter
  Serial.print(',');
  Serial.print(pitch_k  , 3); //pitch Kalman filter
  // Serial.print(pitch, 3); //pitch complementary filter
  // Serial.print(',');  
  Serial.print(',');  
  Serial.println(pitch_no_filter); // pitch no filter

  delay(10);
}


//  -- END OF FILE --
