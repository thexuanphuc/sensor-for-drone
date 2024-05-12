//
//    FILE: GY521_test_main.ino
// PURPOSE: output raw and compare complementary filter and Kalman filter.


#include "GY521.h"

GY521 sensor(0x68);
uint32_t counter = 0;
float last_micro;
float pre_roll_k = 0;
float pre_pitch_k = 0;
float rate_filted = 0;

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

  // sensor.setThrottle();
  // Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = -0.008422607;
  sensor.aye = 0.013572266;
  sensor.aze = 0.063187007;
  sensor.gxe = 2.481618404;
  sensor.gye = -1.788206100;
  sensor.gze = 1.447893142;
  last_micro = 0.0;
}

void loop() {
  // this part to find the duration between sampling
  // float now = micros();
  // float duration = (now - last_micro) * 1e-6;  //  duration in seconds.
  // last_micro = now;
  // Serial.println();
  // Serial.println(duration, 10);
  sensor.read_with_KF();
  float ax = sensor.getAccelX();
  float ay = sensor.getAccelY();
  float az = sensor.getAccelZ();
  float gx = sensor.getGyroX();
  float gy = sensor.getGyroY();
  float gz = sensor.getGyroZ();

    //raw acceleration in [g]
  Serial.print(float(ax*10), 4);
  Serial.print(',');
  Serial.print(float(ay*10), 4);
  Serial.print(',');
  Serial.print(float(az*10), 4);
  Serial.print(',');
  // raw data from gyroscope [rad / s]
  Serial.print(float(gx), 8);
  Serial.print(',');
  Serial.print(float(gy), 8);
  Serial.print(',');
  Serial.println(float(gz), 8);
  // Serial.println(',');

  delay(11);
}


//  -- END OF FILE --
