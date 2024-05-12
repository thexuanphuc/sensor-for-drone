// this file just get raw value for calibration
//I2C device found at address 0x1E  
/*Arduino Uno Pin Layout
----------------------
    (Sensor) - (Arduino)
    VIN - +3V3
    GND - GND
    SCL - A5
    SDA - A4
r
*/
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
int loopcount = 0;

void setup()
{

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);


  // // Enable bypass mode
  // mpu.setI2CMasterModeEnabled(false);
  // mpu.setI2CBypassEnabled(true);
  // mpu.setSleepEnabled(false);
  // set up for magnetometer
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
   
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

  compass.setOffset(0,0,0);

}

float duration;
float now;
float last_micro = 0;
void loop()
{
  Vector raw = compass.readRaw();//after adding offset 
  Vector norm = compass.readNormalize(); // after adding offset + normalize by scale sensitivity
  float measured[3];
  measured[0] = norm.XAxis ;
  measured[1] = norm.YAxis ;
  measured[2] = norm.ZAxis ;
  Serial.print(measured[0],8); Serial.print(",");
  Serial.print(measured[1],8); Serial.print(",");
  Serial.println(measured[2],8); 

  // // this part to find the duration between sampling
  // float now = micros();
  // float duration = (now - last_micro) * 1e-6;  //  duration in seconds.
  // last_micro = now;
  // Serial.println();
  // Serial.println(duration, 10);
  delay(15);

}
