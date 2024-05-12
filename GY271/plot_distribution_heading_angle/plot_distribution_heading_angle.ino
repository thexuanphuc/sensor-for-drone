    // // Transmit over serial
    // Serial.print(MagEvent.magnetic.x, 6); Serial.print(",");
    // Serial.print(MagEvent.magnetic.y, 6); Serial.print(",");
    // Serial.println(MagEvent.magnetic.z, 6);


#include <Wire.h>
#include <MechaQMC5883.h>
#include "GY521.h"
MechaQMC5883 qmc;

const float SENSITIVITY_GAUSS = 12000.0f; // for range 2G, then value[gauss] =  LSB / 12000
const float SENSITIVITY_mT = 120.0f; // for range 2G, then value[micro Tesla] =  100* LSB / 12000 
// float matrix_calib[3][3] = {
//     {1.15187187,  0.00776285 , 0.00788897},
//     {0.00776285,  1.07379589, -0.12601771},
//     {0.00788897, -0.12601771 , 1.24860912}
// };
// float offset[3] = {7.67224465, 7.13783423, 9.08017193};
float matrix_calib[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float offset[3] = {0,0,0};

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println();
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_50Hz,RNG_2G,OSR_256);

}

void matrix_multiplication(float matrix[3][3], float measured[3], float result[3]){
  
  for (int i = 0; i< 3; i++){
    result[i]=0;
    for(int j = 0; j< 3; j++){
      result[i] += matrix[i][j] * measured[j];
    }
  }
}
// void low_pass_filter(float x_in[3], float y_out[3]) {  //Low Pass Butterworth Filter
//   y_out[0] = A[0] * x_in[0] + A[1] * x_in[1] + A[2] * x_in[2] - B[1] * y_out[1] - B[2] * y_out[2];
//   y_out[2] = y_out[1];
//   y_out[1] = y_out[0];
//   x_in[2] = x_in[1];
//   x_in[1] = x_in[0];
// }


void loop() {
  int x,y,z;

  qmc.read(&x,&y,&z);
  // float measured[3];
  // measured[0] = float(x/SENSITIVITY_mT) - offset[0];
  // measured[1] = float(y/SENSITIVITY_mT) - offset[1];
  // measured[2] = float(z/SENSITIVITY_mT) - offset[2];
  Serial.print(float(x/SENSITIVITY_mT), 8);
  Serial.print(",");
  Serial.print(float(y/SENSITIVITY_mT), 8);
  Serial.print(",");
  Serial.print(float(z/SENSITIVITY_mT), 8);
  Serial.println();

  delay(10);
}
