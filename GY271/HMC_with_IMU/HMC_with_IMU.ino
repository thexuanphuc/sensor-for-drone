#include <Wire.h>
#include <HMC5883L.h>
#include "GY521.h"
HMC5883L compass;

GY521 sensor(0x68);
uint32_t counter = 0;
float last_micro;
float pre_roll_k = 0;
float pre_pitch_k = 0;
float rate_filted = 0;
const float MAGNETO_COV = 1.3225;
const float GYROS_RATE_COV = 0.4;// static covariance is 0.04
const float GYROS_MEAN_COV = 0.01;// static covariance is 0.0001
const float BIAS_GYROS = -0.007;
// const float MAGNETO_COV = 1.2769;

float yaw_magnetic_init = 0;
float _gain_k = 0;
float matrix_calib[3][3] =  {
                            {1.1180947 , 0.00760405, 0.03886638},
                            {0.00760405, 1.12722608, 0.0049638},
                            {0.03886638, 0.0049638 , 1.11248237}
};
float offset[3] = {-4.98677796 , 2.38519419, 14.97449162};
// float matrix_calib[3][3] = {
//     {1, 0, 0},
//     {0, 1, 0},
//     {0, 0, 1}
// };
float state_K[3] = {0,BIAS_GYROS, 0} ; // initial state assumed =0;
float A[3][3] = {
                {0,  1 , 0},
                {0,  1, 0},
                {0, 0 , 1}  // need to update dt each time call A
};
float B[3] = {1,0,0};
float P[3][3] = {
                {1,  0 , 0},
                {0,  1, 0},
                {0, 0 , 1}};

float DRAFT[3][3] = {
                    {0,  0, 0},
                    {0,  0, 0},
                    {0,  0, 0}};

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println();
  Wire.begin();

  delay(100);
  while (sensor.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(1);   //  250 degrees/s

  sensor.setThrottle();
  Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = -0.008422607;
  sensor.aye = 0.013572266;
  sensor.aze = 0.063187007;
  sensor.gxe = 2.481618404;
  sensor.gye = -1.788206100;
  sensor.gze = 1.447893142;
  last_micro = 0.0;
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  compass.setRange(HMC5883L_RANGE_0_88GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_2);
  compass.setOffset(0,0,0);

  
  // find initial angle
  for(int n = 0; n < 300; n++){

    Vector norm = compass.readNormalize(); // after adding offset (0,0,0)+ normalize by scale sensitivity
    float measured_ini[3];
    measured_ini[0] = norm.XAxis - offset[0];
    measured_ini[1] = norm.YAxis - offset[1];
    measured_ini[2] = norm.ZAxis - offset[2];
    float result_ini[3];
    multi_AxX(matrix_calib,measured_ini,result_ini);
    float yaw_magnetic = atan2(result_ini[1],result_ini[0])*180/3.1415926;
    yaw_magnetic_init = (yaw_magnetic + yaw_magnetic_init) ;
  }
  yaw_magnetic_init = yaw_magnetic_init / 300;
  Serial.println(yaw_magnetic_init);
}

void multi_AxX(const float (*_A)[3], const float* _X, float* _result){
  for (int i = 0; i< 3; i++){
    _result[i] = 0;
    for(int j = 0; j< 3; j++){
      _result[i] += _A[i][j] * _X[j];
    }
  }
}
void multi_AxB(const float (*_A)[3],const float (*_B)[3],float (*_result)[3]){
  for (int i = 0; i< 3; i++){
    for (int j = 0; j< 3; j++){
      _result[i][j] = _A[i][0]*_B[0][j] +  _A[i][1]*_B[1][j] +  _A[i][2]*_B[2][j];
    }
  }
}
void multi_AxBxA(const float (*_A)[3],const float (*_B)[3],float (*_result)[3]){
  float result1[3][3];
  for (int i = 0; i< 3; i++){
    for (int j = 0; j< 3; j++){
      result1[i][j] = _A[i][0]*_B[0][j] +  _A[i][1]*_B[1][j] +  _A[i][2]*_B[2][j];
    }
  }
  for (int i = 0; i< 3; i++){
    for (int j = 0; j< 3; j++){
      _result[i][j] = result1[i][0]*_A[j][0] +  result1[i][1]*_A[j][1] +  result1[i][2]*_A[j][2];
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

// state = [omega, rate_mean, yaw_angle]
void _3dkalman(float* Xk , float (*Pk)[3],float duration, float rate, float measured_angle){
  float P_tem[3][3] = {
                    {0,  0, 0},
                    {0,  0, 0},
                    {0,  0, 0}};
  A[2][0] = duration;
  // prediction step
  // Ax+Bu
  Xk[2] = duration*Xk[0] + Xk[2];
  if(Xk[2] >= 360){
    Xk[2] = Xk[2] - 360;
  } else if(Xk[2] <= -360){
    Xk[2] = Xk[2] + 360;
  }
  Xk[0] = Xk[1] + rate;

  // multi_AxX(A, Xk,  )
  // update matrix uncertainty 
  // Serial.println(A[2][1]);  

  multi_AxBxA(A,Pk,P_tem);

  P_tem[0][0] += GYROS_RATE_COV;
  P_tem[1][1] += GYROS_MEAN_COV;

  // measurement step
  float delta_y = measured_angle - Xk[2];
  float S = 1/(P_tem[2][2] + MAGNETO_COV);

  float Kk[3] = {0, 0, 0};
  for (int i = 0; i< 3; i++){
    Kk[i] = P_tem[i][2] * S;
    Xk[i] = Xk[i] + Kk[i]*delta_y; 
  }

  // update error cov matrix
  float IKC[3][3] = {
    {1,  0, -Kk[0]},
    {0,  1, -Kk[1]},
    {0,  0, 1-Kk[2]}};
  multi_AxB(IKC, P_tem, Pk);

}


void loop() {
  Vector norm = compass.readNormalize(); // after adding offset (0,0,0)+ normalize by scale sensitivity
  float measured[3];
  measured[0] = norm.XAxis - offset[0];
  measured[1] = norm.YAxis - offset[1];
  measured[2] = norm.ZAxis - offset[2];
  // Serial.print(float(x/SENSITIVITY_mT), 6);
  // Serial.print(",");
  // Serial.print(float(y/SENSITIVITY_mT), 6);
  // Serial.print(",");
  // Serial.print(float(z/SENSITIVITY_mT), 6);
  // Serial.println();


  float result[3];
  multi_AxX(matrix_calib,measured,result);
  // Serial.print("X is heading ");  
  // Serial.print(float(atan2(result[1],result[0])*180/3.14 ),1); 
  // Serial.println("from north(left +; right -)");  
  // Serial.println(float(atan2(result[1],result[0])*180/3.14 ),1); 
  float yaw_magnetic = atan2(result[1],result[0])*180/3.1415926;


          // Serial.print(offset[0]);Serial.print(",");
          // Serial.print(offset[1]);Serial.print(",");
          // Serial.println(offset[2]);
  sensor.read_with_KF();
  float dt = sensor.get_duration();
  float yaw_rate = sensor.get_gyro_rate_z();

  _3dkalman(state_K , P, dt, yaw_rate, yaw_magnetic-yaw_magnetic_init );
  // for (int i = 0; i< 3; i++){
  //   for (int j = 0; j< 3; j++){
  //     P[i][j] = DRAFT[i][j];
  //   }
  // }
  
  // Serial.println(dt ,6); // -> dt = 0.020852
  Serial.print(yaw_magnetic-yaw_magnetic_init);
  Serial.print(",");
  Serial.println(state_K[2]);

  delay(14);
}
