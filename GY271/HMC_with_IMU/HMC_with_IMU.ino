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
const float MAGNETO_COV = 1.32; // plot with python : 1.3225
const float GYROS_RATE_COV = 0.04;// static covariance is 0.04
const float GYROS_MEAN_COV = 0.0001;// static covariance is 0.0001
const float BIAS_GYROS = -0.0;
// const float MAGNETO_COV = 1.2769;

float yaw_magnetic_init = 0;
float _gain_k = 0;
float matrix_calib[3][3] =  {
                            {1.10205367,  0.01427127,  0.03926751},
                            {0.01427127,  1.08821656, -0.0132617},
                            {0.03926751, -0.0132617,   1.10191648}
};
float offset[3] = {-6.38618123,  6.93689073, 13.99404331};
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
                {1,  0, 0},
                {0,  1, 0},
                {0,  0, 1}};

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
  // Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = -0.050081543;
  sensor.aye = 0.007971925;
  sensor.aze = 0.061343741;
  sensor.gxe = 2.558282375;
  sensor.gye = -1.817236709;
  sensor.gze = 1.352961778;

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
  for(int n = 0; n < 1000; n++){

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
  yaw_magnetic_init = yaw_magnetic_init / 1000;
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

// vector representation
struct vector_3D {
private:
  static const int Num = 3;
  float vect[Num];
public:
  // vector_3D() : vect {} {}
  vector_3D(float x,float y, float z){
    vect[0] = x;
    vect[1] = y;
    vect[2] = z;
  }

  float dot(const vector_3D& Y){
    float _result = 0;
    for(int i = 0; i< Num; i++){
      _result = _result + vect[i]*Y.vect[i];
    }
    return _result;
  }
  vector_3D operator* (const float x) const{
    return vector_3D(vect[0]*x, vect[1]*x, vect[2]*x);
  }

  vector_3D operator- (const vector_3D& Y) const{
    return vector_3D(vect[0] - Y.vect[0], vect[1] - Y.vect[1], vect[2] - Y.vect[2]);
  }
  vector_3D operator+ (const vector_3D& Y) const{
    return vector_3D(vect[0] + Y.vect[0], vect[1] + Y.vect[1], vect[2] + Y.vect[2]);
  }
  float getX(){return vect[0];}
  float getY(){return vect[1];}
  float getZ(){return vect[2];}

};

// function to compute yaw angle with tilt compensation, 
float compute_yaw(float acc0, float acc1, float acc2, float mag0, float mag1, float mag2){
  vector_3D acce_vec(acc0,acc1,acc2);
  vector_3D magnet_vec(mag0,mag1,mag2);
  vector_3D vector_north = magnet_vec - (acce_vec*(magnet_vec.dot(acce_vec)/ acce_vec.dot(acce_vec)));
  
  float magnet_value_tilt[3];
  magnet_value_tilt[0] = vector_north.getX();
  magnet_value_tilt[1] = vector_north.getY();
  magnet_value_tilt[2] = vector_north.getZ();
  Serial.print("magnet part on Z axis is ");
  Serial.println(magnet_value_tilt[2]);
  float _result[3];
  multi_AxX(matrix_calib,magnet_value_tilt,_result);
  return atan2(_result[1], _result[0]) * 180/3.14159;
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
  // Serial.println(duration);
  // prediction step
  // Ax+Bu

  // Serial.print(rate);

  Xk[2] = duration*Xk[0] + Xk[2];
  Xk[0] = Xk[1] + rate; //this is the next step n+1, but actually the angle we get from magnet in measurement step is next step too

  if(Xk[2] >= 360){
    Xk[2] = Xk[2] - 360.0;
  } else if(Xk[2] <= -360){
    Xk[2] = Xk[2] + 360.0;
  }
  Serial.print(Xk[0]);
  Serial.print(",");
  Serial.print(Xk[1]);
  Serial.print(",");
  Serial.print(Xk[2]);
  Serial.print(",");

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
  for (int i = 0; i< 3; i++){ // wtf why it correct the state 0 and 1?
    Kk[i] = P_tem[i][2] * S;
    Xk[i] = Xk[i] + Kk[i] * delta_y; 
  }

  // update error cov matrix
  float IKC[3][3] ={
    {1,  0, -Kk[0]},
    {0,  1, -Kk[1]},
    {0,  0, 1-Kk[2]}};
  multi_AxB(IKC, P_tem, Pk);

}


void loop() {
  sensor.read_with_KF();
  float dt = sensor.get_duration();
  float yaw_rate = sensor.get_gyro_rate_z();
  float acce_value[3];
  acce_value[0] = sensor.getAccelX();
  acce_value[1] = sensor.getAccelY();
  acce_value[2] = sensor.getAccelZ();
  float pitch = sensor.get_roll_kalman() *3.14159 / 180;
  float roll =  - sensor.get_pitch_kalman() *3.14159 / 180;
  Vector norm = compass.readNormalize(); // after adding offset (0,0,0)+ normalize by scale sensitivity
  float magnet_raw[3];
  magnet_raw[0] = norm.XAxis - offset[0];
  magnet_raw[1] = norm.YAxis - offset[1];
  magnet_raw[2] = norm.ZAxis - offset[2];
                              // float magnet_value1[3];
                              // multi_AxX(matrix_calib,magnet_raw,magnet_value1);
                              // Serial.print(magnet_value1[0]);
                              // Serial.print(",");
                              // Serial.print(magnet_value1[1]);
                              // Serial.print(",");
                              // Serial.print(magnet_value1[2]);
  // Serial.print(float(x/SENSITIVITY_mT), 6);
  // Serial.print(",");
  // Serial.print(float(y/SENSITIVITY_mT), 6);
  // Serial.print(",");
  // Serial.print(float(z/SENSITIVITY_mT), 6);
  // Serial.println();
  
  /* if we use the formula of projection on the plan, perpendicular with gravity vector*/
  // float yaw_magnetic = compute_yaw(-acce_value[1], acce_value[0], acce_value[2],magnet_raw[0],magnet_raw[1],magnet_raw[2]);

  
  // angle without tilt compensation
                                  float magnet_value[3];
                                  multi_AxX(matrix_calib,magnet_raw,magnet_value);
                                  float yaw_without_tilt =  atan2(-magnet_value[1],magnet_value[0])*180/3.14159;
                                  
                                  // angle with tilt compensation from this note https://www.artekit.eu/resources/ak-mag3110/doc/AN4248.pdf
                                  float magnet_value_tilt[3];
                                  magnet_value_tilt[0] = magnet_raw[0] * cos(pitch) + magnet_raw[1]*sin(roll)*sin(pitch) + magnet_raw[2]*sin(pitch)*cos(roll);
                                  magnet_value_tilt[1] = magnet_raw[1] * cos(roll) - magnet_raw[2]*sin(roll);
                                  magnet_value_tilt[2] = -magnet_raw[0] * sin(pitch) + magnet_raw[1]*cos(pitch)*sin(roll) + magnet_raw[2]*cos(pitch)*cos(roll);
                                  multi_AxX(matrix_calib,magnet_value_tilt,magnet_value);      
                                  // Serial.print(",");
                                  // Serial.print(magnet_value[0]);
                                  // Serial.print(",");
                                  // Serial.print(magnet_value[1]);
                                  // Serial.print(",");
                                  // Serial.print(magnet_value[2]);
                                  // Serial.println();
                                  
                                  float yaw_with_tilt =  atan2(-magnet_value[1],magnet_value[0])*180/3.14159;
                                  // // plot distribution of angle from magnet after tilt compensation
                                  //         // Serial.print(offset[0]);Serial.print(",");
                                  //         // Serial.print(offset[1]);Serial.print(",");
                                  //         // Serial.println(offset[2]);

                                  _3dkalman(state_K , P, dt, yaw_rate, yaw_with_tilt);
                                  
                                  // // print 3 angles 
                                    Serial.print(yaw_without_tilt);
                                    Serial.print(",");
                                    Serial.println(yaw_with_tilt);
                                    // Serial.print(",");
                                    // Serial.print(state_K[2]);

                                  // // test all output state of KF
                                  // // Serial.println(dt ,6); // -> dt = 0.020852
                                      // Serial.print(state_K[0]);
                                      // Serial.print(",");
                                      // Serial.print(state_K[1]);
                                      // Serial.print(",");
                                      // Serial.println(state_K[2]);

  delay(14);
}
