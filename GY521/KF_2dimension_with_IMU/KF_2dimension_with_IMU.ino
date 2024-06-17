#include <Wire.h>
#include <HMC5883L.h>
#include "GY521.h"
HMC5883L compass;

GY521 sensor(0x68);
uint32_t counter = 0;
float last_micro;
float pre_roll_k = 0;

float t_delay = 0;
const float ACCE_COV = 0.036; // plot with python : 1.3225
const float GYROS_RATE_COV = 0.01;// static covariance is 0.007
const float GYROS_MEAN_COV = 0.00015;// static covariance is 0.0000015
const float BIAS_GYROS = 0.0;
// const float MAGNETO_COV = 1.2769;

float state_K_roll[3] = {0,BIAS_GYROS, 0} ; // initial state assumed =0;
float state_K_pitch[3] = {0,BIAS_GYROS, 0} ; // initial state assumed =0;

float A[3][3] = {
                {0,  1, 0},
                {0,  1, 0},
                {0.02,  0, 1}  // need to update dt each time call A
};
float B[3] = {1,0,0};
float P_roll[3][3] = {
                {1,  0, 0},
                {0,  1, 0},
                {0,  0, 1}};

float P_pitch[3][3] = {
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
  sensor.axe = -0.038889158;
  sensor.aye = -0.011673583;
  sensor.aze = 0.072055888;
  sensor.gxe = 1.612763404;
  sensor.gye = 0.983435058;
  sensor.gze = 0.703801536;


  last_micro = 0.0;
  t_delay = micros();  
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
  // A[2][0] = duration;
  // Serial.println(duration);
  // prediction step
  // Ax+Bu

  // Serial.print(rate);
  Xk[1] = 0;
  Xk[2] =  A[2][0]*Xk[0] + Xk[2];
  Xk[0] = Xk[1] + rate; //this is the next step n+1, but actually the angle we get from magnet in measurement step is next step too

  if(Xk[2] >= 180){
    Xk[2] = Xk[2] - 360.0;
  } else if(Xk[2] <= -180){
    Xk[2] = Xk[2] + 360.0;
  }
  // Serial.print(Xk[0]);
  // Serial.print(",");
  // Serial.print(Xk[1]);
  // Serial.print(",");
  // Serial.print(Xk[2]);
  // Serial.print(",");

  // update matrix uncertainty 
  // Serial.println(A[2][1]);  

  multi_AxBxA(A,Pk,P_tem);

  P_tem[0][0] += GYROS_RATE_COV;
  P_tem[1][1] += GYROS_MEAN_COV;

  // measurement step
  float delta_y = measured_angle - Xk[2];
  if(delta_y >= 180){
    delta_y = delta_y - 360.0;
  } else if(delta_y <= -180){
    delta_y = delta_y + 360.0;
  }
  float S = 1/(P_tem[2][2] + ACCE_COV);

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

float duration = 0;
void loop() {
  sensor.read();
  float acce_value[3];
  acce_value[0] = sensor.getAccelX();
  acce_value[1] = sensor.getAccelY();
  acce_value[2] = sensor.getAccelZ();
    //  prepare for Pitch Roll Yaw
  float _ax2 = acce_value[0] * acce_value[0];
  float _ay2 = acce_value[1] * acce_value[1];
  float _az2 = acce_value[2] * acce_value[2];

  //  calculate angle
  float _aax = atan2(       acce_value[1] ,  sqrt(_ax2 + _az2)) * 57.29578;
  float _aay = atan2(-1.0 * acce_value[0] , sqrt(_ay2 + _az2)) * 57.29578;

  float rate_value[2]; // in degree/s
  rate_value[0] = sensor.getGyroX();
  rate_value[1] = sensor.getGyroY();
  float rate_z = sensor.getGyroZ();

                               
  // plot distribution of angle
          Serial.print(acce_value[0],6);Serial.print(",");
          Serial.print(acce_value[1],6);Serial.print(",");
          Serial.print(acce_value[2],6);Serial.print(",");
          Serial.print(rate_value[0],6);Serial.print(",");
          Serial.print(rate_value[1],6);Serial.print(",");
          Serial.print(rate_z,6);Serial.println();
  
  // test all output state of KF
  // Serial.println(dt ,6); // -> dt = 0.020852
            // Serial.print(rate_value[0],4);
            // Serial.print(",");
            // Serial.print(state_K_roll[1],4);
            // Serial.print(",");

            // // print angles 
            // Serial.print(state_K_roll[2],4);
            // Serial.print(",");
            // Serial.print(_aax,4);
            // Serial.println();

    // Serial.print(_aay);
    // Serial.print(",");
    // Serial.println(state_K_pitch[2]);
    // Serial.print(",");
    // Serial.print(state_K[2]);

    

    // ----------delay untill 10 ms--------------
  // delayMicroseconds((int)(12500 - 85- (micros() - t_delay)));

  delayMicroseconds((int)(20000 - (micros() - t_delay)));
  // Serial.println(micros() - t_delay);
  t_delay = micros();


}
