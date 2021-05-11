#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Device.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

#define TIME_STEP 64
using namespace webots;

#define deltaT 0.01f
#define PI 3.14159265358979f
#define gyroMeanError PI * (5.0f / 180.0f)
#define beta sqrt(3.0f/4.0f) * gyroMeanError

struct quaternion {
    float q1;
    float q2;
    float q3;
    float q4;
};

extern struct quaternion q_est;

struct quaternion quat_mult (struct quaternion q_L, struct quaternion q_R);

static inline void quat_scalar(struct quaternion * q, float scalar){
    q -> q1 *= scalar;
    q -> q2 *= scalar;
    q -> q3 *= scalar;
    q -> q4 *= scalar;
}

static inline void quat_add(struct quaternion * Sum, struct quaternion L, struct quaternion R) {
    Sum -> q1 = L.q1 + R.q1;
    Sum -> q2 = L.q2 + R.q2;
    Sum -> q3 = L.q3 + R.q3;
    Sum -> q4 = L.q4 + R.q4;
}

static inline void quat_sub(struct quaternion * Sum, struct quaternion L, struct quaternion R) {
    Sum -> q1 = L.q1 - R.q1;
    Sum -> q2 = L.q2 - R.q2;
    Sum -> q3 = L.q3 - R.q3;
    Sum -> q4 = L.q4 - R.q4;
}

static inline struct quaternion quat_conjugate(struct quaternion q){
    q.q2 = -q.q2;
    q.q3 = -q.q3;
    q.q4 = -q.q4;
    return q;
}

static inline float quat_Norm (struct quaternion q)
{
    return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

static inline void quat_Normalization(struct quaternion * q){
    float norm = quat_Norm(*q);
    q -> q1 /= norm;
    q -> q2 /= norm;
    q -> q3 /= norm;
    q -> q4 /= norm;
}

struct quaternion q_est = { 1, 0, 0, 0};

struct quaternion quat_mult (struct quaternion L, struct quaternion R){
    struct quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}

void imu_filter(float ax, float ay, float az, float gx, float gy, float gz){
    struct quaternion q_est_prev = q_est;
    struct quaternion q_est_dot = {0};
    const struct quaternion q_g_ref = {0, 0, 0, 1};
    struct quaternion q_a = {0, ax, ay, az};
    
    float F_g [3] = {0};
    float J_g [3][4] = {0};
    
    struct quaternion gradient = {0};
    
    struct quaternion q_w;
    q_w.q1 = 0;
    q_w.q2 = gx;
    q_w.q3 = gy;
    q_w.q4 = gz;
    
    quat_scalar(&q_w, 0.5);
    
    q_w = quat_mult(q_est_prev, q_w);
    quat_scalar(&q_w, deltaT);
    quat_add(&q_w, q_w, q_est_prev); 
    
    quat_Normalization(&q_a);

    F_g[0] = 2*(q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
    F_g[1] = 2*(q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3* q_est_prev.q4) - q_a.q3;
    F_g[2] = 2*(0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;
    
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] =  2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] =  2 * q_est_prev.q2;
    
    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;
    
    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;
    
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];
    
    quat_Normalization(&gradient);
    quat_scalar(&gradient, (beta));
    quat_sub(&q_est_dot, q_w, gradient);
    quat_scalar(&q_est_dot, deltaT);
    quat_add(&q_est, q_est_prev, q_est_dot);
    quat_Normalization(&q_est);
}

void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw){
    *yaw = atan2f((2*q.q2*q.q3 - 2*q.q1*q.q4), (2*q.q1*q.q1 + 2*q.q2*q.q2 -1));
    *pitch = -asinf(2*q.q2*q.q4 + 2*q.q1*q.q3);
    *roll  = atan2f((2*q.q3*q.q4 - 2*q.q1*q.q2), (2*q.q1*q.q1 + 2*q.q4*q.q4 -1));
    
    *yaw *= (180.0f / PI);
    *pitch *= (180.0f / PI);
    *roll *= (180.0f / PI);
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  InertialUnit *IMU = robot->getInertialUnit("inertial unit");
  Accelerometer *accelerometer = robot->getAccelerometer("accelerometer");
  Gyro *gyro = robot->getGyro("gyro");
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  accelerometer->enable(TIME_STEP);
  gyro->enable(TIME_STEP);  
  IMU->enable(TIME_STEP);
  float rpy[3],vel[3],gy[3];

  
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = 4.0;
    double rightSpeed = 4.0;
    
    wheels[0]->setVelocity(leftSpeed/2);
    wheels[1]->setVelocity(rightSpeed/2);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
    
    rpy[0]=IMU->getRollPitchYaw()[0];
    rpy[1]=IMU->getRollPitchYaw()[1];
    rpy[2]=IMU->getRollPitchYaw()[2];
    vel[0]=accelerometer->getValues()[0];
    vel[1]=accelerometer->getValues()[1];
    vel[2]=accelerometer->getValues()[2];
    gy[0]=gyro->getValues()[0];
    gy[1]=gyro->getValues()[1];
    gy[2]=gyro->getValues()[2];
    
    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    imu_filter(vel[0], vel[1], vel[2], gy[0], gy[1], gy[2]);
    eulerAngles(q_est, &roll, &pitch, &yaw);
    
    std::cout<<"===========\n"<<std::setprecision(3)
      <<"roll: "<<rpy[0] 
      <<"  pitch: "<<rpy[1]
      <<"  yaw: "<<rpy[2]<<std::endl
      <<"roll2: "<< roll
      <<"  pitch2: "<< pitch
      <<"  yaw2: "<< yaw <<std::endl;
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}