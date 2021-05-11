#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Device.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <iostream>
#include <iomanip>
#define TIME_STEP 64
using namespace webots;

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
    
    std::cout<<"===========\n"<<std::setprecision(3)
      <<"roll: "<<rpy[0] 
      <<"  pitch: "<<rpy[1]
      <<"  yaw: "<<rpy[2]<<std::endl
      <<"acceleration:\ny: "<<vel[0]
      <<" x: "<<vel[1]
      <<" z: "<<vel[2]<<std::endl
      <<"gyro:\n"<<gy[0]<<" "
      <<gy[1]<<" "
      <<gy[2]<<std::endl;
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}