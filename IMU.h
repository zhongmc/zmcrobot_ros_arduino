
#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>

#include <CurieIMU.h>

//#include "kalmanFilter.h"

//#include "MyMPU6050.h"
//#include "MPU6050.h"

#include <Kalman.h>

#define GYRO_RATE 200

class IMU
{
public:
  IMU();
  void resetKalman();
  void init();
  void getIMUInfo(double *buf, double dt);
  //filter paramaters
  double KG_ANG;


  void readIMU(double dt);
  void calculateAngle(double dt);

private:
  //传感器角度（atan（ax/ay）, kalman, madgwick filter, Kalman1
  double m_sensor_angle, m_kalman_angle, m_km_angle; //m_madgwick_angle
  double m_gyro;

  double KG, m_x_angle; // m_x_angle 通过融合滤波得到，用于判断小车被提起
  //     MPU6050 accelgyro;
  //        MyMPU6050 mpu6050;

  double convertRawAcceleration(int aRaw);
  double convertRawGyro(int gRaw);

  //一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
  // angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
  double estima_cal(double angle, double g_angle, double gyro, double dt, double KG);

  Kalman kalman;
  // KalmanFilter km;

private:

  int m_aix, m_aiy, m_aiz;
  int m_gix, m_giy, m_giz;

  void sendIMUInfo();
};

#endif /* _BALANCE_SUPERVISOR_H_ */
