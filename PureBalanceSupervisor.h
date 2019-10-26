
#ifndef _PURE_BALANCE_SUPERVISOR_H_
#define _PURE_BALANCE_SUPERVISOR_H_

#include <Arduino.h>

#include "Controller.h"
#include "Robot.h"
#include "BalanceRobot.h"

#include <CurieIMU.h>
#include "kalmanFilter.h"

//#include "MyMPU6050.h"
//#include "MPU6050.h"

#include <Kalman.h>

#define GYRO_RATE 200

class PureBalanceSupervisor
{
public:
  PureBalanceSupervisor();
  void execute(long left_ticks, long right_ticks, double dt);
  void reset(Vector goal, double v, double d_fw);
  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void resetKalman();
  void setGoal(double v, double w);
  void setGotoGoal(double x, double y, double theta, double v);

  void startGotoGoal();

  void stopDrive();
  void init();

  void setSettings(SETTINGS settings);
  SETTINGS getSettings();
  PIDParam getBalancePIDParam();
  PIDParam getSpeedPIDParam();
  PIDParam getThetaPIDParam();

  void setBalancePIDParam(PIDParam pid);
  void setSpeedPIDParam(PIDParam pid);
  void setThetaPIDParam(PIDParam pid);

  void updateSettings(SETTINGS settings);
  SETTINGS getSettings(byte settingsType);

  void getRobotInfo()
  {
    Serial.println("Balance robot info, current state:");
    Serial.print("exec time:");
    Serial.print(execTime);
    Serial.print(", max_pwm:");
    Serial.println(max_pwm);

    // Serial.print(", pwm_diff:");
    // Serial.print(pwm_diff);
    // Serial.print(", pwm_zero:");
    // Serial.println(pwm_zero);

    Serial.print("pwmInfo(b,s,t, wl，wr):");
    Serial.print(mBalancePWM);
    Serial.print(",");
    Serial.print(mSpeedPWM);
    Serial.print(",");
    Serial.print(mThetaPWM);
    Serial.print(",");
    Serial.print(mwPWM_L);
    Serial.print(",");
    Serial.println(mwPWM_R);

    Serial.print("Input(v, theta):");
    Serial.print(m_input.v);
    Serial.print(", ");
    Serial.println(m_input.theta);

    robot.getRobotInfo();
    Serial.println("balance:");
    Serial.print("KP:");
    Serial.print(b_kp);
    Serial.print(", kd: ");
    Serial.println(b_kd);

    Serial.println("Speed:");
    Serial.print("KP:");
    Serial.print(s_kp);
    Serial.print(", ki: ");
    Serial.print(s_ki);
    Serial.print(", si: ");
    Serial.println(sIntegral);

    Serial.println("theta:");
    Serial.print("KP:");
    Serial.print(t_kp);
    Serial.print(", KI:");
    Serial.print(t_ki);
    Serial.print(", si: ");
    Serial.println(tIntegral);
  }

  //        void setGoal(double x, double y, int theta);

  Position getRobotPosition();
  //will return the pitch/angle of the robot
  // void getIRDistances(double dis[5]);

  void getBalanceInfo(double *buf);
  void getIMUInfo(double *buf, double dt);

  //filter paramaters
  double KG_ANG;

  PWM_OUT pwm;
  double mBalancePWM, mSpeedPWM, mSpeedDelta, mThetaPWM, mThetaDelta, mwPWM_L, mwPWM_R;
  Vel mVel;

  int pwm_diff, pwm_zero, max_pwm;

  double pwm_l, pwm_r;

  // int angleType; //the angle used to control balance 0 sensor 1 kalman 2 es
  // void setAngleType(int val)
  // {
  //   angleType = val;
  //   Serial.print("Change angle type to:");
  //   Serial.println(val);
  // }

  bool mSimulateMode;
  bool mIgnoreObstacle;
  bool mSendIMUInfo;

  bool mSpeedLoop, mThetaLoop;

  void setBeSpeedLoop(bool val);
  void setBeThetaLoop(bool val);
  void setBeSendIMUInfo(bool val);
  void readIMU(double dt);
  void calculateAngle(double dt);

private:
  void check_states();
  int m_state;

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

  bool layingDown, hangUp;

  Kalman kalman;
  // KalmanFilter km;

  bool progress_made;
  bool at_goal;
  bool at_obstacle;
  bool unsafe;

  //used to count the robot velocity
  // double m_per_tick;
  long prev_left_ticks, prev_right_ticks;
  double m_right_ticks, m_left_ticks;
  int speedCounter;

private:
  //balance control
  double b_kp, b_kd;

  //speed control
  double s_kp, s_ki, sIntegral;

  //theta control
  double t_kp, t_ki, tIntegral;

  void balanceOut(double dt);
  void speedOut(long leftTicks, long rightTicks, double dt);
  void thetaOut(double dt);

private:
  // Robot(double R, double L, double ticksr, double maxRpm double minRpm)
  // Robot robot;

  int m_aix, m_aiy, m_aiz;
  int m_gix, m_giy, m_giz;

  BalanceRobot robot;

  double normalize(double in, double limit);

  void sendIMUInfo();

  void sendCtrlInfo();

  long execTime;

  //the target to go!
  Vector m_Goal;

  double mTheta;  //目标方向
  double mW;      //转弯
  double curW;    //当前的转弯状态
  bool keepTheta; //是否需要保存当前方向
  int keepThetaTimer;

  long thetaPrevMillis;

  double d_fw; //distance to follow wall
  double d_stop;
  double d_at_obs;
  double d_unsafe;
  double d_prog;

  Input m_input;
  Output m_output;
};

#endif /* _BALANCE_SUPERVISOR_H_ */
