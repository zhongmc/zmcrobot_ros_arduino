
#ifndef _DRIVE_SUPERVISOR_H_
#define _DRIVE_SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"

#include "VelocityController.h"

#define S_STOP 0
#define S_GTG 1
#define S_AVO 2
#define S_FW 3

class DriveSupervisor
{
public:
  DriveSupervisor();

  void execute(long left_ticks, long right_ticks, double gyro, double dt);

  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void setGoal(double v, double w);

  void getRobotInfo()
  {
    Serial.println("Drive robot info:");

    Serial.print("v:");
    Serial.print(m_output.v);
    Serial.print(",w:");
    Serial.print(m_output.w);

    Serial.print(",vel-l:");
    Serial.print(mVel.vel_l);
    Serial.print(",vel-r:");
    Serial.println(mVel.vel_r);
    // long c1, c2;
    // c1 = (long)m_left_ticks;
    // c2 = (long)m_right_ticks;
    Serial.print("c1:");
    Serial.print(m_left_ticks);
    Serial.print(",c2:");
    Serial.println(m_right_ticks);

    robot.getRobotInfo();
    m_Controller.PrintInfo();
  }

  //val: 0 none, 1,yes, 2 use ultro sonic
  void setHaveIRSensor(int idx, byte val);

  void setIRFilter(bool open, float val);

  void readIRDistances(double dis[5]);

  void getIRDistances(double dis[5]);
  void getRobotVel(double dis[5]);

  Position getRobotPosition();
  void setRobotPosition(double x, double y, double theta);

  void updateSettings(SETTINGS settings);

  void init();

  bool mSimulateMode;
  bool mIgnoreObstacle;
  bool mUseIMU;
  double alpha;

private:
  void check_states();

  double v, w;
  Vel mVel;
  bool unsafe;
  bool danger;

  int m_state;
  double m_right_ticks, m_left_ticks;

private:
  VelocityController m_Controller;
  //   Robot robot;
  RearDriveRobot robot;

  double d_unsafe;
  Input m_input;
  Output m_output;
};

#endif /* _DRIVE_SUPERVISOR_H_ */
