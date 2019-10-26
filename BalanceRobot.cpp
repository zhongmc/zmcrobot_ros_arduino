
#include "BalanceRobot.h"

BalanceRobot::BalanceRobot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  // init(0.0334, 0.158, 390, 390, 50, 190, GP2Y0A41);
  init(0.0332, 0.159, 390, 390, 50, 180, GP2Y0A21);

  mPIDSettings.kp = 30;
  mPIDSettings.ki = 0.0;
  mPIDSettings.kd = 0.02;

  irSensors[0] = new IRSensor(-0.045, 0.05, PI / 2, A1, GP2Y0A21);
  irSensors[1] = new IRSensor(0.08, 0.04, PI / 4, A2, GP2Y0A21); //0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.162, 0.0, 0, A3, GP2Y0A41);
  irSensors[3] = new IRSensor(0.08, -0.04, -PI / 4, A4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.045, -0.05, -PI / 2, A5, GP2Y0A21);

  haveIrSensor[0] = false;
  haveIrSensor[1] = false;
  haveIrSensor[2] = false;
  haveIrSensor[3] = false;
  haveIrSensor[4] = false;
  //  mSettings.kp = 5;
  //   mSettings.ki = 0.01;
  //   mSettings.kd = 0.05;
}

PWM_OUT BalanceRobot::getPWMOut(double v, double w)
{
  PWM_OUT pwm;
  pwm.pwm_l = 0;
  pwm.pwm_r = 0;

  return pwm;
}

Vel BalanceRobot::ensure_w(double v, double w)
{

  Vel vel = uni_to_diff(v, w);
  double vel_l, vel_r;

  double vel_min, vel_max;
  vel_min = vel.vel_l;
  vel_max = vel.vel_r;

  if (vel_min > vel.vel_r)
  {
    vel_min = vel.vel_r;
    vel_max = vel.vel_l;
  }

  // stop one motor to support large angle turning
  double minVel = 0;
  if (abs(w) < 0.2)
    minVel = min_vel;

  if (vel_max > max_vel)
  {
    vel_r = vel.vel_r - (vel_max - max_vel);
    vel_l = vel.vel_l - (vel_max - max_vel);
  }
  else if (vel_min < minVel)
  {
    vel_r = vel.vel_r + (minVel - vel_min);
    vel_l = vel.vel_l + (minVel - vel_min);
  }
  else
  {
    vel_r = vel.vel_r;
    vel_l = vel.vel_l;
  }

  if (vel_l < minVel)
    vel_l = minVel;
  else if (vel_l > max_vel)
    vel_l = max_vel;

  if (vel_r < minVel)
    vel_r = minVel;
  else if (vel_r > max_vel)
    vel_r = max_vel;

  vel.vel_l = vel_l;
  vel.vel_r = vel_r;
  return vel;
}

double BalanceRobot::vel_l_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);
  if (nvel < min_vel - 0.1)
    return 0;

  if (nvel > max_vel)
    nvel = max_vel;

  // double retVal = 0.5729 * nvel * nvel - 5.1735 * nvel + 86.516;
  // double retVal = 9.1631 * nvel + 27.898; //6.393 * nvel + 13.952;
  //y = 0.1086x2 + 3.4864x + 60.919

  double retVal = 0.1086 * nvel * nvel + 3.4864 * nvel + 60.919;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double BalanceRobot::vel_r_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);

  if (nvel < min_vel - 0.1)
    return 0; //nvel = min_vel;

  if (nvel > max_vel)
    nvel = max_vel;

  // double retVal = 0.5649 * nvel * nvel - 4.3156 * nvel + 80.706;
  // double retVal = 9.1631 * nvel + 27.898; // 6.2798 * nvel + 18.787;
  //  y = 0.5649x2 - 4.3156x + 80.706
  //y = 0.1194x2 + 3.0069x + 63.031
  double retVal = 0.1194 * nvel * nvel + 3.0069 * nvel + 63.031;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double BalanceRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 60) //14
    return 0;

  // double ticks = dt * (-0.024 * npwm * npwm + 12.097 * npwm - 426.23);
  //y = -0.0264x2 + 16.836x - 882.53
  double ticks = dt * (-0.0264 * npwm * npwm + 16.836 * npwm - 882.53);

  //5.7132x - 180.16
  //  double ticks = dt * (5.7132 * npwm - 180.16);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double BalanceRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 60)
    return 0;

  // double ticks = dt * (-0.0218 * npwm * npwm + 11.634 * npwm - 358.83);
  //5.7049x - 131.73
  //  double ticks = dt * (5.7049 * npwm - 131.73);
  //y = -0.0312x2 + 18.344x - 974.3

  double ticks = dt * (-0.0312 * npwm * npwm + 18.344 * npwm - 974.3);

  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
