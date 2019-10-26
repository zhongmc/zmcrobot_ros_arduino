#include "ZMCRobot.h"
#include <CurieBle.h>

#if CAR_TYPE == DRIVE_CAR
#include "Supervisor.h"
#include "DriveSupervisor.h"
#else
#include "PureBalanceSupervisor.h"
//#include "Kalman.h"

#endif

#if CAR_TYPE == DRIVE_CAR
extern Supervisor supervisor;
extern DriveSupervisor driveSupervisor;
#else
extern PureBalanceSupervisor balanceSupervisor;
#endif

extern long trigTime, echoTime;
extern double ultrasonicDistance;

static char comData[32];
int comDataCount;

//extern Supervisor supervisor;
extern double gp2y0a41[3][4];
extern long count1, count2;

extern bool openDebug;

extern double batteryVoltage;

void checkSerialData()
{

  //read speed setting from serial
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      char ch = Serial.read();
      comData[comDataCount++] = ch;
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        processCommand(comData, comDataCount);
        comDataCount = 0;
      }
      if (comDataCount > 30) //some error
      {
        Serial.print("UnKnow cmd:");
        comData[comDataCount] = 0;
        Serial.println(comData);
        comDataCount = 0;
      }
    }
  }
}

//  Serial.println("\r\n\r\n***** BLE Commands *******************************");
//  Serial.println(" ST stop!");
//  Serial.println(" RS reset robot");
//  Serial.println(" RP[type] require for settings, 1,2,3,4,5");
//  Serial.println(" RT[type] Set robot type 0 normal 1 balance");
//  Serial.println(" SM[val] Set simulate mode 1 true");
//  Serial.println(" GO Start goto goal");
//  Serial.println(" GG[][][] Set goto goal goan x,y,Q");
//  Serial.println(" GD Start drive mode");
//  Serial.println(" GB Start balance mode");
//  Serial.println(" SD[v][w] set drive goal");
//  Serial.println(" BS stop balance drive or goto goal\r\n\r\n");
//
// Serial.println(" ?; print this info");
// Serial.println(" gr[]; controller infos");
// Serial.println(" sb; start balance");
// Serial.println(" st; stop robot");
// Serial.println(" gd; distance of ultraSonic");
// Serial.println(" od; open debug");
// Serial.println(" cd; close debug");
// Serial.println(" ci; read count");

// Serial.println(" mm[pwm]; move motor with pwm");
// Serial.println(" ml[pwm]; move left motor with pwm");
// Serial.println(" mr[pwm]; move right motor with pwm");
// Serial.println(" gg[xd]; start go to goal");

// Serial.println(" sm[0/1]; simulate mode");
// Serial.println(" io[0/1]; ignore obstacle");
// Serial.println(" rs; reset");
// Serial.println(" tl; turn around test left");
// Serial.println(" tr; turn around test right");

//  Serial.println(" bp[]; balance kp");
//  Serial.println(" bi[]; balance ki");
//  Serial.println(" bd[]; balance kd");
//
//  Serial.println(" vp[]; velocity kp");
//  Serial.println(" vi[]; velocity ki");
//  Serial.println(" vd[]; velocity kd");

void processCommand(char *buffer, int bufferLen)
{
  *(buffer + bufferLen) = 0;
  if (bufferLen <= 2)
  {
    if (buffer[0] == 'b') //get baundrate
      Serial.println(115200);
    else if (buffer[0] == 'p')
      Serial.println(100);
    else
      Serial.println("Na");
    return;
  }
  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  if (ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n\r\n====");
    Serial.print("BAT:");
    Serial.print(batteryVoltage);
    Serial.print(", uc dis:");
    Serial.println(ultrasonicDistance);
#if CAR_TYPE == DRIVE_CAR
    Serial.println("\r\n==");
    supervisor.getRobotInfo();
    Serial.println("\r\n==");
    driveSupervisor.getRobotInfo();
#else
    Serial.println("\r\n====");
    balanceSupervisor.getRobotInfo();
#endif
  }
  else if (ch0 == 's' && ch1 == 't') //stop
  {
    // Serial.println("Stop!");
    stopRobot();
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
    printCountInfo();
  }

  else if (ch0 == 'm' && ch1 == 'l') // move left motor
  {
    int pwm = atoi(buffer + 2);
    printCountInfo();
    MoveLeftMotor(pwm);
  }

  else if (ch0 == 'm' && ch1 == 'r') // move right motor
  {
    int pwm = atoi(buffer + 2);
    printCountInfo();
    MoveRightMotor(pwm);
  }

  else if (ch0 == 'm' && ch1 == 'm') // move motor
  {
    int pwm = atoi(buffer + 2);
    motorSpeed(pwm);
    MoveMotor(0);
  }
  else if (ch0 == 's' && ch1 == 'p') //speed test
  {
    int pwm0, pwm1, step = 0;
    pwm0 = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');
    pwm1 = atoi(buf + 1);
    buf = strchr((buf + 1), ',');
    step = atoi(buf + 1);

    // Serial.print("SP:");
    // Serial.print(pwm0);
    // Serial.print(",");
    // Serial.print(pwm1);
    // Serial.print(",");
    // Serial.println(step);
    log("SP:%d,%d,%d\n", pwm0, pwm1, step);
    if (step == 0)
      step = 10;

    speedTest(pwm0, pwm1, step);
  }

  else if (ch0 == 'r' && ch1 == 's') //RESET
  {
    ResetRobot();
  }

  else if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);

    setDriveGoal(v, w);
  }
  else if (ch0 == 's' && ch1 == 'm') //simulate mode
  {
    int val = atoi(buffer + 2);
    SetSimulateMode(val);

    //     if (val == 1)
    //         SetSimulateMode(true);
    // else
    //   SetSimulateMode(false);
  }

  else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test
  {
    int pwm = atoi(buffer + 2);
    turnAround(pwm);
  }
  else if (ch0 == 'm' && ch1 == 'g') //go to goal
  {
    // count1 = 0;
    // count2 = 0;
    // supervisor.reset(0, 0);
    // supervisor.resetRobot();
    // float x = atof(buffer + 2);
    // float y = 0;
    // char *buf = strchr(buffer, ',');
    // if (buf != NULL)
    //   y = atof(buf + 1);
    // setGoal(x, y, 0);
    // startGoToGoal();
    manuaGoal();
  }

#if CAR_TYPE == DRIVE_CAR
  else if (ch0 == 'g' && ch1 == 'o') //start go to goal
  {
    startGoToGoal();
  }

  else if (ch0 == 'g' && ch1 == 'g') //set goto goal goal
  {

    double fvs[4];
    getDoubleValues(buffer + 2, 4, fvs);
    setGoal(fvs[0], fvs[1], fvs[2], fvs[3]);
  }

  else if (ch0 == 'o' && ch1 == 'd') //set obstacle distance
  {
    double ods[5];
    getDoubleValues(buffer + 2, 5, ods);
    supervisor.setObstacleDistance(ods);
  }
  else if (ch0 == 'r' && ch1 == 'p') //set robot position
  {
    double fvs[3];
    getDoubleValues(buffer + 2, 3, fvs);
    supervisor.setRobotPosition(fvs[0], fvs[1], fvs[2]);
    //to do
  }

  // else if (ch0 == 's' && ch1 == 'r') // step response
  // {
  //   int pwm = atoi(buffer + 2);
  //   stepResponseTest(pwm);
  // }

  else if (ch0 == 'p' && ch1 == 'i') //pid
  {
    setPID(buffer + 2);
  }

  else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test
  {
    int pwm = atoi(buffer + 2);
    turnAround(pwm);
  }
  else if (ch0 == 'i' && ch1 == 'o') //ignore atObstacle
  {
    int val = atoi(buffer + 2);
    SetIgnoreObstacle(val);

    // if (val == 1)
    //   SetIgnoreObstacle(true);
    // else
    //   SetIgnoreObstacle(false);
  }

#endif

#if CAR_TYPE == BALANCE_CAR
  else if (ch0 == 'c' && ch1 == 'l') //control loop
  {
    int val = *(buffer + 3) - '0';
    bool bv = true;
    if (val == 0)
      bv = false;
    if (*(buffer + 2) == '0') // speed
    {
      balanceSupervisor.setBeSpeedLoop(bv);
    }
    else
    {
      balanceSupervisor.setBeThetaLoop(bv);
    }
  }
  else if (ch0 == 'i' && ch1 == 'u') //pring imu info
  {
    int val = *(buffer + 2) - '0';
    bool bv = true;
    if (val == 0)
      bv = false;
    balanceSupervisor.setBeSendIMUInfo(bv);
  }

  else if (ch0 == 'g' && ch1 == 'o') //start balance
  {
    Serial.println("Start balance!");
    startBalance();
  }
  else if (ch0 == 's' && ch1 == 'b') //start balance
  {
    Serial.println("Start balance!");
    startBalance();
  }
  else if (ch0 == 'b' && ch1 == 's') //stop balance
  {
    Serial.println("stop balance!");
    stopBalance();
  }

  else if (ch0 == 'c' && ch1 == 'm')
  {
    CalibrateIMU();
  }

  else if (ch0 == 'p' && ch1 == 'i') //pid234;2 balance 3 speed 4 turning
  {

    PIDParam pidParam;
    if (bufferLen <= 5)
    {

      if (*(buffer + 3) == '2')
      {
        pidParam = balanceSupervisor.getBalancePIDParam();
        sendPID("PID2", pidParam.kp, pidParam.ki, pidParam.kd, 1000);
      }
      else if (*(buffer + 3) == '3')
      {
        pidParam = balanceSupervisor.getSpeedPIDParam();
        sendPID("PID3", pidParam.kp, pidParam.ki, pidParam.kd, 1000);
      }
      else if (*(buffer + 3) == '4')
      {
        pidParam = balanceSupervisor.getThetaPIDParam();
        sendPID("PID4", pidParam.kp, pidParam.ki, pidParam.kd, 1000);
      }
    }
    else
    {
      pidParam = getPID(buffer + 4);

      if (*(buffer + 3) == '2')
      {
        balanceSupervisor.setBalancePIDParam(pidParam);
        Serial.println("Set PID of Balance:");
      }
      else if (*(buffer + 3) == '3')
      {
        balanceSupervisor.setSpeedPIDParam(pidParam);
        Serial.println("Set PID of speed:");
      }
      else if (*(buffer + 3) == '4')
      {
        balanceSupervisor.setThetaPIDParam(pidParam);
        Serial.println("Set PID of turning:");
      }
    }
  }
#endif
  else
  {
    Serial.println("Unknow");
  }
}

#if CAR_TYPE == BALANCE_CAR
/*
SETTINGS getPID(char *buffer)
{
  double p, i, d;
  p = atof((buffer));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);

  Serial.print("PID:");
  Serial.print(p);
  Serial.print(",");
  Serial.print(i);
  Serial.print(",");
  Serial.println(d);

  SETTINGS settings;
  settings.sType = 1;
  settings.kp = p;
  settings.ki = i;
  settings.kd = d;
  return settings;
}
*/

PIDParam getPID(char *buffer)
{
  double p, i, d;
  p = atof((buffer));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);

  Serial.print("PID:");
  Serial.print(p);
  Serial.print(",");
  Serial.print(i);
  Serial.print(",");
  Serial.println(d);

  PIDParam settings;
  settings.kp = p;
  settings.ki = i;
  settings.kd = d;
  return settings;
}

void sendPID(char *pref, double kp, double ki, double kd, int scale)
{
  String retStr;

  String pre = pref;
  char tmp[10];
  itoa((int)(scale * kp), tmp, 10);
  String skp = tmp;
  itoa((int)(scale * ki), tmp, 10);
  String ski = tmp;
  itoa((int)(scale * kd), tmp, 10);
  String skd = tmp;

  retStr = pre + skp + "," + ski + "," + skd;
  Serial.println(retStr);
}

void CalibrateIMU()
{
  Serial.println("Internal sensor offsets BEFORE calibration( xAcc,yAcc,zAcc, xGyro, yGyro, zGyro ...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t"); // -235
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\t"); // 168
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

  // To manually configure offset compensation values,
  // use the following methods instead of the autoCalibrate...() methods below
  //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
  //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
  //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
  //CurieIMU.setGyroOffset(X_AXIS,7.869);
  //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
  //CurieIMU.setGyroOffset(Z_AXIS,15.494);

  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(5000);

  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");

  Serial.print("Starting Acceleration calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t"); // -2359
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\t"); // 1688
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
}
#endif

void printCountInfo()
{

  log("CI:%d, %d.\n", count1, count2);

  // Serial.print(millis());
  // Serial.print(',');
  // Serial.print(count1);
  // Serial.print(',');
  // Serial.println(count2);

  // Serial.print("C1=");
  // Serial.print(count1);
  // Serial.print(", C2=");
  // Serial.println(count2);
  // Serial.print("time:");
  // Serial.println(millis());
}

void speedTest(int pwm0, int pwm1, int step)
{
  // long c1, c2, lt;
  for (int i = pwm0; i < pwm1; i += step)
  {
    motorSpeed(i);
  }
  MoveMotor(0);
}

void motorSpeed(int pwm)
{
  long c1, c2, lt;
  MoveMotor(pwm);
  log("%d,", pwm);

  // Serial.print(pwm);
  // Serial.print(',');
  delay(500);
  c1 = count1;
  c2 = count2;
  lt = millis();
  delay(1000);

  c1 = count1 - c1;
  c2 = count2 - c2;
  lt = millis() - lt;
  log("%d,%d,%d\n", lt, c1, c2);
}

void turnAround(int pwm)
{
  // Serial.print("TR:");
  // Serial.println(pwm);
  log("TR:%d, %d, %d;\n", pwm, count1, count2);

  if (pwm > 0)
  {
    count1 = 0;
    MoveLeftMotor(pwm);
  }
  else
  {
    count2 = 0;
    MoveRightMotor(-pwm);
  }

  while (true)
  {
    if ((pwm > 0 && count1 > 2000) || (pwm < 0 && count2 > 2000))
    // if (count1 > 1700 || count2 > 1700)
    {
      StopMotor();
      break;
    }
    delay(50);
    log("TR:%d, %d, %d;\n", pwm, count1, count2);
  }

  delay(100);

  log("ci:%d,%d;\n", count1, count2);
}

void manuaGoal()
{
  count1 = 0;
  count2 = 0;
  MoveMotor(90);
  delay(3000);
  StopMotor();
  delay(500);
  log("%d,%d", count2, count2);

  // Serial.print(count1);
  // Serial.print(',');
  // Serial.println(count2);
}

void getDoubleValues(char *buffer, int c, double *fvs)
{
  char *buf = buffer;
  for (int i = 0; i < c; i++)
  {
    fvs[i] = (float)atoi(buf) / 1000.0;
    buf = strchr(buf, ',');
    if (buf == NULL)
      break;
    buf++;
    // Serial.print(fvs[i]);
    // Serial.print(',');
  }
  // Serial.println(';');
}

int formatStr(char *buf, char *format, ...)
{
  int c = 0;
  va_list vArgList;
  va_start(vArgList, format);
  c = vsprintf(buf, format, vArgList); //_vsnprintf(buf, 256, format, vArgList);
  va_end(vArgList);
  // *(buf + c) = 0;
  return c;
}

void log(char *format, ...)
{
  char tmp[500];
  va_list vArgList;
  va_start(vArgList, format);
  vsprintf(tmp, format, vArgList);
  va_end(vArgList);
  Serial.print(tmp);
}

char tmp[20][15];

char *floatToStr(int idx, double val)
{

  return floatToStr(idx, (signed char)6, (unsigned char)3, val);
}

char *floatToStr(int idx, signed char width, unsigned char prec, double val)
{
  if (idx >= 19)
    return NULL;
  return dtostrf(val, width, prec, tmp[idx]);
}

#if CAR_TYPE == DRIVE_CAR

void setPID(char *buffer)
{
  double p, i, d;
  p = atof((buffer));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);

  log("PID:%s, %s, %s;\n",
      floatToStr(0, p),
      floatToStr(1, i),
      floatToStr(2, d));

  // Serial.print("PID:");
  // Serial.print(p);
  // Serial.print(",");
  // Serial.print(i);
  // Serial.print(",");
  // Serial.println(d);

  SETTINGS settings;
  settings.sType = 1;
  settings.kp = p;
  settings.ki = i;
  settings.kd = d;
  supervisor.updateSettings(settings);
  driveSupervisor.updateSettings(settings);
}

// void stepResponseTest(int pwm)
// {
//   Serial.print("SR: ");
//   Serial.println(pwm);
//   printCountInfo();
//   printCountInfo();
//   int count = 100;
//   while (count > 0)
//   {
//     delay(20);
//     printCountInfo();
//   }
//   printCountInfo();
//   MoveMotor(0);
// }

/*
void log(int level, char *format, ...)
{
  char str_tmp[256];
  int i = 0;
  va_list vArgList;                              //定义一个va_list型的变量,这个变量是指向参数的指针.
  va_start(vArgList, format);                    //用va_start宏初始化变量,这个宏的第二个参数是第一个可变参数的前一个参数,是一个固定的参数
  i = _vsnprintf(str_tmp, 256, format, vArgList); //注意,不要漏掉前面的_
  va_end(vArgList);                              //用va_end宏结束可变参数的获取
  return i;                                      //返回参数的字符个数中间有逗号间隔
}
*/

#endif
