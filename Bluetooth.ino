#include "ZMCRobot.h"
#include <CurieBle.h>

#include "robot.h"

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

extern bool doCheckBattleVoltage; // = true;
extern bool openDebug;            // = false;
extern byte settingsReqQueue[8];
extern short queueLen; // = 0;

extern long count1, count2;

#if CAR_TYPE == DRIVE_CAR

bool blTL = false;
int blTLPWM = 80;

#endif

BLEPeripheral blePeripheral;
// BLE Peripheral Device (the board you're programming)

BLEService zmcRobotService("3a37"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"

BLECharacteristic zmcRobotSettingsChar("3a38",
                                       // standard 16-bit characteristic UUID
                                       BLERead | BLEWrite | BLENotify, 19); //KP KI KD atObstacle unsafe velocity

BLECharacteristic zmcRobotDriveChar("3a39",
                                    // standard 16-bit characteristic UUID
                                    BLERead | BLEWrite, 19); //CMD:2, datas;    speed tl tr | BLENotify

BLECharacteristic zmcRobotStateChar("3a3a",
                                    // standard 16-bit characteristic UUID
                                    BLERead | BLENotify, 19); // x,y,theta,irdis 0-4, volt of bat | BLENotify

bool bleConnected = false;

// remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications

void initBluetooth()
{

  Serial.println("init BLE...");

  bleConnected = false;
  /*The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("ZMC Robot");

  blePeripheral.setAdvertisedServiceUuid(zmcRobotService.uuid());
  // add the service UUID
  blePeripheral.addAttribute(zmcRobotService);

  // add the Heart Rate Measurement characteristic
  blePeripheral.addAttribute(zmcRobotSettingsChar);
  blePeripheral.addAttribute(zmcRobotDriveChar);
  blePeripheral.addAttribute(zmcRobotStateChar);

  /* Now activate the BLE device. It will start continuously transmitting BLE
    advertising packets and will be visible to remote BLE central devices
    until it receives a new connection */

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  zmcRobotSettingsChar.setEventHandler(BLEWritten, configCharacteristicWritten);
  zmcRobotDriveChar.setEventHandler(BLEWritten, driveCharacteristicWritten);

  blePeripheral.begin();
  Serial.println(("BLE start..."));
}

void blePeripheralConnectHandler(BLECentral &central)
{

  bleConnected = true;
  // central connected event handler
  Serial.print("BLE Conn: ");
  Serial.println(central.address());

  Serial.print("UUID:");
  Serial.println(zmcRobotService.uuid());

  //  sendRobotConfigValue();
}

void blePeripheralDisconnectHandler(BLECentral &central)
{

  bleConnected = false;
  // central disconnected event handler
  Serial.print("BLE disc: ");
  Serial.println(central.address());
}

void configCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // central wrote new value to characteristic, update LED
  Serial.println("cc w: ");
  setConfigValue(characteristic.value());
}

void driveCharacteristicWritten(BLECentral &central, BLECharacteristic &characteristic)
{
  // central wrote new value to characteristic, update LED
  Serial.print("drv,cmd:");
  //the first two chas as CMD
  unsigned char *data = (unsigned char *)characteristic.value();

  char cmd[3];
  cmd[2] = '\0';
  cmd[0] = toupper(data[0]);
  cmd[1] = toupper(data[1]);
  Serial.println(cmd);

  if (cmd[0] == 'S' && cmd[1] == 'T') //stop
  {
    stopRobot();
  }
  else if (cmd[0] == 'R' && cmd[1] == 'P') //Required for settings
  {
    Serial.println("Req sof:");
    Serial.println(data[2]);

    if (queueLen < 7)
      settingsReqQueue[queueLen++] = data[2];
    else
      Serial.println("Q Ovf");

    //    requireForSettings = true;
    //    requiredSettingsType = data[2];
  }
  // else if (cmd[0] == 'R' && cmd[1] == 'T') //robot type 0 3wheel car ; 1 balance robot
  // {
  //   Serial.print("Set robot type;");
  //   Serial.println(data[2]);
  // }
  // else if (cmd[0] == 'C' && cmd[1] == 'B') // check battle voltage
  // {
  //   if (data[2] == 1)
  //     doCheckBattleVoltage = true;
  //   else
  //     doCheckBattleVoltage = false;
  // }

  else if (cmd[0] == 'S' && cmd[1] == 'M') //simulate mode
  {
    if (data[2] == 1)
      SetSimulateMode(true);
    else
      SetSimulateMode(false);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'O') //ignore atObstacle
  {
    if (data[2] == 1)
      SetIgnoreObstacle(true);
    else
      SetIgnoreObstacle(false);
  }

  else if (cmd[0] == 'R' && cmd[1] == 'S') //RESET
  {
    ResetRobot();
  }

#if CAR_TYPE == DRIVE_CAR

  else if (cmd[0] == 'G' && cmd[1] == 'O') // action...
  {
    startGoToGoal();
  }

  // else if (cmd[0] == 'G' && cmd[1] == 'D') //start drive mode
  // {
  //   startDrive();
  // }

  else if (cmd[0] == 'G' && cmd[1] == 'G') // Go To Goal: x, y, theta
  {
    double x, y, v;
    int theta;
    x = byteToFloat((byte *)(data + 2), 100);
    y = byteToFloat((byte *)(data + 4), 100);
    theta = byteToInt((byte *)(data + 6));
    v = byteToFloat((byte *)(data + 8), 100);

    setGoal(x, y, theta, v);

    log("GTG:%s,%s,%d,%s\n",
        floatToStr(0, x),
        floatToStr(1, y),
        theta,
        floatToStr(2, v));

    // Serial.print("GTG:");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.println(theta);
  }

  else if (cmd[0] == 'M' && cmd[1] == 'G') //go to goal
  {
    float d = atof((char *)(data + 2));
    Serial.print("m gtg:");
    Serial.println(d);

    count1 = 0;
    count2 = 0;
    supervisor.reset(0, 0);

    setGoal(d, 0, 0, 0.12);
    startGoToGoal();
  }

  // else if (cmd[0] == 'S' && cmd[1] == 'R') //step response
  // {
  //   startStepResponse(90);
  // }
  else if (cmd[0] == 'T' && cmd[1] == 'L') //turn around
  {
    if (!blTL)
    {
      int pwm = atoi((char *)(data + 2));
      blTL = true;
      blTLPWM = pwm;
    }
    // turnAround(pwm);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'F') // set ir filter IF0/1,0.6;
  {
    bool val = *(data + 2) - '0';
    float filter = atof((char *)(data + 3));
    log("IR flt:%d,%s\n", val, floatToStr(0, filter));
    supervisor.setIRFilter(val, filter);
    driveSupervisor.setIRFilter(val, filter);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'R')
  {
    short idx = *(data + 2) - '0';
    byte val = *(data + 3) - '0';
    log("S IR:%d,%d\n", idx, val);

    supervisor.setHaveIRSensor(idx, val);
    driveSupervisor.setHaveIRSensor(idx, val);
  }

  else if (cmd[0] == 'P' && cmd[1] == 'I') //pid
  {
    setPID((char *)(data + 2));
  }
#else
  else if (cmd[0] == 'C' && cmd[1] == 'L') //control loop
  {
    bool val = false;
    if (data[3] == 1)
      val = true;

    if (data[2] == 0)
    {
      if (val)
        Serial.println("add speed loop!");
      else
      {
        Serial.println("remove speed loop!");
      }

      balanceSupervisor.setBeSpeedLoop(val);
    }
    else
    {
      if (val)
        Serial.println("add theta loop!");
      else
      {
        Serial.println("remove theta loop!");
      }

      balanceSupervisor.setBeThetaLoop(val);
    }
  }
  else if (cmd[0] == 'G' && cmd[1] == 'B') //start Balance mode
  {
    startBalance();
  }
  else if (cmd[0] == 'B' && cmd[1] == 'S') //stop balance drive or goto goal
  {
    stopBalance();
    // balanceSupervisor.stopDrive();
  }
  else if (cmd[0] == 'G' && cmd[1] == 'O') // action...
  {
    startGoToGoal();
  }

  else if (cmd[0] == 'G' && cmd[1] == 'G') // Go To Goal: x, y, theta
  {
    double x, y, v;
    int theta;
    x = byteToFloat((byte *)(data + 2), 100);
    y = byteToFloat((byte *)(data + 4), 100);
    theta = byteToInt((byte *)(data + 6));
    v = byteToFloat((byte *)(data + 8), 100);
    setGoal(x, y, theta, v);

    Serial.print("Go to Goal:");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(theta);
  }

  else if (cmd[0] == 'M' && cmd[1] == 'G') //go to goal
  {
    float d = atof((char *)(data + 2));
    Serial.print("m gtg:");
    Serial.println(d);

    count1 = 0;
    count2 = 0;
    balanceSupervisor.reset(0, 0);

    setGoal(d, 0.0, 0, 0.12);
    startGoToGoal();
  }

#endif

  else if (cmd[0] == 'S' && cmd[1] == 'D') //set drive Goal
  {
    double v, w;
    v = byteToFloat((byte *)(data + 2), 100);
    w = byteToFloat((byte *)(data + 4), 100);
    log("v=%s,w=%s\n",
        floatToStr(0, v),
        floatToStr(1, w));
    setDriveGoal(v, w);
  }
}

void setConfigValue(const unsigned char *cfgArray)
{
  int settingsType = (int)cfgArray[0];
  Serial.print("cfg v BT:");
  Serial.println(settingsType);

  SETTINGS settings;
  settings.sType = settingsType;

  if (settingsType == 1 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {
    settings.kp = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.ki = byteToFloat((byte *)(cfgArray + 3), 1000);
    settings.kd = byteToFloat((byte *)(cfgArray + 5), 1000);

    log("KP:%s, KI:%s, KD:%s\n",
        floatToStr(0, settings.kp),
        floatToStr(1, settings.ki), floatToStr(2, settings.kd));
    // Serial.print("KP:");
    // Serial.print(settings.kp);
    // Serial.print(" KI:");
    // Serial.print(settings.ki);
    // Serial.print(" KD:");
    // Serial.println(settings.kd);
  }

#if CAR_TYPE == DRIVE_CAR
  else if (settingsType == 5)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.dfw = byteToFloat((byte *)(cfgArray + 5), 100);
    settings.velocity = byteToFloat((byte *)(cfgArray + 7), 100);

    settings.max_rpm = byteToInt((byte *)(cfgArray + 9));
    settings.min_rpm = byteToInt((byte *)(cfgArray + 11));

    settings.radius = byteToFloat((byte *)(cfgArray + 13), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 15), 1000);

    // settings.pwm_diff = (int)cfgArray[13]; //byteToInt((byte *)(cfgArray + 13) );
    // settings.pwm_zero = (int)cfgArray[14];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 15), 100);

    log("atObs:%s, unsafe:%s, dfw:%s, v:%s, max_rmp:%d, min_rpm:%d, R:%s, L:%s\n",
        floatToStr(0, settings.atObstacle),
        floatToStr(1, settings.unsafe),
        floatToStr(2, settings.dfw),
        floatToStr(3, settings.velocity),
        settings.max_rpm,
        settings.min_rpm,
        floatToStr(4, settings.radius),
        floatToStr(5, settings.length)

    );
    // Serial.print(" atObstacle:");
    // Serial.print(settings.atObstacle);

    // Serial.print(" unsafe:");
    // Serial.print(settings.unsafe);
    // Serial.print(" dfw:");
    // Serial.print(settings.dfw);

    // Serial.print(" v:");
    // Serial.print(settings.velocity);
    // Serial.print(" max_rpm:");
    // Serial.print(settings.max_rpm);
    // Serial.print(" min_rpm:");
    // Serial.print(settings.min_rpm);
    // Serial.print(" radius:");
    // Serial.print(settings.radius);
    // Serial.print(" length:");
    // Serial.println(settings.length);
  }

#else

  else if (settingsType == 6)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.max_pwm = byteToInt((byte *)(cfgArray + 5));
    settings.pwm_zero = (int)cfgArray[7];
    settings.pwm_diff = (int)cfgArray[8];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 9), 100);

    settings.radius = byteToFloat((byte *)(cfgArray + 11), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 13), 1000);

    settings.velocity = byteToFloat((byte *)(cfgArray + 15), 100);

    Serial.print(" atObstacle:");
    Serial.print(settings.atObstacle);

    Serial.print(" unsafe:");
    Serial.print(settings.unsafe);
    Serial.print(" max_pwm:");
    Serial.print(settings.max_pwm);
    Serial.print(" pwm_zero:");
    Serial.print(settings.pwm_zero);
    Serial.print(" pwm_diff:");
    Serial.print(settings.pwm_diff);
    Serial.print(" angle_off:");
    // Serial.print(settings.angleOff);
    Serial.print(" radius:");
    Serial.println(settings.radius);
    Serial.print(" wheel distance:");
    Serial.println(settings.length);
  }
#endif

#if CAR_TYPE == DRIVE_CAR
  if (settingsType == 1 || settingsType == 5 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {
    supervisor.updateSettings(settings);
    driveSupervisor.updateSettings(settings);
  }
#else
  if (settingsType == 2 || settingsType == 3 || settingsType == 4 || settingsType == 6)
  {
    balanceSupervisor.updateSettings(settings);
  }
#endif

  // setSettings(settings);
  //  updateConfigToMenu();
}

/*
void sendRobotConfigValue()
{

  if ( !bleConnected )
    return;

  Serial.println("Send the robot settings!");
  byte settingsArray[18];

  floatToByte(settingsArray, mSettings.kp, 100 );
  floatToByte(settingsArray+2, mSettings.ki, 1000 );
  floatToByte(settingsArray+4, mSettings.kd, 1000 );
  floatToByte(settingsArray+6, mSettings.atObstacle, 100 );  
  floatToByte(settingsArray+8, mSettings.unsafe, 100 );  
  floatToByte(settingsArray+10, mSettings.dfw, 100 );  
  floatToByte(settingsArray+12, mSettings.velocity, 100 );

  intToByte(settingsArray+14, mSettings.max_rpm );
  intToByte(settingsArray+16, mSettings.min_rpm );
  
  zmcRobotSettingsChar.setValue( settingsArray, 18 );
}
*/

void intToByte(byte *arrayBuf, int val)
{
  if (val > 0)
  {
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val;
  }
  else
  {
    val = -val;
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val | 0x80;
  }
}

void floatToByte(byte *arrayBuf, double val, double scale)
{
  int tmp = (int)(val * scale);
  if (tmp > 0)
  {
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp;
  }
  else
  {
    tmp = -tmp;
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp | 0x80;
  }
}

double byteToFloat(byte *arrayBuf, double scale)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;
  return (double)val / scale;
}

int byteToInt(byte *arrayBuf)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;

  return val;
}

#if CAR_TYPE == DRIVE_CAR

//type, x, y, theta, d0,d1,d2,d3,d4,voltage
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage)
{
  if (!bleConnected)
    return;

  byte buf[19];
  memset(buf, 0, 19);

  buf[0] = stateType;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  //    Serial.print( voltage);
  //    Serial.print( ", " );

  scale = 100;
  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
    //  Serial.print( irDistance[i] );
    //    Serial.print( "," );
  }
  //    Serial.println( ";" );

  floatToByte(buf + 17, voltage, scale);

  bool ret = zmcRobotStateChar.setValue(buf, 19);
  if (!ret)
  {
    Serial.println("wt bt F!");
  }
}

void checkBLTL()
{
  if (!blTL)
    return;

  turnAround(blTLPWM);
  blTL = false;
}

#else
//type angle1,2,3,voltage
void sendBalanceRobotStateValue(Position pos, double irDistance[5], double voltage)
{
  byte buf[19];

  memset(buf, 0, 19);
  buf[0] = 2;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  scale = 100;

  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
  }
  floatToByte(buf + 17, voltage, scale);
  if (!bleConnected)
    return;
  // zmcRobotStateChar.canw
  bool ret = zmcRobotStateChar.setValue(buf, 19);
  if (!ret)
  {
    Serial.println("BLE err!");
  }
}

#endif

void processSetingsRequire()
{
  if (queueLen == 0)
    return;
  byte sType = settingsReqQueue[0];
  for (int i = 0; i < queueLen - 1; i++)
    settingsReqQueue[i] = settingsReqQueue[i + 1];
  queueLen--;

  SETTINGS settings;
  settings.sType = sType;
  // // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: PID theta  5: settings for robot; 6: settings for balance robot;

#if CAR_TYPE == DRIVE_CAR
  settings = supervisor.getSettings(sType);
#else
  settings = balanceSupervisor.getSettings(sType);
#endif
  SendSettings(settings);
}

void SendSettings(SETTINGS settings)
{
  int settingsType = settings.sType;

  Serial.print("Send settings:");
  Serial.println(settingsType);

  byte settingsArray[18];
  settingsArray[0] = (byte)settingsType;
  int len = 7;
  if (settingsType == 1 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {

    floatToByte(settingsArray + 1, settings.kp, 100);
    floatToByte(settingsArray + 3, settings.ki, 1000);
    floatToByte(settingsArray + 5, settings.kd, 1000);
  }
#if CAR_TYPE == DRIVE_CAR

  else if (settingsType == 5)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    floatToByte(settingsArray + 5, settings.dfw, 100);
    floatToByte(settingsArray + 7, settings.velocity, 100);

    intToByte(settingsArray + 9, settings.max_rpm);
    intToByte(settingsArray + 11, settings.min_rpm);

    floatToByte(settingsArray + 13, settings.radius, 1000);

    // settingsArray[13] = (byte)settings.pwm_diff;
    // settingsArray[14] = (byte)settings.pwm_zero;

    floatToByte(settingsArray + 15, settings.length, 1000);

    len = 13;
  }

#else

  else if (settingsType == 6)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    intToByte(settingsArray + 5, settings.max_pwm);
    settingsArray[7] = (byte)settings.pwm_zero;
    settingsArray[8] = (byte)settings.pwm_diff;

    // speedLoop, thetaLoop
    settingsArray[9] = (byte)balanceSupervisor.mSpeedLoop;
    settingsArray[10] = (byte)balanceSupervisor.mThetaLoop;

    //    floatToByte(settingsArray + 9, 0, 1000); //settings.angleOff, 1000);

    floatToByte(settingsArray + 11, settings.radius, 1000);
    floatToByte(settingsArray + 13, settings.length, 1000);

    floatToByte(settingsArray + 15, settings.velocity, 100);

    settingsArray[17] = (byte)balanceSupervisor.mSimulateMode;
    len = 18;
  }
#endif
  zmcRobotSettingsChar.setValue(settingsArray, 18);
}
