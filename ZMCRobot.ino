#include "ZMCRobot.h"
#include <CurieBLE.h>
#include <Arduino.h>

#include "CurieTimerOne.h"
#include "IRSensor.h"
#include "Robot.h"

#if CAR_TYPE == DRIVE_CAR
#include "Supervisor.h"
#include "DriveSupervisor.h"
#include "BlinkLed.h"
//#include "BlinkMatrixLed.h"
#else
#include "MyMPU6050.h"
#include <CurieIMU.h>
// #include <MadgwickAHRS.h>
#include "PureBalanceSupervisor.h"

#include "BlinkMatrixLed.h"
#include "IRReceiver.h"
//#include "Kalman.h"
#endif

//#include "MyKey.h"
//#include "MyMenu.h"

#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 11
#define ULTRASONIC_TRIG 10

//states
#define STATE_IDLE 0
#define STATE_MENU 1
#define STATE_DRIVE 2
#define STATE_GOTOGOAL 3
#define STATE_BALANCE 4
#define STATE_CALIBRATE 5

#define START_KEY 0
#define LEFT_KEY 1
#define RIGHT_KEY 2
#define RET_KEY 3

#define START_KEY_PIN 11
#define LEFT_KEY_PIN 10
#define RIGHT_KEY_PIN 9
#define RET_KEY_PIN 8

byte currentState = STATE_IDLE;

//MyKey myKey;

#if CAR_TYPE == DRIVE_CAR
Supervisor supervisor;
DriveSupervisor driveSupervisor;
BlinkLed blinkLed;
//BlinkMatrixLed blinkLed;
bool bExecDrive, bExecGTG;

#else
PureBalanceSupervisor balanceSupervisor;
BlinkMatrixLed blinkLed;
IRReceiver irRecv(12);
IRCode ircode;

double drive_v, drive_w;
bool bExecBalance;
#endif

Position pos;

long trigTime, echoTime;
double ultrasonicDistance;

bool doCheckBattleVoltage = true;
bool openDebug = false;
byte settingsReqQueue[8];
short queueLen = 0;

extern long count1, count2;
extern int comDataCount;

//char *titles[] = {"Self balance", "Cruise", "Speed ", "Start", "Remote by BLE", "To Target", "Target X:", "Target Y:", "Start",
//                         "PID of Balance", "KP: ", "KI: ", "KD: ", "Config", "Calibrate Motor", "balance angle"
//                        };

//menu_item menuItems[17];
//MyMenu menu(&menuItems[0]);

unsigned long millisPrevKey, millisPrev;

//bool backLightOn = false;
// LiquidCrystal_I2C lcd(0x27, 16, 2);

//  GP2Y0A41 = 0,     //4-30cm
//  GP2Y0A21 = 1     //10-80cm

IRSensor irSensor(GP2Y0A41);

// SETTINGS mSettings;
// Madgwick filter;
// unsigned long microsPerReading, microsPrevious;
// double accelScale, gyroScale;

static double batteryVoltage;  // Measured battery level
static uint8_t batteryCounter; // Counter used to check if it should check the battery level

// int m_p = 0;
// int bCount = 0;

double irDistance[5];

void setup()
{

#if CAR_TYPE == BALANCE_CAR

  drive_v = 0;
  drive_w = 0;

#endif

  Serial.begin(115200);
  delay(100);
  //lcd.init(); // initialize the lcd
  //lcd.backlight(); //Open the backlight
  //  lcd.noBacklight();
  //  lcd.print(" "); // Print a message to the LCD.
  // showMainTips();

  /*
  Serial.println("Initialze key...");
  myKey.addKey(RET_KEY, RET_KEY_PIN);
  myKey.addKey(RIGHT_KEY, RIGHT_KEY_PIN);
  myKey.addKey(LEFT_KEY, LEFT_KEY_PIN);
  myKey.addKey(START_KEY, START_KEY_PIN);


  myKey.initKey();
*/
  // // start the IMU and filter
  // CurieIMU.begin();
  // CurieIMU.setGyroRate(GYRO_RATE);
  // CurieIMU.setAccelerometerRate(GYRO_RATE);
  // // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);

  // filter.begin(GYRO_RATE);

  initMotor();

  // initialize variables to pace updates to correct rate
  //  microsPerReading = 1000000 / GYRO_RATE;  //25
  //  microsPrevious = micros();

  initBluetooth();
  //  Serial.println(F("Initialze MENU..."));
  //
  //void MyMenu::setKeyId(byte stKey, byte rtKey, byte lKey, byte rKey)
  //  menu.setKeyId(START_KEY, RET_KEY,  LEFT_KEY, RIGHT_KEY);
  //  initMenu();

  //  const int oneSecInUsec = 1000000;   // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  //  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &timedBlinkIsr);  // set timer and callback

  //  Serial.println(sizeof(long));

  pinMode(13, OUTPUT);

  comDataCount = 0;
  count1 = 0;
  count2 = 0;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(ULTRASONIC_ECHO, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // ultr sound echo intterupt
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO), UltrasonicEcho, CHANGE);
#if CAR_TYPE == DRIVE_CAR
  SETTINGS mSettings;
  mSettings.sType = 0;

  mSettings.sType = 0;
  mSettings.kp = 5;
  mSettings.ki = 0.01;
  mSettings.kd = 0.05;

  mSettings.max_rpm = 200;
  mSettings.min_rpm = 40; //45

  mSettings.atObstacle = 0.25; //0.15
  mSettings.unsafe = 0.1;
  mSettings.dfw = 0.2;      //0.25
  mSettings.velocity = 0.2; //0.3

  // supervisor.updateSettings(mSettings);
  // driveSupervisor.updateSettings(mSettings);

  supervisor.init();
  driveSupervisor.init();

  bExecDrive = false;
  bExecGTG = false;

  blinkLed.init();
#else

  SETTINGS mSettings;
  mSettings.sType = 0;
  mSettings.kp = 30;
  mSettings.ki = 0.0;
  mSettings.kd = 0.02;

  mSettings.max_rpm = 140;
  mSettings.min_rpm = 0; //45
  mSettings.max_pwm = 150;

  mSettings.pwm_zero = 0;
  mSettings.pwm_diff = 0;

  mSettings.atObstacle = 0.25; //0.15
  mSettings.unsafe = 0.8;      //0.8
  mSettings.dfw = 0.30;        //0.25
  mSettings.velocity = 0.4;    //0.3

  // balanceSupervisor.updateSettings(mSettings);

  //
  //  mSettings.sType = 2;
  //  mSettings.kp = 30;
  //  mSettings.ki = 0.0;
  //  mSettings.kd = 0.02;
  //  balanceSupervisor.updateSettings(mSettings);
  //
  mSettings.sType = 3;
  mSettings.kp = 200;
  mSettings.ki = 10;
  mSettings.kd = 0.0;
  // balanceSupervisor.updateSettings(mSettings);
  balanceSupervisor.init();
  blinkLed.init();
  //  blinkLed.normalBlink();

  const int oneSecInUsec = 1000000;                           // A second in mirco second unit.
  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &balanceIsr); // set timer and callback

  bExecBalance = false;

#endif

  // bCount = 0;

  millisPrevKey = millis();
  millisPrev = millisPrevKey; //millis();
}

void loop()
{
  checkSerialData();
  blinkLed.beSureToBlink();
  //ble cmd process
  processSetingsRequire();
  //ultrasonic process
  processUltrasonic();

#if CAR_TYPE == BALANCE_CAR

  if (irRecv.readIRCode(ircode) == 0)
  {
    if (ircode.code_h + ircode.code_l == 255)
      irRemoteProcess(ircode.code_l);
  }

  if (bExecBalance)
  {
    bExecBalance = false;
    if (currentState == STATE_BALANCE)
      balanceSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.005); // 1.0 / (double)GYRO_RATE);
    else
    {
      balanceSupervisor.calculateAngle(0.005);
    }

    // if (currentState == STATE_BALANCE)
    //   balanceSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.005); // 1.0 / (double)GYRO_RATE);
    // else
    //   balanceSupervisor.readIMU(0.005);
  }

#else

  checkBLTL(); //检查BT 转圈指令
  if (bExecDrive)
  {
    bExecDrive = false;
    driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05); //1/20
  }

  if (bExecGTG)
  {
    bExecGTG = false;
    supervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05);
  }

#endif

  unsigned long millisNow = millis();
  if (millisNow - millisPrev >= 100)
  {

    millisPrev = millisNow;

#if CAR_TYPE == DRIVE_CAR
    if (currentState == STATE_GOTOGOAL)
    {
      //report states
      supervisor.getIRDistances(irDistance);
      pos = supervisor.getRobotPosition();
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
    else if (currentState == STATE_DRIVE)
    {
      driveSupervisor.getIRDistances(irDistance);
      pos = driveSupervisor.getRobotPosition();
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
    else
    {
      supervisor.readIRDistances(irDistance);
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
#else

    if (currentState == STATE_BALANCE)
    {
      balanceSupervisor.getBalanceInfo(irDistance);
      pos = balanceSupervisor.getRobotPosition();
      sendBalanceRobotStateValue(pos, irDistance, batteryVoltage);
      /*
              Serial.print(balanceSupervisor.pwm.pwm_l);
              Serial.print(",");
              Serial.print(balanceSupervisor.pwm.pwm_r);
              Serial.print(",");

              Serial.print(balanceSupervisor.mVel.vel_l);
              Serial.print(",");
              Serial.print(balanceSupervisor.mVel.vel_r);
              Serial.print(",");

              Serial.print(balanceSupervisor.mBalancePWM );
              Serial.print(",");

              Serial.print(irDistance[0]);
              Serial.print(",");
              Serial.print(irDistance[1]);
              Serial.print(",");
              Serial.println(irDistance[2]);
      */
    }
    else
    {
      balanceSupervisor.getBalanceInfo(irDistance);
      // balanceSupervisor.getIMUInfo(irDistance, 0.05);
      sendBalanceRobotStateValue(pos, irDistance, batteryVoltage);
    }

#endif

    /*
    if ( LED_ON )
    {
      LED_ON = false;
      digitalWrite( 13, HIGH);
    }
    else
    {
      LED_ON = true;
      digitalWrite( 13, LOW);
    }
*/

    batteryCounter++;
    if (batteryCounter >= 2)
    { // Measure battery every 1s
      batteryCounter = 0;
      if (isBatteryLow())
      {
        if (doCheckBattleVoltage && isBatteryLow()) //read again
        {
          if (currentState != STATE_IDLE)
          {
            Serial.println("Bat L...");
            stopAndReset();
            currentState = STATE_IDLE;

#if CAR_TYPE == BALANCE_CAR

            stopBalance();
#endif
          }
          blinkLed.slowBlink();
        }
      }
      // batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0352771 + 0.2; /// 65.7424242f; 0.2 二极管压降
      // // v = D * 3.3*(R1+R2)/(R2*1023);  R1 = 46.5 R2 = 4.68 V = d* 0.0352771;
      // // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 56k-15k voltage divider - 1023.0/(3.3/(15.0/(15.0+56.0))) = 63.050847458

      // if ((batteryVoltage < 9 && batteryVoltage < 7.2) || (batteryVoltage > 9 && batteryVoltage < 11.5)) // && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
      // {
      //   if (doCheckBattleVoltage)
      //   {
      //     if (currentState != STATE_IDLE)
      //       Serial.println("Bat lower...");
      //     stopAndReset();
      //     currentState = STATE_IDLE;
      //     stopBalance();
      //     // stopRobot();
      //     blinkLed.slowBlink();
      //   }
      // }
      // else
      // {
      //   if (currentState == STATE_IDLE)
      //     blinkLed.normalBlink();
      // }
    }
  }
}

#if CAR_TYPE == DRIVE_CAR

void setGoal(double x, double y, int theta, double v)
{
  // count1 = 0;
  // count2 = 0;

  supervisor.setGoal(x, y, theta, v);
}

void startGoToGoal()
{
  if (currentState == STATE_GOTOGOAL)
    return;

  stopRobot(); //stop currentState

  // supervisor.updateSettings(mSettings);
  Serial.print("Start GTG:");
  Serial.print(supervisor.m_Goal.x);
  Serial.print(",");
  Serial.println(supervisor.m_Goal.y);

  // to test set goal y to 0
  supervisor.reset(readLeftEncoder(), readRightEncoder());
  currentState = STATE_GOTOGOAL;

  const int oneSecInUsec = 1000000;                     // A second in mirco second unit.
  CurieTimerOne.start(oneSecInUsec / 20, &goToGoalIsr); // set timer and callback //the controller loop need 30ms to exec
}

void goToGoalIsr()
{
  bExecGTG = true;
  // supervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05);
}

void ResetRobot()
{
  supervisor.resetRobot();
  driveSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

void startDrive()
{
  // if ( currentState >= 2 )
  //   return;

  Serial.println("Start DRV!");
  currentState = STATE_DRIVE;

  // to test set goal y to 0
  driveSupervisor.reset(readLeftEncoder(), readRightEncoder());
  //   currentState = STATE_DRIVE;

  const int oneSecInUsec = 1000000; // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  CurieTimerOne.start(oneSecInUsec / 20, &driveIsr); // set timer and callback
}

void driveIsr()
{
  bExecDrive = true;
  // driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05); //1/20
}

void SetSimulateMode(bool val)
{
  // supervisor.mSimulateMode = val;
  supervisor.setSimulateMode(val);
  driveSupervisor.mSimulateMode = val;
  if (val)
  {
    doCheckBattleVoltage = false;
    Serial.println("simulate.");
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate!");
  }
}

void SetIgnoreObstacle(bool igm)
{
  Serial.print("ignore obs:");
  Serial.println(igm);
  supervisor.mIgnoreObstacle = igm;
  driveSupervisor.mIgnoreObstacle = igm;
}

void stopRobot()
{
  blinkLed.normalBlink();
  CurieTimerOne.kill();

  if (currentState == STATE_DRIVE)
  {
    Position pos = driveSupervisor.getRobotPosition();
    supervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
  else if (currentState == STATE_GOTOGOAL)
  {
    Position pos = supervisor.getRobotPosition();
    driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
  currentState = STATE_IDLE;
  stopAndReset();
}

#else

void irRemoteProcess(int code)
{
  if (code == 28) //OK key
  {
    if (currentState == STATE_BALANCE)
    {
      if (drive_w != 0)
      {
        drive_w = 0;
        balanceSupervisor.setGoal(drive_v, drive_w);
      }
      else if (drive_v != 0)
      {
        drive_v = 0;
        balanceSupervisor.setGoal(drive_v, drive_w);
      }
      else
      {
        stopBalance();
        blinkLed.normalBlink();
        return;
      }

      if (drive_v == 0 && drive_w == 0)
      {
        balanceSupervisor.stopDrive();
        blinkLed.balanceBlink();
      }
    }
    else
    {
      startBalance();
    }
    return;
  }

  if (code == 22) //*
  {
    balanceSupervisor.setBeSpeedLoop(true);
    Serial.println("speed loop.");
    return;
  }

  if (code == 13) //#
  {
    balanceSupervisor.setBeThetaLoop(true);
    Serial.println("turn loop.");
    return;
  }

  if (currentState != STATE_BALANCE) //ignore speed ctrl
    return;

  bool drv = false;

  if (code == 24) //up
  {
    if (drive_v == 0)
      drive_v = 0.1;
    else
    {
      drive_v = drive_v + 0.02;
      if (drive_v > 0.3)
        drive_v = 0.3;
    }

    drv = true;
  }
  else if (code == 82) //down
  {
    if (drive_v == 0)
    {
      drive_v = -0.1;
    }
    else
    {
      drive_v = drive_v - 0.02;
      if (drive_v < -0.3)
        drive_v = -0.3;
    }

    drv = true;
  }
  else if (code == 8) //LEFT
  {
    drive_w = drive_w + 0.1;
    if (drive_w > 1.2)
    {
      drive_w = 1.2;
    }
    drv = true;
  }
  else if (code == 90)
  {
    drive_w = drive_w - 0.1;
    if (drive_w < -1.2)
    {
      drive_w = -1.2;
    }
    drv = true;
  }

  if (drv == true)
  {
    balanceSupervisor.setGoal(drive_v, drive_w);
    blinkLed.runingBlink();
  }
  if (drive_v == 0 && drive_w == 0)
  {
    balanceSupervisor.stopDrive();
    blinkLed.balanceBlink();
  }
}

void setGoal(double x, double y, int theta, double v)
{
  balanceSupervisor.setGotoGoal(x, y, theta, v);
}

void startGoToGoal()
{
  if (currentState == STATE_BALANCE)
  {
    balanceSupervisor.startGotoGoal();
    blinkLed.runingBlink();
  }
}

void startBalance()
{
  if (currentState >= 2)
    return;

  blinkLed.balanceBlink();

  currentState = STATE_BALANCE;

  Serial.print("Start balance!");
  balanceSupervisor.reset(readLeftEncoder(), readRightEncoder());
  // const int oneSecInUsec = 1000000;                           // A second in mirco second unit.
  // CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &balanceIsr); // set timer and callback
}

void balanceIsr()
{
  bExecBalance = true;
  balanceSupervisor.readIMU(0.005);

  // if (currentState == STATE_BALANCE)
  //   balanceSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.005); // 1.0 / (double)GYRO_RATE);
  // else
  //   balanceSupervisor.readIMU(0.005);
}

void balanceRecovered()
{
  blinkLed.balanceBlink();
}

void balanceUnnormal()
{
  blinkLed.laydownBlink();
}

void SetIgnoreObstacle(bool igm)
{
  Serial.println("set ignore obstacle mode: " + igm);

  balanceSupervisor.mIgnoreObstacle = igm;
}

void SetSimulateMode(bool val)
{
  balanceSupervisor.mSimulateMode = val;
  if (val)
  {
    doCheckBattleVoltage = false;
    Serial.println("Set to simulate mode!");
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate mode!");
  }
}

void ResetRobot()
{
  balanceSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

//为了共用，banlance时 stop 表示stop goto goal 或 drive
void stopRobot()
{
  balanceSupervisor.stopDrive();
  blinkLed.balanceBlink();
}

void stopBalance()
{
  Serial.println("Stop balance.");

  blinkLed.normalBlink();
  currentState = STATE_IDLE;
  balanceSupervisor.reset(readLeftEncoder(), readRightEncoder());
  // CurieTimerOne.kill();
  stopAndReset();
}

#endif

void setDriveGoal(double v, double w)
{
#if CAR_TYPE == DRIVE_CAR
  if (currentState == STATE_DRIVE)
  {
    if (abs(v) < 0.001 && abs(w) < 0.001) //stop
      stopRobot();
    // else
    driveSupervisor.setGoal(v, w);
  }
  else
  {
    stopRobot(); //stop currentState
    startDrive();
    driveSupervisor.setGoal(v, w);
  }

#else
  if (currentState == STATE_BALANCE)
  {
    drive_v = v;
    drive_w = w;

    balanceSupervisor.setGoal(v, w);
    if (v != 0 || w != 0)
    {
      blinkLed.runingBlink();
    }
    else
    {
      blinkLed.balanceBlink();
    }
  }
#endif
}

bool isBatteryLow()
{
  batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0352771 + 0.2;
  if ((batteryVoltage < 9 && batteryVoltage < 7.2) || (batteryVoltage > 9 && batteryVoltage < 11.5)) // && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
    return true;
  else
    return false;
}

bool waitForEcho = false;
long lastTrigTimer = 0;

void processUltrasonic()
{
  if (waitForEcho)
  {
    if (echoTime > 0)
    {
      ultrasonicDistance = 0.00017 * echoTime;
      if (ultrasonicDistance > MAX_ULTRASONIC_DIS)
        ultrasonicDistance = MAX_ULTRASONIC_DIS - 0.01;
      waitForEcho = false;
      echoTime = 0;
    }
    else if (millis() - lastTrigTimer > 50)
    {
      waitForEcho = false;
      //      Serial.println("u ...");
      ultrasonicDistance = 1.2; //MAX_ULTRASONIC_DIS;
    }
  }
  else
  {
    long curTime = millis();
    if (curTime - lastTrigTimer < 40)
      return;
    lastTrigTimer = curTime;
    waitForEcho = true;

    digitalWrite(ULTRASONIC_TRIG, HIGH); //trig the ultrosonic

    long curMicros = micros();
    while (true) //10us pules
    {
      if (micros() - curMicros > 10)
        break;
    }
    digitalWrite(ULTRASONIC_TRIG, LOW); //trig the ultrosonic
  }
}

//the ultrasonic isr service
void UltrasonicEcho()
{

  int echoSig = digitalRead(ULTRASONIC_ECHO);
  if (echoSig == HIGH)
  {
    trigTime = micros();
    echoTime = 0;
  }
  else
    echoTime = micros() - trigTime;
}
