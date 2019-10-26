
#include "Balance.h"

#include "ZMCRobot.h"

//static float KP = 15, KI = 0.0, KD = 0.32; // PID variables

Balance::Balance()
{
  Kp = 15;
  Ki = 0.0;
  Kd = 0.32;
  lastError = 0;
  lastErrorIntegration = 0;
}

void Balance::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
}

void Balance::execute(Robot *robot, Input *input, Output *output, double dt)
{

  float e_k, e_I, e_D, pidValue;
  /* Update PID values */
  e_k = robot->angle - input->targetAngle; // - robot->angleOff; //roll;  // pitch;

  e_I = lastErrorIntegration + e_k * dt;
  e_D = (e_k - lastError) / dt;
  pidValue = Kp * e_k + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  if (abs(lastErrorIntegration) > 1000)
    lastErrorIntegration = 0;
  lastError = e_k;

  output->w = pidValue;

  /*
    if ( millis() - reportTimer > 1000)
    {
      reportTimer = millis();
      Serial.print( PIDValue );
      Serial.print(",");

      Serial.print(",");
      Serial.print( wheelVelocity );

      Serial.print(",");
      Serial.print( currentAngle );

      Serial.print(",");
      Serial.println(restAngle);
    }
  */
}
