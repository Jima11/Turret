/*
 * Notes:
 * 1) I read that going above microstep of 8 or 10 is a waste of time as you just lose torque at no real smoothness gain.
 *    It might not be true, but testing of 8 gives smooth operation
 */

#include <AccelStepper.h>

//setup steppers: mode, clock(step/pulse), direction
AccelStepper H(AccelStepper::DRIVER, 2, 3); //Turret Horizontal Movement
AccelStepper V(AccelStepper::DRIVER, 5, 6); //Turret Vertical Movement
AccelStepper T(AccelStepper::DRIVER, 8, 9); //Turret Trigger Movement

//stepper constants
const int H_RAW_STEPS = 200; //number of raw motor steps
const int H_MICROSTEP = 8; //microstep multiplier
const int H_GEAR_RATIO = 3; //pulley gear ratio
const int H_STEPS_PER_REV = H_RAW_STEPS * H_MICROSTEP * H_GEAR_RATIO; //total number of steps per rev on output shaft
const int H_POLARITY = -1; //axis polarity

const int V_RAW_STEPS = 200; //number of raw motor steps
const int V_MICROSTEP = 8; //microstep multiplier
const int V_GEAR_RATIO = 3; //pulley gear ratio
const int V_STEPS_PER_REV = V_RAW_STEPS * V_MICROSTEP * V_GEAR_RATIO; //total number of steps per rev on output shaft
const int V_POLARITY = -1; //axis polarity

const int T_RAW_STEPS = 200; //number of raw motor steps
const int T_MICROSTEP = 8; //microstep multiplier
const int T_GEAR_RATIO = 1; //pulley gear ratio
const int T_STEPS_PER_REV = T_RAW_STEPS * T_MICROSTEP * T_GEAR_RATIO; //total number of steps per rev on output shaft
const int T_POLARITY = 1; //axis polarity

//initialise global variables
long H_stepPosition = 0;
long V_stepPosition = 0;
long T_stepPosition = 0;

void setup()
{
  //Setup H stepper
  H.setMaxSpeed(stepRateFromRPS(5, H_STEPS_PER_REV));
  H.setAcceleration(stepRateFromRPS(0.1, H_STEPS_PER_REV));
  
  //Setup V stepper
  V.setMaxSpeed(stepRateFromRPS(5, V_STEPS_PER_REV));
  V.setAcceleration(stepRateFromRPS(0.1, V_STEPS_PER_REV));
  
  //Setup T stepper
  T.setMaxSpeed(stepRateFromRPS(5, T_STEPS_PER_REV));
  T.setAcceleration(stepRateFromRPS(5, T_STEPS_PER_REV));
  
  //init serial
  Serial.begin(115200);
}

void loop()
{
  if (Serial.available() > 0)
  {
    String pythonMessage = Serial.readStringUntil('\n');
    Serial.println("Received: " + pythonMessage);
    String axisCode = pythonMessage.substring(0,1); //get 1st character
    String axisVal = pythonMessage.substring(1); //get remaining characters

    //determine which axis to update
    if (axisCode == "H")
    {
      //update H axis position
      float deg = axisVal.toFloat(); //expecting an angle in degrees
      H_stepPosition = H_POLARITY * stepsFromDeg(deg, H_STEPS_PER_REV); //calculate number of steps that corresponds to angle
      H.move(H_stepPosition); //set target position
    }
    else if (axisCode == "V")
    {
      //update V axis position
      float deg = axisVal.toFloat(); //expecting an angle in degrees
      V_stepPosition = V_POLARITY * stepsFromDeg(deg, V_STEPS_PER_REV); //calculate number of steps that corresponds to angle
      V.move(V_stepPosition); //set target position
    }
    else if (axisCode == "T")
    {
      //update T axis position
      int trig = axisVal.toInt();
      if (trig == 1)
      {
        //move to 180deg if trigger pulled
        T_stepPosition = T_POLARITY * stepsFromDeg(180, T_STEPS_PER_REV);
      }
      else
      {
        //move to 0deg if trigger not pulled
        T_stepPosition = T_POLARITY * stepsFromDeg(0, T_STEPS_PER_REV);
      }
      T.moveTo(T_stepPosition);
    }
    else
    {
      Serial.println("Invalid message received...");
    } 
  }
  
  //run to position set by moveTo()
  H.run();
  V.run();
  T.run();
}

/*
 * Calculate step rate (steps per second) from rpm (rev/min), in whole numbers
 * spr = steps per rev
 */
float stepRateFromRPM(float _rpm, int _spr)
{
  return _rpm * (float)_spr / 60;
}

/*
 * Calculate step rate (steps per second) from rps (rev/s), in whole numbers
 * spr = steps per rev
 */
float stepRateFromRPS(float _rps, int _spr)
{
  return _rps * (float)_spr;
}

/*
 * Calculate step rate (steps per second) from dps (degrees/s), in whole numbers
 * spr = steps per rev
 */
float stepRateFromDPS(float _dps, int _spr)
{
  return _dps * (float)_spr / 360;
}

/*
 * Calculate number of steps in a given angle
 * spr = steps per rev
 */
long stepsFromDeg(float _deg, int _spr)
{
  return (long)((_deg / 360) * (float)_spr);
}
