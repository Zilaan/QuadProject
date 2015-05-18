/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
#include "lps25h.h"
#include "debug.h"
#include "led.h"
#include "matrix.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

/*
 * User defined variables. The size of kMatrix and
 * krMatrix may change.
 */
void matrixAdd(float target[], float A[], float B[], uint8_t row, uint8_t col) {
  uint16_t len = row * col;
  uint16_t i;
  for(i = 0; i < len; i++) {
      target[i] = A[i] + B[i];
  }
}

void matrixMultiply(float target[], float K[], float u[], uint8_t m1, uint8_t n1, uint8_t m2, uint8_t n2) {
  uint8_t i, j, k, m3, n3;
  m3 = m1;
  n3 = n2;

  if (n1 == m2) {
    for(i = 0; i < m3; i++) {
      for(j = 0; j < n3; j++)
        target[j + i * n3] = 0.0f;
    }

    for (i = 0; i < m3; i++) {
      for(j = 0; j < n3; j++) {
        for(k = 0; k < n1; k++) {
          target[j + i * n3] = target[j + i * n3] + K[k + i * n1] * u[j + k * n2];
        }
      }
    }
  }
}


/* u = Kr * r + K * x
 *     4x3 3x1 + 4x5 5x1
 */
float mulTemp1[4], mulTemp2[4], tempMat[4];
static float ref[3] = {0,
                          0,
                          0}; // Ref from user (r)

static float states[8] = {0,
                             0,
                             0,
                             0,
                             0, 0, 0, 0}; // States (x)
float controlSig[4] = {0,
                             0,
                             0,
                             0}; // Control signal (u)

#define TO_RAD 0.01745329251

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0;
static float accMAG    = 0.0;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust


RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, (const signed char * const)STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // References given by user
      //ref[0] = -eulerYawDesired * M_PI / 180.0f;
      //ref[1] = eulerRollDesired * M_PI / 180.0f;
      //ref[2] = -eulerPitchDesired * M_PI / 180.0f;

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
        // Get states

        states[0] = gyro.x * TO_RAD;
        states[1] = gyro.y * TO_RAD;
        states[2] = (eulerYawDesired - gyro.z) * TO_RAD;
        states[3] = (eulerRollDesired - eulerRollActual) * TO_RAD;
        states[4] = (eulerPitchDesired - eulerPitchActual) * TO_RAD;

        // Calculate control signal
        matrixMultiply(controlSig, kMatrix, states, 4, 6, 6, 1);
        //for(i = 0; i < ; i++)
          //controlSig[0] = controlSig[0] + kMatrix[0*]*states[i];



      }

      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&actuatorThrust);
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }

      if (actuatorThrust > 0)
      {
        // Send control signal to the motors and add user thrus to it
        motorPowerM1 = limitThrust((uint16_t) (controlSig[0] + actuatorThrust));
        motorPowerM2 = limitThrust((uint16_t) (controlSig[1] + actuatorThrust));
        motorPowerM3 = limitThrust((uint16_t) (controlSig[2] + actuatorThrust));
        motorPowerM4 = limitThrust((uint16_t) (controlSig[3] + actuatorThrust));

        motorsSetRatio(MOTOR_M1, motorPowerM1);
        motorsSetRatio(MOTOR_M2, motorPowerM2);
        motorsSetRatio(MOTOR_M3, motorPowerM3);
        motorsSetRatio(MOTOR_M4, motorPowerM4);

        //motorsSetRatio(MOTOR_M2, limitThrust((uint16_t) (controlSig[1] + actuatorThrust)));
        
      }
      else
      {
        distributePower(0, 0, 0, 0);
      }
    }
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  // We don't need these!
  int16_t r = 0;
  int16_t p = 0;
  motorPowerM1 = limitThrust(thrust - r + p + yaw);
  motorPowerM2 = limitThrust(thrust - r - p - yaw);
  motorPowerM3 =  limitThrust(thrust + r - p + yaw);
  motorPowerM4 =  limitThrust(thrust + r + p - yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)

