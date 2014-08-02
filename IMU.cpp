#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

void getEstimatedAttitude();

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  #if defined(NUNCHUCK)
    annexCode();
    while((uint16_t)(micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    ACC_getADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((uint16_t)(micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    f.NUNCHUKDATA = 1;
    while(f.NUNCHUKDATA) ACC_getADC(); // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      imu.gyroData[axis] = (imu.gyroADC[axis]*3+gyroADCprevious[axis])>>2;
      gyroADCprevious[axis] = imu.gyroADC[axis];
    }
  #else
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  imu.gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    uint8_t t=0;
    while((uint16_t)(micros()-timeInterleave)<650) t=1; //empirical, interleaving delay between 2 consecutive reads
    if (!t) annex650_overrun_count++;
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  imu.gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      imu.gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]>>1;
      if (!ACC) imu.accADC[axis]=0;
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      imu.gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+imu.gyroData[axis]+1 ) / conf.Smoothing[axis]);
      gyroSmooth[axis] = imu.gyroData[axis];
    }
  #elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    imu.gyroData[YAW] = (gyroYawSmooth*2+imu.gyroData[YAW])/3;
    gyroYawSmooth = imu.gyroData[YAW];
  #endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;
#if MAG
  static t_int32_t_vector EstM32;
  static t_fp_vector EstM;
#endif

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  static uint16_t previousT;
  uint16_t currentT = micros();

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis]  * scale; // radian

    accLPF32[axis]    -= accLPF32[axis]>>ACC_LPF_FACTOR;
    accLPF32[axis]    += imu.accADC[axis];
    imu.accSmooth[axis]    = accLPF32[axis]>>ACC_LPF_FACTOR;

    accMag += (int32_t)imu.accSmooth[axis]*imu.accSmooth[axis] ;
  }

  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif

  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
    #if MAG
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + imu.magADC[axis]) * INV_GYR_CMPFM_FACTOR;
      EstM32.A[axis] = EstM.A[axis];
    #endif
  }
  
  if ((int16_t)EstG32.A[2] > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  att.angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  att.angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

  #if MAG
    att.heading = _atan2(
      EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
      (EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y)*invG ); 
    att.heading += conf.mag_declination; // Set from GUI
    att.heading /= 10;
  #endif

  #if defined(THROTTLE_ANGLE_CORRECTION)
    cosZ = EstG.V.Z / ACC_1G * 100.0f;                                                        // cos(angleZ) * 100 
    throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain(100 - cosZ, 0, 100) >>3;  // 16 bit ok: 200*150 = 30000  
  #endif
}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21

//#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now
#define ACC_Z_DEADBAND 15 // dropped to 15

#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

#if BARO

uint8_t getEstimatedAltitude(){
  int32_t  BaroAlt;
  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
  static float vel = 0.0f;
  static unsigned long previousT;
  unsigned long currentT = micros();
  unsigned long dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    logBaroGroundPressureSum = log(baroPressureSum);
    baroGroundTemperatureScale = (baroTemperature + 27315) *  29.271267f;
    calibratingB--;
  }

  // baroGroundPressureSum is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

  #if (defined(BARO)) || !defined(SUPPRESS_BARO_ALTHOLD)

/* not using acc
    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ = (imu.accSmooth[ROLL] * EstG32.V.X + imu.accSmooth[PITCH] * EstG32.V.Y + imu.accSmooth[YAW] * EstG32.V.Z) * invG;

    static int16_t accZoffset = 0;
    if (!f.ARMED) {
      accZoffset -= accZoffset>>3;
      accZoffset += accZ;
    }  
    accZ -= accZoffset>>3;
    applyDeadband(accZ, ACC_Z_DEADBAND);
*/

    static int32_t lastBaroAlt;
    int16_t baroVel = (alt.EstAlt - lastBaroAlt) * 1000000.0f / dTime;
    //int16_t baroVel = (alt.EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
    lastBaroAlt = alt.EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 5); // to reduce noise near zero

/* not using acc
    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    alt.vario = vel;
    applyDeadband(alt.vario, 1);
*/

   #define MAXALTRATE 40 // max rate to climb or fall in cm/s
   #define ALTCMDIFF 4 // for each ALTCMDIFF increase or decrease throttle by THRINC
   #define THRINC 2
   #define MAXTHRINC 200 // absolute maximum to increase or decrease throttle, excluding vel corrections
   // at (MAXTHRINC/THRINC)*ALTCMDIFF cm from AltHold and beyond thr would increase or decrease MAXTHRINC

   static int16_t maxaltrate = MAXALTRATE; // this is modified if swings are too high
   int16_t thrMod = 0; // velocity modifier
   static int16_t baseThrMod = 0; // base modifier
   static uint8_t updown = 0; // 1 means last loop was climbing, 2 means last loop was falling
   static int16_t minSwing = 0;
   static int16_t maxSwing = 0;
   static uint8_t swingLoop = 0;

   if (f.BARO_MODE == 1) {

     if (updown == 0) {
       // this is the start of AltHold, just enabled

       // set swing points to current alt
       minSwing = alt.EstAlt;
       maxSwing = alt.EstAlt;
     }

     if (alt.EstAlt < AltHold) {
       // below AltHold

       if (swingLoop == 2) {
         // a full loop has been completed

         // if initialThrottleHold was set perfectly, staying almost perfectly level before initiating alt hold
         // then the smoothest transition takes place between up and down
         // in most cases initialThrottleHold is not set perfectly, for example it is triggered while 
         // increasing or decreasing in altitude

         // this attempts to fine tune and correct that over the course of multiple loops
         // by looking at maxSwing and minSwing to determine which way initialThrottleHold is off
         // and modify it in real time

         // maxSwing is the highest Altitude in the top part of the wave
         // minSwing is the lowest Altitude in the bottom part of the wave
         // if maxSwing-AltHold is higher we decrease initialThrottleHold and if AltHold-minSwing is higher we increase it
         // if they are the same, we do nothing
         // this will constantly bounce back and forth, accounting for voltage decrease as well

         if (maxSwing-AltHold > AltHold-minSwing) {
           baseThrMod -= THRINC*4;
         } else if (AltHold-minSwing > maxSwing-AltHold) {
           baseThrMod += THRINC*4;
         }

         if (maxSwing-minSwing > 200 && swingLoop == 0) {
           // current swing is larger than 2m
           // decrease maxaltrate to shrink swings
           if (maxaltrate > 10) {
             // should never be less than 10
             maxaltrate -= 1;
           }
         }

         // set swingLoop to 1 so this doesn't repeat until next raise phase
         swingLoop = 1;
       }

       if (updown == 2) {
         // this is a switch in direction past AltHold
         // because last cycle was 2 which is opposite
         // we have gone from above AltHold to below AltHold

         // reset low point in swing
         minSwing = AltHold;

       }

       if (alt.EstAlt < minSwing) {
         // current alt is lower than min swing, set it
         minSwing = alt.EstAlt;
       }

       if (baroVel >= maxaltrate) {
         // we are moving up faster than we are supposed to be
         // for every ALTCMDIFF cm/s faster than maxaltrate modify throttle by THRINC
         // this is inverse to the throttle applied on altitude difference, keeping things stable
         thrMod = -(((abs(baroVel)-maxaltrate)/ALTCMDIFF)*THRINC);
       }

       // for each ALTCMDIFF from AltHold add THRINC
       int16_t tt = (((AltHold-alt.EstAlt)/ALTCMDIFF)*THRINC);
       BaroPID = constrain(tt, -MAXTHRINC, MAXTHRINC)+thrMod+baseThrMod;

       if (alt.EstAlt == minSwing) {
         // first half of this half cycle
       } else {
         // second half of this half cycle
       }

       updown = 1;

     } else if (alt.EstAlt > AltHold) {
       // above AltHold

       if (updown == 1) {
         // this is a switch in direction past AltHold
         // because last cycle was 1 which is opposite
         // we have gone from below AltHold to above AltHold

         // reset high point in swing
         maxSwing = AltHold;

         // set swingLoop to 2 to indicate a full loop has taken place
         swingLoop = 2;
       }

       if (alt.EstAlt > maxSwing) {
         // current alt is higher than max swing, set it
         maxSwing = alt.EstAlt;
       }

       if (baroVel <= -maxaltrate) {
         // we are moving down faster than we are supposed to be
         // for every ALTCMDIFF cm/s faster than maxaltrate modify throttle by THRINC
         // this is inverse to the throttle applied on altitude difference, keeping things stable
         // THRINC is tripled here because you are fighting gravity on the down swing
         thrMod = ((abs(baroVel)-maxaltrate)/ALTCMDIFF)*(THRINC*3);
       }

       // for each ALTCMDIFF from AltHold remove THRINC
       int16_t tt = -(((alt.EstAlt-AltHold)/ALTCMDIFF)*THRINC);
       BaroPID = constrain(tt, -MAXTHRINC, MAXTHRINC)+thrMod+baseThrMod;

       if (alt.EstAlt == maxSwing) {
         // first half of this half cycle
       } else {
         // second half of this half cycle
       }

       updown = 2;

     }

   } else {
     // f.BARO_MODE != 1
     // alt hold has been turned off
     // reset everything
     updown = 0;
     minSwing = 0;
     maxSwing = 0;
     swingLoop = 0;
     maxaltrate = MAXALTRATE;
   }

   // debug for charting, debug[>4] doesn't work with standard MSP Protocol
   /*
   debug[0] = baroVel; // RATE IN CM/S
   debug[1] = BaroPID; // WHAT WE ARE GOING TO INCREASE THROTTLE BY
   debug[2] = AltHold; // ALT TO HOLD AT
   debug[3] = alt.EstAlt; // CURRENT ALT
   debug[4] = maxaltrate;
   debug[5] = thrMod; // velocity mod to throttle
   debug[6] = minSwing; //min swing
   debug[7] = maxSwing; //max swing
   */

  #endif
  
  return 1;
}
#endif //BARO

