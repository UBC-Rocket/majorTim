#ifndef APDET_H
#define APDET_H

/* Constants used in apdet.cpp */
#define LOCN_ALT                         785    /* (m)        altitude of Hanna, Alberta */
#define MIN_APOGEE_DEPLOY                 20    /* (m)        height below which we don't deploy drogue, payload or main */
#define LAUNCH_ACCEL                    1050    /* (millig)   assume we've launched when we detect accel >= 1050 */
#define STBY_ACCEL_EPSILON                50    /* (millig)   maximum variation of the accelerometer at standby. 
                                                                Should be LAUNCH_ACCEL - 1 */
#define ACCEL_NEAR_APOGEE                150    /* (millig)   accel magnitude <= 150 indicates we are close to apogee */
#define MAIN_DEPLOY_HEIGHT              2000    /* (ft)       height at which we deploy main */

/* Constants for safety checks */
#define NUM_CHECKS                         5    /* each condition has to pass 5 times */
#define ARR_SIZE                          10    /* size of state change checking array */

/* Comparison constant */
#define EPSILON                        0.005    /* used for floating point value comparison */

/* Duration of pauses of program */
#define POST_DROGUE_DEPLOYMENT_PAUSE    3000    /* (ms) regulation pause after deploying drogue */
#define PRESSURE_NORMALIZATION_PAUSE    2000    /* (ms) duration of sensor-querying pause 
                                                            while pressure normalizes in the rocket */

/* Constants for altitude calculation */
#define P_0                          1013.25    /* (mb)         pressure at 0 altitude */
#define T_0                           288.15    /* (K)          temperature at 0 altitude */
#define L                            -0.0065    /* (K/m)        lapse rate (valid for heights up to 11km) */
#define R                            287.053    /* (J/(kg*K))   gas constant for air */
#define g                            9.80665    /* (m/s^2)      gravitational acceleration */

/* File paths for logging */
static const char* logPath = "/sd/log.bin";
static const char* sdBaseVarsPath = "/sd/baseVars.bin";
static const char* sdCurrStatePath = "/sd/currState.bin";
static const char* sdMountPt = "sd";

static FILE* logFP;
static FILE* baseVarsFP;
static FILE* currStateFP;

#endif