/*
Hardware-independent functions.
*/
#include "mbed.h"
#include <i2c_driver.h>
#include <general.h>
#include <math.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"


/* CONSTANTS ================================================================================================ */

#define SIM_LAUNCH_ACCEL             1050  /* Assume we've launched when we detect accel >= 1050 millig */
                                           /* should be accel of least accelerating rocket */
#define SIM_BURNOUT_ACCEL_DELTA      4000  /* 4s till burnout (data is pretty trash during powered ascent) */
#define ACCEL_NEAR_APOGEE             150  /* accel magnitude <= 150 millig indicates we are close to apogee */
#define MAIN_DEPLOY_HEIGHT           2000  /* height at which we deploy main (ft) */
#define MIN_APOGEE_DEPLOY              20  /* height above which we want to deploy drogue, payload and main (m) */

#define NUM_CHECKS                      5  /* each condition has to pass 5 times */
#define ARR_SIZE                       10  /* size of state change checking array */

#define LOCN_ALT                      785  /* altitude of Hanna, Alberta (m) */
#define EPSILON                     0.005  /* used for floating point value equality */
#define STBY_ACCEL_EPSILON
                (SIM_LAUNCH_ACCEL - 1000)  /* maximum variation of the accelerometer at standby */

#define IREC_REGULATION_POST_DROGUE_DEPLOYMENT_PAUSE 3000
#define PRESSURE_NORMALIZATION_PAUSE                  2000

#define P_0                       1013.25  /* pressure at 0 altitude (mb) */
#define T_0                        288.15  /* temperature at 0 altitude (K) */
#define L                         -0.0065  /* lapse rate (valid for heights between 0km and 11km) (K/m) */
#define R                         287.053  /* gas constant for air (J/(kg K))*/
#define g                         9.80665  /* gravitational acceleration (m/s^2) */

static const char* logPath = "/sd/log.bin";
static const char* sdBaseVarsPath = "/sd/baseVars.bin";
static const char* sdCurrStatePath = "/sd/currState.bin";
static const char* sdMountPt = "sd";

/* STATIC VARS ============================================================================================== */

Timer timer;

/* DRIVERS ================================================================================================== */

/**
  * @brief  Drogue deployment driver.
  * @return Status
  */
extern status_t deployDrogue()
{
    /* TODO: Actuator */
    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] Drogue deployed but not really.\n", timestamp);
    fclose(pFile);
    return STATUS_OK;
}

/**
  * @brief  Payload deployment driver.
  * @return Status
  */
extern status_t deployPayload()
{
    /* TODO */
    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] Payload deployed but not really.\n", timestamp);
    fclose(pFile);
    return STATUS_OK;
}

/**
  * @brief  Main deployment driver.
  * @return Status
  */
extern status_t deployMain()
{
    /* TODO */
    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] Main deployed but not really.\n", timestamp);
    fclose(pFile);
    return STATUS_OK;
}

/* STATIC HELPER FUNCTIONS ================================================================================== */

/**
  * @brief  Altitude calculator.
  * @param  curr_pres   The current pressure.
  * @param  alt         A pointer to store the current altitude IN METERS.
  * @return Status
  */
extern status_t calcAlt(float curr_pres, float *alt)
{
    *alt = (T_0/L) * (pow((curr_pres/P_0),(-L*R/g)) - 1);
    return STATUS_OK;
}

/**
  * @brief  Height above ground (base altitude) calculator.
  * @param  curr_pres   The current pressure.
  * @param  base_alt    The base altitude.
  * @param  height      A pointer to store the current height IN METERS.
  * @return Status
  */
extern status_t calcHeight(float curr_pres, float base_alt, float *height)
{
    /* Height calculation */
    float curr_alt;
    calcAlt(curr_pres, &curr_alt);
    *height = curr_alt - base_alt;

    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] [Height] %0.4f \n", timestamp, *height);
    fclose(pFile);
    return STATUS_OK;
}

extern status_t convertToFeet(float *height_in_ft, float height_in_m)
{
	*height_in_ft = height_in_m * 3.28084;
    return STATUS_OK;
}

/**
  * @brief  Returns acceleration magnitude.
  * @param  accel_x     The x-component of the acceleration.
  * @param  accel_y     The y-component of the acceleration.
  * @param  accel_z     The z-component of the acceleration.
  * @param  accel       A pointer to store the magnitude of the acceleration.
  * @return Status
  */
extern status_t accelMagnitude(int16_t *accel, int16_t accel_x, int16_t accel_y, int16_t accel_z)
{
	*accel = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));
    return STATUS_OK;
}


extern status_t accelerometerGetAndLog(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    /* Query data from accelerometer */
    status_t retval = accelerometerGetData(accel_x, accel_y, accel_z);
    if (retval != STATUS_OK) {
        return retval;
    }

    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d][Accelerometer] X: %d Y: %d Z: %d \n", timestamp, *accel_x, *accel_y, *accel_z);
    fclose(pFile);

    return STATUS_OK;
}

extern status_t barometerGetAndLog(float *curr_pres) 
{
    /* Query data from barometer */
    status_t retval = barometerGetCompensatedPressure(curr_pres);
    if (retval != STATUS_OK) {
        return retval;
    }

    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] [Barometer] Pres: %.4f \n", timestamp, *curr_pres);
    fclose(pFile);

    return STATUS_OK;
}

extern status_t barometerGetPresTempAndLog(float *curr_pres, float *curr_temp) 
{
    /* Query data from barometer */
    status_t retval = barometerGetCompensatedValues(curr_pres, curr_temp);
    if (retval != STATUS_OK) {
        return retval;
    }

    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] [Barometer] Pres: %.4f Temp: %.4f \n", timestamp, *curr_pres, *curr_temp);
    fclose(pFile);

    return STATUS_OK;
}

extern status_t changeState(state_t *curr_state, state_t state) 
{
    /* Change the state */
    *curr_state = state;
    int8_t buffer[] = { (int8_t)state };
    FILE *pFile = fopen(sdCurrStatePath, "w");
    fseek(pFile, 0, SEEK_SET);
    fwrite(buffer, sizeof(int8_t), sizeof(buffer), pFile);
    fclose(pFile);

    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf("[%d] [State Change] %d\n", timestamp, (int8_t)state);
    fclose(pFile);
    return STATUS_OK;
}

extern status_t writeBaseVars(float base_pres, float base_temp, float base_alt) 
{
    float buffer[] = {base_pres, base_temp, base_alt};
    FILE *pFile = fopen(sdBaseVarsPath, "w");
    fseek(pFile, 0, SEEK_SET);
    fwrite(buffer, sizeof(float), sizeof(buffer), pFile);
    fclose(pFile);
    return STATUS_OK;
}

extern status_t recoverLastState(state_t *curr_state)
{
    int8_t buf[1];
    FILE *pFile = fopen(sdCurrStatePath, "r");
    fseek(pFile, 0, SEEK_SET);
    fread(buf, sizeof(int8_t), sizeof(buf), pFile);
    *curr_state = (state_t)buf[0];
    fclose(pFile);
    return STATUS_OK;
}

extern status_t recoverBaseVars(float *base_pres, float *base_temp, float *base_alt) 
{
    float buf[3];
    FILE *pFile = fopen(sdCurrStatePath, "r");
    fread(buf, sizeof(float), sizeof(buf), pFile);
    *base_pres = buf[0];
    *base_temp = buf[1];
    *base_alt = buf[2];
    fclose(pFile);
    return STATUS_OK;
}

extern status_t recoverAll(state_t *curr_state, float *base_pres, float *base_temp, float *base_alt) 
{
    recoverLastState(curr_state);
    recoverBaseVars(base_pres, base_temp, base_alt);
    return STATUS_OK;
}

extern int sumArrElems(int arr[], int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum;
}


/* ROCKET FLIGHT STATE TRANSITION DETECTION FUNCTIONS ======================================================= */

/* 
TODO LIST
- Check units:
    - height in m vs ft
    - accel in m/s^2 vs g vs millig
- Incorporate Kalman data filtering?
- Make sure a program clears the SD card / base variable file before every flight BUT NOT AFTER A BLACKOUT
- Documentation updates
- Good practice to make everything return status_t for the future
- Check pointer passing when just need value vs passing address to write
- Make input parameter order consistent (first or last arg in function)
- Get accelerometer log to find maximum variation of the accelerometer at standby
- Comments in main
- Try to move gets into main for logging purposes
- Find out how long it takes for pressure to normalize in the rocket (waits)
- Organize constants
- Wait after main deployment or nah
*/

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occured).
  * @param  accel       The current magnitude of the acceleration.
  * @param  curr_pres   The current pressure (if in standby, this will be the base pressure).
  * @param  curr_temp   The current temperature (if in standby, this will be the base temperature).
  * @param  curr_alt    The current altitude (if in standby, this will be the base altitude).
  * @return Boolean
  */
static bool testStandby(int16_t accel, float curr_pres, float curr_temp, float curr_alt)
{
    bool standby_accel = (fabs(accel - 1000) <= STBY_ACCEL_EPSILON);
    bool standby_alt = (fabs(curr_alt - LOCN_ALT) <= MIN_APOGEE_DEPLOY); /* In case we launch on a hill */
    return (standby_accel && standby_alt);
}

/** 
  * @brief  Detects launch.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool detectLaunch(int16_t accel)
{
    return (accel >= SIM_LAUNCH_ACCEL);
}

/** 
  * @brief  Detects burnout.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool detectBurnout(int16_t *prev_accel, int16_t accel)
{
    /* 
    Barometer data is probably not be stable at this point.
    TODO: Accelerometer data may not be either. In that case, we'll just wait ~4s for burnout.
     (depends on rocket though)
     */
    bool burnout = (accel <= *prev_accel);
    *prev_accel = accel;
    return burnout;
}

/** 
  * @brief  Determines whether rocket is nearing apogee.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool nearingApogee(int16_t accel, float base_alt, float curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);

    return (accel <= ACCEL_NEAR_APOGEE && height > MIN_APOGEE_DEPLOY);
}

/** 
  * @brief  Determines whether rocket has passed apogee.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
static bool testApogee(float base_alt, float curr_pres, float *height)
{

    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return ((*height - prev_height) <= 0);
}

/** 
  * @brief  Verifies that rocket's height <= MAIN_DEPLOY_HEIGHT ft.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The current height.
  * @return Boolean
  */
static bool detectMainAlt(float base_alt, float curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);
    float height_in_ft;
    convertToFeet(&height_in_ft, height);

    return (height_in_ft <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta = 0.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
static bool detectLanded(float base_alt, float curr_pres, float *height)
{
    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return (fabs(*height - prev_height) <= EPSILON);
}


/* MAIN ===================================================================================================== */

/** 
  * @brief Apogee Detection board routine - hardware-independent implementation.
  * @return Status
  */
int main()
{
    status_t retval;

    do {
        retval = barometerInit();
    } while (retval != STATUS_OK);
    
    do {
        retval = accelerometerInit();
    } while (retval != STATUS_OK);
    
    SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);

    int ret;

    do {
        ret = sd.init();
    } while (ret != 0);
    
    FATFileSystem fs(sdMountPt, &sd);

    /* RECOVER IN CASE OF BLACKOUT */
    float base_pres;
    float base_temp;
    float base_alt;

    /* For detectBurnout function */
    int16_t bo_det_accel  = 0;

    /* For testApogee function */
    float test_ap_height  = 0;

    /* For detectLanded function */
    float land_det_height = 0;
    
    /* don't mind resetting these */
    int launch_count_arr[ARR_SIZE];
    int burnout_count_arr[ARR_SIZE];
    int coasting_count_arr[ARR_SIZE];
    int apogee_count_arr[ARR_SIZE];
    int main_count_arr[ARR_SIZE];
    int land_count_arr[ARR_SIZE];

    /* is this too much setup in case of a blackout? */
    memset(launch_count_arr,    '\0', ARR_SIZE);
    memset(burnout_count_arr,   '\0', ARR_SIZE);
    memset(coasting_count_arr,  '\0', ARR_SIZE);
    memset(apogee_count_arr,    '\0', ARR_SIZE);
    memset(main_count_arr,      '\0', ARR_SIZE);
    memset(land_count_arr,      '\0', ARR_SIZE);

    /* don't mind resetting these either*/
    int launch_count_idx    = 0;
    int burnout_count_idx   = 0;
    int coasting_count_idx  = 0;
    int apogee_count_idx    = 0;
    int main_count_idx      = 0;
    int land_count_idx      = 0;

    state_t curr_state;
    changeState(&curr_state, APDET_STATE_TESTING);

    while (1) {

        int16_t accel, accel_x, accel_y, accel_z;
        float curr_pres;

        switch(curr_state) {
            case APDET_STATE_TESTING:
                {
                    /* TODO: location calbration with SD (check if space in memory is null) */
                    /* Get acceleration */
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    accelMagnitude(&accel, accel_x, accel_y, accel_z);

                    /* Get pressure and temperature */
                    retval = barometerGetPresTempAndLog(&base_pres, &base_temp);
                    if (retval != STATUS_OK) {
                        break;
                    }

                    /* Calculate altitude */
                    calcAlt(base_pres, &base_alt);
                    if (testStandby(accel, base_pres, base_temp, base_alt)) {
                        writeBaseVars(base_pres, base_temp, base_alt);
                        changeState(&curr_state, APDET_STATE_STANDBY);
                    } else {
                        recoverAll(&curr_state, &base_pres, &base_temp, &base_alt);
                    }
                    break;
                }

            case APDET_STATE_STANDBY:
                {
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    accelMagnitude(&accel, accel_x, accel_y, accel_z);


                    if (detectLaunch(accel)) {
                        launch_count_arr[launch_count_idx] = 1;
                        launch_count_idx = (launch_count_idx + 1) % ARR_SIZE;
                        if (sumArrElems(launch_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_POWERED_ASCENT);
                        }
                    } else {
                        launch_count_arr[launch_count_idx] = 0;
                        launch_count_idx = (launch_count_idx + 1) % ARR_SIZE;
                        //update base values
                        retval = barometerGetPresTempAndLog(&base_pres, &base_temp);
                        if (retval != STATUS_OK) {
                            break;
                        }
                        calcAlt(base_pres, &base_alt);
                        writeBaseVars(base_pres, base_temp, base_alt);
                    }
                    break;
                }

            case APDET_STATE_POWERED_ASCENT:
                {
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    accelMagnitude(&accel, accel_x, accel_y, accel_z);
                    if (detectBurnout(bo_det_accel, accel)) {
                        burnout_count_arr[burnout_count_idx] = 1;
                        if (sumArrElems(burnout_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_COASTING);
                        }
                    } else {
                        burnout_count_arr[burnout_count_idx] = 0;
                    }
                    burnout_count_idx = (burnout_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_COASTING:
                {
                    retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    accelMagnitude(&accel, accel_x, accel_y, accel_z);
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (nearingApogee(accel, base_alt, curr_pres)) {
                        coasting_count_arr[coasting_count_idx] = 1;
                        if (sumArrElems(coasting_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_APOGEE_TESTING);
                        }
                    } else {
                        coasting_count_arr[coasting_count_idx] = 0;
                    }
                    coasting_count_idx = (coasting_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_APOGEE_TESTING:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (testApogee(base_alt, curr_pres, test_ap_height)) {
                        apogee_count_arr[apogee_count_idx] = 1;
                        if (sumArrElems(apogee_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_DEPLOY_DROGUE);
                        }
                    } else {
                        apogee_count_arr[apogee_count_idx] = 0;
                    }
                    apogee_count_idx = (apogee_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_DEPLOY_DROGUE:
                {
                    deployDrogue();
                    wait_ms(IREC_REGULATION_POST_DROGUE_DEPLOYMENT_PAUSE);
                    changeState(&curr_state, APDET_STATE_DEPLOY_PAYLOAD);
                    break;
                }

            case APDET_STATE_DEPLOY_PAYLOAD:
                {
                    deployPayload();
                    changeState(&curr_state, APDET_STATE_INITIAL_DESCENT);
                    wait_ms(PRESSURE_NORMALIZATION_PAUSE);
                    break;
                }

            case APDET_STATE_INITIAL_DESCENT:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectMainAlt(base_alt, curr_pres)) {
                        main_count_arr[main_count_idx] = 1;
                        if (sumArrElems(main_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_DEPLOY_MAIN);
                        }
                    } else {
                        main_count_arr[main_count_idx] = 0;
                    }
                    main_count_idx = (main_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_DEPLOY_MAIN:
                {
                    deployMain();
                    changeState(&curr_state, APDET_STATE_FINAL_DESCENT);
                    break;
                }

            case APDET_STATE_FINAL_DESCENT:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLanded(base_alt, curr_pres, land_det_height)){
                        land_count_arr[land_count_idx] = 1;
                        if (sumArrElems(land_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(&curr_state, APDET_STATE_LANDED);
                        }
                    } else {
                        land_count_arr[land_count_idx] = 0;
                    }
                    land_count_idx = (land_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_LANDED:
                {
                    /* LANDED STATE */
                    break;
                }

            default:
                {
                    /* ERROR STATE */
                    break;
                }
                
        }
    }

    return 0;
}