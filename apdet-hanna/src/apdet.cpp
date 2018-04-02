/*
Hardware-independent functions.
*/
#include "mbed.h"
#include <i2c_driver.h>
#include <general.h>
#include <math.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include <apdet.h>

/* STATIC VARS ============================================================================================== */

Timer timer;
DigitalOut led1(LED1, false);

/* ACTUATORS ================================================================================================ */

/**
  * @brief  Drogue and payload deployment actuator.
  * @return Status
  */
status_t deployDrogueAndPayload()
{
    /* TODO: Actuator */
    /* Logging */
    int timestamp = timer.read_ms();
    if (fprintf(logFP, "[%d] Drogue and payload deployed but not really.\n", timestamp) < 20) {
        return STATUS_ERROR; //must log at least 20 chars
    }
    fflush(logFP);
    return STATUS_OK;
}

/**
  * @brief  Main deployment driver.
  * @return Status
  */
status_t deployMain()
{
    /* TODO: Actuator */
    /* Logging */
    int timestamp = timer.read_ms();
    if (fprintf(logFP, "[%d] Main deployed but not really.\n", timestamp) < 20) {
        return STATUS_ERROR; //must log at least 20 characters
    }
    fflush(logFP);
    return STATUS_OK;
}

/* STATIC HELPER FUNCTIONS ================================================================================== */

/**
  * @brief  Computes altitude based on pressure.
  * @param  curr_pres   The current pressure.
  * @param  alt         A pointer to store the current altitude IN METERS.
  * @return Status
  */
status_t calcAlt(float curr_pres, float *alt)
{
    *alt = (T_0/L) * (pow((curr_pres/P_0),(-L*R/g)) - 1);
    return STATUS_OK;
}

/**
  * @brief  Computes height above ground relative to base altitude.
  * @param  curr_pres   The current pressure.
  * @param  base_alt    The base altitude.
  * @param  height      A pointer to store the current height IN METERS.
  * @return Status
  */
status_t calcHeight(float curr_pres, float base_alt, float *height)
{
    /* Height calculation */
    float curr_alt;
    calcAlt(curr_pres, &curr_alt);
    *height = curr_alt - base_alt;

    /* Logging */
    int timestamp = timer.read_ms();
    if(fprintf(logFP, "[%d] [Height] %0.4f \n", timestamp, *height) < 8){
        return STATUS_ERROR; //must log at least 8 characters
    }
    fflush(logFP);
    return STATUS_OK;
}

/**
  * @brief  Computes acceleration magnitude.
  * @param  accel_x     The x-component of the acceleration.
  * @param  accel_y     The y-component of the acceleration.
  * @param  accel_z     The z-component of the acceleration.
  * @param  accel       A pointer to store the magnitude of the acceleration.
  * @return Status
  */
status_t accelGetMagnitudeAndLog(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t *accel)
{
    *accel = (int16_t)sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

    /* Logging */
    int timestamp = timer.read_ms();
    if (fprintf(logFP, "[%d] [Accel Magnitude] %d\n", *accel, timestamp) < 20) {
        return STATUS_ERROR; //must log at least 20 chars
    }
    fflush(logFP);
    return STATUS_OK;
}

/**
  * @brief  Queries and logs acceleration data from accelerometer.
  * @param  accel_x     A pointer to store the x-component of the acceleration.
  * @param  accel_y     A pointer to store the y-component of the acceleration.
  * @param  accel_z     A pointer to store the z-component of the acceleration.
  * @return Status
  */
status_t accelerometerGetAndLog(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    /* Query data from accelerometer */
    status_t retval = accelerometerGetData(accel_x, accel_y, accel_z);
    if (retval != STATUS_OK) {
        return retval;
    }

    /* Logging */
    int timestamp = timer.read_ms();
    if(fprintf(logFP, "[%d][Accelerometer] X: %d Y: %d Z: %d \n", timestamp, *accel_x, *accel_y, *accel_z) < 10){
        return STATUS_ERROR; //log at least 10 characters
    }
    fflush(logFP);

    return STATUS_OK;
}

/**
  * @brief  Queries and logs pressure and temperature data from barometer.
  * @param  curr_pres   A pointer to store the current pressure.
  * @param  curr_temp   A pointer to store the current temperature.
  * @return Status
  */
status_t barometerGetAndLog(float *curr_pres, float *curr_temp) 
{
    /* Query data from barometer */
    status_t retval = barometerGetCompensatedValues(curr_pres, curr_temp);
    if (retval != STATUS_OK) {
        return retval;
    }

    /* Logging */
    int timestamp = timer.read_ms();
    if(fprintf(logFP, "[%d] [Barometer] Pres: %.4f Temp: %.4f \n", timestamp, *curr_pres, *curr_temp) < 10) {
        return STATUS_ERROR; //log at least 10 characters
    }
    fflush(logFP);

    return STATUS_OK;
}

status_t resetCheckArrAndIdx(int arr[], int size, int *idx) {
    bzero(arr, sizeof(arr[0]) * size);
    *idx = 0;
    return STATUS_OK;
}


/**
  * @brief  Changes state, writes current state to SD and logs state change.
  * @param  state       The state being transitioned to.
  * @param  curr_state  A pointer holding the current state.
  * @return Status
  */
status_t changeStateAndResetChecks(state_t state, state_t *curr_state, int arr[], int size, int *idx) 
{
    /* Change the state */
    *curr_state = state;
    fseek(currStateFP, 0, SEEK_SET); //reset since we want to overwrite the old one
    if (fwrite(curr_state, sizeof(state_t), 1, currStateFP) < 1) {
        return STATUS_ERROR; //make sure we logged the state
    }
    fflush(currStateFP);

    /* Set all array elements to 0 and idx to 0 */
    resetCheckArrAndIdx(arr, size, idx);

    /* Logging */
    int timestamp = timer.read_ms();
    if(fprintf(logFP, "[%d] [State Change] %d\n", timestamp, state) < 10)   {
        return STATUS_ERROR; //log at least 10 characters
    }
    fflush(logFP);
    return STATUS_OK;
}

/**
  * @brief  Writes base variables to SD.
  * @param  base_pres   Pressure at launch site.
  * @param  base_temp   Temperature at launch site.
  * @param  baseVars.base_alt    Launch site altitude.
  * @return Status
  */
status_t writeBaseVars(baseVarStruct baseVars) 
{
    fseek(baseVarsFP, 0, SEEK_SET); //reset to 0 so we overwrite the old vars
    if (fwrite(&baseVars, sizeof(baseVars), 1, baseVarsFP) < 1) {
        return STATUS_ERROR; //must log the struct
    }
    fflush(baseVarsFP);
    return STATUS_OK;
}

/**
  * @brief  Recovers previous state from SD.
  * @param  curr_state  Pointer holding the current state.
  * @return Status
  */
status_t recoverLastState(state_t *curr_state)
{

    //fseek(currStateFP, 0, SEEK_SET); 
    //shouldn't need to set to 0 since the file pointer points back to beginning if we lose power
    if (fread (curr_state, sizeof(state_t), 1, currStateFP) < 1) {
        return STATUS_ERROR; //must read the state (1 item)
    }
    return STATUS_OK;
}

/**
  * @brief  Recovers base variables from SD.
  * @param  base_pres   Pointer holding the pressure at the launch site.
  * @param  base_temp   Pointer holding the temperature at the launch site.
  * @param  baseVars.base_alt    Pointer holding the launch site's altitude.
  * @return Status
  */
status_t recoverBaseVars(baseVarStruct *baseVars) 
{
    if (fread (baseVars, sizeof(baseVarStruct), 1, baseVarsFP) < 1) {
        return STATUS_ERROR; //must read the state (1 item)
    }
    return STATUS_OK;
}

/**
  * @brief  Recovers previous state and base variables from SD.
  * @param  curr_state  Pointer holding the current state.
  * @param  base_pres   Pointer holding the pressure at the launch site.
  * @param  base_temp   Pointer holding the temperature at the launch site.
  * @param  baseVars.base_alt    Pointer holding the launch site's altitude.
  * @return Status
  */
status_t recoverAll(state_t *curr_state, baseVarStruct *baseVars) 
{
    if (recoverLastState(curr_state) != STATUS_OK || 
        recoverBaseVars(baseVars) != STATUS_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

//returns true if a file exists and has 0 size
bool isNullOrEmpty(FILE* fp)
{
    if(fp == NULL) {
        return true;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    return (size == 0);
}

/**
  * @brief  Sums the elements of an array
  * @param  arr[]       Array of integers
  * @param  n           Size of arr[]
  * @return Sum
  */
int sumArrElems(int arr[], int size)
{
    int sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum;
}

float getMedian(float x[], int n) {
    float temp;
    int i, j;
    /* Selection sort */
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
    if(n % 2 == 0) {
        return((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        return x[n/2];
    }
}

/* ROCKET FLIGHT STATE TRANSITION DETECTION FUNCTIONS ======================================================= */

/* 
TODO LIST
- Incorporate Kalman data filtering?
- Make sure a program clears the SD card / base variable file before every flight BUT NOT AFTER A BLACKOUT
- Documentation updates
- Get accelerometer log to find maximum variation of the accelerometer at standby
- Comments in main
- Try to move gets into while before the switch statement for logging purposes
    - for altitudes,
*/

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occurred).
  * @param  accel       The current magnitude of the acceleration.
  * @param  curr_pres   The current pressure (if in standby, this will be the base pressure).
  * @param  curr_temp   The current temperature (if in standby, this will be the base temperature).
  * @param  curr_alt    The current altitude (if in standby, this will be the base altitude).
  * @return Boolean
  */
bool testStandby(int16_t accel, float curr_alt)
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
bool detectLaunch(int16_t accel)
{
    return (accel >= LAUNCH_ACCEL);
}

/** 
  * @brief  Detects burnout.
  * @param  prev_accel  The last recorded magnitude of acceleration.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
bool detectBurnout(int16_t *prev_accel, int16_t accel)
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
  * @param  baseVars.base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @return Boolean
  */
bool nearingApogee(int16_t accel, float base_alt, float curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);

    return (accel <= ACCEL_NEAR_APOGEE && height > MIN_APOGEE_DEPLOY);
}

/** 
  * @brief  Determines whether rocket has passed apogee.
  * @param  baseVars.base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
bool testApogee(float base_alt, float curr_pres, float *height)
{

    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return ((*height - prev_height) <= 0);
}

/** 
  * @brief  Verifies that rocket's height <= MAIN_DEPLOY_HEIGHT ft.
  * @param  baseVars.base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @return Boolean
  */
bool detectMainAlt(float base_alt, float curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);
    float height_in_ft;

    return (height_in_ft <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta ≈ 0.
  * @param  baseVars.base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
bool detectLanded(float base_alt, float curr_pres, float *height)
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
    { //scope retval
        status_t retval;
        do {
            retval = barometerInit();
        } while (retval != STATUS_OK);

        do {
            retval = accelerometerInit();
        } while (retval != STATUS_OK);
    }

    SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);
    FATFileSystem fs(sdMountPt, &sd);

    /* RECOVER IN CASE OF BLACKOUT */

    baseVarStruct baseVars;

    baseVarsFP = fopen(sdBaseVarsPath, "r");

    if (isNullOrEmpty(baseVarsFP)) {
        /* first initialization, not a blackout */
        float base_alt_arr[ARR_SIZE];
        float curr_pres, curr_temp, curr_alt;
        for (int i = 0; i < ARR_SIZE; i++) {
            barometerGetAndLog(&curr_pres, &curr_temp);
            calcAlt(curr_pres, &curr_alt);
            base_alt_arr[i] = curr_alt;
        }
        float median = getMedian(base_alt_arr, ARR_SIZE);
        baseVars.base_alt = median;
    }

    logFP = fopen(logPath, "a");
    baseVarsFP = fopen(sdBaseVarsPath, "w");
    currStateFP = fopen(sdCurrStatePath, "w");

    /* For detectBurnout function */
    int16_t bo_det_accel  = 0;

    /* For testApogee function */
    float test_ap_height  = 0;

    /* For detectLanded function */
    float land_det_height = 0;

    /* state transition checking array */
    int state_change_check_arr[ARR_SIZE];

    /* current index of state transition checking array */
    int state_change_check_idx;

    /* set all array elements to 0 and idx to 0 */
    resetCheckArrAndIdx(state_change_check_arr, ARR_SIZE, &state_change_check_idx);

    state_t curr_state;
    changeStateAndResetChecks(APDET_STATE_TESTING, &curr_state,
      state_change_check_arr, ARR_SIZE, &state_change_check_idx);

    /* turn on LED to indicate initialization has succeeded*/
    led1 = !led1;

    while (1) {

        int16_t accel, accel_x, accel_y, accel_z;
        float curr_pres, curr_temp, curr_alt;

        //move all gets to here 

        switch(curr_state) {
            case APDET_STATE_TESTING:
            {
                    /* TODO: location calbration with SD (check if space in memory is null) */
                    /* Get acceleration */
                status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                if (retval != STATUS_OK) {
                    break;
                }
                accelGetMagnitudeAndLog(accel_x, accel_y, accel_z, &accel);

                    /* Get pressure and temperature */
                retval = barometerGetAndLog(&curr_pres, &curr_temp);
                baseVars.base_pres = curr_pres;
                baseVars.base_temp = curr_temp;
                if (retval != STATUS_OK) {
                    break;
                }

                if (testStandby(accel, baseVars.base_alt)) {

                    /* Update base values */
                    writeBaseVars(baseVars);
                    changeStateAndResetChecks(APDET_STATE_STANDBY, &curr_state,
                      state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                } else {
                    recoverAll(&curr_state, &baseVars);
                }
                break;
            }

            case APDET_STATE_STANDBY:
            {
                    /* Get acceleration */
                status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                if (retval != STATUS_OK) {
                    break;
                }

                /* Calculate magnitude of acceleration */
                accelGetMagnitudeAndLog(accel_x, accel_y, accel_z, &accel);

                if (detectLaunch(accel)) {
                        /* 1 means we have a passing check */
                    state_change_check_arr[state_change_check_idx] = 1;
                        /* increment index with wrap-around */
                    state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_POWERED_ASCENT, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                        /* 0 means we have a failing check */
                    state_change_check_arr[state_change_check_idx] = 0;
                    state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;

                        /* Update pressure and temperature */
                    retval = barometerGetAndLog(&curr_pres, &curr_temp);
                    baseVars.base_pres = curr_pres;
                    baseVars.base_temp = curr_temp;
                    if (retval != STATUS_OK) {
                        break;
                    }

                        /* Update base values */
                    writeBaseVars(baseVars);
                }
                break;
            }

            case APDET_STATE_POWERED_ASCENT:
            {
                status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                if (retval != STATUS_OK) {
                    break;
                }
                accelGetMagnitudeAndLog(accel_x, accel_y, accel_z, &accel);
                
                if (detectBurnout(&bo_det_accel, accel)) {
                    state_change_check_arr[state_change_check_idx] = 1;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_COASTING, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    state_change_check_arr[state_change_check_idx] = 0;
                }
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                break;
            }

            case APDET_STATE_COASTING:
            {
                status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                if (retval != STATUS_OK) {
                    break;
                }
                accelGetMagnitudeAndLog(accel_x, accel_y, accel_z, &accel);
                retval = barometerGetAndLog(&curr_pres, &curr_temp);
                if (retval != STATUS_OK) {
                    break;
                }
                if (nearingApogee(accel, baseVars.base_alt, curr_pres)) {
                    state_change_check_arr[state_change_check_idx] = 1;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_APOGEE_TESTING, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    state_change_check_arr[state_change_check_idx] = 0;
                }
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                break;
            }

            case APDET_STATE_APOGEE_TESTING:
            {
                status_t retval = barometerGetAndLog(&curr_pres, &curr_temp);
                if (retval != STATUS_OK) {
                    break;
                }
                if (testApogee(baseVars.base_alt, curr_pres, &test_ap_height)) {
                    state_change_check_arr[state_change_check_idx] = 1;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_DEPLOY_DROGUE_AND_PAYLOAD, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    state_change_check_arr[state_change_check_idx] = 0;
                }
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                break;
            }

            case APDET_STATE_DEPLOY_DROGUE_AND_PAYLOAD:
            {
                deployDrogueAndPayload();
                changeStateAndResetChecks(APDET_STATE_INITIAL_DESCENT, &curr_state,
                  state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                wait_ms(PRESSURE_NORMALIZATION_PAUSE);
                break;
            }

            case APDET_STATE_INITIAL_DESCENT:
            {
                status_t retval = barometerGetAndLog(&curr_pres, &curr_temp);
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectMainAlt(baseVars.base_alt, curr_pres)) {
                    state_change_check_arr[state_change_check_idx] = 1;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_DEPLOY_MAIN, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    state_change_check_arr[state_change_check_idx] = 0;
                }
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                break;
            }

            case APDET_STATE_DEPLOY_MAIN:
            {
                deployMain();
                changeStateAndResetChecks(APDET_STATE_FINAL_DESCENT, &curr_state,
                  state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                break;
            }

            case APDET_STATE_FINAL_DESCENT:
            {
                status_t retval = barometerGetAndLog(&curr_pres, &curr_temp);
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectLanded(baseVars.base_alt, curr_pres, &land_det_height)){
                    state_change_check_arr[state_change_check_idx] = 1;
                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_LANDED, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    state_change_check_arr[state_change_check_idx] = 0;
                }
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
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
