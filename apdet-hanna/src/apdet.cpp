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
    if (fprintf(logFP, "[%d] [Height] %0.4f \n", timestamp, *height) < 8) {
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
    if(fprintf(logFP, "[%d] [Accelerometer] X: %d Y: %d Z: %d \n", timestamp, *accel_x, *accel_y, *accel_z) < 10){
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
  * @param  base_alt    Launch site altitude.
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
  * @param  base_alt    Pointer holding the launch site's altitude.
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
  * @param  base_alt    Pointer holding the launch site's altitude.
  * @return Status
  */
status_t recoverAll(state_t *curr_state, baseVarStruct *baseVars) 

{
    if (recoverLastState(curr_state) != STATUS_OK || 
        recoverBaseVars(baseVars) != STATUS_OK) {
        return STATUS_ERROR;
    } else {
        return STATUS_OK;
    }
}

//returns true if a file exists and has 0 size
bool isNullOrEmpty(FILE* fp)
{
    if (fp == NULL) {
        return true;
    } else {

        fseek(fp, 0, SEEK_END);
        int size = ftell(fp);

        return (size == 0);
    }
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

float getMedian(float arr[], int size) {
    float temp;
    int i, j;

    /* Selection sort */
    for(i = 0; i < size - 1; i++) {
        for(j = i + 1; j < size; j++) {
            if(arr[j] < arr[i]) {
                temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
    }

    if (size % 2 == 0) {
        return ((arr[size/2] + arr[size/2 - 1]) / 2.0);
    } else {
        return arr[size/2];
    }
}

/* ROCKET FLIGHT STATE TRANSITION DETECTION FUNCTIONS ======================================================= */

/*
TODO LIST
- Incorporate Kalman data filtering?
- Make sure a program clears the SD card / base variable file before every flight BUT NOT AFTER A BLACKOUT
- Documentation updates
- Comments in main
- Review testStandby and LOCN_ALTITUDE
*/

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occurred).
  * @param  accel       The current magnitude of the acceleration.
  * @param  curr_pres   The current pressure (if in standby, this will be the base pressure).
  * @param  curr_temp   The current temperature (if in standby, this will be the base temperature).
  * @param  curr_alt    The current altitude (if in standby, this will be the base altitude).
  * @return Boolean
  */
bool testStandby(int16_t accel, float curr_height)
{
    bool standby_accel = (fabs(accel - 1000) <= STBY_ACCEL_EPSILON);
    bool standby_height = (curr_height <= EPSILON);

    return (standby_accel && standby_height);
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
    TODO: Accelerometer data may not be stable either. In that case, we'll just wait ~4s for burnout.
     (depends on rocket though)
     */
    bool burnout = (accel <= *prev_accel);
    *prev_accel = accel;

    return burnout;
}

/**
  * @brief  Determines whether rocket is nearing apogee.
  * @param  accel       The current magnitude of the acceleration.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @return Boolean
  */
bool nearingApogee(int16_t accel, float base_alt, float curr_pres, float height)
{
    return (accel <= ACCEL_NEAR_APOGEE && height > MIN_APOGEE_DEPLOY);
}

/**
  * @brief  Determines whether rocket has passed apogee.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
bool testApogee(float base_alt, float curr_pres, float *prev_height, float height)
{
    bool apogee = ((height - *prev_height) <= 0);
    *prev_height = height;

    return apogee;
}

/**
  * @brief  Verifies that rocket's height <= MAIN_DEPLOY_HEIGHT ft.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @return Boolean
  */
bool detectMainAlt(float base_alt, float curr_pres, float height)
{
    return (height <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta ≈ 0.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
bool detectLanded(float base_alt, float curr_pres, float *prev_height, float height)
{
    bool landed = ((height - *prev_height) <= EPSILON);
    *prev_height = height;

    return landed;
}

/* MAIN ===================================================================================================== */

/**
  * @brief Apogee Detection board routine - hardware-independent implementation.
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
    } else {
        /* Recover altitude only */
        baseVarStruct temp;
        recoverBaseVars(&temp);
        baseVars.base_alt = temp.base_alt;
    }

    logFP = fopen(logPath, "a");
    baseVarsFP = fopen(sdBaseVarsPath, "w");
    currStateFP = fopen(sdCurrStatePath, "w");

    /* To store previous accel in detectBurnout function */
    int16_t bo_det_accel  = 0;

    /* To store previous height in testApogee function */
    float test_ap_height  = 0;

    /* To store previous height in detectLanded function */
    float land_det_height = 0;

    /* State transition checking array */
    int state_change_check_arr[ARR_SIZE];

    /* Current index of state transition checking array */
    int state_change_check_idx;

    /* Set all state_change_check_arr elements to 0 and state_change_check_idx to 0 */
    resetCheckArrAndIdx(state_change_check_arr, ARR_SIZE, &state_change_check_idx);

    /* Enter state machine into APDET_STATE_TESTING state */
    state_t curr_state;
    changeStateAndResetChecks(APDET_STATE_TESTING, &curr_state,
      state_change_check_arr, ARR_SIZE, &state_change_check_idx);

    /* Turn on LED to indicate initialization has succeeded*/
    led1 = !led1;

    while (1) {

        int16_t accel, accel_x, accel_y, accel_z;
        float curr_pres, curr_temp, curr_height;
        
        status_t retval;

        /* Get acceleration */
        retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
        if (retval != STATUS_OK) {
            break;
        }

        /* Get acceleration magnitude */
        retval = accelGetMagnitudeAndLog(accel_x, accel_y, accel_z, &accel);
        if (retval != STATUS_OK) {
            break;
        }

        /* Get pressure and temperature */
        retval = barometerGetAndLog(&curr_pres, &curr_temp);
        if (retval != STATUS_OK) {
            break;
        }

        /* Get current relative altitude (height above ground) */
        calcHeight(curr_pres, baseVars.base_alt, &curr_height);


        switch(curr_state) {

            case APDET_STATE_TESTING:
            {
                if (testStandby(accel, curr_height)) {
                    /* Update and write pressure and temperature */
                    baseVars.base_pres = curr_pres;
                    baseVars.base_temp = curr_temp;
                    writeBaseVars(baseVars);
                    /* Change state */
                    changeStateAndResetChecks(APDET_STATE_STANDBY, &curr_state,
                      state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                } else {
                    /* recover base variables and state */
                    recoverAll(&curr_state, &baseVars);
                }
                break;
            }

            case APDET_STATE_STANDBY:
            {
                if (detectLaunch(accel)) {

                    /* 1 means we have a passing check */
                    state_change_check_arr[state_change_check_idx] = 1;

                    if (sumArrElems(state_change_check_arr, ARR_SIZE) >= NUM_CHECKS) {
                        changeStateAndResetChecks(APDET_STATE_POWERED_ASCENT, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    /* 0 means we have a failing check */
                    state_change_check_arr[state_change_check_idx] = 0;

                    /* Update and write base values */
                    baseVars.base_pres = curr_pres;
                    baseVars.base_temp = curr_temp;
                    writeBaseVars(baseVars);
                }
                /* increment index with wrap-around */
                state_change_check_idx = (state_change_check_idx + 1) % ARR_SIZE;
                break;
            }

            case APDET_STATE_POWERED_ASCENT:
            {
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
                if (nearingApogee(accel, baseVars.base_alt, curr_pres, curr_height)) {
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
                if (testApogee(baseVars.base_alt, curr_pres, &test_ap_height, curr_height)) {
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
                if (detectMainAlt(baseVars.base_alt, curr_pres, curr_height)) {
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
                if (detectLanded(baseVars.base_alt, curr_pres, &land_det_height, curr_height)){
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
