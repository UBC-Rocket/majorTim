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


/*
TODO LIST
- Incorporate Kalman data filtering?
- Make sure the SD card / base variable file is reformatted/cleared before every flight
- Timer backup (ask Joren for state times)
- Tweak ACCEL_NEAR_APOGEE
- Open with binary or not? Test this (replace w with wb)
- Set up serial
- Close file before opening in another mode?
*/

/* STATIC VARS ============================================================================================== */

Timer timer;

DigitalOut led(LED, 0);
DigitalOut ig1(IG1, 0);
DigitalOut ig1_test_out(IG1_TEST_OUT, 1);
DigitalIn ig1_test_in(IG1_TEST_IN);
DigitalOut ig2(IG2, 0);
DigitalOut ig2_test_out(IG2_TEST_OUT, 1);
DigitalIn ig2_test_in(IG2_TEST_IN);

/* ACTUATORS ================================================================================================ */

/**
  * @brief  Drogue and payload deployment actuator.
  * @return Status
  */
status_t deployDrogueAndPayload()
{
    /* Signal to the ignitor */
    ig1.write(1);
    /* Logging */
    int timestamp = timer.read_ms();
    if (fprintf(logFP, "[%d] Drogue and payload deployed.\n", timestamp) < 20) {
        return STATUS_ERROR; //must log at least 20 chars
    }
    fflush(logFP);
    return STATUS_OK;
}

/**
  * @brief  Main deployment actuator.
  * @return Status
  */
status_t deployMain()
{
    /* Signal to the ignitor */
    ig2.write(1);
    /* Logging */
    int timestamp = timer.read_ms();
    if (fprintf(logFP, "[%d] Main deployed.\n", timestamp) < 20) {
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
  * @param  accel_x     The x-component of acceleration.
  * @param  accel_y     The y-component of acceleration.
  * @param  accel_z     The z-component of acceleration.
  * @param  accel       A pointer to store the magnitude of acceleration.
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
  * @param  accel_x     A pointer to store the x-component of acceleration.
  * @param  accel_y     A pointer to store the y-component of acceleration.
  * @param  accel_z     A pointer to store the z-component of acceleration.
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
  * @param  base_Vars   Struct containing base variables.
  * @return Status
  */
status_t writeBaseVars(baseVarStruct baseVars) 
{
    fseek(baseVarsFP, 0, SEEK_SET); //reset to 0 so we overwrite the old vars
    //TODO: is this necessary if we opened the file to "write" mode?
    if (fwrite(&baseVars, sizeof(baseVars), 1, baseVarsFP) < 1) {
        return STATUS_ERROR; //must log the struct
    }
    fflush(baseVarsFP);
    return STATUS_OK;
}

/**
  * @brief  Recovers previous state from SD.
  * @param  curr_state  Pointer to store the current state.
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
  * @param  base_Vars   Pointer to store the base variables.
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
  * @param  curr_state  Pointer to store the current state.
  * @param  base_Vars   Pointer to store the base variables.
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

/**
  * @brief  Determines if a file has been previously created and/or written to.
  * @param  fp          Pointer to a file
  * @return Boolean
  */
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
  * @brief  Sums the elements of an array.
  * @param  arr[]       Array of integers
  * @param  size        Size of arr[]
  * @return int
  */
int sumArrElems(int arr[], int size)
{
    int sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum;
}

/**
  * @brief  Sorts an array and computes the median of its elements.
  * @param  arr[]       Array of floats
  * @param  size        Size of arr[]
  * @return float
  */
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

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occurred).
  * @param  accel       The current magnitude of acceleration.
  * @param  height      The current height relative to the ground.
  * @return Boolean
  */
bool testStandby(int16_t accel, float height)
{
    bool standby_accel = (fabs(accel - 1000) <= STBY_ACCEL_EPSILON);
    bool standby_height = (height <= EPSILON);

    return (standby_accel && standby_height);
}

/**
  * @brief  Detects launch.
  * @param  accel       The current magnitude of acceleration.
  * @return Boolean
  */
bool detectLaunch(int16_t accel)
{
    return (accel >= LAUNCH_ACCEL);
}

/**
  * @brief  Detects burnout.
  * @param  prev_accel  The last recorded magnitude of acceleration.
  * @param  accel       The current magnitude of acceleration.
  * @return Boolean
  */
bool detectBurnout(int16_t *prev_accel, int16_t accel)
{
    /*
    Barometer data is probably not stable at this point.
    TODO: Accelerometer data may not be stable either. In that case, we'll just wait 
    ~7s for burnout in 10k rockets and (TODO) for 30k rockets
     */
    bool burnout = (accel <= *prev_accel);
    *prev_accel = accel;

    return burnout;
}

/**
  * @brief  Determines whether rocket is nearing apogee.
  * @param  accel       The current magnitude of acceleration.
  * @param  height      The current height.
  * @return Boolean
  */
bool nearingApogee(int16_t accel, float height)
{
    return (accel <= ACCEL_NEAR_APOGEE && height > MIN_APOGEE_DEPLOY);
}

/**
  * @brief  Determines whether rocket has passed apogee.
  * @param  prev_height The last recorded height.
  * @param  height      The current height.
  * @return Boolean
  */
bool testApogee(float *prev_height, float height)
{
    bool at_apogee = ((height - *prev_height) <= 0);
    *prev_height = height;

    return at_apogee;
}

/**
  * @brief  Verifies that rocket's height <= MAIN_DEPLOY_HEIGHT ft.
  * @param  height      The current height.
  * @return Boolean
  */
bool detectMainAlt(float height)
{
    return (height <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta ≈ 0.
  * @param  prev_height The last recorded height.
  * @param  height      The current height.
  * @return Boolean
  */
bool detectLanded(float *prev_height, float height)
{
    bool landed = ((height - *prev_height) <= EPSILON);
    *prev_height = height;

    return landed;
}

/* MAIN ===================================================================================================== */

/**
  * @brief Apogee Detection board routine.
  */
int main()
{
    { /* scope retval */
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

    /* Open or create logging file in append mode */
    logFP = fopen(logPath, "a");

    baseVarStruct baseVars;

    baseVarsFP = fopen(sdBaseVarsPath, "r+");

    if (isNullOrEmpty(baseVarsFP)) {
        /* First initialization, not a blackout */
        /* Get base altitude */
        float base_alt_arr[ARR_SIZE];
        float curr_pres, curr_temp, curr_alt;
        for (int i = 0; i < ARR_SIZE; i++) {
            barometerGetAndLog(&curr_pres, &curr_temp);
            calcAlt(curr_pres, &curr_alt);
            base_alt_arr[i] = curr_alt;
        }
        float median = getMedian(base_alt_arr, ARR_SIZE);
        baseVars.base_alt = median;
        baseVars.base_pres = curr_pres;
        baseVars.base_temp = curr_temp;
        writeBaseVars(baseVars);
    } else {
        /* Recover altitude only */
        baseVarStruct temp;
        recoverBaseVars(&temp);
        baseVars.base_alt = temp.base_alt;
    }

    currStateFP = fopen(sdCurrStatePath, "r+");

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
    state_t enter_state = APDET_STATE_TESTING;
    changeStateAndResetChecks(enter_state, &curr_state,
      state_change_check_arr, ARR_SIZE, &state_change_check_idx);

    /* turn on LED to indicate initialization has succeeded */
    led.write(1);

    while (1) {

        int16_t accel, accel_x, accel_y, accel_z;
        float curr_pres, curr_temp, curr_height;
        
        status_t retval;

        // logFP = fopen(logPath, "a");
        // baseVarsFP = fopen(sdBaseVarsPath, "r+");
        // currStateFP = fopen(sdCurrStatePath, "r+");

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
                    /* Change state and log */
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
                        /* Change state, log, and reset state change checking array */
                        changeStateAndResetChecks(APDET_STATE_POWERED_ASCENT, &curr_state,
                          state_change_check_arr, ARR_SIZE, &state_change_check_idx);
                    }
                } else {
                    /* 0 means we have a failing check */
                    state_change_check_arr[state_change_check_idx] = 0;

                    /* Update base altitude (rolling median) and base pres and temp */
                    float base_alt_arr[ARR_SIZE];
                    /* Insert old base altitude as first data point */
                    base_alt_arr[0] = baseVars.base_alt;
                    float curr_alt; /* curr_pres and curr_temp declared in while loop body */
                    for (int i = 1; i < ARR_SIZE; i++) {
                        barometerGetAndLog(&curr_pres, &curr_temp);
                        calcAlt(curr_pres, &curr_alt);
                        base_alt_arr[i] = curr_alt;
                    }
                    float median = getMedian(base_alt_arr, ARR_SIZE);

                    /* Update and write base values */
                    baseVars.base_alt = median;
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
                if (nearingApogee(accel, curr_height)) {
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
                if (testApogee(&test_ap_height, curr_height)) {
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
                if (detectMainAlt(curr_height)) {
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
                if (detectLanded(&land_det_height, curr_height)){
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
