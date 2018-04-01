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

/* ACTUATORS ================================================================================================ */

/**
  * @brief  Drogue and payload deployment actuator.
  * @return Status
  */
extern status_t deployDrogueAndPayload()
{
    /* TODO: Actuator */
    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] Drogue and payload deployed but not really.\n", timestamp);
    fclose(pFile);
}

/**
  * @brief  Main deployment driver.
  * @return Status
  */
extern status_t deployMain()
{
    /* TODO: Actuator */
    /* Logging */
    int timestamp = timer.read_ms();
    FILE *pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] Main deployed but not really.\n", timestamp);
    fclose(pFile);
    return STATUS_OK;
}

/* STATIC HELPER FUNCTIONS ================================================================================== */

/**
  * @brief  Computes altitude based on pressure.
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
  * @brief  Computes height above ground relative to base altitude.
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

/**
  * @brief  Computes height above ground relative to base altitude.
  * @param  height_in_m   The current height in meters.
  * @param  height_in_ft  A pointer to store the current height in feet.
  * @return Status
  */
extern status_t convertToFeet(float height_in_m, float *height_in_ft)
{
    *height_in_ft = height_in_m * 3.28084;
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
extern status_t accelMagnitude(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t *accel)
{
    *accel = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));
    return STATUS_OK;
}

/**
  * @brief  Queries and logs acceleration data from accelerometer.
  * @param  accel_x     A pointer to store the x-component of the acceleration.
  * @param  accel_y     A pointer to store the y-component of the acceleration.
  * @param  accel_z     A pointer to store the z-component of the acceleration.
  * @return Status
  */
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

/**
  * @brief  Queries and logs pressure data from barometer.
  * @param  curr_pres   A pointer to store the current pressure.
  * @return Status
  */
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

/**
  * @brief  Queries and logs pressure and temperature data from barometer.
  * @param  curr_pres   A pointer to store the current pressure.
  * @param  curr_temp   A pointer to store the current temperature.
  * @return Status
  */
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

/**
  * @brief  Changes state, writes current state to SD and logs state change.
  * @param  state       The state being transitioned to.
  * @param  curr_state  A pointer holding the current state.
  * @return Status
  */
extern status_t changeState(state_t state, state_t *curr_state) 
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
    pFile = fopen(logPath, "a");
    fprintf(pFile, "[%d] [State Change] %d\n", timestamp, (int8_t)state);
    fclose(pFile);
    return STATUS_OK;
}

/**
  * @brief  Writes base variables to SD.
  * @param  base_pres   Pressure at launch site.
  * @param  base_temp   Temperature at launch site.
  * @param  base_alt    Launch site altitude.
  * @return Status
  */
extern status_t writeBaseVars(float base_pres, float base_temp, float base_alt) 
{
    float buffer[] = {base_pres, base_temp, base_alt};
    FILE *pFile = fopen(sdBaseVarsPath, "w");
    fseek(pFile, 0, SEEK_SET);
    fwrite(buffer, sizeof(float), sizeof(buffer), pFile);
    fclose(pFile);
    return STATUS_OK;
}

/**
  * @brief  Recovers previous state from SD.
  * @param  curr_state  Pointer holding the current state.
  * @return Status
  */
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

/**
  * @brief  Recovers base variables from SD.
  * @param  base_pres   Pointer holding the pressure at the launch site.
  * @param  base_temp   Pointer holding the temperature at the launch site.
  * @param  base_alt    Pointer holding the launch site's altitude.
  * @return Status
  */
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

/**
  * @brief  Recovers previous state and base variables from SD.
  * @param  curr_state  Pointer holding the current state.
  * @param  base_pres   Pointer holding the pressure at the launch site.
  * @param  base_temp   Pointer holding the temperature at the launch site.
  * @param  base_alt    Pointer holding the launch site's altitude.
  * @return Status
  */
extern status_t recoverAll(state_t *curr_state, float *base_pres, float *base_temp, float *base_alt) 
{
    recoverLastState(curr_state);
    recoverBaseVars(base_pres, base_temp, base_alt);
    return STATUS_OK;
}

/**
  * @brief  Sums the elements of an array
  * @param  arr[]       Array of integers
  * @param  n           Size of arr[]
  * @return Sum
  */
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
- Incorporate Kalman data filtering?
- Make sure a program clears the SD card / base variable file before every flight BUT NOT AFTER A BLACKOUT
- Documentation updates
- Get accelerometer log to find maximum variation of the accelerometer at standby
- Comments in main
- Try to move gets into while before the switch statement for logging purposes
- Find out how long it takes for pressure to normalize in the rocket (waits)
- Refactor state transition functions using comparison with prev values
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
    return (accel >= LAUNCH_ACCEL);
}

/** 
  * @brief  Detects burnout.
  * @param  prev_accel  The last recorded magnitude of acceleration.
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
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
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
  * @return Boolean
  */
static bool detectMainAlt(float base_alt, float curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);
    float height_in_ft;
    convertToFeet(height, &height_in_ft);

    return (height_in_ft <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta ≈ 0.
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
    changeState(APDET_STATE_TESTING, &curr_state);

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
                    accelMagnitude(accel_x, accel_y, accel_z, &accel);

                    /* Get pressure and temperature */
                    retval = barometerGetPresTempAndLog(&base_pres, &base_temp);
                    if (retval != STATUS_OK) {
                        break;
                    }

                    /* Calculate base altitude */
                    calcAlt(base_pres, &base_alt);

                    if (testStandby(accel, base_pres, base_temp, base_alt)) {

                        /* Update base values */
                        writeBaseVars(base_pres, base_temp, base_alt);
                        changeState(APDET_STATE_STANDBY, &curr_state);
                    } else {
                        recoverAll(&curr_state, &base_pres, &base_temp, &base_alt);
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
                    accelMagnitude(accel_x, accel_y, accel_z, &accel);

                    if (detectLaunch(accel)) {
                        launch_count_arr[launch_count_idx] = 1;
                        launch_count_idx = (launch_count_idx + 1) % ARR_SIZE;
                        if (sumArrElems(launch_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(APDET_STATE_POWERED_ASCENT, &curr_state);
                        }
                    } else {
                        launch_count_arr[launch_count_idx] = 0;
                        launch_count_idx = (launch_count_idx + 1) % ARR_SIZE;

                        /* Get pressure and temperature */
                        retval = barometerGetPresTempAndLog(&base_pres, &base_temp);
                        if (retval != STATUS_OK) {
                            break;
                        }

                        /* Calculate base altitude */
                        calcAlt(base_pres, &base_alt);

                        /* Update base values */
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
                    accelMagnitude(accel_x, accel_y, accel_z, &accel);
                    if (detectBurnout(&bo_det_accel, accel)) {
                        burnout_count_arr[burnout_count_idx] = 1;
                        if (sumArrElems(burnout_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(APDET_STATE_COASTING, &curr_state);
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
                    accelMagnitude(accel_x, accel_y, accel_z, &accel);
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (nearingApogee(accel, base_alt, curr_pres)) {
                        coasting_count_arr[coasting_count_idx] = 1;
                        if (sumArrElems(coasting_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(APDET_STATE_APOGEE_TESTING, &curr_state);
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
                    if (testApogee(base_alt, curr_pres, &test_ap_height)) {
                        apogee_count_arr[apogee_count_idx] = 1;
                        if (sumArrElems(apogee_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(APDET_STATE_DEPLOY_DROGUE_AND_PAYLOAD, &curr_state);
                        }
                    } else {
                        apogee_count_arr[apogee_count_idx] = 0;
                    }
                    apogee_count_idx = (apogee_count_idx + 1) % ARR_SIZE;
                    break;
                }

            case APDET_STATE_DEPLOY_DROGUE_AND_PAYLOAD:
                {
                    deployDrogueAndPayload();
                    changeState(APDET_STATE_INITIAL_DESCENT, &curr_state);
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
                            changeState(APDET_STATE_DEPLOY_MAIN, &curr_state);
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
                    changeState(APDET_STATE_FINAL_DESCENT, &curr_state);
                    break;
                }

            case APDET_STATE_FINAL_DESCENT:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLanded(base_alt, curr_pres, &land_det_height)){
                        land_count_arr[land_count_idx] = 1;
                        if (sumArrElems(land_count_arr, ARR_SIZE) >= NUM_CHECKS) {
                            changeState(APDET_STATE_LANDED, &curr_state);
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