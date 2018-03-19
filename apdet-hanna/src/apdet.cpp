/*
Hardware-independent functions.
*/
#include "mbed.h"
#include <i2c_driver.h>
#include <general.h>
#include <math.h>


/* CONSTANTS ================================================================================================ */

#define SIM_LAUNCH_ACCEL               40  /* 40 is old value, ask Ollie assuming sims are accurate, units? */
                                           /* should be accel of least accelerating rocket */
#define SIM_BURNOUT_ACCEL_DELTA         4  /* 4s till burnout (data is pretty trash during powered ascent) */
#define ACCEL_NEAR_APOGEE            0.15  /* accel magnitude <= 0.15g indicates we are close to apogee */
#define MAIN_DEPLOY_HEIGHT           1500  /* height at which we deploy main (ft) */
#define MIN_APOGEE_DEPLOY              65  /* height above which we want to deploy drogue, payload and main (in ft) */

#define NUM_CHECKS                      5  /* each condition has to pass 5 times */
#define NUM_WRITE_ATTEMPTS              5  /* 5 is temp value, tbd from testing */

#define LOCN_ALT                      785  /* altitude of Hanna, Alberta */
#define EPSILON                     0.005  /* */

#define P_0                       1013.25  /* pressure at 0 altitude (mb) */
#define T_0                        288.15  /* temperature at 0 altitude (K) */
#define L                         -0.0065  /* lapse rate (valid for heights between 0km and 11km) (K/m) */
#define R                         287.053  /* gas constant for air (J/(kg K))*/
#define g                         9.80665  /* gravitational acceleration (m/s^2) */

//TODO: define memory constants for flash stuff

/* DRIVERS ================================================================================================== */

/**
  * @brief  Drogue deployment driver.
  * @return Status
  */
extern status_t deployDrogue()
{
    /* TODO */
    printf("Drogue deployed.\n");
    return STATUS_OK;
}

/**
  * @brief  Payload deployment driver.
  * @return Status
  */
extern status_t deployPayload()
{
    /* TODO */
    printf("Payload deployed.\n");
    return STATUS_OK;
}

/**
  * @brief  Main deployment driver.
  * @return Status
  */
extern status_t deployMain()
{
    printf("Main deployed.\n");
    /* TODO */
    return STATUS_OK;
}

/* STATIC HELPER FUNCTIONS ================================================================================== */

/**
  * @brief  Altitude calculator.
  * @param  curr_pres   The current pressure.
  * @param  alt         A pointer to store the current altitude IN METERS.
  * @return Status
  */
extern status_t calcAlt(float *curr_pres, float *alt)
{
    *alt = (T_0/L) * (pow((*curr_pres/P_0),(-L*R/g)) - 1);

    return STATUS_OK;
}

/**
  * @brief  Height above ground (base altitude) calculator.
  * @param  curr_pres   The current pressure.
  * @param  base_alt    The base altitude.
  * @param  height      A pointer to store the current height IN METERS.
  * @return Status
  */
extern status_t calcHeight(float *curr_pres, float *base_alt, float *height)
{
    float curr_alt;
    calcAlt(curr_pres, &curr_alt);
    *height = curr_alt - *base_alt;

    return STATUS_OK;
}

extern status_t convertToFeet(float *height_in_ft, float *height_in_m)
{
	*height_in_ft = (*height_in_m) * 3.28084;
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
extern status_t accelMagnitude(int16_t *accel, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
	*accel = sqrt(pow(*accel_x,2)+pow(*accel_y,2)+pow(*accel_z,2));

	return STATUS_OK;
}

extern status_t accelerometerGetAndLog(&accel_x, &accel_y, &accel_z) {
    retval = accelerometerGetData(&accel_x, &accel_y, &accel_z);
    /* TODO: Log accel_x, accel_y, accel_z to logging file */
    return retval;
}

extern status_t barometerGetAndLog(&curr_pres) {
    retval = barometerGetCompensatedPressure(&curr_pres);
    /* TODO: Log curr_pres to logging file */
    return retval;
}

extern status_t barometerGetPresTempAndLog(&curr_pres) {
    retval = barometerGetCompensatedPressure(&curr_pres);
    /* TODO: Log curr_pres to logging file */
    return retval;
}

extern void changeState(state_t *curr_state, state_t state) {
    *curr_state = state;
    /* TODO: Log curr_state to state file */
}


/* ROCKET FLIGHT STATE TRANSITION DETECTION FUNCTIONS ======================================================= */

/* 
TODO LIST
- Check units:
    - height in m vs ft
    - accel in m/s^2 vs g
- Incorporate Kalman data filtering?
- Make sure a program clears the SD card / base variable file before every flight BUT NOT AFTER A BLACKOUT
- Implement 7/11 split checks
- Documentation updates
- Make wrapper for sensor query functions to also log sensor data whenever I query it
- Make wrapper to change states and write to SD
- Three files: state file, logging file and base variables file
*/

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occured).
  * @param  accel       The current magnitude of the acceleration.
  * @param  curr_pres   The current pressure (if in standby, this will be the base pressure).
  * @param  curr_temp   The current temperature (if in standby, this will be the base temperature).
  * @param  curr_alt    The current altitude (if in standby, this will be the base altitude).
  * @return Boolean
  */
static bool testStandby(int16_t *accel, float *curr_pres, float *curr_temp, float *curr_alt)
{
    barometerGetCompensatedValues(curr_pres, curr_temp);
    /* TODO: log */
    calcAlt(curr_pres, curr_alt); 
    bool standby_accel = (fabs(*accel - g) <= EPSILON);
    bool standby_alt = (fabs(*curr_alt - LOCN_ALT) < MIN_APOGEE_DEPLOY); /* In case we launch on a hill */
    return (standby_alt && standby_accel);
}

/** 
  * @brief  Detects launch.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool detectLaunch(int16_t *accel)
{
    return (*accel >= SIM_LAUNCH_ACCEL);
}

/** 
  * @brief  Detects burnout.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool detectBurnout(int16_t *prev_accel, int16_t *accel)
{
    /* Barometer data is not be stable at this point. */
    if (*accel <= *prev_accel) {
        return true;
    } else {
        *prev_accel = *accel;
        return false;
    }
}

/** 
  * @brief  Determines whether rocket is nearing apogee.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool nearingApogee(int16_t *accel, float *base_alt, float *curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);

    return (*accel <= ACCEL_NEAR_APOGEE && height > MIN_APOGEE_DEPLOY);
}

/** 
  * @brief  Determines whether rocket has passed apogee.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
static bool testApogee(float *base_alt, float *curr_pres, float *height)
{

    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return ((*height - prev_height) <= 0);
}

/** 
  * @brief  Verifies that rocket's height <= 1500 ft.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The current height.
  * @return Boolean
  */
static bool detectMainAlt(float *base_alt, float *curr_pres)
{
    float height;
    calcHeight(curr_pres, base_alt, &height);
    float height_in_ft;
    convertToFeet(&height_in_ft, &height);

    return (height_in_ft <= MAIN_DEPLOY_HEIGHT);
}

/** 
  * @brief  Detects landing by checking that altitude delta = 0.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The last recorded height.
  * @return Boolean
  */
static bool detectLanded(float *base_alt, float *curr_pres, float *height)
{
    float prev_height = *height; //threshold floats, you'll never get equality, too precise
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

    do {
        retval = sd.init();
    } while (retval != 0);

    /* RECOVER IN CASE OF BLACKOUT */
    float base_pres;
    float base_temp;
    float base_alt;

    /* For detectBurnout function */
    float bo_det_accel = 0;

    /* For testApogee function */
    float test_ap_height = 0;

    /* For detectLanded function */
    float land_det_height = 0;
    
    /* don't mind resetting these */
    int launch_count = NUM_CHECKS;
    int burnout_count = NUM_CHECKS;
    int coasting_count = NUM_CHECKS;
    int apogee_count = NUM_CHECKS;
    int main_count = NUM_CHECKS;
    int land_count = NUM_CHECKS;

    state_t curr_state;
    changeState(&curr_state, APDET_STATE_TESTING);
    //printf("Entering testing state.");

    while (1) {
        //printf("curr_state = %d\n", curr_state);

        int16_t accel_x, accel_y, accel_z;
        int16_t accel;
        float curr_pres;

        switch(curr_state) {
            case APDET_STATE_TESTING:
                {
                    /* TODO: location calbration with SD (check if space in memory is null) */
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (testStandby(&accel, &base_pres, &base_temp, &base_alt)) {
                        /* TODO: Update/store base_pres, temp and alt in flash */
                        changeState(&curr_state, APDET_STATE_STANDBY);
                    } else {
                        /* TODO: Get them from flash memory (assume we already set them earlier) */
                        //printf("Retrieveing state and variables from flash memory.");
                        
                    }
                    break;
                }

            case APDET_STATE_STANDBY:
                {
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLaunch(&accel)) {
                        launch_count--;
                        if (launch_count <= 0) {
                            changeState(&curr_state, APDET_STATE_POWERED_ASCENT);
                        }
                    } else {
                        launch_count = NUM_CHECKS; /* ensures "in a row" */
                        /* TODO: Update/store base_pres, temp and alt in flash */
                    }
                    break;
                }

            case APDET_STATE_POWERED_ASCENT:
                {
                    status_t retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectBurnout(&bo_det_accel, &accel)) {
                        burnout_count--;
                        if (burnout_count <= 0) {
                            changeState(&curr_state, APDET_STATE_COASTING);
                        }
                    } else {
                        burnout_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_COASTING:
                {
                    retval = accelerometerGetAndLog(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (nearingApogee(&accel, &base_alt, &curr_pres)) {
                        coasting_count--;
                        if (coasting_count <= 0) {
                            changeState(&curr_state, APDET_STATE_APOGEE_TESTING);
                        }
                    } else {
                        coasting_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_APOGEE_TESTING:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (testApogee(&base_alt, &curr_pres, &test_ap_height)) {
                        apogee_count--;
                        if (apogee_count <= 0) {
                            changeState(&curr_state, APDET_STATE_DEPLOY_DROGUE);
                        }
                    } else {
                        apogee_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_DEPLOY_DROGUE:
                {
                    deployDrogue();
                    wait_ms(3000);
                    changeState(&curr_state, APDET_STATE_DEPLOY_PAYLOAD);
                    break;
                }

            case APDET_STATE_DEPLOY_PAYLOAD:
                {
                    deployPayload();
                    changeState(&curr_state, APDET_STATE_INITIAL_DESCENT);
                    wait_ms(5000);
                    break;
                }

            case APDET_STATE_INITIAL_DESCENT:
                {
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectMainAlt(&base_alt, &curr_pres)) {
                        main_count--;
                        if (main_count <= 0) {
                            changeState(&curr_state, APDET_STATE_DEPLOY_MAIN);
                        }
                    } else {
                        main_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_DEPLOY_MAIN:
                {
                    deployMain();
                    changeState(&curr_state, APDET_STATE_FINAL_DESCENT);
                    wait_ms(2000); /* to satisfy Rai's paranoia */
                    break;
                }

            case APDET_STATE_FINAL_DESCENT:
                {
                    float curr_pres;
                    status_t retval = barometerGetAndLog(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLanded(&base_alt, &curr_pres, &land_det_height)){
                        land_count--;
                        if (land_count <= 0) {
                            changeState(&curr_state, APDET_STATE_LANDED);
                        }
                    } else {
                        land_count = NUM_CHECKS;
                    }
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