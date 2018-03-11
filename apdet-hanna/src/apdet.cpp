/*
Hardware-independent functions.
*/
#include "mbed.h"
#include <i2c_driver.h>
#include <general.h>
#include <math.h>


/* CONSTANTS ================================================================================================ */

#define SIM_LAUNCH_ACCEL               40  /* 40 is old value, ask Ollie assuming sims are accurat */
                                           /* should be accel of least accelerating rocket */
#define SIM_BURNOUT_ACCEL_DELTA         4  /* 4s till burnout (data is pretty trash during powered ascent) */
#define ACCEL_NEAR_APOGEE            0.15  /* accel <= 0.15g indicates we are close to apogee */

#define NUM_CHECKS                      5  /* each condition has to pass 5 times */
#define NUM_WRITE_ATTEMPTS              5  /* 5 is temp value, tbd from testing */

#define LOCN_ALT                      785  /* altitude of Hanna, Alberta */

#define P_0                       1013.25  /* pressure at 0 altitude (mb) */
#define T_0                        288.15  /* temperature at 0 altitude (K) */
#define L                         -0.0065  /* lapse rate (valid for heights between 0km and 11km) (K/m) */
#define R                         287.053  /* gas constant for air (J/(kg K))*/
#define g                         9.80665  /* gravitational acceleration (m/s^2) */

/* DRIVERS ================================================================================================== */

/**
  * @brief  Drogue deployment driver.
  * @return Status
  */
extern status_t deployDrogue()
{
    /* TODO */
    return STATUS_OK;
}

/**
  * @brief  Payload deployment driver.
  * @return Status
  */
extern status_t deployPayload()
{
    /* TODO */
    return STATUS_OK;
}

/**
  * @brief  Main deployment driver.
  * @return Status
  */
extern status_t deployMain()
{
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

/* ROCKET FLIGHT STATE TRANSITION DETECTION FUNCTIONS ======================================================= */

/* 
TODO LIST
- Check units:
    - height in m vs ft
    - accel in m/s^2 vs g
- Stop getting data from barometer and accelerometer at certain points:
    - payload deployment
    - sonic boom?
- Incorporate data filtering?
*/

/**
  * @brief  Returns true if rocket is in standby (otherwise assume blackout occured).
  * @param  accel       The current magnitude of the acceleration.
  * @param  curr_pres   The current pressure (if in standby, this will be the base pressure).
  * @param  curr_pres   The current temperature (if in standby, this will be the base temperature).
  * @param  curr_alt    The current altitude (if in standby, this will be the base altitude).
  * @return Boolean
  */
static bool testStandby(int16_t *accel, float *curr_pres, float *curr_temp, float *curr_alt)
{
    barometerGetCompensatedValues(curr_pres, curr_temp);
    calcAlt(curr_pres, curr_alt);
    bool standby_accel = (*accel == 0); /* TODO: error bound? */
    bool standby_alt = (*curr_alt == LOCN_ALT); /* TODO: error bound? */
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
static bool detectBurnout(int16_t *accel)
{
    /* Involve time to double check / as a backup? Check reasonable altitude?
    Data may not be stable at this point */
    return (*accel <= 0);
}

/** 
  * @brief  Determines whether rocket is nearing apogee.
  * @param  accel       The current magnitude of the acceleration.
  * @return Boolean
  */
static bool nearingApogee(int16_t *accel)
{
    /* Add altitude or time backup? */
    return (*accel <= ACCEL_NEAR_APOGEE);
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
    /* check that acceleration is 0 or positive downwards (acc >= 0 ) too? */

    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return (*height <= prev_height);
}

/** 
  * @brief  Verifies that rocket's height <= 3000 ft.
  * @param  base_alt    The base altitude.
  * @param  curr_pres   The current pressure.
  * @param  height      The current height.
  * @return Boolean
  */
static bool detectMainAlt(float *base_alt, float *curr_pres, float *height)
{
    calcHeight(curr_pres, base_alt, height);
    float height_in_ft;
    convertToFeet(&height_in_ft, height);

    return (height_in_ft <= 3000);
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
    float prev_height = *height;
    calcHeight(curr_pres, base_alt, height);

    return (*height == prev_height);
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

    /* RECOVER IN CASE OF BLACKOUT */
    float base_pres;
    float base_temp;
    float base_alt;

    /* For testApogee function */
    float test_ap_height = 0;

    /* For detectLanded function */
    float land_det_height = 0;
    
    /* don't mind resetting these */
    //int test_count = NUM_CHECKS;
    int launch_count = NUM_CHECKS;
    int burnout_count = NUM_CHECKS;
    int coasting_count = NUM_CHECKS;
    int apogee_count = NUM_CHECKS;
    int main_count = NUM_CHECKS;
    int land_count = NUM_CHECKS;

    state_t curr_state = APDET_STATE_TESTING; /* write to and read from flash memory */
    printf("Testing state.");

    while (1) {
        printf("curr_state = %d\n", curr_state);
        switch(curr_state) {
            case APDET_STATE_TESTING:
                {
                    int16_t accel_x, accel_y, accel_z;
                    status_t retval = accelerometerGetData(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    int16_t accel;
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (testStandby(&accel, &base_pres, &base_temp, &base_alt)) {
                    /* TODO: Update/store base_pres, temp and alt in flash */
                        //printf("Still in standby.");
                    } else {
                    /* TODO: Get them from flash memory (assume we already set them earlier) */
                        //printf("Retrieveing state and variables from flash memory.");
                    }
                    break;
                }

            case APDET_STATE_STANDBY:
                {
                    int16_t accel_x, accel_y, accel_z;
                    status_t retval = accelerometerGetData(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    int16_t accel;
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLaunch(&accel)) {
                        launch_count--;
                        if (launch_count <= 0) {
                            curr_state = APDET_STATE_POWERED_ASCENT;
                            //printf("Powered ascent.");
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                        }
                    } else {
                    launch_count = NUM_CHECKS; /* ensures "in a row" */
                    }
                    break;
                }

            case APDET_STATE_POWERED_ASCENT:
                {
                    int16_t accel_x, accel_y, accel_z;
                    status_t retval = accelerometerGetData(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    int16_t accel;
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectBurnout(&accel)) {
                        burnout_count--;
                        if (burnout_count <= 0) {
                            curr_state = APDET_STATE_COASTING;
                            //printf("Coasting.");
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                        }
                    } else {
                        burnout_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_COASTING:
                {
                    int16_t accel_x, accel_y, accel_z;
                    retval = accelerometerGetData(&accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    int16_t accel;
                    retval = accelMagnitude(&accel, &accel_x, &accel_y, &accel_z);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (nearingApogee(&accel)) {
                        coasting_count--;
                        if (coasting_count <= 0) {
                            curr_state = APDET_STATE_APOGEE_TESTING;
                            //printf("Testing apogee.");
                            /* TODO: Save state to flash memory */
                            /* TODO: Write state to SD card using SPI */
                        }
                    } else {
                        coasting_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_APOGEE_TESTING:
                {
                    float curr_pres;
                    status_t retval = barometerGetCompensatedPressure(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (testApogee(&base_alt, &curr_pres, &test_ap_height)) {
                        apogee_count--;
                        if (apogee_count <= 0) {
                            curr_state = APDET_STATE_DEPLOY_DROGUE;
                            //printf("Deploying drogue.");
                            /* TODO: Save state to flash memory */
                            /* TODO: Write state to SD card using SPI */
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
                    curr_state = APDET_STATE_DEPLOY_PAYLOAD;
                    //printf("Deploying payload.");
                    /* TODO: Save state to flash memory */
                    /* TODO: Write state to SD card using SPI */
                    break;
                }

            case APDET_STATE_DEPLOY_PAYLOAD:
                {
                    deployPayload();
                    curr_state = APDET_STATE_INITIAL_DESCENT;
                    //printf("Initial descent.");
                    /* TODO: Save state to flash memory */
                    /* TODO: Write state to SD card using SPI */
                    break;
                }

            case APDET_STATE_INITIAL_DESCENT:
                {
                    float curr_pres;
                    float height = 0;
                    status_t retval = barometerGetCompensatedPressure(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectMainAlt(&base_alt, &curr_pres, &height)) {
                        main_count--;
                        if (main_count <= 0) {
                            curr_state = APDET_STATE_DEPLOY_MAIN;
                            //printf("Deploying main.");
                            /* TODO: Save state to flash memory */
                            /* TODO: Write state to SD card using SPI */
                        }
                    } else {
                        main_count = NUM_CHECKS;
                    }
                    break;
                }

            case APDET_STATE_DEPLOY_MAIN:
                {
                    deployMain();
                    curr_state = APDET_STATE_FINAL_DESCENT;
                    //printf("Final descent.");
                    /* TODO: Save state to flash memory */
                    /* TODO: Write state to SD card using SPI */
                    break;
                }

            case APDET_STATE_FINAL_DESCENT:
                {
                    float curr_pres;
                    status_t retval = barometerGetCompensatedPressure(&curr_pres);
                    if (retval != STATUS_OK) {
                        break;
                    }
                    if (detectLanded(&base_alt, &curr_pres, &land_det_height)){
                        land_count--;
                        if (land_count <= 0) {
                            curr_state = APDET_STATE_LANDED;
                            //printf("Landed.");
                            /* TODO: Save state to flash memory */
                            /* TODO: Write state to SD card using SPI */
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