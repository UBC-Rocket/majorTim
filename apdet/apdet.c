/*
Hardware-independent functions from apdet.h
*/
#include <apdet.h>
#include <shared/i2c_driver.c>
#include <general.h>

/* CONSTANTS ============================================================================================= */

#define SIM_LAUNCH_ACCEL               40  /* 40 is old value, ask Ollie assuming sims are accurate, > 40 */
                                           /* should be accel of least accelerating rocket */
#define SIM_BURNOUT_ACCEL_DELTA         4  /* 4 is old value, ask Ollie */

#define NUM_CHECKS                      5  /* each condition has to pass 5 times */
#define NUM_WRITE_ATTEMPTS              5  /* 5 is temp value, tbd from testing */

#define P_0                       1013.25  /* pressure at 0 altitude (mb) */
#define T_0                        288.15  /* temperature at 0 altitude (K) */
#define L                         -0.0065  /* lapse rate (valid for heights between 0km and 11km) (K/m) */
#define R                         287.053  /* gas constant for air (J/(kg K))*/
#define g                         9.80665  /* gravitational acceleration (m/s^2) */

/* STATIC HELPER FUNCTIONS / DRIVERS ===================================================================== */

/*
@brief Drogue deployment driver.
@return Status
*/
extern status_t deployDrogue()
{
    /* TODO */
    return STATUS_OK;
}

/*
@brief Payload deployment driver.
@return Status
*/
extern status_t deployPayload()
{
    /* TODO */
    return STATUS_OK;
}

/*
@brief Main deployment driver.
@return Status
*/
extern status_t deployMain()
{
    /* TODO */
    return STATUS_OK;
}

/*
@brief Altitude calculator.
@param curr_pres The current pressure.
@param alt A pointer to store the current altitude.
@return Altitude IN METERS.
*/
extern status_t calcAlt(uint32_t *curr_pres, uint32_t *alt)
{
    *alt = T_0/L*((*curr_pres/P_0)^(-L*R/g)-1);

    return STATUS_OK;
}

/*
@brief Height above ground (base altitude) calculator.
@param curr_pres The current pressure.
@param base_alt The base altitude.
@param height A pointer to store the current height.
@return Height IN METERS (can convert to feet)
*/
extern status_t calcHeight(uint32_t *curr_pres, uint32_t *base_alt, uint32_t *height)
{
    uint32_t *curr_alt;
    calcAlt(curr_pres, curr_alt);
    *height = *curr_alt - *base_alt;

    return STATUS_OK;
}

/* ROCKET FLIGHT STATE FUNCTIONS ================================================================= */

/* TODO: Documentation */

/*
@brief Returns true if rocket is in standby, else assume blackout
@return Boolean
*/
static bool testStandby(int16_t *accel, uint32_t *base_pres, uint32_t *base_temp, uint32_t *base_alt)
{
    barometerGetCompensatedValues(base_pres, base_temp);
    calcAlt(base_pres, base_alt);
    /* TODO: check that acceleration is ≈ 0 AND base_alt is around what we expect */
}

/*
@brief Detects launch
@return Boolean
*/
static bool detectLaunch(int16_t *accel)
{
    /* expect giant spike in acceleration */
    return (*accel >= SIM_LAUNCH_ACCEL);
    /* TODO: account for all directions of acceleration */
}

/*
@brief Transitions from POWERED_ASCENT to COASTING after seeing NUM_CHECKS negative accelerations in a row.
@return Boolean
*/
static bool detectBurnout(int16_t *accel)
{
    /* involve time to double check / as a backup? Check reasonable altitude?
    Data may not be stable at this point */
    return (*accel <= 0);
    /* TODO: account for all directions of acceleration */
}

/*
@brief Transition from COASTING to DEPLOY DROGUE by checking altitude delta is negative.
@param base_alt The base altitude.
@return Boolean
*/
static bool testApogee(uint32_t *base_alt, uint32_t *curr_pres, uint32_t *height)
{

    /* check that acceleration is 0 or positive downwards (acc >= 0 ) too? */

    uint32_t *prev_height = *height
    calcHeight(curr_pres, base_alt, height);

    return (*height <= *prev_height);
}

/*
@brief Transitions from INITIAL_DESCENT to DEPLOY_MAIN after rocket's alt <= 3000 ft.
@param base_alt The base altitude.
@return Boolean
*/
static bool detectMainAlt(uint32_t *base_alt)
{
    calcHeight(curr_pres, base_alt, height);

    return (*height <= 3000);
}

/*
@brief Transitions from transitions from FINAL_DESCENT to LANDED.
@param base_alt The base altitude.
@return Boolean
*/
static bool detectLanded(uint32_t *base_alt, uint32_t *curr_pres, uint32_t *height)
{
    *prev_height = *height
    calcHeight(curr_pres, base_alt, height);

    return (*height == *prev_height);
}


/* MAIN =============================================================================== */

/**
 * @brief Apogee Detection board routine - hardware-independent implementation.
 * @return Status
 */
int main()
{
    status_t retval;

    do {
        retval = i2cInit();
    } while (retval != STATUS_OK);

    do {
        retval = barometerInit();
    } while (retval != STATUS_OK);
    
    do {
        retval = accelerometerInit();
    } while (retval != STATUS_OK);

    /* DON'T RESET THESE IN CASE OF BLACKOUT */
    uint32_t *base_pres;
    uint32_t *base_temp;
    uint32_t *base_alt;

    /* For testApogee function */
    uint32_t *test_ap_height = 0;

    /* For detectLanded function */
    uint32_t *land_det_height = 0;
    
    /* don't mind resetting these */
    int reset_count = NUM_CHECKS
    int launch_count = NUM_CHECKS;
    int burnout_count = NUM_CHECKS;
    int apogee_count = NUM_CHECKS;
    int main_count = NUM_CHECKS;
    int land_count = NUM_CHECKS;

    state_t curr_state APDET_STATE_TESTING; /* write to and read from flash memory */

    while (1) {
        switch(curr_state) {
            case APDET_STATE_TESTING:
                int16_t *accel;
                status_t retval = accelerometerGetData(accel); /* TODO: account for all directions of acceleration */
                if (retval != STATUS_OK) {
                    break;
                }
                if (testStandby(accel, base_pres, base_temp, base_alt)) {
                    /* TODO: Update/store base_pres,temp and alt in flash */
                } else {
                    /* TODO: Get them from flash memory (assume we already set them earlier) */
                }
                break;

            case APDET_STATE_STANDBY:
                int16_t *accel;
                status_t retval = accelerometerGetData(accel); /* TODO: account for all directions of acceleration */
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectLaunch(accel)) {
                    launch_count--;
                    if (launch_count <= 0) {
                        curr_state = APDET_STATE_POWERED_ASCENT;
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                    }
                } else {
                    launch_count = NUM_CHECKS; /* ensures "in a row" */
                }
                break;

            case APDET_STATE_POWERED_ASCENT:
                int16_t *accel;
                status_t retval = accelerometerGetData(accel);
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectBurnout(accel)) {
                    burnout_count--;
                    if (burnout_count <= 0) {
                        curr_state = APDET_STATE_COASTING;
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                    }
                } else {
                    burnout_count = NUM_CHECKS;
                }
                break;

            case APDET_STATE_COASTING:
                uint32_t *curr_pres;
                status_t retval = barometerGetCompensatedPressure(curr_pres);
                if (retval != STATUS_OK) {
                    break;
                }
                if (testApogee(base_alt, curr_pres, test_ap_height)) {
                    apogee_count--;
                    if (apogee_count <= 0) {
                        curr_state = APDET_STATE_DEPLOY_DROGUE;
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                    }
                } else {
                    apogee_count = NUM_CHECKS;
                }
                break;

            case APDET_STATE_DEPLOY_DROGUE:
                deploy_drogue();
                curr_state = APDET_STATE_DEPLOY_PAYLOAD;
                /* TODO: Save state to flash memory */
                /* TODO: Write state to SD card using SPI */
                delay(3); /* ^ in case blackout occurs here */
                break;

            case APDET_STATE_DEPLOY_PAYLOAD:
                deploy_payload();
                curr_state = APDET_STATE_INITIAL_DESCENT;
                /* TODO: Save state to flash memory */
                /* TODO: Write state to SD card using SPI */
                break;

            case APDET_STATE_INITIAL_DESCENT:
                uint32_t *curr_pres;
                uint32_t *height = 0;
                status_t retval = barometerGetCompensatedPressure(curr_pres);
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectMainAlt(base_alt, curr_pres, height)) {
                    main_count--;
                    if (main_count <= 0) {
                        curr_state = APDET_STATE_DEPLOY_MAIN;
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                    }
                } else {
                    main_count = NUM_CHECKS;
                }
                break;

            case APDET_STATE_DEPLOY_MAIN:
                deploy_main();
                curr_state = APDET_STATE_FINAL_DESCENT;
                /* TODO: Save state to flash memory */
                /* TODO: Write state to SD card using SPI */
                break;

            case APDET_STATE_FINAL_DESCENT:
                uint32_t *curr_pres;
                status_t retval = barometerGetCompensatedPressure(curr_pres);
                if (retval != STATUS_OK) {
                    break;
                }
                if (detectLanded(base_alt, curr_pres, land_det_height)){
                    land_count--;
                    if (land_count <= 0) {
                        curr_state = APDET_STATE_LANDED
                        /* TODO: Save state to flash memory */
                        /* TODO: Write state to SD card using SPI */
                    }
                } else {
                    land_count = NUM_CHECKS;
                }
                break;

            case APDET_STATE_LANDED:
                /* should I continue instead of breaking? */
                break;
        }
    }

    while (1) {}
    return 0;
}