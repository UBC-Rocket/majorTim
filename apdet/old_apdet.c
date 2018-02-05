/*
Hardware-independent functions from apdet.h
*/
#include <apdet.h>
#include <shared/i2c_driver.c>
#include <general.h>

/* CONSTANTS ============================================================================================= */

#define SIM_LAUNCH_ACCEL               40  /* 40 is old value, ask Ollie assuming sims are accurate, > 40 */
#define SIM_BURNOUT_ACCEL_DELTA         4  /* 40 is old value, ask Ollie */

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

/*
@brief Transitions from STANDBY to POWERED_ASCENT after seeing NUM_CHECKS positive accelerations in a row.
@return Status
*/
static status_t detectLaunch(void)
{
	/* expect giant spike in acceleration */
	int launch_count = 0;
	int16_t *accel;
	
	while (launch_count < NUM_CHECKS) { 

		status_t retval = accelerometerGetData(accel);
		if (retval != STATUS_OK) {
			continue;
		}

		if (*accel >= SIM_LAUNCH_ACCEL) {
			launch_count++;
		} else {
			launch_count = 0;
		}
	}

    /* HOW DO I WRITE APDET STATE TO THE I2C BUS? */
	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_POWERED_ASCENT);
	}
	return STATUS_OK;
}

/*
@brief Transitions from POWERED_ASCENT to COASTING after seeing NUM_CHECKS negative accelerations in a row.
@return Status
*/
static status_t detectBurnout(void)
{
	/* involve time to double check / as a backup? Check reasonable altitude?
	Data may not be stable at this point */
	int burnout_count = 0;
	int16_t *accel;

	while (burnout_count < NUM_CHECKS) {

        status_t retval = accelerometerGetData(accel);
        if (retval != STATUS_OK) {
            continue;
        }

		if (*accel <= 0) {
			burnout_count++;
		} else {
			burnout_count = 0;
		}
	}

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_COASTING);
	}
	return STATUS_OK;
}

/*
@brief Transition from COASTING to DEPLOY DROGUE by checking altitude delta is negative.
@param base_alt The base altitude.
@return Status
*/
static status_t coastingAndTestApogee(uint32_t *base_alt)
{

	/* check that acceleration is 0 or positive downwards (acc >= 0 ) too? */
	int apogee_count = 0;

    uint32_t *curr_pres;
	uint32_t *height = 0;
	uint32_t *prev_height = 0;

	while (apogee_count < NUM_CHECKS) {

        status_t retval = barometerGetCompensatedPressure(curr_pres);
        if (retval != STATUS_OK) {
            continue;
        }

        *prev_height = *height
        calcHeight(curr_pres, base_alt, height);

		if (*height <= *prev_height) {
			apogee_count++;
		} else {
			apogee_count = 0;
		}
	}

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_DEPLOY_DROGUE);
	}
	return STATUS_OK;
}


/*
@brief Actually deploys the drogue, waits 3s and transitions from DEPLOY_DROGUE to DEPLOY_PAYLOAD.
@return Status
*/
static status_t deployDrogueState(void)
{
	deploy_drogue();
	delay(3);

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_DEPLOY_PAYLOAD);
	}
	return STATUS_OK;
}

/*
@brief Actually deploys the payload, then transitions from DEPLOY_PAYLOAD to INITIAL_DESCENT.
@return Status
*/
static status_t deployPayloadState(void)
{
	deploy_payload();

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_INITIAL_DESCENT);
	}
	return STATUS_OK;
}

/*
@brief transitions from INITIAL_DESCENT to DEPLOY_MAIN after rocket's alt <= 3000 ft.
@param base_alt The base altitude.
@return Status
*/
static status_t detectMainAlt(uint32_t *base_alt)
{
	calcHeight(curr_pres, base_alt, height);

	return (*height <= 3000);
}

/*
@brief Actually deploys the main parachute, then transitions from DEPLOY_MAIN to FINAL_DESCENT.
@return Status
*/
static status_t deployMainState(void)
{
	deploy_main();

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_FINAL_DESCENT);
	}
	return STATUS_OK;
}

/*
@brief Actually deploys the main parachute, then transitions from transitions from FINAL_DESCENT to LANDED.
@param base_alt The base altitude.
@return Status
*/
static status_t finalDescent(uint32_t *base_alt)
{
	int land_count = 0;

    uint32_t *curr_pres;
    uint32_t *height = 0;
    uint32_t *prev_height = 0;

	while (land_count < NUM_CHECKS) {

		status_t retval = barometerGetCompensatedPressure(curr_pres);
        if (retval != STATUS_OK) {
            continue;
        }

        *prev_height = *height
        calcHeight(curr_pres, base_alt, height);

		if (*height == *prev_height) { /* give error bound? */
			land_count++;
		} else {
			land_count = 0;
		}
	}

	status_t retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_LANDED);
	}
	return STATUS_OK;
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

	uint32_t *base_pres;
	uint32_t *base_temp;
    uint32_t *base_alt;
	barometerGetCompensatedValues(base_pres, base_temp); /* put into detect launch? Take the latest */
    calcAlt(base_pres, base_alt);
	
    detectLaunch();
	detectBurnout();
	coastingAndTestApogee(base_alt);
	deployDrogueState();
	deployPayloadState();
	detectMainAlt(base_alt);
	deployMainState();
	finalDescent(base_alt);

	while (1) {}
	return 0;
}