/*
Hardware-independent functions from apdet.h
*/
#include <apdet.h>
#include <shared/i2c_driver.c>
#include <general.h>

/* CONSTANTS ============================================================================================= */

#define APDET_STATE_STANDBY             0
#define APDET_STATE_POWERED_ASCENT      1
#define APDET_STATE_COASTING            2
#define APDET_STATE_DEPLOY_DROGUE       3
#define APDET_STATE_DEPLOY_PAYLOAD      4
#define APDET_STATE_INITIAL_DESCENT     5
#define APDET_STATE_DEPLOY_MAIN         6
#define APDET_STATE_FINAL_DESCENT       7
#define APDET_STATE_LANDED              8

#define SIM_LAUNCH_ACCEL               40  /* 40 is old value, ask Ollie assuming sims are accurate, > 40 */
#define SIM_BURNOUT_ACCEL_DELTA         4  /* 40 is old value, ask Ollie */

#define NUM_CHECKS                      5  /* each condition has to pass 5 times */
#define NUM_WRITE_ATTEMPTS              5  /* 5 is temp value, tbd from testing */

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
@brief Altitude calculator. Implementation of the international barometric formula (no temperature needed)
Pressure is in millibars.
@return altitude IN METERS (can convert to feet)
*/
extern status_t calcAlt(int32_t *curr_pres, int32_t *base_pres, uint32_t *alt)
{
	*alt = 44330*(1-( *curr_pres/ *base_pres)^(1/5.255));

	return STATUS_OK;
}

/*
@brief Pressure converter from milibar to ___.
Pressure is in millibars.
@return altitude IN METERS (can convert to feet)
*/
extern status_t presConvert(int32_t *pres)
{
	*pres = /* convert me */

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
	float accel = 0;
	
	while (launch_count < NUM_CHECKS) { 

		can_id_t can_id;
		float can_msg;
		void retval = canRead(&can_id, (uint64_t *) &can_msg);
		if (retval != STATUS_OK || can_id != CAN_ID_SENSOR_ACCEL_Z) {
			continue;
		}

		accel = can_msg;

		if (accel >= SIM_LAUNCH_ACCEL) {
			launch_count++;
		} else {
			launch_count = 0;
		}
	}

	void retval = STATUS_ERROR;
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
	float accel = 0;

	while (burnout_count < NUM_CHECKS) {

		can_id_t can_id;
		float can_msg;
		void retval = canRead(&can_id, (uint64_t *) &can_msg);
		if (retval != STATUS_OK || can_id != CAN_ID_SENSOR_ACCEL_Z) {
			continue;
		}

		accel = can_msg;

		if (accel <= 0) {
			burnout_count++;
		} else {
			burnout_count = 0;
		}
	}

	void retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_COASTING);
	}
	return STATUS_OK;
}

/*
@brief Transition from COASTING to DEPLOY DROGUE by checking altitude delta is negative.
@return Status
*/
static status_t coastingAndTestApogee(void)
{

	/* check that acceleration is 0 or positive downwards (acc >= 0 ) too? */
	int apogee_count = 0;

	float alt = 0;
	float prev_alt = 0;

	while (apogee_count < NUM_CHECKS) {

		can_id_t can_id;
		float can_msg;
		void retval = canRead(&can_id, (uint64_t *) &can_msg);
		if (retval != STATUS_OK || can_id != CAN_ID_SENSOR_CALC_ALTITUDE) {
			continue;
		}

		prev_alt = alt;
		alt = can_msg;

		if (alt <= prev_alt) {
			apogee_count++;
		} else {
			apogee_count = 0;
		}
	}

	void retval = STATUS_ERROR;
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

	void retval = STATUS_ERROR;
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

	void retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_INITIAL_DESCENT);
	}
	return STATUS_OK;
}

/*
@brief transitions from INITIAL_DESCENT to DEPLOY_MAIN after rocket's alt <= 3000 ft.
@return Status
*/
static status_t detectMainAlt(void)
{
	int main_count = 0;
	float alt = 0;

	while (main_count < NUM_CHECKS) {

		can_id_t can_id;
		float can_msg;
		void retval = canRead(&can_id, (uint64_t *) &can_msg);
		if (retval != STATUS_OK || can_id != CAN_ID_SENSOR_CALC_ALTITUDE) {
			continue;
		}

		alt = can_msg;

		if (alt <= 3000) {
			main_count++;
		} else {
			main_count = 0;
		}
	}

	void retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_DEPLOY_MAIN);
	}
	return STATUS_OK;
}

/*
@brief Actually deploys the main parachute, then transitions from DEPLOY_MAIN to FINAL_DESCENT.
@return Status
*/
static status_t deployMainState(void)
{
	deploy_main();

	void retval = STATUS_ERROR;
	for (int i = 0; i < NUM_WRITE_ATTEMPTS && retval != STATUS_OK; i++) {
		retval = canWrite(CAN_ID_APDET_STATE, APDET_STATE_FINAL_DESCENT);
	}
	return STATUS_OK;
}

/*
@brief Actually deploys the main parachute, then transitions from transitions from FINAL_DESCENT to LANDED.
@return Status
*/
static status_t finalDescent(void)
{
	int land_count = 0;

	float alt = 0;
	float prev_alt = 0;

	while (land_count < NUM_CHECKS) {

		can_id_t can_id;
		float can_msg;
		void retval = canRead(&can_id, (uint64_t *) &can_msg);
		if (retval != STATUS_OK || can_id != CAN_ID_SENSOR_CALC_ALTITUDE) {
			continue;
		}

		prev_alt = alt;
		alt = can_msg;

		if (alt == prev_alt) { /* give error bound? */
			land_count++;
		} else {
			land_count = 0;
		}
	}

	void retval = STATUS_ERROR;
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
	barometerGetCompensatedValues(base_pres, base_temp); /* put into detect launch? */

	detectLaunch();
	detectBurnout();
	coastingAndTestApogee();
	deployDrogueState();
	deployPayloadState();
	detectMainAlt();
	deployMainState();
	finalDescent();

	while (1) {}
	return 0;
}