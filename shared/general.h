#ifndef GENERAL_H
#define GENERAL_H

/* Return codes */
typedef enum status_enum {
	STATUS_OK        =  0,
	STATUS_ERROR     = -1,
} status_t;

/* States */
typedef enum state {
	APDET_STATE_TESTING			= 0, /* determines if STANDBY or what's stored in flash memory */
	APDET_STATE_STANDBY 		= 1,
	APDET_STATE_POWERED_ASCENT 	= 2,
	APDET_STATE_COASTING 		= 3,
	APDET_STATE_DEPLOY_DROGUE 	= 4,
	APDET_STATE_DEPLOY_PAYLOAD 	= 5,
	APDET_STATE_INITIAL_DESCENT = 6,
	APDET_STATE_DEPLOY_MAIN 	= 7,
	APDET_STATE_FINAL_DESCENT 	= 8,
	APDET_STATE_LANDED 			= 9,
} state_t;

#endif