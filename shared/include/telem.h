/*
Header file for Telemetry board
*/

#ifndef TELEM_H
#define TELEM_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h> //for time_t


#define SENSOR_COUNT 16
#define LISTEN_QUANTUM 500
#define BUFFER_SIZE 8
typedef double canbus_t;



/* Hardware-dependent functions */
status_t initSend(int* fd, struct sockaddr_un* addr);
status_t canListen(int* id, canbus_t* canbusData);

/* Hardware-independent functions */
#include "telemMesg.pb.h"
#include "pb_encode.h"
#include "general.h"

extern int main(void);

//sets up registers, attempt to link to ground station
status_t telemInit();


status_t listenOnBus(TelemMesg* telemDataBuffer);

status_t pbPackage(pb_byte_t* targetBuffer, pb_ostream_t* stream, size_t targetBufferSize, TelemMesg* data);

status_t pbSend(pb_ostream_t* stream, pb_byte_t buffer[TelemMesg_size]);

#endif
