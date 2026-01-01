#ifndef autonomous_car_h
#define autonomous_car_h

#include <stdio.h>

#define CLEAR_HOME "\033[2J\033[H"
#define TIMEWAIT_MILLIS(X) busy_wait_us((X)*1000)
#define TIMEWAIT_MICROS(X) busy_wait_us_32(X)

enum drive_motor_direction {
  forward,
  reverse
};

enum steering_motor_direction {
  left,
  right,
  none
};

// FIFO message is a 32bit int, with RF or RR plus angle (1 byte) plus distance (1 byte)
#define ENCODE_FIFO_MESSAGE(X) ((uint32_t)(X[0]<<24|X[1]<<16|X[2]<<8|X[3]))

#define RADAR_READY_FIFO_MESSAGE           ENCODE_FIFO_MESSAGE("_RDY")
// R(ange) |   R(ear)  | angle# | distance#, RFad
#define RADAR_REAR_RANGE_FIFO_MESSAGE      ENCODE_FIFO_MESSAGE("RRad")
// R(ange) | (Forward) | angle# | distance#, RFad
#define RADAR_FORWARD_RANGE_FIFO_MESSAGE   ENCODE_FIFO_MESSAGE("RFad")

// Car - forward/reverse | left/right/none | speed
#define CAR_FIFO_MESSAGE ENCODE_FIFO_MESSAGE("Cdts")

#define CORE1_DEBUG_FIFO_MESSAGE           ENCODE_FIFO_MESSAGE("D___")
// D(Debug) |   (value 1)  | (value2) | (value 3)
#endif
