#ifndef autonomous_car_h
#define autonomous_car_h

#include <stdio.h>

#define CLEAR "\033[2J"
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

#define ENCODE_FIFO_MESSAGE(X) ((uint32_t)(X[0]<<24|X[1]<<16|X[2]<<8|X[3]))

#define RADAR_READY_FIFO_MESSAGE        ENCODE_FIFO_MESSAGE("_RDY")
// Rearward Range angle distance
#define RADAR_REAR_RANGE_FIFO_MESSAGE      ENCODE_FIFO_MESSAGE("RRad")
// Forward Range angle distance
#define RADAR_FORWARD_RANGE_FIFO_MESSAGE   ENCODE_FIFO_MESSAGE("RFad")

// Drive motor forward, reverse, speed
#define DRIVE_FORWARD_FIFO_MESSAGE ENCODE_FIFO_MESSAGE("DFWD")
#define DRIVE_REVERSE_FIFO_MESSAGE ENCODE_FIFO_MESSAGE("DREV")
#define DRIVE_SPEED_FIFO_MESSAGE   ENCODE_FIFO_MESSAGE("DSPs")

// Steering motor left, right, none
#define STEER_LEFT_FIFO_MESSAGE    ENCODE_FIFO_MESSAGE("SLFT")
#define STEER_RIGHT_FIFO_MESSAGE   ENCODE_FIFO_MESSAGE("SRGH")
#define STEER_NONE_FIFO_MESSAGE    ENCODE_FIFO_MESSAGE("SNON")
#endif
