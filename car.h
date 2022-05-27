#ifndef Car_h
#define Car_h
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include "autonomous_car.h"
#include "radar.h"
#include "lights.h"

#define LOW_SPEED                   220
#define HIGH_SPEED                  255
// Minimum range before emergency stop
// below minimum range the car stops
#define MINIMUM_FORWARD_RANGE_LIMIT  50.0
#define MINIMUM_REAR_RANGE_LIMIT     50.0
// Minimum range for steering decisions
// below steering range the car changes direction
#define STEERING_FORWARD_RANGE_LIMIT 100.0
#define STEERING_REAR_RANGE_LIMIT    100.0

#define DRIVE_MOTOR_DIRECTION_PIN    10
#define STEERING_MOTOR_DIRECTION_PIN 11
#define DRIVE_MOTOR_SPEED_PIN        12
// Do not use pin                    13
#define STEERING_MOTOR_SPEED_PIN     14

// radar
#define WHEELBASE_METERS  0.26
#define AXEL_WIDTH_METERS 0.23

#define STEERING_ANGLE_DEGS 25.0
#define STEERING_ANGLE_RADS ((STEERING_ANGLE_DEGS * 71.0) / 4068.0)

#define FRONT_INNER_TURN_RADIUS_METERS (WHEELBASE_METERS/sin(STEERING_ANGLE_RADS))
#define FRONT_OUTER_TURN_RADIUS_METERS (FRONT_INNER_TURN_RADIUS_METERS+AXEL_WIDTH_METERS)
#define REAR_INNER_TURN_RADIUS_METERS  (sqrt( pow(FRONT_INNER_TURN_RADIUS_METERS,2)-pow(WHEELBASE_METERS,2) ) )
#define REAR_OUTER_TURN_RADIUS_METERS  (REAR_INNER_TURN_RADIUS_METERS+AXEL_WIDTH_METERS)

#define LEFT3       (-3* RADAR_SCAN_STEP_DEGREES)
#define LEFT2       (-2* RADAR_SCAN_STEP_DEGREES)
#define LEFT1       (-1* RADAR_SCAN_STEP_DEGREES)
#define DEAD_AHEAD  ( 0                         )
#define RIGHT1      ( 1* RADAR_SCAN_STEP_DEGREES)
#define RIGHT2      ( 2* RADAR_SCAN_STEP_DEGREES)
#define RIGHT3      ( 3* RADAR_SCAN_STEP_DEGREES)
class Car
{

  public:

    Car();
    void DecodeFifo(               uint32_t);
    void Iterator();
    void HazardLightsOn();
    void HazardLightsOff();
    void DumpRadarMetrics();
    void PlotRadarMetrics(int);

  private:
  
    void SetLights();
    void PlanRoute();
    void PlanSpeed();
    bool Stopped();
    bool Moving();
    bool MovingForward();
    bool MovingBackward();
    bool DirectionIsForward();
    bool DirectionIsReverse();
    bool Turning();
    bool TurningLeft();
    bool TurningRight();
    bool ObstacleDetectedAhead(               float );
    bool ObstacleDetectedBehind(              float );
    bool CollisionAvoidance();
    void SetDriveMotorSpeed(                  uint8_t );
    void EmergencyStop();
    void Drive(                               drive_motor_direction);
    void Steer(                               steering_motor_direction);
    uint8_t GetRadarForwardMeasurements(      int8_t);
    uint8_t GetRadarRearwardMeasurements(     int8_t );
    int16_t ConvertServoAngleToCarAngle(      uint8_t );
    uint8_t ConvertCarAngleToRadarAngle(      int16_t );
    void SendStatus();

    steering_motor_direction  last_steering_motor_direction;
    drive_motor_direction     last_drive_motor_direction;

    uint8_t                   last_drive_motor_speed;
    int                       acceleration;
    int                       deceleration;

    Lights                    lights;

    uint8_t                   radar_forward_measurements[  RADAR_MEASUREMENTS ];
    uint8_t                   radar_rearward_measurements[ RADAR_MEASUREMENTS ];

};


#endif
