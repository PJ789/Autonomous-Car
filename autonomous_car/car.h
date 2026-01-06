#ifndef Car_h
#define Car_h
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include "autonomous_car.h"
#include "radar.h"
#include "lights.h"


#define DRIVE_MOTOR_LOW_SPEED        500
#define DRIVE_MOTOR_HIGH_SPEED       750
#define DRIVE_MOTOR_ACCELERATION      50
#define DRIVE_MOTOR_DECELERATION     100
#define DRIVE_MOTOR_RAMP_MILLIS       20

#define STEERING_MOTOR_LOW_SPEED    6250
#define STEERING_MOTOR_HIGH_SPEED   6250
#define STEERING_MOTOR_ACCELERATION  100
#define STEERING_MOTOR_DECELERATION  100
#define STEERING_MOTOR_RAMP_MILLIS    20

// Minimum range before emergency stop
// below minimum range the car stops
#define MINIMUM_FORWARD_RANGE_LIMIT  150.0
#define MINIMUM_REAR_RANGE_LIMIT     150.0
// Minimum range for steering decisions
// below steering range the car reverses direction
#define STEERING_FORWARD_RANGE_LIMIT 200.0
#define STEERING_REAR_RANGE_LIMIT    200.0

#define DRIVE_MOTOR_DIRECTION_PIN     2
#define STEERING_MOTOR_DIRECTION_PIN  3
#define DRIVE_MOTOR_SPEED_PIN         4
#define STEERING_MOTOR_SPEED_PIN      5

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

    void DriveMotorSpeedControlCallback();
    void SteeringMotorSpeedControlCallback();

  private:
  
    void SetLights();
    void PlanRoute();
    void PlanSpeed();
    bool DriveMotorStopped();
    bool SteeringMotorStopped();
    bool Moving();
    bool Braking();
    bool Accelerating();
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
    void SetDriveMotorSpeed(                  uint16_t );
    void SetSteeringMotorSpeed(                  uint16_t );
    void HardStop();
    void EmergencyStop();
    void Drive(                               drive_motor_direction);
    void Steer(                               steering_motor_direction);
    uint32_t GetRadarForwardMeasurements(      int8_t);
    uint32_t GetRadarRearwardMeasurements(     int8_t );
    int16_t ConvertServoAngleToCarAngle(      uint8_t );
    uint8_t ConvertCarAngleToRadarAngle(      int16_t );
    void SendStatus();

    steering_motor_direction  current_steering_motor_direction;
    drive_motor_direction     current_drive_motor_direction;

    uint16_t                  current_drive_motor_speed;
    uint16_t                  current_steering_motor_speed;
    uint16_t                  target_drive_motor_speed;
    uint16_t                  target_steering_motor_speed;

    struct repeating_timer   drive_motor_speed_control_ticker;
    struct repeating_timer   steering_motor_speed_control_ticker;

    Lights                    lights;

    uint32_t                  radar_forward_measurements[  RADAR_MEASUREMENTS ];
    uint32_t                  radar_rearward_measurements[ RADAR_MEASUREMENTS ];

    uint                      fifo_message_count = 0; // used to force the car to gather fifo messages when starting or changing course
    bool                      status_changed  = true; // used to limit the frequency of status change messages from car to radar
};


#endif
