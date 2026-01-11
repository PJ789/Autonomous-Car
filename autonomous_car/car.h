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
#define DRIVE_MOTOR_RAMP_INTERVAL_MILLIS       20

// Minimum range before emergency stop
// Below minimum range the car performs a hard-stop, and flushes out radar data
// 150 - floor testing
//  25 - bench testing
#define OBSTACLE_AHEAD_EMERGENCY_STOP_RANGE_LIMIT_CM  150.0
#define OBSTACLE_BEHIND_EMERGENCY_STOP_RANGE_LIMIT_CM 150.0
// Minimum range for manoeuvering decisions
// Below steering range the car attempts to reverse direction rather than manoeuvre
// 200 - floor testing
// 100 - bench testing
#define OBSTACLE_AHEAD_MANOEUVERING_RANGE_LIMIT_CM   200.0
#define OBSTACLE_BEHIND_MANOEUVERING_RANGE_LIMIT_CM  200.0

#define DRIVE_MOTOR_DIRECTION_PIN     2
#define DRIVE_MOTOR_SPEED_PIN         4
// Pins 6 & 7 are channels A & B of PWM 3
#define STEERING_MOTOR_PIN            7

// steering motor
// these angles are not car angles, they are servo motor rotations
#define STEERING_MOTOR_MAX_LEFT_ANGLE    (90+20)
#define STEERING_MOTOR_CENTRE_ANGLE      (90)
#define STEERING_MOTOR_MAX_RIGHT_ANGLE   (90-20)

// Scan steps used by heuristics to compare radar readings, and decide on manoeuvres
// these are relative to the car
#define LEFT3       (-3* RADAR_SCAN_STEP_DEGREES)
#define LEFT2       (-2* RADAR_SCAN_STEP_DEGREES)
#define LEFT1       (-1* RADAR_SCAN_STEP_DEGREES)
#define DEAD_AHEAD  ( 0                         )
#define DEAD_BEHIND ( 0                         )
#define RIGHT1      ( 1* RADAR_SCAN_STEP_DEGREES)
#define RIGHT2      ( 2* RADAR_SCAN_STEP_DEGREES)
#define RIGHT3      ( 3* RADAR_SCAN_STEP_DEGREES)

// Biases the planning algorithm to avoid rapid turn changes unless there is a 20cm benefit from better range
#define TURN_CHANGE_HYSTERESIS_CM       20

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

  private:
  
    void SetLights();
    void PlanRoute();
    void PlanSpeed();
    bool DriveMotorStopped();
    bool Moving();
    bool Braking();
    bool Accelerating();
    bool MovingForward();
    bool MovingReverse();
    bool DirectionIsForward();
    bool DirectionIsReverse();
    bool Turning();
    bool TurningLeft();
    bool TurningRight();
    bool ObstacleDetectedAhead(               float );
    bool ObstacleDetectedBehind(              float );
    bool CollisionAvoidance();
    void SetDriveMotorSpeed(                  uint16_t );
    void HardStop();
    void EmergencyStop();
    void Drive(                               drive_motor_direction);
    void Steer(                               steering_motor_direction);
    uint32_t GetDistancetoObstacleAhead(      int8_t);
    uint32_t GetDistancetoObstacleBehind(     int8_t );
    int16_t ConvertRadarAngleToCarAngle(      uint8_t );
    uint8_t ConvertCarAngleToRadarAngle(      int16_t );
    void SendStatus();

    steering_motor_direction  current_steering_motor_direction;
    drive_motor_direction     current_drive_motor_direction;

    uint16_t                  current_drive_motor_speed;
    uint16_t                  target_drive_motor_speed;

    struct repeating_timer    drive_motor_speed_control_ticker;
    
    Lights                    lights;

    uint32_t                  radar_front_measurements[  RADAR_MEASUREMENTS ];
    uint32_t                  radar_rear_measurements[ RADAR_MEASUREMENTS ];

    uint                      fifo_message_count = 0; // used to force the car to gather fifo messages when starting or changing course
    bool                      status_changed  = true; // used to limit the frequency of status change messages from car to radar

    Servo                     steering_motor;
};


#endif
