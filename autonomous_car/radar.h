#ifndef Radar_h
#define Radar_h

#include <stdint.h>
#include <algorithm>
#include <arduino.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include "autonomous_car.h"
#include "servo.h"

 // brown wire - trigger
 // blue wire  - echo
 
#define FRONT_ULTRASOUND_TRIG_PIN       2
#define FRONT_ULTRASOUND_ECHO_PIN       3
#define REAR_ULTRASOUND_TRIG_PIN        4
#define REAR_ULTRASOUND_ECHO_PIN        5

#define RADAR_ACTIVE_PIN                8

#define RADAR_SERVO_MILLIS_PER_DEGREE   5
#define RADAR_SCAN_STEP_DEGREES        25

// 30000us = 525cm (0.035cm/us)
#define RADAR_ECHO_TIMEOUT_MICROS       60000
#define RADAR_ECHO_TIMEOUT_MILLIS       (RADAR_ECHO_TIMEOUT_MICROS/1000)

#define RADAR_MEASUREMENTS_GRANULARITY  5
#define RADAR_MEASUREMENTS              1+(180/RADAR_MEASUREMENTS_GRANULARITY) 

class Radar
{

  public:

    Radar();

    void Iterator();
    void     SetRadarActivePin();
    uint32_t MeasureFront();
    uint32_t MeasureRear();   
    static uint32_t                  ultrasound_pulse_start;
    static uint32_t                  ultrasound_pulse_duration;
    static void     UltrasoundIRQCallback(   uint, uint32_t);
   
  private:
    Servo                     radar_turret;
    steering_motor_direction  last_steering_motor_direction;
    drive_motor_direction     last_drive_motor_direction;
    uint8_t                   last_drive_motor_speed;

    void     TurretRotationSequencer();
    void     RotateTurret(int16_t);
    int8_t   GetTurretDirection();

    float    Measure(                 uint8_t, uint8_t );
    void     DataDebug();
    void     ServoDiagnostics();
    void     SetDriveMotorSpeed(      uint8_t );
    void     Drive(                   drive_motor_direction );
    void     Steer(                   steering_motor_direction );
    void     DecodeFifo(              uint32_t);
    int16_t  ConvertServoAngleToCarAngle(uint8_t);
    uint8_t  ConvertCarAngleToServoAngle(int16_t);
    float    PulseIn(                 uint32_t);

    bool Stopped();
    bool Moving();
    bool MovingForward();
    bool MovingBackward();
    bool DirectionIsForward();
    bool DirectionIsReverse();
    bool Turning();
    bool TurningLeft();
    bool TurningRight();

};



#endif
