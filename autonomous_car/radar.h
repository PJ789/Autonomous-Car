#ifndef Radar_h
#define Radar_h

#include <stdint.h>
#include <algorithm>
#include <arduino.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include "autonomous_car.h"
#include "Servo.h"
#include "Ultrasound.h"

// Pins 6 & 7 are channels A & B of PWM 3
#define RADAR_TURRET_PIN                    6

 // brown wire - trigger
 // blue wire  - echo
 
#define FRONT_ULTRASOUND_TRIG_PIN         10
#define FRONT_ULTRASOUND_ECHO_PIN         11
// Avoid pin                              12
// Do not use pin                         13
#define REAR_ULTRASOUND_TRIG_PIN          14
#define REAR_ULTRASOUND_ECHO_PIN          15

// pin used to signal radar lost messages core1->core0
#define RADAR_LOST_MESSAGE_PIN             8

// speed of servo rotation used to estimate turn time
#define RADAR_SERVO_MILLIS_PER_DEGREE      5
// size of servo turn for each +/- step
#define RADAR_SCAN_STEP_DEGREES           25
// time taken for servo to stabilise after a turn stops
#define RADAR_SERVO_SETTLING_TIME_MILLIS 100

// parameters used to size the array of ultrasound readings
#define RADAR_MEASUREMENTS_GRANULARITY     5
#define RADAR_MEASUREMENTS                 1+(180/RADAR_MEASUREMENTS_GRANULARITY) 

class Radar
{

  public:

    Radar();

    void       Iterator();
    float      MeasureFront();
    float      MeasureRear();

    Ultrasound front_ultrasound;
    Ultrasound rear_ultrasound;
   
  private:
    Servo                     radar_turret;
    uint8_t                   radar_turret_step;
    steering_motor_direction  last_steering_motor_direction;
    drive_motor_direction     last_drive_motor_direction;
    uint8_t                   last_drive_motor_speed;


    void     TurretRotationSequencer();
    void     RotateTurret(int16_t);
    int8_t   GetTurretDirection();

    void     DataDebug();
    void     ServoDiagnostics();
    void     SetDriveMotorSpeed(      uint8_t );
    void     Drive(                   drive_motor_direction );
    void     Steer(                   steering_motor_direction );
    void     DecodeFifo(              uint32_t);
    int16_t  ConvertServoAngleToCarAngle(uint8_t);
    uint8_t  ConvertCarAngleToServoAngle(int16_t);

    bool Stopped();
    bool Moving();
    bool MovingForward();
    bool MovingBackward();
    bool DirectionIsForward();
    bool DirectionIsReverse();
    bool Turning();
    bool TurningLeft();
    bool TurningRight();
    void SetLostMessageWarningLamp();
    void ClearLostMessageWarningLamp();

    void SendFifoDebugMessage(char*);
};



#endif
