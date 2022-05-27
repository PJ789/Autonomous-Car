#ifndef Servo_h
#define Servo_h

#include <stdio.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

// Pins 6 & 7 are channels A & B of PWM 3
#define RADAR_TURRET_PIN                6
// Do not use pin                       7

#define     SERVO_PWM_CYCLE_LENGTH_US     20000.f
#define     SERVO_PWM_MIN_PULSE_LENGTH_US   500.f
#define     SERVO_PWM_MAX_PULSE_LENGTH_US  2500.f
#define     SERVO_PWM_PULSE_LENGTH_RANGE_US (SERVO_PWM_MAX_PULSE_LENGTH_US-SERVO_PWM_MIN_PULSE_LENGTH_US)
#define     SERVO_PWM_CONFIG_TOP_LEVEL    65535.f
#define     SERVO_ROTATION_RANGE_DEGREES    180.f
#define     SERVO_PWM_MAX_PULSE_LEVEL ((SERVO_PWM_MAX_PULSE_LENGTH_US/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP_LEVEL)

class Servo
{
    public:
      Servo();
      void SetDegrees(float);
      uint8_t GetDegrees();

    private:
      uint8_t    servo_pin;
      uint8_t    servo_degrees; //0-180
      uint16_t   servo_pwm_slice;
      pwm_config servo_pwm_config;
 
};


#endif
