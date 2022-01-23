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
#define     SERVO_PWM_CONFIG_TOP           0xFFFF

class Servo
{
    public:
      Servo();
      void SetDegrees(float);

    private:
      uint8_t    servo_pin;
      uint16_t   servo_pwm_slice;
      pwm_config servo_pwm_config;
 
};


#endif
