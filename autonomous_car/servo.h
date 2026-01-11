#ifndef Servo_h
#define Servo_h
#include <Arduino.h>
#include <stdio.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <pico/multicore.h>

// 50Hz - duration of a single cycle is 20000us
#define SERVO_PWM_CYCLE_LENGTH_US     20000.f
// Wrap level - maximum level of the pwm counter
#define SERVO_PWM_CONFIG_TOP_LEVEL    24999.f
// Minium pulse length for the servo
#define SERVO_PWM_MIN_PULSE_LENGTH_US   500.f
// Maximum pulse length for the servo
#define SERVO_PWM_MAX_PULSE_LENGTH_US  2500.f
// Range of pulse length values
#define SERVO_PWM_PULSE_LENGTH_RANGE_US (SERVO_PWM_MAX_PULSE_LENGTH_US-SERVO_PWM_MIN_PULSE_LENGTH_US)
// Servo rotation range
#define SERVO_ROTATION_RANGE_DEGREES    180.f
// Pulse level required for minimum pulse length
#define SERVO_PWM_MIN_PULSE_LEVEL ((SERVO_PWM_MIN_PULSE_LENGTH_US/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP_LEVEL)
// Pulse level required for maximum pulse length
#define SERVO_PWM_MAX_PULSE_LEVEL ((SERVO_PWM_MAX_PULSE_LENGTH_US/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP_LEVEL)


class Servo
{
    public:
      Servo(uint8_t);
      void SetDegrees(float);
      uint8_t GetDegrees();

    private:
      uint8_t    servo_pin;
      uint8_t    servo_degrees; //0-180
      uint16_t   servo_pwm_slice;
      pwm_config servo_pwm_config;
      
      void SendFifoDebugMessage(char*);
 
};


#endif
