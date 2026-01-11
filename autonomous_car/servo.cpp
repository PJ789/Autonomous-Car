#include "autonomous_car.h"
#include "servo.h"

Servo::Servo(uint8_t servo_pin)
{
  this->servo_pin = servo_pin;

  gpio_set_function(servo_pin, GPIO_FUNC_PWM);
  servo_pwm_slice = pwm_gpio_to_slice_num(servo_pin);

  // 50 Hz is the standard servo refresh rate
  // The divider of 100 keeps the PWM clock stable
  // A wrap of 24999 gives you 1 tick = 0.8 µs, which is plenty of resolution
  // 50 Hz: wrap = 24999, clkdiv = 100 
  //   𝑓 = clock_sys /  clkdiv ⋅ ( wrap + 1 )
  pwm_set_wrap(servo_pwm_slice, SERVO_PWM_CONFIG_TOP_LEVEL);
  pwm_set_clkdiv(servo_pwm_slice, 100);
  // don't need to invert the PWM because we're using non-inverting opto-couplers
  //pwm_set_output_polarity(servo_pwm_slice, true, true);

  // Enable PWM
  pwm_set_enabled(servo_pwm_slice, true);

  SetDegrees(90);
}

// Control a servo by degrees

void Servo::SetDegrees(float target_servo_degrees)
{
  float pwm_pulse_length_us;
  uint16_t gpio_level;

  target_servo_degrees = (target_servo_degrees>SERVO_ROTATION_RANGE_DEGREES)? SERVO_ROTATION_RANGE_DEGREES:target_servo_degrees;
  target_servo_degrees = (target_servo_degrees<                           0)? 0:target_servo_degrees;
  servo_degrees = target_servo_degrees;

   pwm_pulse_length_us = SERVO_PWM_MIN_PULSE_LENGTH_US +
                        (
                          target_servo_degrees
                          *(
                            SERVO_PWM_PULSE_LENGTH_RANGE_US
                            /SERVO_ROTATION_RANGE_DEGREES
                          )
                        );


  gpio_level = (pwm_pulse_length_us/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP_LEVEL;


  // allow pwm cycle to complete to smooth transitions
  // max pulse is circa 2000-2500 in a 20000us cycle.
  // so wait until we're in the second half of the
  // cycle (ie, pulse output low, some point between
  // end of a max pulse, and ahead of a new  max pulse)
  // So 2500us to 17500us time period effectively
  uint16_t c = pwm_get_counter(servo_pwm_slice); 
  while (c < SERVO_PWM_MAX_PULSE_LEVEL || c > (SERVO_PWM_CONFIG_TOP_LEVEL - SERVO_PWM_MAX_PULSE_LEVEL)) 
  {
    c = pwm_get_counter(servo_pwm_slice);
  }

  pwm_set_gpio_level(servo_pin, gpio_level);
}

uint8_t Servo::GetDegrees()
{
  return servo_degrees;
}

void Servo::SendFifoDebugMessage(char* message)
{
  uint32_t fifo_message;

  fifo_message = CORE1_DEBUG_FIFO_MESSAGE;
  fifo_message &= 0xFF000000;
  fifo_message |= ((uint32_t)message[0])<<16;
  fifo_message |= ((uint32_t)message[1])<<8;
  fifo_message |= ((uint32_t)message[2])<<0;

  while(!multicore_fifo_wready())
  {
    sleep_ms(1);
  }

  multicore_fifo_push_timeout_us(fifo_message, 100);
}