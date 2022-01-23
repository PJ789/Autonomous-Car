#include "servo.h"

Servo::Servo()
{
  float      pwm_target_frequency_hz;

  pwm_target_frequency_hz = 1000000.0/SERVO_PWM_CYCLE_LENGTH_US;
    
  gpio_set_function(RADAR_TURRET_PIN, GPIO_FUNC_PWM);
  servo_pwm_slice = pwm_gpio_to_slice_num(RADAR_TURRET_PIN);

  servo_pwm_config     = pwm_get_default_config();
  servo_pwm_config.top = SERVO_PWM_CONFIG_TOP;

  float default_frequency_clk_sys_hz = clock_get_hz(clk_sys)/SERVO_PWM_CONFIG_TOP;
  float clock_divider                = default_frequency_clk_sys_hz/pwm_target_frequency_hz;

  pwm_config_set_clkdiv(&servo_pwm_config, clock_divider);

  pwm_init(servo_pwm_slice, &servo_pwm_config, true);

  SetDegrees(90);
}

// Control a servo by degrees

void Servo::SetDegrees(float servo_angle)
{
  float pwm_pulse_length_us;
  uint16_t gpio_level;

  servo_angle = (servo_angle>180)?180:servo_angle;
  servo_angle = (servo_angle<  0)?  0:servo_angle;
  
  pwm_pulse_length_us = SERVO_PWM_MIN_PULSE_LENGTH_US +
                        (
                          servo_angle
                          *(
                            (SERVO_PWM_MAX_PULSE_LENGTH_US-SERVO_PWM_MIN_PULSE_LENGTH_US)
                            /180.f
                          )
                        );

  gpio_level = (pwm_pulse_length_us/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP;

  // allow pwm cycle to complete to smooth transitions
  // max pulse is circa 2000-2500 in a 20000us cycle.
  // so wait until we're in the second half of the
  // cycle (ie, pulse output low)
  while(pwm_get_counter( servo_pwm_slice )>((SERVO_PWM_MAX_PULSE_LENGTH_US/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP));
  while(pwm_get_counter( servo_pwm_slice )<((SERVO_PWM_MAX_PULSE_LENGTH_US/SERVO_PWM_CYCLE_LENGTH_US)*SERVO_PWM_CONFIG_TOP));
  
  pwm_set_gpio_level(RADAR_TURRET_PIN, gpio_level);
}
