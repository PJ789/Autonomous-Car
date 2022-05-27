#include "Ultrasound.h"

Ultrasound::Ultrasound(uint8_t trigger, uint8_t echo)
{
  trigger_pin = trigger;
  echo_pin    = echo;
  
  _gpio_init(trigger_pin);
  gpio_set_dir(trigger_pin, GPIO_OUT);
  
  _gpio_init(echo_pin);
  gpio_set_dir(echo_pin, GPIO_IN);
  gpio_pull_down(echo_pin);
  gpio_set_input_enabled(echo_pin, true);
  gpio_set_irq_enabled_with_callback(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &UltrasoundIRQCallback);
}
float Ultrasound::Measure()
{
  float distance;
  float echo_duration;

  // 10us pulse on trigger pin
  gpio_put(trigger_pin, false);
  TIMEWAIT_MICROS(2);
  gpio_put(trigger_pin, true);
  TIMEWAIT_MICROS(10);
  gpio_put(trigger_pin, false);

  sleep_us( ULTRASOUND_ECHO_TIMEOUT_MICROS ); 

  // Read echo time
  echo_duration = Ultrasound::ultrasound_pulse_duration;
  if (!echo_duration) echo_duration=ULTRASOUND_ECHO_TIMEOUT_MICROS;
  
  // Speed of ultrasound 350m/s (0.035cm/us)
  distance = echo_duration * 0.035 / 2.0; 

  return distance;
}
uint32_t                  Ultrasound::ultrasound_pulse_start;
uint32_t                  Ultrasound::ultrasound_pulse_duration;
void Ultrasound::UltrasoundIRQCallback(uint gpio, uint32_t events)
{
  if (events & GPIO_IRQ_EDGE_RISE )
  {
    Ultrasound::ultrasound_pulse_start    = time_us_32();
    Ultrasound::ultrasound_pulse_duration = 0;
  }
  if (events & GPIO_IRQ_EDGE_FALL)
  {
    Ultrasound::ultrasound_pulse_duration = time_us_32()-Ultrasound::ultrasound_pulse_start;
    Ultrasound::ultrasound_pulse_duration = (Ultrasound::ultrasound_pulse_duration<(ULTRASOUND_ECHO_TIMEOUT_MICROS/2))?Ultrasound::ultrasound_pulse_duration:0;
    Ultrasound::ultrasound_pulse_start    = 0;
  }
}
