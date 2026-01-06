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
  gpio_set_irq_enabled_with_callback( echo_pin,
    GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
    false, // register callback, but keep IRQ OFF until required
    &UltrasoundIRQCallback
    );
}
float Ultrasound::Measure()
{
  float distance;
  float echo_duration;

  Ultrasound::ultrasound_pulse_start    = 0;
  Ultrasound::ultrasound_pulse_duration = 0;

  // Disable IRQ to avoid noise before triggering
  gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  // 10us pulse on trigger pin
  gpio_put(trigger_pin, false);
  TIMEWAIT_MICROS(2);
  gpio_put(trigger_pin, true);
  TIMEWAIT_MICROS(10);
  gpio_put(trigger_pin, false);

  // Enable IRQ only during the echo window
  gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  // Wait for echo OR timeout (non-blocking)
  uint32_t start = time_us_32();
  while ((time_us_32() - start) < ULTRASOUND_ECHO_TIMEOUT_MICROS)
  {
    if (Ultrasound::ultrasound_pulse_duration != 0) break; // falling edge captured
  }
  // Disable IRQ again immediately
  gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);  

  // Read echo time
  echo_duration = Ultrasound::ultrasound_pulse_duration;
  if (!echo_duration)
    echo_duration = ULTRASOUND_ECHO_TIMEOUT_MICROS;

  // Convert to distance (speed of sound ~0.035 cm/us))
  distance = echo_duration * 0.035 / 2.0; 

  return distance;
}

volatile uint32_t Ultrasound::ultrasound_pulse_start;
volatile uint32_t Ultrasound::ultrasound_pulse_duration;
void Ultrasound::UltrasoundIRQCallback(uint gpio, uint32_t events)
{
  uint32_t now = timer_hw->timerawl; // SAFE inside IRQ

  if (events & GPIO_IRQ_EDGE_RISE )
  {
    Ultrasound::ultrasound_pulse_start    = now;
    Ultrasound::ultrasound_pulse_duration = 0;
  }
  if (events & GPIO_IRQ_EDGE_FALL)
  {
    Ultrasound::ultrasound_pulse_duration = now-Ultrasound::ultrasound_pulse_start;
    Ultrasound::ultrasound_pulse_start    = 0;
    // 🔥 CRITICAL: disable IRQ immediately after falling edge 
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  }
}
