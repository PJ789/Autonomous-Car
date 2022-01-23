#include <pico/multicore.h>
#include "car.h"
#include "radar.h"

Car car;

void setup() {
  //stdio_init_all();

  printf("Starting car on core 0\n");

  printf("Starting radar on core 1\n");
  multicore_reset_core1();
  multicore_launch_core1(radar_loop);
     
  printf("Waiting for radar ready signal\n");
  uint32_t fifo_message = 0;
  do
  {
    multicore_fifo_pop_timeout_us(200, &fifo_message);
    car.HazardLightsOn();
  }
  while ( fifo_message != RADAR_READY_FIFO_MESSAGE );
  car.HazardLightsOff();

}


// CORE0 -------------------------------------------------------------------------------------
// Core0 loop - navigation tasks
void loop()
{
  car.Iterator();
}


// CORE1 -------------------------------------------------------------------------------------
// Core1 loop - peripheral tasks
void radar_loop() // Peripherals
{
  Radar radar;

  while (true)
  {
    radar.SetRadarActivePin();
    radar.Iterator();
  }
}
