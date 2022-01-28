#include <pico/multicore.h>
#include "car.h"
#include "radar.h"

Car car;

void setup() {
  //stdio_init_all();
Serial.begin(250000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("hello world");

  Serial.println("Starting radar on core 1\n");
  multicore_reset_core1();
  multicore_launch_core1(radar_loop);
     
  Serial.println("Waiting for radar ready signal\n");
  uint32_t fifo_message = 0;
  do
  {
    multicore_fifo_pop_timeout_us(200, &fifo_message);
    car.HazardLightsOn();
  }
  while ( fifo_message != RADAR_READY_FIFO_MESSAGE );
  Serial.println("Radar ready!\n");
  car.HazardLightsOff();

}


// CORE0 -------------------------------------------------------------------------------------
// Core0 loop - navigation tasks
void loop()
{
  car.Iterator();
  car.DumpRadarMetrics();
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
