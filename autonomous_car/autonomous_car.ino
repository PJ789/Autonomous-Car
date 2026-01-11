#include <pico/multicore.h>
#include "car.h"
#include "radar.h"
#include "debug.h"
#include "TickTwo.h"

extern "C" void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack_bottom, size_t stack_size);

static uint32_t core1_stack[8192];  // 32 KB stack (8192 * 4 bytes)

Car car;

TickTwo DriveMotorSpeedControlTicker(
    [](){ car.DriveMotorSpeedControlCallback(); },
    DRIVE_MOTOR_RAMP_INTERVAL_MILLIS
);


void setup() {
  //stdio_init_all();
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {
  //while (!Serial) {
    ; // wait up to 2 seconds
  }
  SERIALPRINTLN("hello world");
  delay(500);

  SERIALPRINTLN("Starting radar on core 1\n");
  multicore_reset_core1();
  //multicore_launch_core1(radar_loop);
  multicore_launch_core1_with_stack(radar_loop, core1_stack, sizeof(core1_stack));
     
  SERIALPRINTLN("Waiting for radar ready signal\n");
  uint32_t fifo_message = 0;
  do
  {
   multicore_fifo_pop_timeout_us(200, &fifo_message);
   car.HazardLightsOn();
  }
  while ( fifo_message != RADAR_READY_FIFO_MESSAGE );
  SERIALPRINTLN("Radar ready!\n");
  car.HazardLightsOff();
  DriveMotorSpeedControlTicker.start();
}


// CORE0 -------------------------------------------------------------------------------------
// Core0 loop - navigation tasks
void loop()
{
  DriveMotorSpeedControlTicker.update();
  car.Iterator();

}


// CORE1 -------------------------------------------------------------------------------------
// Core1 loop - peripheral tasks
void radar_loop() // Peripherals
{
  Radar radar;

  while (true)
  {
    radar.Iterator();
  }
}
