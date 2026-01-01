#ifndef Ultrasound_h
#define Ultrasound_h

#include <stdint.h>
#include <algorithm>
#include <arduino.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include "autonomous_car.h"
#include "servo.h"


// 30000us = 525cm (0.035cm/us)
#define ULTRASOUND_ECHO_TIMEOUT_MICROS       60000
#define ULTRASOUND_ECHO_TIMEOUT_MILLIS       (ULTRASOUND_ECHO_TIMEOUT_MICROS/1000)

class Ultrasound
{

  public:

    Ultrasound(uint8_t, uint8_t);

    void UltraSound();
    float    Measure();

    static volatile uint32_t                  ultrasound_pulse_start;
    static volatile uint32_t                  ultrasound_pulse_duration;
    static void     UltrasoundIRQCallback(   uint, uint32_t);
   
  private:

    uint8_t  trigger_pin;
    uint8_t  echo_pin;


};



#endif
