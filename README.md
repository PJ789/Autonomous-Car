# Autonomous-Car
An autonomous vehicle based on a Raspberry Pi Pico, using servo motors, DC motors, PWM speed control, ultrasound sensors, interrupts, and multi-core code.

## Overview

This project repurposes an old RC car to become an autonomous vehicle.

The car uses a Raspberry Pi Pico for its brains... running code on both cores, using the _official_ [Arduino Core based on Mbed RTOS which includes support for Pico.](https://github.com/arduino/ArduinoCore-mbed/tree/master/variants/RASPBERRY_PI_PICO).

- Core0 provides DC steering motor & drive motor control, and route planning.

- Core1 operates an ultrasound 'radar' turret mounted on the roof
 
The code provides examples of;
 - A complex Pico project, compiled under the Arduino IDE
 - Multi-core programming using the [Pico C/C++SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
 - SG90 miniature servo position control using PWM
 - DC motor direction control using GPIO to switch a DPST mechanical relay using a MOSFET
 - PWM DC motor speed control using GPIO to switch a MOSFET
 - Interrupt driven measurement from ultrasound sensors

The 'radar' turret is [based on this 3D printed kit](https://www.tinkercad.com/embed/0e6vV6PrGs4?editbtn=1) and a pair of back-to-back HC-SR04 ultrasound sensors. The ultrasound turret sits ontop of a [DOMAN S0090MD metal gear 9g digital servo](http://www.domanrchobby.com/content/?130.html).

The car has two custom circuits, each with an IRLZ44N MOSFET switching a mechanical DPST relay. The relays control forward/backward and left/right DC motor direction (by reversing the polarity of the power to the DC motors). Each circuit has a second MOSFET, used to provide power on/off and PWM speed control.

A 5V/3.3V TXS0108E 8 Channel Bi-Directional Logic Level Converter is used, where necessary, to boost the 3.3V Pico GPIO pins up to 5V.

## Some Pico Programming Notes

- This code compiles under the Arduino IDE, but mostly uses the Pico SDK. This creates some awkward problems; particularly the Pico SDK 'printf' function doesn't work on Core1. I'm still trying to find a workaround.
- Pico PWM settings affect multiple pins in a group. If you use PWM, it seems you should avoid using other pins in the same group, else you can experience strange side-effects. [See also these useful notes; 'making sure not to use two GPIO pins having the same number and letter designation together'](https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/)
- Core1 seems very sensitive to low power conditions; you need a robust 5V power supply, or else Core1 will stop functioning. Adding capacitors to the power supply helps. Devices like servos can draw huge amounts of current.
- To avoid concurrency problems when writing multi-core code, avoid using any Arduino or mbed APIs, particularly on Core1, and particularly so Serial.print. The Arduino/mbed APIS are apparently not designed for multi-core MCUs, and thus using them on Core1 will crash the Pico... leading to flashing LED error codes and requiring a complete reset. 
- To avoid concurrency problems, I also avoided creating heap objects on Core1 (so relying on the Core1 stack almost exclusively, which is not shared with Core0)
- The default stack size on both cores seems quite small, especially if you are using C++ object oriented code libraries, and also using the stack to avoid creating heap objects (see above). So I increased my stack sizes to the limit by setting compiler flags (-DPICO_STACK_SIZE=0x1000 -DPICO_CORE1_STACK_SIZE=0x1000)
- To calculate the PWM frequency for a servo, you take the system clock (default 125mhz), divide it by the maximum PWM counter value (default 65535) to arrive at the current PWM frequency. Then pick a clock divider to align with your servo PWM frequency needs. Take a look at the [servo code](https:autonomous_car/servo.cpp) for an example of how this works.
- If you're going to try your hand at multi-core programming... wire a push button switch to the Pico reset pin. You're going to need it. A lot.

## Some Apologies
Sorry about the code quality. Route planning is elementary at present, tba. And sorry about any bugs.

## Some Caveats

The code is a work in progress, and for interest/reference. Some elements don't work fully, some don't work at all. E&OE.

If you find it useful, please enjoy it. If you have suggestions for improvement, I'd be very interested.

regards
Pete.
