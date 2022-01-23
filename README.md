# Autonomous-Car
An autonomous vehicle based on a Raspberry Pi Pico, using servo motors, DC motors, PWM speed control, ultrasound sensors, multicore code.

## Overview

This project repurposes an old RC car to become an autonomous vehicle.

The car uses a Raspberry Pi Pico for its brains... running code on both cores.

- Core0 provides steering, DC motor control (using MOSFETs to switch mechanical relays), and route planning.

- Core1 operates an ultrasound 'radar' turret mounted on the roof [based on this 3D printed kit](https://www.tinkercad.com/embed/0e6vV6PrGs4?editbtn=1) and a pair of back-to-back HC-SR04 ultrasound sensors. The ultrasound turret sits ontop of a [DOMAN S0090MD metal gear 9g digital servo](http://www.domanrchobby.com/content/?130.html).

The code provides examples of;
 - A complex Pico project, compiled under the Arduino IDE
 - Multi-core programming using the [Pico C/C++SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
 - Servo position control using PWM
 - DC motor direction control using GPIO to switch a DPST mechanical relay using a MOSFET
 - DC motor speed control using PWM via a MOSFET
 - Ultrasound sensor use

The car has two custom circuits, each with a MOSFET switching a mechanical DPST relay. The relays control forward/backward and left/right DC motor direction (by reversing the polarity of the DC motors). Each circuit has a second MOSFETs, used to provide power on/off/PWM speed control.

A 5V/3.3V TXS0108E 8 Channel Bi-Directional Logic Level Converter is used, where necessary, to boost the Pico GPIO pins upto 5V.

## Some Notes

- This code compiles under the Arduino IDE, but uses the Pico SDK. This creates some awkward problems; particularly the Pico SDK 'printf' function doesn't work. I'm still trying to find a workaround.
- Pico PWM settings affect multiple pins in a group. If you use PWM, it seems you should avoid using other pins in the same group, else you can experience strange side-effects.
- Core1 seems very sensitive to low power conditions; you need a robust 5V power supply, or else core1 will stop functioning.
- To avoid concurrency problems when writing multi-core code, avoid using any Arduino or mbed APIs (they are not designed for multi-core MCUs, and may crash the Pico... leading to flashing error codes and requiring a reset). 
- To avoid concurrency problems, I also avoided creating heap objects on Core1 (so relying on the Core1 stack exclusively, which is not shared with Core0)
- The default stack size on both cores seems quite small, especially if you are using C++ object oriented code libraries and using the stack to avoid creating heap objects (see above). I increased my stack sizes by setting compiler flags (-DPICO_STACK_SIZE=0x1000 -DPICO_CORE1_STACK_SIZE=0x1000)
- To calculate the PWM frequency for a servo, you take the system clock (default 125mhz), divide it by the maximum PWM counter value (default 65535) to arrive at the current PWM frequency. Then pick a clock divider to align with your servo PWM frequency needs. Take a look at the servo code for an example of how this works.
- If you're going to try your hand at multi-core programming... wire a push button switch to the Pico reset pin. You're going to need it. A lot.
-  I didn't use the TXS0108E Logic Level Converter for the PWM signals to the turret servo... it doesn't seem to be fast enough. Wiring the servo power to the 5V/ground lines, but hooking the Pico GPIO PWM pin direct to the servo's PWM input worked fine. Mostly.

## Some Apologies

Sorry there are no circuit diagrams. Sorry about the code quality. And sorry about any bugs.

## Some Caveats

The code is a work in progress, and offered for interest only. Some elements don't work fully, some don't work at all. E&OE.

If you find it useful, please enjoy it. If you have suggestions for improvement, I'd be very interested.

regards
Pete.
