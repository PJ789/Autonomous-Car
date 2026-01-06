# Autonomous-Car
An autonomous model vehicle based on a Raspberry Pi Pico, using servo motors, DC motors, PWM speed control, ultrasound sensors, interrupts, and multi-core code.

## Overview

This project repurposes an old remote control model car to become an autonomous model vehicle.

The car uses a Raspberry Pi Pico for its brains... running code on both cores, using the _official_ [Arduino Core based on Mbed RTOS which includes support for Pico.](https://github.com/arduino/ArduinoCore-mbed/tree/master/variants/RASPBERRY_PI_PICO).

- Core0 provides DC steering motor & drive motor control, and route planning.

- Core1 operates an ultrasound 'radar' turret mounted on the roof
 
The code provides examples of;
 - A complex Pico project, compiled under the Arduino IDE
 - Multi-core programming using the [Pico C/C++SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
 - SG90 miniature servo position control using PWM
 - DC motor direction control using GPIO to switch a DPDT mechanical relay using a MOSFET
 - PWM DC motor speed control using GPIO to switch a MOSFET
 - Interrupt driven measurement from ultrasound sensors

The 'radar' turret is [based on this 3D printed kit](https://www.tinkercad.com/embed/0e6vV6PrGs4?editbtn=1) and a pair of back-to-back HC-SR04 ultrasound sensors. The ultrasound turret sits on top of a [DOMAN S0090MD metal gear 9g digital servo](http://www.domanrchobby.com/content/?130.html).

The car has two custom ESC (electronic speed control) circuits. Each of the two ESC circuits has an IRLZ44N MOSFET, for switching a mechanical DPDT relay. The DPDT relays control forward/backward and left/right DC motor direction (by reversing the polarity of the power to the driving and steering DC motors). Each of the two ESC circuits also has a second IRLZ44N MOSFET, used to provide PWM speed control over the power to the motors.

The DC motors & their batteries are completely isolated from the digital circuitry using 817 optocouplers.

A 5V/3.3V TXS0108E 8 Channel Bi-Directional Logic Level Converter is used to interface to the HC-SR04 ultrasound sensor, to boost the 3.3V Pico GPIO trigger pins up to 5V, and bring the HC-SR04 echo pin down to 3.3v.

A buck convertor is used to efficiently supply the steering motor with 5v power, from the 7.2v NiMH battery pack used for the main motor. The digital circuit is powered from a separate high capacity 5v USB powerback.


## FIFO Protocol

The code running on the two cores uses a simple protocol to send vehicle motor status data from core0 (steering motor control, drive motor control, and route planning) to core1 (operating the ultrasound 'radar' turret). And in the reverse direction, ultrasound distance & direction information from core1 to core0.

Message formats are 32 bit values, as follows.

### Sent from core0 to core1

**C[F|R][L|R|N]\<speed as a byte\>** - a car status message conveying forward or reverse state, left right or none turn state, and speed (max 255, not currently used on core1). The direction of travel is used by core1 to orientate the radar turret towards the expected path of the vehicle (ideally both then face in the same direction).

### Sent from core1 to core0 

**_RDY** - a signal that the radar on core1 is up & operational (used at startup to ensure the vehicle does not set off without working radar). This is periodically restransmitted.

**R[F|R]\<radar angle as a byte\>\<obstacle range \(in 10cm units\) as a byte\>** - radar status message conveying forward or reverse sensor reading, turret angle, range from 0 to 2550cm using 10cm unit resolution. The radar data is then used by core0 to perform route planning.

**D\<value 1 as a byte\>\<value 2 as a byte\>\<value 3 as a byte\>** - radar or turret debug message allowing three bytes to be passed from core1 to core 0 to be printed on the Serial console. 

If at any point a message push to the FIFO times out on core1, a warning LED on pin 8 is lit, which is used to indicate loss of radar data sync between core1 and core0. This is a bad situation, because it means the car is navigating without accurate radar information.

## Some Pico Programming Notes

- This code compiles under the Arduino IDE, but also uses the Pico SDK. This requires some awkward workarounds; particularly because the Pico SDK 'printf' function doesn't work on Core1, and various commonplace Arduino time related functions are unstable on core1.
- Pico PWM settings affect multiple pins in a group. If you use PWM, it seems you should avoid using other pins in the same group, else you can experience strange side-effects. [See also these useful notes; 'making sure not to use two GPIO pins having the same number and letter designation together'](https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/)
- Core1 seems very sensitive to low power conditions; you need a robust 5V power supply, or else Core1 will stop functioning. Adding capacitors to the power supply helps. Keep DC motors and digital electronics on different opto-isolated circuits.
- To avoid concurrency problems when writing multi-core code, avoid using any Arduino or mbed APIs, particularly on Core1, and particularly so Serial.print or Arduino timer related functions. The Arduino/mbed APIs are apparently not designed for multi-core MCUs, and thus using them on Core1 will crash the Pico... leading to flashing LED error codes and requiring a complete reset. 
- To avoid concurrency problems, I also avoided creating heap objects on Core1 (so relying on the Core1 stack almost exclusively, which is not shared with Core0)
- The default stack size on both cores seems quite small, especially if you are using C++ object oriented code libraries, and also using the stack to avoid creating heap objects (see above). So I increased my stack sizes to the limit by setting compiler flags (-DPICO_STACK_SIZE=0x1000 -DPICO_CORE1_STACK_SIZE=0x1000), also set a custom stack size for code running on core1 in code.
- To calculate the PWM frequency for a servo, you take the system clock (default 125mhz), divide it by the maximum PWM counter value (default 65535) to arrive at the current PWM frequency. Then pick a clock divider to align with your servo PWM frequency needs. Take a look at the [servo code](https:autonomous_car/servo.cpp) for an example of how this works.
- If you're going to try your hand at multi-core programming... wire a push button switch to the Pico reset pin. You're going to need it. A lot.

## Some Apologies
Sorry about the code quality. Route planning is elementary at present, tba. And sorry about any bugs.

## Some Thoughts

I've worked on this project for a few years, on and off. Now its working, I can offer some suggestions to greatly simplify your life;-

- Use an off the shelf ESC (electronic speed controller) to control your main drive motor. They're cheap, simple to interface using PWM, and avoid lots of the complications of building your own motor control circuits (voltage spikes & so forth particularly). I wish I'd done that.
- Use a servo to control steering. If your RC car donor doesn't have a servo for steering (mine has a DC motor), you should make it so. I will do that at some point. Controlling a servo is a lot easier than trying to control a DC motor, and gives you much better control. I wish I'd done that.
- Use a LIDAR module, not ultrasound. They're ten times more expensive, but ultrasound is simply too slow for a fast car. You need fast accurate sensing to travel at speed, and scanning ultrasound is simply not fast enough to make it reliable, so you have to kill the speed.
- Use a buck convertor module to bring down the battery voltage to 5v, not a voltage regulator. You need a solid power source with lots of amps.
- Use a separate battery for the digital electronics, one that outlasts the main battery. You don't want the control circuit to expire before the motors (because that creates an unguided missile). USB powerbanks are good for the job.
- Use fresh rechargable batteries. Old NiMH batteries are junk, the internal resistance is terrible.. I found that out the hard way, so you don't have to.
- Use opto-couplers to separate the main/steering motors from the digital electronics entirely. It makes a big difference to the stability of your system, the Pico seems quite vulnerable to voltage spikes.
- Twist all your power leads, to reduce electro-magnetic interference. The Pico seems vulnerable to EMI, crashing in strange and unpredictable ways.
- Pay careful attention to interrupt handling code on core1 (used for ultrasound). It is hard enough on core0, but on core1 even more so because you have no way to add debug.

## Roadmap (pun intended)

- Switch chassis, to a hobby grade platform with a 390 motor, servo steering, and ESC controlled main motor
- Implement LIDAR, for higher speeds, using adaptive 270 overlapping scan zones (forward + left, rearward + right)
- Add 'oh hell no' sensors for permanent last resort accident prevention
- Switch to servo controlled steering, for simplicity & finesse
- Add accelerometers for orientation, impact detection, acceleration/deceleration measurement
- Pair the Pico module with an ESP32 S3 CAM module as a co-processor, for wifi monitoring, diagnostics
- Video streaming from the car (see next item)

 For my finale,
- Attempt Nvidia Alpamayo AI integration https://developer.nvidia.com/drive/alpamayo for a truly AI autonomous model vehicle

## Some Caveats

The code is a work in progress, and for interest/reference. Mostly, it's working fully now, apart from the bits where it doesn't and crashes (the car, not the code). E&OE.

If you find it useful, please enjoy it. If you have suggestions for improvement, I'd be very interested.

regards
Pete.
