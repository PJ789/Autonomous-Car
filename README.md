# Autonomous-Car
An autonomous model vehicle based on a Raspberry Pi Pico, using servo motors, DC motors, PWM speed control, ultrasound sensors, interrupts, and multi-core code.

<img src="autonomous car picture - Copy.png"> 

## Overview

This project repurposes an old remote control model car to become an autonomous vehicle.

[Watch a video of a test run.](https://github.com/PJ789/Autonomous-Car/raw/refs/heads/main/car_test.mp4)

The car is capable of 'Partial Automation'; controlling steering & acceleration/braking simultaneously,  maintaining distance from obstacles in its path, manoeuvering, but needs human supervision. 

The car uses a Raspberry Pi Pico for its brains... running code on both cores, using the _official_ [Arduino Core based on Mbed RTOS which includes support for Pico.](https://github.com/arduino/ArduinoCore-mbed/tree/master/variants/RASPBERRY_PI_PICO).

- Core0 provides DC steering motor & drive motor control, and route planning.

- Core1 operates an ultrasound 'radar' turret mounted on the roof
 
The code provides examples of;
 - A complex Pico project, compiled under the Arduino IDE
 - Multi-core programming using the [Pico C/C++SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
 - SG90 micro servo control using PWM
 - DC motor direction control using GPIO to switch a DPDT mechanical relay using a MOSFET
 - DC motor speed control using GPIO & PWM to switch a MOSFET
 - Interrupt driven measurement from ultrasound sensors

The 'radar' turret is [based on this own design 3D printed kit](https://www.tinkercad.com/embed/0e6vV6PrGs4?editbtn=1) and a pair of back-to-back ultrasound sensors. The ultrasound turret sits on top of a digital servo.

### 8 Volt Main Battery Circuit
The main battery is a 6 cell NiMH battery pack (circa 8v).

The car has a custom ESC (electronic speed control) circuit for the 8v main driving motor. The ESC circuit has a mechanical DPDT relay, switched by an IRLZ44N MOSFET.

The DPDT relays control forward/backward DC motor direction (by reversing the polarity of the power to the driving DC motor).

The ESC circuit also has a second IRLZ44N MOSFET, used to provide PWM speed control over the power to the motor.

### 5 Volt Motor Circuit
A buck convertor is used to efficiently supply 5v power (reducing the voltage from the main battery pack). 

The turret uses a 5v DOMAN S0090MD metal gear 9g digital servo. It rotates 180°, directing forward & rearward facing HC-SR04 ultrasound sensors (giving 360° coverage). NB: The ultrasound sensors are connected to the digital circuit.

The steering rack has been modified to use a 5v EMAX ES08MA II 12g metal gear analogue servo (rather than the DC 'bang-bang' motor it was originally equipped with).

The motors & their batteries are completely isolated from the digital circuitry using 817 optocouplers. There is no common ground between analogue & digital circuits.

### 5 Volt Digital Circuit
The digital electronics are powered from a separate high capacity 5v USB powerbank.

The digital circuit comprises a Raspberry Pico, two HC-SR04  ultrasound sensors, and a TXS0108E 8 Channel Bi-Directional Logic Level Converter.

The logic level converter is used to interface the Pico to the ultrasound sensors, to boost the 3.3v Pico GPIO ultrasound trigger pins up to 5v, and bring the 5v ultrasound echo pin down to 3.3v.

6 leds 'lamps' (2x white headlights, 2x rear reversing lights, 2x rear brake lights) are installed at each corner of the car, and one red warning LED on the roof to signal loss of communication between core0 & core1 of the Pico.

### Circuit Diagram
A full circuit diagram is available [here](https://raw.githubusercontent.com/PJ789/Autonomous-Car/refs/heads/main/Autonomous%20Car%20Circuit%20Diagram.png)

## FIFO Protocol

The tasks running on the two cores use a simple protocol to exchange status information. Core0 (steering motor control, drive motor control, and route planning) sends vehicle motor status data to core1. And in the reverse direction, core 1 (operating the ultrasound 'radar' turret) sends obstacle distance & direction information to core0.

Core0 uses the 'radar' data to decide on the next manoeuvre, core1 uses the vehicle motor information to direct the 'radar' toward the expected path of the vehicle (ideally both then face in the same direction).

Message formats are 32 bit values, as follows.

### Sent from core0 to core1

**C[F|R][L|R|N]\<speed as a byte\>** - a car status message conveying forward or reverse state, left right or none turn state, and speed (max 255, not currently used on core1). 

### Sent from core1 to core0 

**_RDY** - a signal that the radar on core1 is up & operational (used at startup to ensure the vehicle does not set off without working radar). This is periodically restransmitted.

**R[F|R]\<radar angle as a byte\>\<obstacle range \(in 10cm units\) as a byte\>** - radar status message conveying forward or reverse sensor reading, turret angle (expressed relative to servo position, 0-180°, see geometry pictures), obstacle range (from 0 to 2550cm in 10cm units resolution). 

**D\<value 1 as a byte\>\<value 2 as a byte\>\<value 3 as a byte\>** - radar or turret debug message allowing three bytes to be passed from core1 to core0 to be printed on the Serial console. 

If at any point a message push to the FIFO times out on core1, a warning LED on pin 8 is lit, which is used to indicate loss of radar data sync between core1 and core0. This is a bad situation, because it means the car is navigating without accurate radar information.

## Geometry Diagrams

Car geometry is used to abstract from the underlying servo geometry (which is transverse to the car, and anti-clockwise/inverted because the servo is _under_ the roof). Steps Left 3, Left 2, Left 1 etc are simply constants/shorthand for the sequence of 25° angular positions the turret iterates through, and used in the route planning algorithm.

<img src="car_geometry.png" height="650"> <img src="servo_geometry.png" height="650">

## Some Pico Programming Notes

- This code compiles under the Arduino IDE, but also uses the Pico SDK. This requires some awkward workarounds; particularly because the Arduino/mbed SDK 'printf' function doesn't work on Core1, the Servo library also does not work on Core1 (being time sensitive), and various commonplace Arduino/mbed time related functions are unstable on core1.
- Pico PWM settings affect multiple pins in a Pico PWM group. If you use PWM, you should avoid using other pins in the same PWM group unless they share the same PWM characteristics. [See also these useful notes; 'making sure not to use two GPIO pins having the same number and letter designation together'](https://www.etechnophiles.com/raspberry-pi-pico-pinout-specifications-datasheet-in-detail/). This page also offers some good technical insights; [RandomNerdTutorials.com:Raspberry PicoPWM Pins](https://randomnerdtutorials.com/raspberry-pi-pico-w-pinout-gpios/#pwm-pins). I run both my 9g servos (turret & steering) from one group (@50Hz), and the DC main motor from another (@20KHz).
- Core1 seems very sensitive to low power conditions; you need a robust 5V power supply, or else core1 will stop functioning. Adding capacitors to the power supply helps. Keep DC motors and digital electronics on different opto-isolated circuits.
- To avoid concurrency problems when writing multi-core code, avoid using any Arduino or mbed APIs, particularly on core1, and particularly so Serial.print, Servo library, or Arduino timer related functions. The Arduino/mbed APIs are apparently not designed for multi-core MCUs, and thus using them on core1 will crash the Pico... leading to flashing LED error codes and requiring a complete reset. 
- To avoid concurrency problems, I also avoided creating heap objects on core1 (so relying on the core1 stack memory almost exclusively, which is not shared with Core0)
- The default stack size on both cores seems quite small, especially if you are using C++ object oriented code libraries, and also using the stack to avoid creating heap objects (see above). So I increased my stack sizes to the limit by setting compiler flags (-DPICO_STACK_SIZE=0x1000 -DPICO_CORE1_STACK_SIZE=0x1000), also set a custom stack size for code running on core1 in code.
- To calculate the PWM frequency for a servo, you take the system clock (default 125mhz), divide it by the maximum PWM counter value (default 65535) to arrive at the current PWM frequency. Then pick a clock divider to align with your servo PWM frequency needs. Take a look at the [servo code](https:autonomous_car/servo.cpp) for an example of how this works.
- If you're going to try your hand at multi-core programming... wire a push button switch to the Pico reset pin. You're going to need it. A lot. (That yellow push button on the front of the bonnet? Take a wild guess what that's for).

## Some Apologies
Sorry about the code quality, its ok, but not my best work. Route planning is a bit rudimentary at present. And sorry about any bugs.

## Some Thoughts

I've worked on this project for a few years, on and off. Now its working, I can offer some suggestions to greatly simplify your life;-

- Use an off the shelf ESC (electronic speed controller) to control your main drive motor. They're cheap, simple to interface using PWM, and avoid lots of the complications of building your own motor control circuits (voltage spikes & so forth particularly). I wish I'd done that.
- Use an analogue servo (or digital servo if you prefer) to control steering. If your RC car donor doesn't have a servo for steering (toy cars often have a simplistic DC 'bang-bang' motor) you should consider an upgrade. I have converted my car using a 3D printed mount for the servo [See video here](servo_steering.mp4) The old steering DC motor & reduction gears were removed, and an analogue servo installed on a custom 3D printed bracket. The servo horn pushes the old steering pitman arm which is still mounted on the (empty) steering gearbox. The steering gearbox is effectively acting as a simple bearing for the steering pitman arm axle. Controlling a PWM 9g servo is far easier than a DC motor. I wish I'd done that earlier in the project. NB the turret is best controlled by a digital servo; ideally you want accurate turret positioning at least. Steering isn't so critical (until you crash).
- Use a LIDAR module, not ultrasound. They're ten times more expensive, but ultrasound is simply too slow for a fast car. You need fast accurate dependable sensing to travel at speed, and scanning ultrasound is simply not fast enough to make it reliable, so you have to cut your speed. Which sucks. You can't even operate multiple cheap ultrasound module simultaneously; they will all 'hear' each others echo and give you false results, so you can only sample one at a time.
- Use a buck convertor module to bring down the battery voltage to 5v, not a voltage regulator. You need a solid power source with lots of amps.
- Use a separate battery for the digital electronics, one that outlasts the main battery. You don't want the control circuit to expire before the motors (because that creates an unguided missile). USB powerbanks are good for the job.
- Use fresh rechargable batteries. Old NiMH batteries are junk, the internal resistance is terrible.. I found that out the hard way, so you don't have to.
- Use opto-couplers to separate the noisey main/steering motor circuits from the digital electronics _entirely_. It makes a big difference to the stability of your system, the Pico seems quite vulnerable to voltage spikes.
- Twist all your power leads, to reduce electro-magnetic interference. The Pico seems vulnerable to EMI, crashing in strange and unpredictable ways.
- Pay careful attention to interrupt handling code on core1 (used for ultrasound). It is hard enough on core0, but on core1 even more so because you have no way to add debug without sending FIFO messages to core0..

## Roadmap (pun intended)

- Switch chassis, to a hobby grade HSP/Himoto 1/16 buggy platform with a 390 main motor, ESC controlled main motor, and 3 wire servo steering
- Implement LIDAR, for higher speeds, using adaptive 270° overlapping field of view scan zones (180° forward + 90° left side, 180° rearward + 90° right side)
- Implement [wheel hub magnets based on this design](https://www.tinkercad.com/things/0p47F2eLgXn-rc-car-wheel-insert-for-neodymium-magnets) and integrate with [A3214EUA-T Hall Effect sensor](https://www.allegromicro.com/-/media/files/datasheets/a3213-4-datasheet.pdf) for speed measurement
- Add 'oh hell no' sensors for permanent last resort accident prevention
- Integrate [HLK-LD2415H millimeter wave radar module](https://www.hlktech.net/index.php?id=1220) or similar for long range vehicle & obstacle tracking.
- ~~Switch to servo controlled steering, for simplicity & finesse~~ Done! 
- Add accelerometers for orientation, impact detection, acceleration/deceleration measurement
- Pair the Pico module with an ESP32 S3 CAM module as a co-processor, for wifi streaming of monitoring, diagnostics info
- Video streaming from the car (see next item)
- Investigate [Nvidia ROS ](https://developer.nvidia.com/isaac/ros#section-starter-kits)

 For my finale,
- Attempt [Nvidia Alpamayo AI integration](https://developer.nvidia.com/drive/alpamayo) for a truly AI autonomous model vehicle

## Some Caveats

The code is a work in progress, and for interest/reference. Mostly, it's working fully now, apart from the bits where it doesn't and crashes (the car, not the code). E&OE.

If you find it useful, please enjoy it. If you have suggestions for improvement, I'd be very interested.

regards
Pete.
