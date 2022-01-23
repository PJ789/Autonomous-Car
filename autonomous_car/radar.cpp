#include "Radar.h"

Radar::Radar()
{
  _gpio_init(FRONT_ULTRASOUND_TRIG_PIN);
  gpio_set_dir(FRONT_ULTRASOUND_TRIG_PIN, GPIO_OUT);
  _gpio_init(FRONT_ULTRASOUND_ECHO_PIN);
  gpio_set_dir(FRONT_ULTRASOUND_ECHO_PIN, GPIO_IN);
  _gpio_init(REAR_ULTRASOUND_TRIG_PIN);
  gpio_set_dir(REAR_ULTRASOUND_TRIG_PIN, GPIO_OUT);
  _gpio_init(FRONT_ULTRASOUND_ECHO_PIN);
  gpio_set_dir(FRONT_ULTRASOUND_ECHO_PIN, GPIO_IN);
  _gpio_init( RADAR_ACTIVE_PIN);
  gpio_set_dir( RADAR_ACTIVE_PIN,    GPIO_OUT);
  gpio_put(RADAR_ACTIVE_PIN, false);

  RotateTurret(0);
  RotateTurret(-90);
  RotateTurret(0);
  RotateTurret(90);
  RotateTurret(0);
  
  SetDriveMotorSpeed(0);
  Drive(forward);
  Steer(none);
  
  multicore_fifo_push_timeout_us( RADAR_READY_FIFO_MESSAGE, 200 );
}
void Radar::Iterator()
{
  uint32_t fifo_message;

  while (multicore_fifo_pop_timeout_us(200, &fifo_message))
  {
    printf("fifo message received on core1\n");
    DecodeFifo( fifo_message );
  }

  //TurretRotationSequencer();
  ServoDiagnostics();
  
  multicore_fifo_push_timeout_us( RADAR_READY_FIFO_MESSAGE, 200 );
}
void Radar::SetRadarActivePin()
{
  gpio_put(RADAR_ACTIVE_PIN, ((( time_us_32()/1000) % 500)>250)?false:true); 
}
void Radar::TurretRotationSequencer()
{
  uint8_t radar_sequence;
  int16_t target_radar_turret_angle;
  uint32_t time_millis= 0;

   // use pico api to avoid core crashing on millis
  time_millis=time_us_32()/1000;

  if ( Turning() )
  {
    radar_sequence = (time_millis / ((2 * RADAR_ECHO_TIMEOUT_MILLIS) + (RADAR_SCAN_STEP_DEGREES * RADAR_SERVO_MILLIS_PER_DEGREE))) % 4;
    if (
        (DirectionIsForward() && TurningLeft())
        ||
        (DirectionIsReverse() && TurningRight())
        )
    {
      switch(radar_sequence) {
        case 0:
          target_radar_turret_angle = ( -2 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 1:
          target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 2:
          target_radar_turret_angle = (  0 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 3:
          target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
          break;
      }
    }
    else
    {
      switch(radar_sequence) {
        case 0:
          target_radar_turret_angle = (  2 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 1:
          target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 2:
          target_radar_turret_angle = (  0 * RADAR_SCAN_STEP_DEGREES );
          break;
        case 3:
          target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
          break;
      }
    }
  }
  else
  {
    radar_sequence = (time_millis / ((2 * RADAR_ECHO_TIMEOUT_MILLIS) + (RADAR_SCAN_STEP_DEGREES * RADAR_SERVO_MILLIS_PER_DEGREE))) % RADAR_SCAN_STEPS;
    switch(radar_sequence) {
      case 0:
        target_radar_turret_angle = (  2 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 1:
        target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 2:
        target_radar_turret_angle = (  0 );
        break;
      case 3:
        target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 4:
        target_radar_turret_angle = ( -2 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 5:
        target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 6:
        target_radar_turret_angle = (  0 );
        break;
      case 7:
        target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
        break;
    }
  }      
  if (target_radar_turret_angle != TurretDirection() )
  {
    RotateTurret(target_radar_turret_angle);
    MeasureFront();
    MeasureRear();
  }
  
}
uint8_t Radar::TurretDirection()
{
  return (ConvertServoAngleToCarAngle(radar_turret_angle));
}
void Radar::RotateTurret(int16_t car_target_angle)
{
  int16_t  radar_turret_target_angle;
  int16_t  radar_turret_angle_delta;
  uint32_t radar_turret_turn_time;

  car_target_angle = (car_target_angle < -90)?-90:car_target_angle;
  car_target_angle = (car_target_angle >  90)? 90:car_target_angle;

  radar_turret_target_angle = ConvertCarAngleToServoAngle(car_target_angle);
  
  radar_turret_angle_delta = (radar_turret_angle>radar_turret_target_angle)?(radar_turret_angle-radar_turret_target_angle):(radar_turret_target_angle-radar_turret_angle);
  radar_turret_turn_time = radar_turret_angle_delta * RADAR_SERVO_MILLIS_PER_DEGREE;
 
  radar_turret.SetDegrees(radar_turret_target_angle);
  TIMEWAIT_MILLIS(radar_turret_turn_time );

  radar_turret_angle = radar_turret_target_angle;
}
uint32_t Radar::MeasureFront()
{
  uint32_t range;
  uint32_t measurement_message;

  range = Measure( FRONT_ULTRASOUND_TRIG_PIN,FRONT_ULTRASOUND_ECHO_PIN );
  range = (range > 255)?255:range;

  measurement_message = 'R'<<24 | 'F'<<16 | radar_turret_angle<<8 | range;
  multicore_fifo_push_timeout_us(measurement_message,1000);

  return range;
}
uint32_t Radar::MeasureRear()
{
  uint32_t range;
  uint32_t measurement_message;

  range = Measure( REAR_ULTRASOUND_TRIG_PIN,REAR_ULTRASOUND_ECHO_PIN );
  range = (range > 255)?255:range;

  measurement_message = 'R'<<24 | 'R'<<16 | radar_turret_angle<<8 | range;
  multicore_fifo_push_timeout_us(measurement_message,1000);

  return range;
}
float Radar::Measure( uint8_t trigger, uint8_t echo )
{
  float distance;
  uint32_t echo_duration;

  // 10us pulse on trigger pin
  gpio_put(trigger, false);
  TIMEWAIT_MICROS(2);
  gpio_put(trigger, true);
  TIMEWAIT_MICROS(10);
  gpio_put(trigger, false);

  // Read echo time
  echo_duration = PulseIn(echo, HIGH, RADAR_ECHO_TIMEOUT_MICROS);
  
  echo_duration = (echo_duration == 0)?RADAR_ECHO_TIMEOUT_MICROS:echo_duration; // if timed out assume max distance

  // Speed of ultrasound 350m/s (0.035cm/us)
  distance = echo_duration * 0.035 / 2.0; 

  return distance;
}
void Radar::ServoDiagnostics()
{
  for (int test=0;test<366;test++)
  {
    switch (test)
    {
      case 0:
        printf("Servo Calibration\n");
        printf("-90\n");
        RotateTurret(-90);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
        break;
      case 1:
        printf("Zero\n");
        RotateTurret(0);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
  
        break;
      case 2:
        printf("90\n");
        RotateTurret(90);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
  
        break;
      case 3:
        printf("Zero\n");
        RotateTurret(0);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
  
        break;
      case 4:
        printf("Sweep\n");
        break;
      case 5 ... 185:
        RotateTurret(test-95);
        printf("%i\n",test-95);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
  
        break;
      case 186 ... 366:
        RotateTurret(276-test);
        printf("%i\n",276-test);
        printf("Radar measurements (f/r):%lu/%lu\n",MeasureFront(),MeasureRear());
  
        break;
      default:
        printf("Restarting test cycle");
        test=-1;
        break;
    }
  }


  return;
}
void Radar::SetDriveMotorSpeed( uint8_t s )
{
  last_drive_motor_speed = s;
}
void Radar::Drive( drive_motor_direction d )
{
  last_drive_motor_direction = d;
}
void Radar::Steer( steering_motor_direction d )
{
  last_steering_motor_direction = d;
}
bool Radar::Turning()
{
  return ( last_steering_motor_direction != none );
}
bool Radar::TurningLeft()
{
  return ( last_steering_motor_direction == left );
}
bool Radar::TurningRight()
{
  return ( last_steering_motor_direction == right );
}
bool Radar::DirectionIsForward()
{
  return (last_drive_motor_direction == forward);
}
bool Radar::DirectionIsReverse()
{
  return (last_drive_motor_direction == reverse);
}
uint32_t Radar::PulseIn( uint32_t pin, uint32_t state, uint32_t timeout )
{
  uint32_t start_us;
  uint32_t pulse_start_us;
  uint32_t width = 0;

  // wait for pulse start
  start_us = time_us_32();
  while(
      (gpio_get(pin)!=state)
      &&
      ((time_us_32() - start_us)<timeout)
    );

  if((time_us_32() - start_us)<timeout)
  {
    // wait for pulse end
    pulse_start_us = time_us_32();
    while(
        (gpio_get(pin)==state)
        &&
        ((time_us_32() - start_us)<timeout)
      );
    width = (time_us_32() - pulse_start_us);
  
    TIMEWAIT_MICROS( timeout - width ); 
  }

  return width;
}
void Radar::DecodeFifo(uint32_t fifo_message)
{
  char decoded_message[5];

  memset(decoded_message, 0, sizeof(decoded_message));
  decoded_message[0] = (fifo_message & 0xFF000000) >> 24;
  decoded_message[1] = (fifo_message & 0x00FF0000) >> 16;
  decoded_message[2] = (fifo_message & 0x0000FF00) >> 8;
  decoded_message[3] = (fifo_message & 0x000000FF)    ;

  printf("%lu - %s\n", fifo_message, decoded_message );

  switch (decoded_message[0])
  {
    case 'D': // drive direction
      switch (decoded_message[1])
      {
        case 'F': // forward
          printf("Drive motor direction forward signal received");
          Drive(forward);
          break;
        case 'R': // reverse
          printf("Drive motor direction reverse signal received");
          Drive(reverse);
          break;
        case 'S': // speed
          printf("Drive motor speed received: %i",decoded_message[3]);
          SetDriveMotorSpeed(decoded_message[3]);
          break;
        default:
          printf("Unrecognised drive motor message type [%c][%s]\n", decoded_message[1], decoded_message);
          break;
      }
      break;
    case 'S': // steering direction
      switch (decoded_message[1])
      {
        case 'L': // left
          printf("Steering motor steer left signal received");
          Steer(left);
          break;
        case 'R': // reverse
          printf("Steering motor steer right signal received");
          Steer(right);
          break;
        case 'N': // none
          printf("Steering motor steer none signal received");
          Steer(none);
          break;
        default:
          printf("Unrecognised steering motor message type [%c][%s]\n", decoded_message[1], decoded_message);
          break;
      }
      break;
    default:
      printf("Unrecognised motor message type [%c][%s]\n", decoded_message[0], decoded_message);
      break;
  }
}

int16_t Radar::ConvertServoAngleToCarAngle(uint8_t servo_angle)
{
  int16_t car_angle;
  
  servo_angle = (servo_angle>180)?180:servo_angle;
  car_angle = servo_angle;
  car_angle = (-1*(car_angle-90));

  return car_angle;
}

uint8_t Radar::ConvertCarAngleToServoAngle(int16_t car_angle)
{
  uint8_t servo_angle;

  car_angle = (car_angle> 90)? 90:car_angle;
  car_angle = (car_angle<-90)?-90:car_angle;
  car_angle = 90-car_angle;

  servo_angle = car_angle;
  
  return servo_angle;
}
