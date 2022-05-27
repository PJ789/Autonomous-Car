#include "Radar.h"

Radar::Radar():front_ultrasound(FRONT_ULTRASOUND_TRIG_PIN,FRONT_ULTRASOUND_ECHO_PIN),rear_ultrasound(REAR_ULTRASOUND_TRIG_PIN,REAR_ULTRASOUND_ECHO_PIN)
{
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
  
  if (multicore_fifo_wready())
    multicore_fifo_push_blocking( RADAR_READY_FIFO_MESSAGE );
}
void Radar::Iterator()
{
  uint32_t fifo_message;

  while (multicore_fifo_rvalid())
  {
    fifo_message = multicore_fifo_pop_blocking();
    DecodeFifo( fifo_message );
  }

  TurretRotationSequencer();
    
  if (multicore_fifo_wready())
    multicore_fifo_push_blocking( RADAR_READY_FIFO_MESSAGE );

}
void Radar::SetRadarActivePin()
{
  gpio_put(RADAR_ACTIVE_PIN, ((( time_us_32()/1000) % 500)>250)?false:true); 
}
void Radar::TurretRotationSequencer()
{
  uint8_t radar_sequence;
  int16_t target_radar_turret_angle;
  uint64_t time_millis;
  uint64_t rotation_interval_ms;
  uint64_t rotation_steps;

   // use pico api to avoid core crashing on millis
  time_millis=time_us_64()/1000;

  if ( Moving() && !Turning() )
  {
    // base interval, plus 1* ultrasound measurements plus rotation time
    rotation_interval_ms = 100 +(ULTRASOUND_ECHO_TIMEOUT_MILLIS + (RADAR_SCAN_STEP_DEGREES * RADAR_SERVO_MILLIS_PER_DEGREE));
    rotation_steps       = 4;
    radar_sequence = ( time_millis /rotation_interval_ms ) % rotation_steps;

    switch(radar_sequence) {
      case 0:
        target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 1:
      case 3:
        target_radar_turret_angle = (  0 );
        break;
      case 2:
        target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
        break;
    }
  }
  if ( Moving() && Turning() )
  {
    // base interval, plus 1* ultrasound measurements plus rotation time
    rotation_interval_ms = 100 +(ULTRASOUND_ECHO_TIMEOUT_MILLIS + (RADAR_SCAN_STEP_DEGREES * RADAR_SERVO_MILLIS_PER_DEGREE));
    rotation_steps       = 4;
    radar_sequence = ( time_millis /rotation_interval_ms ) % rotation_steps;

    if (
        (MovingForward() && TurningLeft())
        ||
        (MovingBackward() && TurningRight())
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
    else // MovingForward && TurningRight || Moving Backward && TurningLeft
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
  if ( Stopped() )
  {
    // base interval, plus 2* ultrasound measurements plus rotation time
    rotation_interval_ms = 100 +(2*ULTRASOUND_ECHO_TIMEOUT_MILLIS + (RADAR_SCAN_STEP_DEGREES * RADAR_SERVO_MILLIS_PER_DEGREE));
    rotation_steps       = 12;
    radar_sequence = ( time_millis / rotation_interval_ms ) % rotation_steps;
    switch(radar_sequence) {
      case 0:
        target_radar_turret_angle = (  3 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 1:
        target_radar_turret_angle = (  2 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 2:
        target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 3:
        target_radar_turret_angle = (  0 );
        break;
      case 4:
        target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 5:
        target_radar_turret_angle = ( -2 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 6:
        target_radar_turret_angle = ( -3 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 7:
        target_radar_turret_angle = ( -2 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 8:
        target_radar_turret_angle = ( -1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 9:
        target_radar_turret_angle = (  0 );
        break;
      case 10:
        target_radar_turret_angle = (  1 * RADAR_SCAN_STEP_DEGREES );
        break;
      case 11:
        target_radar_turret_angle = (  2 * RADAR_SCAN_STEP_DEGREES );
        break;
    }    
  }

  if (target_radar_turret_angle != GetTurretDirection() )
  {
    RotateTurret(target_radar_turret_angle);
  }

  if ( MovingForward()  || Stopped() )
  {
    MeasureFront();
  }
  if ( MovingBackward() || Stopped() )
  {
    MeasureRear();
  }
  
}
int8_t Radar::GetTurretDirection()
{
  return (ConvertServoAngleToCarAngle(radar_turret.GetDegrees()));
}
void Radar::RotateTurret(int16_t car_target_angle)
{
  int16_t  radar_turret_target_angle;
  int16_t  radar_turret_angle_delta;
  uint32_t radar_turret_turn_time;

  car_target_angle = (car_target_angle < -90)?-90:car_target_angle;
  car_target_angle = (car_target_angle >  90)? 90:car_target_angle;

  radar_turret_target_angle = ConvertCarAngleToServoAngle(car_target_angle);
  
  radar_turret_angle_delta = (radar_turret.GetDegrees()>radar_turret_target_angle)?(radar_turret.GetDegrees()-radar_turret_target_angle):(radar_turret_target_angle-radar_turret.GetDegrees());
  radar_turret_turn_time = radar_turret_angle_delta * RADAR_SERVO_MILLIS_PER_DEGREE;
 
  radar_turret.SetDegrees(radar_turret_target_angle);
  sleep_ms(radar_turret_turn_time );

}
uint32_t Radar::MeasureFront()
{
  uint32_t range;
  uint32_t measurement_message;

  range = front_ultrasound.Measure();
  range = (range > 255)?255:range;

  measurement_message = RADAR_FORWARD_RANGE_FIFO_MESSAGE;
  measurement_message &= 0xFFFF0000;
  measurement_message |= ((uint32_t)radar_turret.GetDegrees())<<8;
  measurement_message |= range;
  
  if (multicore_fifo_wready())
    multicore_fifo_push_blocking(measurement_message);

  return range;
}
uint32_t Radar::MeasureRear()
{
  uint32_t range;
  uint32_t measurement_message;

  range = rear_ultrasound.Measure();
  range = (range > 255)?255:range;

  measurement_message = RADAR_REAR_RANGE_FIFO_MESSAGE;
  measurement_message &= 0xFFFF0000;
  measurement_message |= ((uint32_t)radar_turret.GetDegrees())<<8;
  measurement_message |= range;
  
  if (multicore_fifo_wready())
    multicore_fifo_push_blocking(measurement_message);

  return range;
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
bool Radar::Stopped()
{
  return (last_drive_motor_speed == 0);
}
bool Radar::Moving()
{
  return ( MovingForward() || MovingBackward() );
}
bool Radar::MovingForward()
{
  return ( (last_drive_motor_direction == forward ) && (last_drive_motor_speed > 0) );
}
bool Radar::MovingBackward()
{
  return ( (last_drive_motor_direction == reverse ) && (last_drive_motor_speed > 0) );
}
bool Radar::DirectionIsForward()
{
  return (last_drive_motor_direction == forward);
}
bool Radar::DirectionIsReverse()
{
  return (last_drive_motor_direction == reverse);
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
    case 'C': // Car status
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
        default:
          printf("Unrecognised drive motor message type [%c][%s]\n", decoded_message[1], decoded_message);
          break;
      }
      switch (decoded_message[2])
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
          printf("Unrecognised steering motor message type [%c][%s]\n", decoded_message[2], decoded_message);
          break;
      }
      SetDriveMotorSpeed(decoded_message[3]);
      break;
    default:
      printf("Unrecognised message type [%c][%s]\n", decoded_message[0], decoded_message);
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
