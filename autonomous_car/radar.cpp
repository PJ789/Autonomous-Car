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

  last_drive_motor_direction = forward;
  last_steering_motor_direction = none;
  last_drive_motor_speed = 0;
  
  if (multicore_fifo_wready())
    multicore_fifo_push_timeout_us(RADAR_READY_FIFO_MESSAGE, 100);

}
void Radar::Iterator()
{
  uint32_t fifo_message;
  while (multicore_fifo_rvalid())
  {
    if (multicore_fifo_pop_timeout_us(100, &fifo_message))
    {
      DecodeFifo( fifo_message );
    }
  }

  // Switch to new position and send front, rear, or both measurements
  TurretRotationSequencer();

  // Attempt to send a radar ready message when the measurements have been received
  multicore_fifo_push_timeout_us(RADAR_READY_FIFO_MESSAGE, 100);

}
void Radar::SetRadarActivePin()
{
  gpio_put(RADAR_ACTIVE_PIN, ((( time_us_32()/1000) % 500)>250)?false:true); 
}
void Radar::TurretRotationSequencer()
{
  uint8_t radar_sequence            = 0;
  int16_t target_radar_turret_angle = 0;
  uint64_t time_millis              = 0;
  uint64_t rotation_interval_ms     = 0;
  uint64_t rotation_steps           = 0;

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
  else if ( Moving() && Turning() )
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
  else // if ( Stopped() )
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
  else
  {
      // prevent lockup
    sleep_ms(100);
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
    multicore_fifo_push_timeout_us(measurement_message, 100);

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
    multicore_fifo_push_timeout_us(measurement_message, 100);

  return range;
}
void Radar::ServoDiagnostics()
{
  for (int test=0;test<366;test++)
  {
    switch (test)
    {
      case 0:
        RotateTurret(-90);
        break;
      case 1:
        RotateTurret(0);         
        break;
      case 2:
        RotateTurret(90);        
        break;
      case 3:
        RotateTurret(0);          
        break;
      case 4:
        break;
      case 5 ... 185:
        RotateTurret(test-95);                  
        break;
      case 186 ... 366:
        RotateTurret(276-test);                  
        break;
      default:
        test=-1;
        break;
    }
  }  return;
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

  
  switch (decoded_message[0])
  {
    case 'C': // Car status
      switch (decoded_message[1])
      {
        case 'F': // forward
          Drive(forward);
          break;
        case 'R': // reverse
          Drive(reverse);
          break;
        default:
          break;
      }
      switch (decoded_message[2])
      {
        case 'L': // left
          Steer(left);
          break;
        case 'R': // reverse
          Steer(right);
          break;
        case 'N': // none
          Steer(none);
          break;
        default:
          break;
      }
      SetDriveMotorSpeed(decoded_message[3]);
      break; // Car status from car to radar
    default:
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

void Radar::SendFifoDebugMessage(char* message)
{
  uint32_t fifo_message;

  fifo_message = CORE1_DEBUG_FIFO_MESSAGE;
  fifo_message &= 0xFF000000;
  fifo_message |= ((uint32_t)message[0])<<16;
  fifo_message |= ((uint32_t)message[1])<<8;
  fifo_message |= ((uint32_t)message[2])<<0;

  while(!multicore_fifo_wready())
  {
    sleep_ms(1);
  }

  multicore_fifo_push_timeout_us(fifo_message, 100);
}