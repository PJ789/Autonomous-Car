#include "Radar.h"

Radar::Radar():front_ultrasound(FRONT_ULTRASOUND_TRIG_PIN,FRONT_ULTRASOUND_ECHO_PIN),rear_ultrasound(REAR_ULTRASOUND_TRIG_PIN,REAR_ULTRASOUND_ECHO_PIN)
{
  _gpio_init( RADAR_LOST_MESSAGE_PIN);
  gpio_set_dir( RADAR_LOST_MESSAGE_PIN,    GPIO_OUT);
  gpio_put(RADAR_LOST_MESSAGE_PIN, false);

  RotateTurret(0);
  RotateTurret(-90);
  RotateTurret(0);
  RotateTurret(90);
  RotateTurret(0);

  last_drive_motor_direction = forward;
  last_steering_motor_direction = none;
  last_drive_motor_speed = 0;
  
  for(int i=0;i<10;i++)
  {
    SetLostMessageWarningLamp();
    sleep_ms(50);
    ClearLostMessageWarningLamp();
    sleep_ms(50);
  }

  if (multicore_fifo_wready() && multicore_fifo_push_timeout_us(RADAR_READY_FIFO_MESSAGE, 100))
  {
    // sent fifo message, clear warning lamp for diagnostics
    // fifo messages are being transmitted sucessfully
    ClearLostMessageWarningLamp();
  }
  else
  {
    // failed to send fifo message, set warning lamp for diagnostics
    // if the lamp lights frequently we have a fifo message throughput issue
    SetLostMessageWarningLamp();
  };


}
void Radar::Iterator()
{
  // use pico api to avoid core crashing on millis
  static uint64_t last_ready_message_time_micros=0;
  uint64_t current_time_micros=0;
  uint32_t fifo_message;

  current_time_micros = time_us_64();
  // Attempt to send a radar ready message once every few seconds
  if (
    multicore_fifo_wready()
    &&
    ( 
      (!last_ready_message_time_micros) // first radar ready message?
      ||
      ((current_time_micros-last_ready_message_time_micros)>10000000) // timeout since last radar ready message?
    )
  )
  {
    if (multicore_fifo_push_timeout_us(RADAR_READY_FIFO_MESSAGE, 100))
    {
      // sent fifo message, clear warning lamp for diagnostics
      // fifo messages are being transmitted sucessfully
      ClearLostMessageWarningLamp();
    }
    else
    {
    // failed to send fifo message, set warning lamp for diagnostics
      // if the lamp lights frequently we have a fifo message throughput issue
      SetLostMessageWarningLamp();
    };
    last_ready_message_time_micros = current_time_micros;
  }
  
  while (multicore_fifo_rvalid())
  {
    fifo_message = multicore_fifo_pop_blocking();
    DecodeFifo( fifo_message );
  }

  // Switch to new position and send front, rear, or both measurements
  TurretRotationSequencer();

}
void Radar::SetLostMessageWarningLamp()
{
  gpio_put(RADAR_LOST_MESSAGE_PIN, true); 
}
void Radar::ClearLostMessageWarningLamp()
{
  gpio_put(RADAR_LOST_MESSAGE_PIN, false); 
}
void Radar::TurretRotationSequencer()
{
  int16_t target_radar_turret_angle = 0;

  if ( Moving() && !Turning() )
  {
    if (radar_turret_step>3) radar_turret_step=0;
    switch(radar_turret_step) {
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
    if (
        (MovingForward() && TurningLeft())
        ||
        (MovingBackward() && TurningRight())
        )
    {
      if (radar_turret_step>3) radar_turret_step=0;
      switch(radar_turret_step) {
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
      if (radar_turret_step>3) radar_turret_step=0;
      switch(radar_turret_step) {
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
    if (radar_turret_step>11) radar_turret_step=0;
    switch(radar_turret_step) {
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

  sleep_ms(RADAR_SERVO_SETTLING_TIME_MILLIS);

  if ( MovingForward()  || Stopped() )
  {
    MeasureFront();
  }

  if ( MovingBackward() || Stopped() )
  {
    MeasureRear();
  }
  radar_turret_step +=1;
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
float Radar::MeasureFront()
{
  float range_1cm;
  uint32_t range_10cm;
  uint32_t measurement_message;

  range_1cm = front_ultrasound.Measure();
  range_10cm = range_1cm / 10; // convert the measurement from cm to 10cm resolution
  range_10cm = (range_10cm > 255)?255:range_10cm;

  measurement_message = RADAR_FORWARD_RANGE_FIFO_MESSAGE;
  measurement_message &= 0xFFFF0000;
  measurement_message |= ((uint32_t)radar_turret.GetDegrees())<<8;
  measurement_message |= range_10cm;
  
  if (multicore_fifo_wready() && multicore_fifo_push_timeout_us(measurement_message, 100))
  {
    // sent fifo message, clear warning lamp for diagnostics
    // fifo messages are being transmitted sucessfully
    ClearLostMessageWarningLamp();
  }
  else
  {
    // failed to send fifo message, set warning lamp for diagnostics
    // if the lamp lights frequently we have a fifo message throughput issue
    SetLostMessageWarningLamp();
  };
  return range_1cm;
}
float Radar::MeasureRear()
{
  float    range_1cm;
  uint32_t range_10cm;
  uint32_t measurement_message;

  range_1cm = rear_ultrasound.Measure();
  range_10cm = range_1cm / 10; // convert the measurement from cm to 10cm resolution
  range_10cm = (range_10cm > 255)?255:range_10cm;

  measurement_message = RADAR_REAR_RANGE_FIFO_MESSAGE;
  measurement_message &= 0xFFFF0000;
  measurement_message |= ((uint32_t)radar_turret.GetDegrees())<<8;
  measurement_message |= range_10cm;
  
  if (multicore_fifo_wready() && multicore_fifo_push_timeout_us(measurement_message, 100))
  {
    // sent fifo message, clear warning lamp for diagnostics
    // fifo messages are being transmitted sucessfully
    ClearLostMessageWarningLamp();
  }
  else
  {
    // failed to send fifo message, set warning lamp for diagnostics
    // if the lamp lights frequently we have a fifo message throughput issue
    SetLostMessageWarningLamp();
  };
  return range_1cm;
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
  
  // servo is mounted transverse relative to car, and upside down
  servo_angle = (servo_angle>180)?180:servo_angle;
  car_angle = (-1*(servo_angle-90));

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