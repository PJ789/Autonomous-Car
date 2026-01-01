#include "car.h"
#include "debug.h"

Car::Car(): lights()
{

  absolute_time_t timeout;

  _gpio_init(   DRIVE_MOTOR_DIRECTION_PIN);
  gpio_set_dir( DRIVE_MOTOR_DIRECTION_PIN,    GPIO_OUT);
  _gpio_init(   STEERING_MOTOR_DIRECTION_PIN);
  gpio_set_dir( STEERING_MOTOR_DIRECTION_PIN, GPIO_OUT);

  gpio_set_function(DRIVE_MOTOR_SPEED_PIN, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to pin
  uint16_t drive_motor_slice_num = pwm_gpio_to_slice_num(DRIVE_MOTOR_SPEED_PIN);
  // System clock is 125 MHz by default
  // For 20 kHz, period = 125e6 / 20e3 = 6250 counts
  pwm_set_wrap(drive_motor_slice_num, 6249);
  //  pwm_set_wrap(slice_num, 255);
  pwm_set_gpio_level (DRIVE_MOTOR_SPEED_PIN, 0 );
  // Set the PWM running
  pwm_set_enabled(drive_motor_slice_num, true);

  gpio_set_function(STEERING_MOTOR_SPEED_PIN, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to pin
  uint16_t steering_motor_slice_num = pwm_gpio_to_slice_num(STEERING_MOTOR_SPEED_PIN);
  // System clock is 125 MHz by default
  // For 20 kHz, period = 125e6 / 20e3 = 6250 counts
  pwm_set_wrap(steering_motor_slice_num, 6249);
  //  pwm_set_wrap(slice_num, 255);
  pwm_set_gpio_level (STEERING_MOTOR_SPEED_PIN, 0 );
  // Set the PWM running
  pwm_set_enabled(steering_motor_slice_num, true);

  HardStop();
 
  // initialise radar measurements
  for(int16_t r=0; r<RADAR_MEASUREMENTS; r++)
  {
    radar_forward_measurements[ r]=0;
    radar_rearward_measurements[r]=0;
  }

  lights.Off();
  for (uint8_t i = 0; i<9; i++)
  {
    timeout = make_timeout_time_ms(1000);
    while( !time_reached(timeout) )
    {
      switch(i)
      {
        case 0: lights.FrontLeftFlash(); break;
        case 1: lights.RearLeftFlash(); break;
        case 2: lights.RearRightFlash(); break;
        case 3: lights.FrontRightFlash(); break;
        case 4: lights.LeftFlash(); break;
        case 5: lights.RearFlash(); break;
        case 6: lights.RightFlash(); break;
        case 7: lights.FrontFlash(); break;
        case 8: lights.Flash(); break;
      }
    }
    lights.Off();
  }

}

void Car::Iterator()
{
  // simple motor test
//  SetDriveMotorSpeed(DRIVE_MOTOR_HIGH_SPEED);
  // long m = millis(); 
  //  if ((m & 0x1FFF) > 0x0FFF)
  //  {
  //   SERIALPRINTLN("Left");
  //    this->Steer(left);
  //   }
  //  else
  //  {
  //   SERIALPRINTLN("Right");
  //    this->Steer(right);
  //  }
  //  return;

  uint32_t fifo_message;
  static uint     fifo_message_count = 0;

  while (multicore_fifo_rvalid())
  {
    if(multicore_fifo_pop_timeout_us(100, &fifo_message))
    {
      SERIALPRINTLN("fifo message received on core0");
      DecodeFifo( fifo_message );
      fifo_message_count+=1;
      //PlotRadarMetrics(25);// nb slow
      //DumpRadarMetrics();        
    }
    else
    {
      SERIALPRINTLN("Nothing to receive");
      sleep_ms(250);        
    }

  }

  if (fifo_message_count < 100)
  {
      SERIALPRINTLN("Initializing radar data, no movement");
      SERIALPRINT(fifo_message_count);
      SERIALPRINTLN("% complete");
      PlotRadarMetrics(25);// nb slow
      SetLights();
      return;
  }
  fifo_message_count = 100;

  PlanRoute();

  if (CollisionAvoidance() )
  {
    SERIALPRINTLN("Collision avoidance activated!");
    EmergencyStop();
  }
  else
  {
    SERIALPRINTLN("No collision risk detected");
    PlanSpeed();
  }

  SetLights();

}
void Car::DriveMotorSpeedControlCallback()
{
  if (current_drive_motor_speed < target_drive_motor_speed)
  {
    current_drive_motor_speed += DRIVE_MOTOR_ACCELERATION;
    current_drive_motor_speed = (current_drive_motor_speed>target_drive_motor_speed)?target_drive_motor_speed:current_drive_motor_speed;
  }
  else
  if (current_drive_motor_speed > target_drive_motor_speed)
  {
    current_drive_motor_speed -= min(current_drive_motor_speed, DRIVE_MOTOR_DECELERATION);
  }
  pwm_set_gpio_level (DRIVE_MOTOR_SPEED_PIN, current_drive_motor_speed );
}

void Car::SteeringMotorSpeedControlCallback()
{
  if (current_steering_motor_speed < target_steering_motor_speed)
  {
    current_steering_motor_speed += STEERING_MOTOR_ACCELERATION;
    current_steering_motor_speed = (current_steering_motor_speed>target_steering_motor_speed)?target_steering_motor_speed:current_steering_motor_speed;
  }
  else
  if (current_steering_motor_speed > target_steering_motor_speed)
  {
    current_steering_motor_speed -= min(current_steering_motor_speed, STEERING_MOTOR_DECELERATION);
  }
  pwm_set_gpio_level(STEERING_MOTOR_SPEED_PIN, current_steering_motor_speed);
  SERIALPRINT("Setting PWM GPIO LEVEL ON PIN ");
  SERIALPRINT(STEERING_MOTOR_SPEED_PIN);
  SERIALPRINT(" TO ");
  SERIALPRINTLN(current_steering_motor_speed);
}


void Car::HazardLightsOn()
{
    lights.Flash();
}
void Car::HazardLightsOff()
{
    lights.Off();
}
void Car::SetLights()
{
  if (DriveMotorStopped())
  {
    HazardLightsOn();
    return;
  }

  if (DirectionIsForward())
  {
    lights.RearOff();
    if (TurningLeft())
    {
      lights.FrontLeftFlash();
      lights.FrontRightOff();
    }
    else if (TurningRight())
    {
      lights.FrontRightFlash();
      lights.FrontLeftOff();
    }
    else
    {
      lights.FrontOn();
    }
  }
  else if (DirectionIsReverse())
  {
    lights.FrontOff();
    if (TurningLeft())
    {
      lights.RearLeftFlash();
      lights.RearRightOff();
    }
    else if (TurningRight())
    {
      lights.RearRightFlash();
      lights.RearLeftOff();
    }
    else
    {
      lights.RearOn();
    }
  }
}
void Car::PlanRoute()
{
  SERIALPRINTLN("Plan Route\n");
  if (DirectionIsForward())
  {
    SERIALPRINTLN("Plan Route; current direction is forward\n");
    if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      SERIALPRINTLN("Plan Route; no obstacle ahead\n");
      if (Turning())
      {
        if (TurningLeft())
        {
          if (
              (GetRadarForwardMeasurements(LEFT2)+GetRadarForwardMeasurements(LEFT1))
              >
              (GetRadarForwardMeasurements(LEFT1)+GetRadarForwardMeasurements(DEAD_AHEAD))
             )
          {
            SERIALPRINTLN("Plan Route; decision: steer left->left, drive forward\n");
            Steer(left);
          }
          else
          {
            SERIALPRINTLN("Plan Route; decision: steer left->none, drive forward\n");
            Steer(none); 
          }
        } else
        if (TurningRight())
        {
          if (
              (GetRadarForwardMeasurements(RIGHT2)+GetRadarForwardMeasurements(RIGHT1))
              >
              (GetRadarForwardMeasurements(RIGHT1)+GetRadarForwardMeasurements(DEAD_AHEAD))
             )
          {
            SERIALPRINTLN("Plan Route; decision: steer right->right, drive forward\n");
            Steer(right);
          }
          else
          {
            SERIALPRINTLN("Plan Route; decision: steer right->none, drive forward\n");
            Steer(none);
          }
        }
      }
      else if (!Turning())
      {
        if (
            (GetRadarForwardMeasurements(DEAD_AHEAD)+GetRadarForwardMeasurements(RIGHT1))
            >
            (GetRadarForwardMeasurements(LEFT1)+GetRadarForwardMeasurements(DEAD_AHEAD))
           )
        {
          SERIALPRINTLN("Plan Route; decision: steer none->right, drive forward\n");
          Steer(right);
        } else
        if (
            (GetRadarForwardMeasurements(LEFT1)+GetRadarForwardMeasurements(DEAD_AHEAD))
            >
            (GetRadarForwardMeasurements(DEAD_AHEAD)+GetRadarForwardMeasurements(RIGHT1))
           )
        {
          SERIALPRINTLN("Plan Route; decision: steer none->left, drive forward\n");
          Steer(left);
        }
      }
    }
    else if (!(ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT)))
    {
      SERIALPRINTLN("Plan Route; obstacle ahead, no obstacle behind");
      SERIALPRINTLN("Plan Route; decision: steer none, drive reverse");
      Steer(none);
      Drive(reverse);
    }
    else
    {
      SERIALPRINTLN("Plan Route; obstacle ahead & behind");
      SERIALPRINTLN("Plan Route; decision: no option to change course");
      Steer(none);
    }
  }
  else if (DirectionIsReverse())
  {
    SERIALPRINTLN("Plan Route; current direction is reverse\n");
    if (!ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT))
    {
      SERIALPRINTLN("Plan Route; no obstacle behind\n");
      if (Turning())
      {
        if (TurningLeft()) // nb right in reverse
        {
          if (
              (GetRadarRearwardMeasurements(RIGHT2)+GetRadarRearwardMeasurements(RIGHT1))
              >
              (GetRadarRearwardMeasurements(RIGHT1)+GetRadarRearwardMeasurements(DEAD_AHEAD))
             )
          {
            SERIALPRINTLN("Plan Route; decision: steer left->left, drive reverse\n");
            Steer(left);
          }
          else
          {
            SERIALPRINTLN("Plan Route; decision: steer left->none, drive reverse\n");
            Steer(none);
          }
        } else
        if (TurningRight()) // nb left in reverse
        {
          if (
              (GetRadarRearwardMeasurements(LEFT2)+GetRadarRearwardMeasurements(LEFT1))
              >
              (GetRadarRearwardMeasurements(LEFT1)+GetRadarRearwardMeasurements(DEAD_AHEAD))
             )
          {
            SERIALPRINTLN("Plan Route; decision: steer right->right, drive reverse\n");
            Steer(right);
          }
          else
          {
            SERIALPRINTLN("Plan Route; decision: steer right->none, drive reverse\n");
            Steer(none);
          }
        }
      }
      else if (!Turning())
      {
        if (
            (GetRadarRearwardMeasurements(DEAD_AHEAD)+GetRadarRearwardMeasurements(RIGHT1))
            >
            (GetRadarRearwardMeasurements(LEFT1)+GetRadarRearwardMeasurements(DEAD_AHEAD))
           )
        {
          SERIALPRINTLN("Plan Route; decision: steer none->left, drive reverse\n");
          Steer(left);
        } else
        if (
            (GetRadarRearwardMeasurements(LEFT1)+GetRadarRearwardMeasurements(DEAD_AHEAD))
            >
            (GetRadarRearwardMeasurements(DEAD_AHEAD)+GetRadarRearwardMeasurements(RIGHT1))
           )
        {
          SERIALPRINTLN("Plan Route; decision: steer none->right, drive reverse\n");
          Steer(right);
        }
      }
    }
    else if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      SERIALPRINTLN("Plan Route; obstacle behind, no obstacle ahead");
      SERIALPRINTLN("Plan Route; decision: steer none, drive forward");
      Steer(none);
      Drive(forward);
    }
    else
    {
      SERIALPRINTLN("Plan Route; obstacle ahead & behind");
      SERIALPRINTLN("Plan Route; decision: no option to change course");
      Steer(none);
      Drive(reverse);
    }
  }

}
void Car::PlanSpeed()
{
  SERIALPRINTLN("Plan Speed");

  if (DirectionIsForward())
  {
    if (ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      SERIALPRINTLN("Forward Stop");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        SERIALPRINTLN("Forward Low Speed");
        SetDriveMotorSpeed(DRIVE_MOTOR_LOW_SPEED);
      }
      else
      {
        SERIALPRINTLN("Forward High Speed");
        SetDriveMotorSpeed(DRIVE_MOTOR_HIGH_SPEED);
      }
    }
  }
  else if (DirectionIsReverse())
  {
    if (ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT))
    {
      SERIALPRINTLN("Reverse Stop");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        SERIALPRINTLN("Reverse Low Speed");
        SetDriveMotorSpeed(DRIVE_MOTOR_LOW_SPEED);
      }
      else
      {
        SERIALPRINTLN("Reverse High Speed");
        SetDriveMotorSpeed(DRIVE_MOTOR_HIGH_SPEED);
      }
    }
  }
}
bool Car::DriveMotorStopped()
{
  return (current_drive_motor_speed == 0);
}
bool Car::SteeringMotorStopped()
{
  return (current_steering_motor_speed == 0);
}
bool Car::Moving()
{
  return ( MovingForward() || MovingBackward() );
}
bool Car::MovingForward()
{
  return ( (current_drive_motor_direction == forward ) && (current_drive_motor_speed > 0) );
}
bool Car::MovingBackward()
{
  return ( (current_drive_motor_direction == reverse ) && (current_drive_motor_speed > 0) );
}
bool Car::DirectionIsForward()
{
  return (current_drive_motor_direction == forward);
}
bool Car::DirectionIsReverse()
{
  return (current_drive_motor_direction == reverse);
}
bool Car::Turning()
{
  return ( current_steering_motor_direction != none );
}
bool Car::TurningLeft()
{
  return ( current_steering_motor_direction == left );
}
bool Car::TurningRight()
{
  return ( current_steering_motor_direction == right );
}
bool Car::ObstacleDetectedAhead(float range_limit)
{
  float range = 0;
  uint16_t start_index = -1;
  uint16_t end_index = 1;
  bool obstacle_detected = false;

  if (TurningLeft())
  {
    start_index = -2;
    end_index   = 0;
  }
  if (TurningRight())
  {
    start_index = 0;
    end_index   = 2;
  }

  for (int16_t i = start_index; i <= end_index; i++)
  {
    range = GetRadarForwardMeasurements(i * RADAR_SCAN_STEP_DEGREES);
    if (range < range_limit)
    {
      SERIALPRINT("Obstacle ahead. Range ");
      SERIALPRINT(range);
      SERIALPRINT(" cm (min ");
      SERIALPRINT(range_limit);
      SERIALPRINT(" cm) @angle=");
      SERIALPRINTLN(i * RADAR_SCAN_STEP_DEGREES);
      obstacle_detected = true;
    }

  }
  return obstacle_detected;
}
bool Car::ObstacleDetectedBehind(float range_limit)
{
  float range = 0;
  uint16_t start_index = -1;
  uint16_t end_index = 1;
  bool obstacle_detected = false;

  if (TurningLeft())
  {
    start_index = 0;
    end_index   = 2;
  }
  if (TurningRight())
  {
    start_index = -2;
    end_index   = 0;
  }

  for (int16_t i = start_index; i <= end_index; i++)
  {
    range = GetRadarRearwardMeasurements(i * RADAR_SCAN_STEP_DEGREES);
    if (range < range_limit)
    {
      SERIALPRINT("Obstacle behind. Range ");
      SERIALPRINT(range);
      SERIALPRINT(" cm (min ");
      SERIALPRINT(range_limit);
      SERIALPRINT(" cm) @angle=");
      SERIALPRINTLN(i * RADAR_SCAN_STEP_DEGREES);

      obstacle_detected = true;
    }

  }
  return obstacle_detected;
}
bool Car::CollisionAvoidance()
{
  if (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE_LIMIT))
  {
    SERIALPRINTLN("Vehicle direction is forward, but obstacle too close to front of vehicle!");
  }
  else if (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE_LIMIT))
  {
    SERIALPRINTLN("Vehicle direction is reverse, but obstacle too close to rear of vehicle!");
  }

  return (
           (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE_LIMIT))
           ||
           (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE_LIMIT))
         );
}
void Car::SetDriveMotorSpeed(uint16_t s)
{
  if (s != current_drive_motor_speed)
  {
    SERIALPRINT("Changing drive motor speed from ");
    SERIALPRINT(current_drive_motor_speed);
    SERIALPRINT(" to ");
    SERIALPRINTLN( s);

    target_drive_motor_speed = s;
  }
  else
  {
    SERIALPRINT("Drive motor speed at ");
    SERIALPRINTLN(current_drive_motor_speed);    
  }
}
void Car::SetSteeringMotorSpeed(uint16_t s)
{
  if (s != current_steering_motor_speed)
  {
    SERIALPRINT("Changing steering motor speed from ");
    SERIALPRINT(current_steering_motor_speed);
    SERIALPRINT(" to ");
    SERIALPRINTLN( s);

    target_steering_motor_speed = s;
  }
  else
  {
    SERIALPRINT("Steering motor speed at ");
    SERIALPRINTLN(current_steering_motor_speed);    
    SERIALPRINT("Target steering motor speed at ");
    SERIALPRINTLN(target_steering_motor_speed);
  }
}
void Car::HardStop()
{
  // Power down drive motor
  pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, 0);
  // Power down steering motor
  pwm_set_gpio_level(STEERING_MOTOR_SPEED_PIN, 0);

  // Power down steering relay
  gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
  // Power down steering motor
  gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );

  current_drive_motor_direction = forward;
  current_steering_motor_direction = none;
  current_drive_motor_speed    = 0;
  current_steering_motor_speed = 0;
  target_drive_motor_speed     = 0;
  target_steering_motor_speed  = 0;

  SendStatus();

}
void Car::EmergencyStop()
{
  SERIALPRINTLN("<<< EMERGENCY STOP >>>");

  HardStop();
  
  SERIALPRINTLN("8888888888 888b     d888 8888888888 8888888b.   .d8888b.  8888888888 888b    888  .d8888b. Y88b   d88P");
  SERIALPRINTLN("888        8888b   d8888 888        888   Y88b d88P  Y88b 888        8888b   888 d88P  Y88b Y88b d88P");
  SERIALPRINTLN("888        88888b.d88888 888        888    888 888    888 888        88888b  888 888    888  Y88o88P");
  SERIALPRINTLN("8888888    888Y88888P888 8888888    888   d88P 888        8888888    888Y88b 888 888          Y888P");
  SERIALPRINTLN("888        888 Y888P 888 888        8888888P'  888  88888 888        888 Y88b888 888           888");
  SERIALPRINTLN("888        888  Y8P  888 888        888 T88b   888    888 888        888  Y88888 888    888    888");
  SERIALPRINTLN("888        888   '   888 888        888  T88b  Y88b  d88P 888        888   Y8888 Y88b  d88P    888");
  SERIALPRINTLN("8888888888 888       888 8888888888 888   T88b  'Y8888P88 8888888888 888    Y888  'Y8888P'     888");
  SERIALPRINTLN("");
  SERIALPRINTLN("");
  SERIALPRINTLN("                           .d8888b. 88888888888 .d88888b.  8888888b.  888");
  SERIALPRINTLN("                          d88P  Y88b    888    d88P' 'Y88b 888   Y88b 888");
  SERIALPRINTLN("                          Y88b.         888    888     888 888    888 888");
  SERIALPRINTLN("                           'Y888b.      888    888     888 888   d88P 888");
  SERIALPRINTLN("                              'Y88b.    888    888     888 8888888P'  888");
  SERIALPRINTLN("                                '888    888    888     888 888        Y8P");
  SERIALPRINTLN("                          Y88b  d88P    888    Y88b. .d88P 888         !");
  SERIALPRINTLN("                           'Y8888P'     888     'Y88888P'  888        888");
}
void Car::Drive( drive_motor_direction direction )
{
  // Change direction
  switch (direction) {
    case forward:
      if (!DirectionIsForward())
      {
        if ( !DriveMotorStopped() )
        {
          SERIALPRINTLN("Stopping drive motor for direction change to forward..");
          // Slow to a stop
          SetDriveMotorSpeed( 0 );
        }
        else
        {
          SERIALPRINTLN("Changing drive motor direction to forward");

          gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
          current_drive_motor_direction = forward;
        }
      } else
      {
        SERIALPRINTLN("Maintaining drive motor forward direction");
      }
      break;
    case reverse:
      if (!DirectionIsReverse())
      {
        if ( !DriveMotorStopped() )
        {
          SERIALPRINTLN("Stopping drive motor for direction change to reverse..");
          // Slow to a stop
          SetDriveMotorSpeed( 0 );
        }
        else
        {
          SERIALPRINTLN("Changing drive motor direction to reverse");
   
          gpio_put(DRIVE_MOTOR_DIRECTION_PIN, true );
          current_drive_motor_direction = reverse;
        }
      } else
      {
        SERIALPRINTLN("Maintaining drive motor reverse direction");
      }
      break;
  }

}
void Car::Steer( steering_motor_direction direction )
{
  // Change direction
  switch (direction) {
    case left:
      if (!TurningLeft())
      {
        if (!SteeringMotorStopped())
        {
          SERIALPRINTLN("Stopping steering motor for direction change to left..");
          // Slow to a stop
          SetSteeringMotorSpeed( 0 ); 
        }
        else
        {
          SERIALPRINTLN("Changing steering motor direction to left");
          gpio_put(STEERING_MOTOR_DIRECTION_PIN, true );
          SetSteeringMotorSpeed(STEERING_MOTOR_HIGH_SPEED);
          current_steering_motor_direction = left; 
        }
      } else
      {
        SERIALPRINTLN("Maintaining left steering direction");
      }
      break;
    case right:
      if (!TurningRight())
      {
        if (!SteeringMotorStopped())
        {
          SERIALPRINTLN("Stopping steering motor for direction change to right..");
          // Slow to a stop
          SetSteeringMotorSpeed( 0 ); 
        }
        else
        {
          SERIALPRINTLN("Changing steering motor direction to right");
          gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
          SetSteeringMotorSpeed(STEERING_MOTOR_HIGH_SPEED);
          current_steering_motor_direction = right; 
        }
      } else
      {
        SERIALPRINTLN("Maintaining right steering direction");
      }
      break;
    case none:
      if (Turning())
      {
        if (!SteeringMotorStopped())
        {
          SERIALPRINTLN("Stopping steering motor for direction change to none..");
          // Slow to a stop
          SetSteeringMotorSpeed( 0 );
        }
        else
        {
          SERIALPRINTLN("Steering none");

          gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
          SetSteeringMotorSpeed(0);
          current_steering_motor_direction = none;
        }
      }
      else
      {
        SERIALPRINTLN("Maintaining forward (no steering) direction");
      }

      break;    
  }
}
uint8_t Car::GetRadarForwardMeasurements(int8_t angle)
{
  angle = (angle<-90)?-90:angle;
  angle = (angle> 90)? 90:angle;

  uint8_t index = ConvertCarAngleToRadarAngle(angle) / RADAR_MEASUREMENTS_GRANULARITY;
  uint8_t distance = radar_forward_measurements[index];

  return distance;
}
uint8_t Car::GetRadarRearwardMeasurements(int8_t angle)
{
  angle = (angle<-90)?-90:angle;
  angle = (angle> 90)? 90:angle;

  uint8_t index = ConvertCarAngleToRadarAngle(angle) / RADAR_MEASUREMENTS_GRANULARITY;
  uint8_t distance = radar_rearward_measurements[index];

  return distance;
}
void Car::DecodeFifo(uint32_t fifo_message)
{
  char decoded_message[5];
  uint8_t index;
  uint8_t angle;
  uint8_t range;

  if (fifo_message == RADAR_READY_FIFO_MESSAGE)
  {
    SERIALPRINTLN("Radar ready");
    SendStatus();
    return;
  }

  for(uint8_t i=0; i<5; i++) decoded_message[i]=0;
  decoded_message[0] = (fifo_message & 0xFF000000) >> 24;
  decoded_message[1] = (fifo_message & 0x00FF0000) >> 16;
  decoded_message[2] = (fifo_message & 0x0000FF00) >>  8;
  decoded_message[3] = (fifo_message & 0x000000FF) >>  0;

  switch (decoded_message[0])
  {
    case 'R': // radar measurement
      SERIALPRINT("Radar measurement ");
      angle = decoded_message[2];
      range = decoded_message[3];
      index = (angle / RADAR_MEASUREMENTS_GRANULARITY);
      switch (decoded_message[1])
      {
        case 'F': // forward
          SERIALPRINT("forward ");
          radar_forward_measurements[index] = range;
          break;
        case 'R': // rearward
          SERIALPRINT("rearward ");
          radar_rearward_measurements[index] = range;
          break;
        default:
          break;
      }
      SERIALPRINT(angle);
      SERIALPRINT(" degs, range ");
      SERIALPRINTLN(range);
      break;
    case 'D': // debug message
      SERIALPRINT("DEBUG: ");
      SERIALPRINTLN((char*)&decoded_message[1]);
      break;
    default:
      SERIALPRINT("Radar sent unexpected message type: ");
      SERIALPRINTLN(decoded_message);
      break;
  }
}

int16_t Car::ConvertServoAngleToCarAngle(uint8_t angle)
{
  return (-1*(angle-90));
}
uint8_t Car::ConvertCarAngleToRadarAngle(int16_t angle)
{
  return (90-angle);
}

void Car::SendStatus()
{
  uint32_t message = CAR_FIFO_MESSAGE;
  message = message & 0xFF000000; // C___
  
  switch (current_steering_motor_direction) {
  case left:
    message = message | 'L' << 8;
    break;
  case right:
    message = message | 'R' << 8;
    break;
  default:
    message = message | 'N' << 8;
    break;      
  }

  switch (current_drive_motor_direction) {
    case forward:
      message = message | 'F' << 16;
      break;
    case reverse:
      message = message | 'R' << 16;
      break;
  }

  message = message | current_drive_motor_speed;

  if (multicore_fifo_wready())
    multicore_fifo_push_timeout_us( message, 100 ); 
}
void Car::DumpRadarMetrics()
{
  SERIALPRINTLN("........................................................................................");
  for(int16_t r=0; r<RADAR_MEASUREMENTS; r++)
  {
    SERIALPRINT(r*RADAR_MEASUREMENTS_GRANULARITY);
    SERIALPRINT(":\t");
    SERIALPRINT(ConvertServoAngleToCarAngle(r*RADAR_MEASUREMENTS_GRANULARITY));
    SERIALPRINT(":\t");
    SERIALPRINT(radar_forward_measurements[ r]);
    SERIALPRINT(",\t");
    SERIALPRINT(radar_rearward_measurements[r]);
    SERIALPRINTLN("");
  }
  SERIALPRINTLN("........................................................................................");
  
}
void Car::PlotRadarMetrics(int diameter)
{
  float opposite,adjacent,hypotenuse,tangent,m,scale;
  int i,radius, starty, endy;
  radius = diameter>>1; // divide by two
  scale = 255/((float) radius);

  starty = (DriveMotorStopped()||DirectionIsForward())?radius:0;
  endy   = (DriveMotorStopped()||DirectionIsReverse())?-radius:0;

  for (int y=starty; y>=endy; y--)
  {
    for (int x=-radius; x<=radius; x++)
    {
      adjacent = x*scale;
      opposite = abs(y*scale);
      hypotenuse = sqrt(pow(adjacent,2)+pow(opposite,2));

      if (y>0)
      {
        tangent = (atan2(opposite,adjacent)*57.295779513);
        i = round (tangent / RADAR_MEASUREMENTS_GRANULARITY);
        m = radar_forward_measurements[i];
      }
      else
      {
        tangent = 180 - (atan2(opposite,adjacent)*57.295779513);
        i = round (tangent / RADAR_MEASUREMENTS_GRANULARITY);
        m = radar_rearward_measurements[i];
      }

      if (y==0)
      {
        SERIALPRINT( (x==0)? "+":"-");
      }
      else if (x==0)
      {
        if (hypotenuse<m)
        {
          SERIALPRINT("#");
        }
        else
        {
          SERIALPRINT( "|");
        }
      }
      else if (hypotenuse<m)
      {
        SERIALPRINT("#");
      }
      else if ((hypotenuse/255)>1)
      {
        SERIALPRINT(" ");
      }
      else
      {
        if(hypotenuse<STEERING_FORWARD_RANGE_LIMIT)
        {
          SERIALPRINT((hypotenuse<MINIMUM_FORWARD_RANGE_LIMIT)?".":"¤");
        }
        else
        {
          SERIALPRINT(".");
        }
      }
    }      
    SERIALPRINTLN("");   
  }
 
}
