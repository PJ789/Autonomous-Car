#include "car.h"
#include "debug.h"

Car::Car(): lights(), steering_motor(STEERING_MOTOR_PIN)
{

  absolute_time_t timeout;

  _gpio_init(   DRIVE_MOTOR_DIRECTION_PIN);
  gpio_set_dir( DRIVE_MOTOR_DIRECTION_PIN,    GPIO_OUT);

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

  HardStop();
 
  lights.Off();
  lights.BrakeOff();
  for (uint8_t i = 0; i<11; i++)
  {
    timeout = make_timeout_time_ms(1000);
    while( !time_reached(timeout) )
    {
      switch(i)
      {
        case  0: lights.FrontLeftFlash(); break;
        case  1: lights.RearLeftFlash(); break;
        case  2: lights.BrakeLeftFlash(); break;
        case  3: lights.BrakeRightFlash(); break;
        case  4: lights.RearRightFlash(); break;
        case  5: lights.FrontRightFlash(); break;
        case  6: lights.LeftFlash(); break;
        case  7: lights.RearFlash(); lights.BrakeFlash(); break;
        case  8: lights.RightFlash(); break;
        case  9: lights.FrontFlash(); break;
        case 10: lights.Flash(); break;
      }
    }
    lights.Off();
    lights.BrakeOff();
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


  while (multicore_fifo_rvalid())
  {
    fifo_message=multicore_fifo_pop_blocking();
    SERIALPRINT(CLEAR_HOME);
    SERIALPRINTLN("fifo message received on core0");
    DecodeFifo( fifo_message );
    fifo_message_count+=1;
    //DumpRadarMetrics();// nb slow
    //SetLights();
  }

  if (status_changed)
  {
    SERIALPRINTLN("Sending speed/direction status to radar");
    SendStatus();
    status_changed = false;
  }

  if (fifo_message_count < 32)
  {
      SERIALPRINTLN("Initializing radar data, suppressing movement");
      SERIALPRINT(fifo_message_count*(100/32));
      SERIALPRINTLN("% complete");
      DumpRadarMetrics();// nb slow
      delay(50);
      SetLights();
      return;
  }

  fifo_message_count = 32;

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
    status_changed=true;
  }
  else
  if (current_drive_motor_speed > target_drive_motor_speed)
  {
    current_drive_motor_speed -= min(current_drive_motor_speed, DRIVE_MOTOR_DECELERATION);
    status_changed=true;
  }
  pwm_set_gpio_level (DRIVE_MOTOR_SPEED_PIN, current_drive_motor_speed );
}



void Car::HazardLightsOn()
{
    lights.Flash();
    lights.BrakeFlash();
}
void Car::HazardLightsOff()
{
    lights.Off();
    lights.BrakeOff();
}
void Car::SetLights()
{
  if (DriveMotorStopped())
  {
    HazardLightsOn();
    return;
  }

  if (Braking())
  {
    lights.RearOff();
    lights.BrakeOn();
    return;
  }
  lights.BrakeOff();

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

  // Change direction if a turn manouevre is not possible

  if (
    (DirectionIsForward() && ObstacleDetectedAhead(OBSTACLE_AHEAD_MANOEUVERING_RANGE_LIMIT_CM))
    ||
    (DirectionIsReverse() && ObstacleDetectedBehind(OBSTACLE_BEHIND_MANOEUVERING_RANGE_LIMIT_CM))
    )
  {
    if (DirectionIsForward() && !ObstacleDetectedBehind(OBSTACLE_BEHIND_MANOEUVERING_RANGE_LIMIT_CM))
    {
      SERIALPRINTLN("Plan Route; obstacle ahead, no obstacle Behind");
      SERIALPRINTLN("Plan Route; decision: steer none, drive reverse");
      Steer(none);
      Drive(reverse);
    }
    else
    if (DirectionIsReverse() && !ObstacleDetectedAhead(OBSTACLE_AHEAD_MANOEUVERING_RANGE_LIMIT_CM))
    {
      SERIALPRINTLN("Plan Route; obstacle Behind, no obstacle ahead");
      SERIALPRINTLN("Plan Route; decision: steer none, drive forward");
      Steer(none);
      Drive(forward);
    }
    else
    {
      SERIALPRINTLN("Plan Route; obstacles ahead & Behind");
      SERIALPRINTLN("Plan Route; decision: no option to change course");
      Steer(none);
      Drive(forward);
        // three point turn?
    }
  }

  // Plan manoeuvre

  if (DirectionIsForward())
  {
    SERIALPRINTLN("Plan Route; current direction is forward\n");
    if (Turning())
    {
      if (TurningLeft())
      {
        if (
          (GetDistancetoObstacleAhead(DEAD_AHEAD) > (GetDistancetoObstacleAhead(LEFT1)+TURN_CHANGE_HYSTERESIS_CM))
          &&
          (GetDistancetoObstacleAhead(DEAD_AHEAD) > (GetDistancetoObstacleAhead(LEFT2)+TURN_CHANGE_HYSTERESIS_CM))
          )
        {
          SERIALPRINTLN("Plan Route; decision: steer left->none, drive forward\n");
          Steer(none);
          Drive(forward); 
        }
      } else
      if (TurningRight())
      {
        if (
          (GetDistancetoObstacleAhead(DEAD_AHEAD) > (GetDistancetoObstacleAhead(RIGHT1)+TURN_CHANGE_HYSTERESIS_CM))
          &&
          (GetDistancetoObstacleAhead(DEAD_AHEAD) > (GetDistancetoObstacleAhead(RIGHT2)+TURN_CHANGE_HYSTERESIS_CM))
          )
        {
          SERIALPRINTLN("Plan Route; decision: steer right->none, drive forward\n");
          Steer(none);
          Drive(forward);
        }
      }
    }
    else // not turning
    {
      if (
          (GetDistancetoObstacleAhead(RIGHT1) > (GetDistancetoObstacleAhead(DEAD_AHEAD)+TURN_CHANGE_HYSTERESIS_CM))
        &&
          (GetDistancetoObstacleAhead(RIGHT1) > (GetDistancetoObstacleAhead(LEFT1)+TURN_CHANGE_HYSTERESIS_CM))
        )
      {
        SERIALPRINTLN("Plan Route; decision: steer none->right, drive forward\n");
        Steer(right);
        Drive(forward);
      } else
      if (
          (GetDistancetoObstacleAhead(LEFT1) > (GetDistancetoObstacleAhead(DEAD_AHEAD)+TURN_CHANGE_HYSTERESIS_CM))
        &&
          (GetDistancetoObstacleAhead(LEFT1) > (GetDistancetoObstacleAhead(RIGHT1)+TURN_CHANGE_HYSTERESIS_CM))
        )
      {
        SERIALPRINTLN("Plan Route; decision: steer none->left, drive forward\n");
        Steer(left);
        Drive(forward);
      }     
    }
  }
  else if (DirectionIsReverse())
  {
    SERIALPRINTLN("Plan Route; current direction is reverse\n");
    if (Turning())
    {
      if (TurningLeft()) // nb right in reverse
      {
        if (
          (GetDistancetoObstacleBehind(DEAD_BEHIND) > (GetDistancetoObstacleBehind(RIGHT1)+TURN_CHANGE_HYSTERESIS_CM))
          &&
          (GetDistancetoObstacleBehind(DEAD_BEHIND) > (GetDistancetoObstacleBehind(RIGHT2)+TURN_CHANGE_HYSTERESIS_CM))
          )
        {
          SERIALPRINTLN("Plan Route; decision: steer left->none, drive reverse\n");
          Steer(none);
          Drive(reverse);
        }
      } else
      if (TurningRight()) // nb left in reverse
      {
        if (
          (GetDistancetoObstacleBehind(DEAD_BEHIND) > (GetDistancetoObstacleBehind(LEFT1)+TURN_CHANGE_HYSTERESIS_CM))
          &&
          (GetDistancetoObstacleBehind(DEAD_BEHIND) > (GetDistancetoObstacleBehind(LEFT2)+TURN_CHANGE_HYSTERESIS_CM))
          )
        {
          SERIALPRINTLN("Plan Route; decision: steer right->none, drive reverse\n");
          Steer(none);
          Drive(reverse);
        }
      }
    }
    else // not turning
    {
      if (
          (GetDistancetoObstacleBehind(RIGHT1) > (GetDistancetoObstacleBehind(DEAD_BEHIND)+TURN_CHANGE_HYSTERESIS_CM))
        &&
          (GetDistancetoObstacleBehind(RIGHT1) > (GetDistancetoObstacleBehind(LEFT1)+TURN_CHANGE_HYSTERESIS_CM))
        )
      {
        SERIALPRINTLN("Plan Route; decision: steer none->left, drive reverse\n");
        Steer(left);
        Drive(reverse);
      } else
      if (
          (GetDistancetoObstacleBehind(LEFT1) > (GetDistancetoObstacleBehind(DEAD_BEHIND)+TURN_CHANGE_HYSTERESIS_CM))
        &&
          (GetDistancetoObstacleBehind(LEFT1) > (GetDistancetoObstacleBehind(RIGHT1)+TURN_CHANGE_HYSTERESIS_CM))
        )
      {
        SERIALPRINTLN("Plan Route; decision: steer none->right, drive reverse\n");
        Steer(right);
        Drive(reverse);
      }
    }
  }
}

void Car::PlanSpeed()
{
  SERIALPRINTLN("Plan Speed");

  if (DirectionIsForward())
  {
    if (ObstacleDetectedAhead(OBSTACLE_AHEAD_MANOEUVERING_RANGE_LIMIT_CM))
    {
      // We're inside the car's turning circle, and facing an obstacle. time to stop.
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
    if (ObstacleDetectedBehind(OBSTACLE_BEHIND_MANOEUVERING_RANGE_LIMIT_CM))
    {
      // We're inside the car's turning circle, and facing an obstacle. time to stop.
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

bool Car::Moving()
{
  return ( MovingForward() || MovingReverse() );
}
bool Car::Braking()
{
  return (current_drive_motor_speed > target_drive_motor_speed);
}
bool Car::Accelerating()
{
  return (current_drive_motor_speed < target_drive_motor_speed);
}
bool Car::MovingForward()
{
  return ( (current_drive_motor_direction == forward ) && (current_drive_motor_speed > 0) );
}
bool Car::MovingReverse()
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
    range = GetDistancetoObstacleAhead(i * RADAR_SCAN_STEP_DEGREES);
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
    range = GetDistancetoObstacleBehind(i * RADAR_SCAN_STEP_DEGREES);
    if (range < range_limit)
    {
      SERIALPRINT("Obstacle Behind. Range ");
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
  if (DirectionIsForward() && ObstacleDetectedAhead(OBSTACLE_AHEAD_EMERGENCY_STOP_RANGE_LIMIT_CM))
  {
    SERIALPRINTLN("Vehicle direction is forward, but obstacle too close ahead vehicle!");
    return true;
  }
  else if (DirectionIsReverse() && ObstacleDetectedBehind(OBSTACLE_BEHIND_EMERGENCY_STOP_RANGE_LIMIT_CM))
  {
    SERIALPRINTLN("Vehicle direction is reverse, but obstacle too close behind vehicle!");
    return true;
  }

  return false;
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

void Car::HardStop()
{
  status_changed = true;
  // Power down drive motor
  pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, 0);
  // Power down drive motor relay
  gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );

  // Cancel steering
  steering_motor.SetDegrees(STEERING_MOTOR_CENTRE_ANGLE);

  current_drive_motor_direction = forward;
  current_steering_motor_direction = none;
  current_drive_motor_speed    = 0;
  target_drive_motor_speed     = 0;

  // Initialise/reset radar measurements (something bad has happened, so clear it out)
  for(int16_t r=0; r<RADAR_MEASUREMENTS; r++)
  {
    radar_front_measurements[ r]=0;
    radar_rear_measurements[r]=0;
  }

  // force the car to gather fifo messages before moving off
  fifo_message_count = 0;

}
void Car::EmergencyStop()
{
  SERIALPRINTLN("<<< EMERGENCY STOP >>>");


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

  DumpRadarMetrics();
  HardStop();

  SERIALPRINTLN("<<< END EMERGENCY STOP, CAR REINITIALISING >>>");
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
          status_changed = true;
          gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
          current_drive_motor_direction = forward;
          // force the car to gather fifo messages before moving off
          fifo_message_count = 0;
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
          status_changed = true;
          gpio_put(DRIVE_MOTOR_DIRECTION_PIN, true );
          current_drive_motor_direction = reverse;
          // force the car to gather fifo messages before moving off
          fifo_message_count = 0;
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
        SERIALPRINTLN("Changing steering motor direction to left");
        status_changed = true;
        steering_motor.SetDegrees(STEERING_MOTOR_MAX_LEFT_ANGLE);
        current_steering_motor_direction = left; 
      } else
      {
        SERIALPRINTLN("Maintaining left steering direction");
      }
      break;
    case right:
      if (!TurningRight())
      {
        SERIALPRINTLN("Changing steering motor direction to right");
        status_changed = true;
        steering_motor.SetDegrees(STEERING_MOTOR_MAX_RIGHT_ANGLE);
        current_steering_motor_direction = right; 
      } else
      {
        SERIALPRINTLN("Maintaining right steering direction");
      }
      break;
    case none:
      if (Turning())
      {
        SERIALPRINTLN("Steering none");
        status_changed = true;
        steering_motor.SetDegrees(STEERING_MOTOR_CENTRE_ANGLE);
        current_steering_motor_direction = none;
      }
      else
      {
        SERIALPRINTLN("Maintaining no steering direction");
      }
      break;    
  }
}
uint32_t Car::GetDistancetoObstacleAhead(int8_t angle)
{
  angle = (angle<-90)?-90:angle;
  angle = (angle> 90)? 90:angle;

  uint8_t index = ConvertCarAngleToRadarAngle(angle) / RADAR_MEASUREMENTS_GRANULARITY;
  uint32_t distance = radar_front_measurements[index];

  return distance;
}
uint32_t Car::GetDistancetoObstacleBehind(int8_t angle)
{
  angle = (angle<-90)?-90:angle;
  angle = (angle> 90)? 90:angle;

  uint8_t index = ConvertCarAngleToRadarAngle(angle) / RADAR_MEASUREMENTS_GRANULARITY;
  uint32_t distance = radar_rear_measurements[index];

  return distance;
}
void Car::DecodeFifo(uint32_t fifo_message)
{
  char decoded_message[5];
  uint8_t index;
  uint8_t angle;
  uint8_t range_10cm;
  uint32_t range_1cm;

  if (fifo_message == RADAR_READY_FIFO_MESSAGE)
  {
    SERIALPRINTLN("Radar ready");
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
      range_10cm = decoded_message[3];
      range_1cm = 10 * range_10cm; // convert the measurement back from 10cm to 1cm resolution
      index = (angle / RADAR_MEASUREMENTS_GRANULARITY);
      switch (decoded_message[1])
      {
        case 'F': // forward
          SERIALPRINT("forward ");
          radar_front_measurements[index] = range_1cm; 
          break;
        case 'R': // rearward
          SERIALPRINT("rearward ");
          radar_rear_measurements[index] = range_1cm;
          break;
        default:
          break;
      }
      SERIALPRINT(angle);
      SERIALPRINT(" degs, range ");
      SERIALPRINT(range_1cm);
      SERIALPRINTLN("cm");
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

int16_t Car::ConvertRadarAngleToCarAngle(uint8_t angle)
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
  
  // Cd__ - Add forward/reverse status
  switch (current_drive_motor_direction) {
    case forward:
      message = message | 'F' << 16;
      break;
    case reverse:
      message = message | 'R' << 16;
      break;
  }

  // Cdt_ - Add steering status
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

  uint16_t speed = (current_drive_motor_speed>255)? 255:current_drive_motor_speed;
  // Cdt_ - Add speed status
  message = message | speed;

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
    SERIALPRINT(ConvertRadarAngleToCarAngle(r*RADAR_MEASUREMENTS_GRANULARITY));
    SERIALPRINT(":\t");
    SERIALPRINT(radar_front_measurements[ r]);
    SERIALPRINT(",\t");
    SERIALPRINT(radar_rear_measurements[r]);
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
        m = radar_front_measurements[i];
      }
      else
      {
        tangent = 180 - (atan2(opposite,adjacent)*57.295779513);
        i = round (tangent / RADAR_MEASUREMENTS_GRANULARITY);
        m = radar_rear_measurements[i];
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
        if(hypotenuse<OBSTACLE_AHEAD_MANOEUVERING_RANGE_LIMIT_CM)
        {
          SERIALPRINT((hypotenuse<OBSTACLE_AHEAD_EMERGENCY_STOP_RANGE_LIMIT_CM)?".":"¤");
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

