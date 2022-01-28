#include "car.h"

Car::Car(): lights()
{
  absolute_time_t timeout;
  acceleration = 1;
  deceleration = 1;

  _gpio_init(   DRIVE_MOTOR_DIRECTION_PIN);
  gpio_set_dir( DRIVE_MOTOR_DIRECTION_PIN,    GPIO_OUT);
  _gpio_init(   STEERING_MOTOR_DIRECTION_PIN);
  gpio_set_dir( STEERING_MOTOR_DIRECTION_PIN, GPIO_OUT);
  _gpio_init(   STEERING_MOTOR_SPEED_PIN);
  gpio_set_dir( STEERING_MOTOR_SPEED_PIN,     GPIO_OUT);

  gpio_set_function(DRIVE_MOTOR_SPEED_PIN, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  uint16_t slice_num = pwm_gpio_to_slice_num(DRIVE_MOTOR_SPEED_PIN);
  pwm_set_wrap(slice_num, 255);
  pwm_set_gpio_level (DRIVE_MOTOR_SPEED_PIN, 0 );
  // Set the PWM running
  pwm_set_enabled(slice_num, true);
    
  // initialise radar measurements
  for(int16_t r=0; r<RADAR_MEASUREMENTS; r++) radar_forward_measurements[ r]=0;
  for(int16_t f=0; f<RADAR_MEASUREMENTS; f++) radar_rearward_measurements[f]=0;

  SetDriveMotorSpeed(0);
  Drive(forward);
  Steer(none);

  printf("Testing lights\n");
  lights.Off();
  for (uint8_t i = 0; i<4; i++)
  {
    timeout = make_timeout_time_ms(1500);
    while( !time_reached(timeout) )
    {
      switch(i)
      {
        case 0: lights.LeftFlash(); break;
        case 1: lights.RearFlash(); break;
        case 2: lights.RightFlash(); break;
        case 3: lights.FrontFlash(); break;
      }
    }
    lights.Off();
  }

}
void Car::Iterator()
{
  uint32_t fifo_message;

  while (multicore_fifo_pop_timeout_us(200, &fifo_message))
  {
    DecodeFifo( fifo_message );
  }

  if (CollisionAvoidance() )
  {
    printf("Collision Avoidance!\n");
    EmergencyStop();
  }

  printf("Plan route\n");
  PlanRoute();
  printf("Plan speed\n");
  PlanSpeed();

  SetLights();
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
  if (Stopped())
  {
    HazardLightsOn();
    return;
  }

  if (DirectionIsForward())
  {
    if (TurningLeft())
    {
      lights.LeftFlash();
      lights.RightOff();
    }
    else if (TurningRight())
    {
      lights.RightFlash();
      lights.LeftOff();
    }
    else
    {
      lights.FrontOn();
      lights.RearOff();
    }
  }
  else if (DirectionIsReverse())
  {
    if (TurningLeft())
    {
      lights.LeftFlash();
      lights.RightOff();
    }
    else if (TurningRight())
    {
      lights.RightFlash();
      lights.LeftOff();
    }
    else
    {
      lights.FrontOff();
      lights.RearOn();
    }
  }
}
void Car::PlanRoute()
{
  printf("Plan Route\n");
  if (DirectionIsForward())
  {
    printf("Plan Route; current direction is forward\n");
    if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE))
    {
      printf("Plan Route; no obstacle ahead\n");
      printf("Plan Route; decision: steer none, drive forward\n");
      Steer(none);
      Drive(forward);
    }
    else if (!(ObstacleDetectedBehind(STEERING_REAR_RANGE)))
    {
      printf("Plan Route; obstacle ahead, no obstacle behind\n");
      printf("Plan Route; decision: steer none, drive reverse\n");
      Steer(none);
      Drive(reverse);
    }
    else
    {
      printf("Plan Route; obstacle ahead & behind\n");
      printf("Plan Route; decision: no option to change course\n");
      Steer(none);
      Drive(forward);
    }
  }
  else if (DirectionIsReverse())
  {
    printf("Plan Route; current direction is reverse\n");

    if (!(ObstacleDetectedBehind(STEERING_REAR_RANGE)))
    {
      printf("Plan Route; no obstacle behind\n");
      printf("Plan Route; decision: steer none, drive reverse\n");
      Steer(none);
      Drive(reverse);
    }
    else if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE))
    {
      printf("Plan Route; obstacle behind, no obstacle ahead\n");
      printf("Plan Route; decision: steer none, drive forward\n");
      Steer(none);
      Drive(forward);
    }
    else
    {
      printf("Plan Route; obstacle ahead & behind\n");
      printf("Plan Route; decision: no option to change course\n");
      Steer(none);
      Drive(reverse);
    }
  }

}
void Car::PlanSpeed()
{
  printf("Plan Speed\n");

  if (DirectionIsForward())
  {
    if (ObstacleDetectedAhead(STEERING_FORWARD_RANGE))
    {
      printf("Forward Stop\n");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        printf("Forward Low Speed\n");
        SetDriveMotorSpeed(LOW_SPEED);
      }
      else
      {
        printf("Forward High Speed\n");
        SetDriveMotorSpeed(HIGH_SPEED);
      }
    }
  }
  else if (DirectionIsReverse())
  {
    if (ObstacleDetectedBehind(STEERING_REAR_RANGE))
    {
      printf("Reverse Stop\n");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        printf("Reverse Low Speed\n");
        SetDriveMotorSpeed(LOW_SPEED);
      }
      else
      {
        printf("Reverse High Speed\n");
        SetDriveMotorSpeed(HIGH_SPEED);
      }
    }
  }
}
/*void Car::PlanRoute1()
{
  if (DirectionIsForward())
  {
    if (ObstacleDetectedAhead(STEERING_FORWARD_RANGE))
    {
      SetDriveMotorSpeed(LOW_SPEED); // slow
      if (TurningLeft())
      {
        Steer(none);
      }
      else if (TurningRight())
      {
        Steer(none);
      }
      else
      {
        uint8_t left_distance    = GetRadarForwardMeasurements(-1 * RADAR_SCAN_STEP_DEGREES);
        uint8_t centre_distance  = GetRadarForwardMeasurements(0 * RADAR_SCAN_STEP_DEGREES);
        uint8_t right_distance   = GetRadarForwardMeasurements(1 * RADAR_SCAN_STEP_DEGREES);
        if ( // we can saftely steer to left
          (left_distance > STEERING_FORWARD_RANGE)
          &&
          (centre_distance > STEERING_FORWARD_RANGE)
          &&
          (left_distance > right_distance)
        )
        {
          Steerleft;
        }
        else if ( // we can saftely steer to right
          (right_distance > STEERING_FORWARD_RANGE)
          &&
          (centre_distance > STEERING_FORWARD_RANGE)
          &&
          (right_distance > left_distance)
        )
        {
          Steerright;
        }
        else // can't avoid, so reverse
        {
          Steer(none);
          Drive(reverse);
        }
      }
    }
    else //direction is forward and no obstacle ahead
    {
      SetDriveMotorSpeed(HIGH_SPEED); // fast
    }
  }
  else if (DirectionIsReverse())
  {
    if (ObstacleDetectedBehind(STEERING_REAR_RANGE))
    {
      SetDriveMotorSpeed(LOW_SPEED); // slow
      if (TurningLeft())
      {
        Steer(none);
      }
      else if (TurningRight())
      {
        Steer(none);
      }
      else
      {
        uint8_t left_distance    = GetRadarRearwardMeasurements( 1 * RADAR_SCAN_STEP_DEGREES);
        uint8_t centre_distance  = GetRadarRearwardMeasurements( 0 * RADAR_SCAN_STEP_DEGREES);
        uint8_t right_distance   = GetRadarRearwardMeasurements(-1 * RADAR_SCAN_STEP_DEGREES);

        if ( // we can saftely steer to left
          (left_distance > STEERING_REAR_RANGE)
          &&
          (centre_distance > STEERING_REAR_RANGE)
          &&
          (left_distance > right_distance)
        )
        {
          Steerleft;
        }
        else if ( // we can saftely steer to right
          (right_distance > STEERING_REAR_RANGE)
          &&
          (centre_distance > STEERING_REAR_RANGE)
          &&
          (right_distance > left_distance)
        )
        {
          Steerright;
        }
        else // can't avoid, so go forward
        {
          Steer(none);
          Drive(forward);
        }
      }
    }
    else //no obstacle behind
    {
      SetDriveMotorSpeed(HIGH_SPEED); // fast
    }
  }
}*/
bool Car::Stopped()
{
  return (last_drive_motor_speed == 0);
}
bool Car::Moving()
{
  return ( MovingForward() || MovingBackward() );
}
bool Car::MovingForward()
{
  return ( (last_drive_motor_direction == forward ) && (last_drive_motor_speed > 0) );
}
bool Car::MovingBackward()
{
  return ( (last_drive_motor_direction == reverse ) && (last_drive_motor_speed > 0) );
}
bool Car::DirectionIsForward()
{
  return (last_drive_motor_direction == forward);
}
bool Car::DirectionIsReverse()
{
  return (last_drive_motor_direction == reverse);
}
bool Car::Turning()
{
  return ( last_steering_motor_direction != none );
}
bool Car::TurningLeft()
{
  return ( last_steering_motor_direction == left );
}
bool Car::TurningRight()
{
  return ( last_steering_motor_direction == right );
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
      Serial.print("Obstacle ahead. Range ");
      Serial.print(range);
      Serial.print(" cm (min ");
      Serial.print(range_limit);
      Serial.print(" cm) @angle=");
      Serial.println(i * RADAR_SCAN_STEP_DEGREES);
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
      Serial.print("Obstacle behind. Range ");
      Serial.print(range);
      Serial.print(" cm (min ");
      Serial.print(range_limit);
      Serial.print(" cm) @angle=");
      Serial.println(i * RADAR_SCAN_STEP_DEGREES);

      obstacle_detected = true;
    }

  }
  return obstacle_detected;
}
bool Car::CollisionAvoidance()
{
  if (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE))
  {
    Serial.println("Vehicle direction is forward, but obstacle too close to front of vehicle!");
  }
  else if (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE))
  {
    Serial.println("Vehicle direction is reverse, but obstacle too close to rear of vehicle!");
  }

  return (
           (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE))
           ||
           (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE))
         );
}
void Car::SetDriveMotorSpeed(uint8_t s)
{
  printf("Set drive speed\n");

  if (s == last_drive_motor_speed)
  {
    printf("No speed change\n");
    return;
  }
  printf("Changing speed from %iu to %iu\n",
         last_drive_motor_speed, s);

  if ( s > last_drive_motor_speed )
  {
    printf("Accelerating\n");

    for (int i = last_drive_motor_speed; (i <= s) && !(CollisionAvoidance()); i++) {
      pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, i);
      last_drive_motor_speed = i;
    }
  } else if ( s < last_drive_motor_speed )
  {
    printf("Decelerating\n");

    for (int i = last_drive_motor_speed; (i >= s) && !(CollisionAvoidance()); i--) {
      pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, i);
      last_drive_motor_speed = i;
    }
  }

  uint32_t speed_message = DRIVE_SPEED_FIFO_MESSAGE | s;
  multicore_fifo_push_timeout_us(speed_message,1000);

}
void Car::EmergencyStop()
{
  printf("<<< EMERGENCY STOP >>>\n");

  // Power down drive motor
  pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, 0);
  last_drive_motor_speed = 0;
  // Power down main motor relay - causes strange behavior if vehicle is in reverse with obstacle behind
  // Drive( forward );
  // Power down steering motor
  Steer( none );

  Serial.println("8888888888 888b     d888 8888888888 8888888b.   .d8888b.  8888888888 888b    888  .d8888b. Y88b   d88P");
  Serial.println("888        8888b   d8888 888        888   Y88b d88P  Y88b 888        8888b   888 d88P  Y88b Y88b d88P");
  Serial.println("888        88888b.d88888 888        888    888 888    888 888        88888b  888 888    888  Y88o88P");
  Serial.println("8888888    888Y88888P888 8888888    888   d88P 888        8888888    888Y88b 888 888          Y888P");
  Serial.println("888        888 Y888P 888 888        8888888P'  888  88888 888        888 Y88b888 888           888");
  Serial.println("888        888  Y8P  888 888        888 T88b   888    888 888        888  Y88888 888    888    888");
  Serial.println("888        888   '   888 888        888  T88b  Y88b  d88P 888        888   Y8888 Y88b  d88P    888");
  Serial.println("8888888888 888       888 8888888888 888   T88b  'Y8888P88 8888888888 888    Y888  'Y8888P'     888");
  Serial.println("");
  Serial.println("");
  Serial.println("                           .d8888b. 88888888888 .d88888b.  8888888b.  888");
  Serial.println("                          d88P  Y88b    888    d88P' 'Y88b 888   Y88b 888");
  Serial.println("                          Y88b.         888    888     888 888    888 888");
  Serial.println("                           'Y888b.      888    888     888 888   d88P 888");
  Serial.println("                              'Y88b.    888    888     888 8888888P'  888");
  Serial.println("                                '888    888    888     888 888        Y8P");
  Serial.println("                          Y88b  d88P    888    Y88b. .d88P 888         !");
  Serial.println("                           'Y8888P'     888     'Y88888P'  888        888");

}
void Car::Drive( drive_motor_direction direction )
{
  int s = last_drive_motor_speed;

  // Change direction
  switch (direction) {
    case forward:
      if (!DirectionIsForward())
      {
        printf("Changing direction to forward\n");

        // Slow to a stop
        SetDriveMotorSpeed( 0 );
        gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
      } else
      {
        printf("Maintaining forward direction\n");

      }
      last_drive_motor_direction = forward;
      multicore_fifo_push_timeout_us(DRIVE_FORWARD_FIFO_MESSAGE,1000);
      break;
    case reverse:
      if (!DirectionIsReverse())
      {
        printf("Changing direction to reverse\n");

        // Slow to a stop
        SetDriveMotorSpeed( 0 );
        gpio_put(DRIVE_MOTOR_DIRECTION_PIN, true );
      } else
      {
        printf("Maintaining reverse direction\n");

      }
      last_drive_motor_direction = reverse;
      multicore_fifo_push_timeout_us(DRIVE_REVERSE_FIFO_MESSAGE,1000);
      break;
  }

  // Accelerate back to speed
  SetDriveMotorSpeed( s );
}
void Car::Steer( steering_motor_direction direction )
{
  // Change direction
  switch (direction) {
    case left:
      last_steering_motor_direction = left;
      printf("Steering left\n");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, true );
      gpio_put(STEERING_MOTOR_SPEED_PIN, true );

      multicore_fifo_push_timeout_us(STEER_LEFT_FIFO_MESSAGE,1000);
      break;
    case right:
      last_steering_motor_direction = right;
      printf("Steering to right\n");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
      gpio_put(STEERING_MOTOR_SPEED_PIN, true );

      multicore_fifo_push_timeout_us(STEER_RIGHT_FIFO_MESSAGE,1000);
      break;
    default:
      last_steering_motor_direction = none;
      printf("Steering cancel\n");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
      gpio_put(STEERING_MOTOR_SPEED_PIN, false );

      multicore_fifo_push_timeout_us(STEER_NONE_FIFO_MESSAGE,1000);
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
    //lights.RearLeftFlash();
    return;
  }
  //lights.RearRightFlash();

  for(uint8_t i=0; i<5; i++) decoded_message[i]=0;
  decoded_message[0] = (fifo_message & 0xFF000000) >> 24;
  decoded_message[1] = (fifo_message & 0x00FF0000) >> 16;

  switch (decoded_message[0])
  {
    case 'R': // radar measurement
      angle = ((fifo_message & 0x0000FF00) >> 8);
      range = (fifo_message & 0x000000FF);
      index = (angle / RADAR_MEASUREMENTS_GRANULARITY);
      switch (decoded_message[1])
      {
        case 'F': // forward
          radar_forward_measurements[index] = range;
          break;
        case 'R': // rearward
          radar_rearward_measurements[index] = range;
          break;
        default:
          break;
      }
      break;
    default:
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

void Car::DumpRadarMetrics()
{
  Serial.print(CLEAR);
  Serial.println("........................................................................................");
  for(int16_t r=0; r<RADAR_MEASUREMENTS; r++)
  {
    Serial.print(r*RADAR_MEASUREMENTS_GRANULARITY);
    Serial.print(":\t");
    Serial.print(ConvertServoAngleToCarAngle(r*RADAR_MEASUREMENTS_GRANULARITY));
    Serial.print(":\t");
    Serial.print(radar_forward_measurements[ r]);
    Serial.print(",\t");
    Serial.print(radar_rearward_measurements[r]);
    Serial.println();
  }
    Serial.println("........................................................................................");
  
}
