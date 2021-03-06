#include "car.h"

Car::Car(): lights()
{
  absolute_time_t timeout;
  acceleration = 1;
  deceleration = 1;

  _gpio_init(   DRIVE_MOTOR_DIRECTION_PIN);
  gpio_set_dir( DRIVE_MOTOR_DIRECTION_PIN,    GPIO_OUT);
  gpio_put(     DRIVE_MOTOR_DIRECTION_PIN,    false );
  
  _gpio_init(   STEERING_MOTOR_DIRECTION_PIN);
  gpio_set_dir( STEERING_MOTOR_DIRECTION_PIN, GPIO_OUT);
  gpio_put(     STEERING_MOTOR_DIRECTION_PIN, false );
  
  _gpio_init(   STEERING_MOTOR_SPEED_PIN);
  gpio_set_dir( STEERING_MOTOR_SPEED_PIN,     GPIO_OUT);
  gpio_put(     STEERING_MOTOR_SPEED_PIN,     false );

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
  uint32_t fifo_message;

  while (multicore_fifo_rvalid())
  {
    fifo_message = multicore_fifo_pop_blocking();
    Serial.println("fifo message received on core0");
    DecodeFifo( fifo_message );
  }

  Serial.println("Plan route");
  PlanRoute();

  if (CollisionAvoidance() )
  {
    Serial.println("Collision avoidance activated!");
    EmergencyStop();
  }
  else
  {
    Serial.println("Plan speed");
    PlanSpeed();
  }

  SetLights();
  PlotRadarMetrics(25);
  delay(250);
  //DumpRadarMetrics();
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
  Serial.println("Plan Route\n");
  if (DirectionIsForward())
  {
    Serial.println("Plan Route; current direction is forward\n");
    if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      Serial.println("Plan Route; no obstacle ahead\n");
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
            Serial.println("Plan Route; decision: steer left->left, drive forward\n");
            Steer(left);
          }
          else
          {
            Serial.println("Plan Route; decision: steer left->none, drive forward\n");
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
            Serial.println("Plan Route; decision: steer right->right, drive forward\n");
            Steer(right);
          }
          else
          {
            Serial.println("Plan Route; decision: steer right->none, drive forward\n");
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
          Serial.println("Plan Route; decision: steer none->right, drive forward\n");
          Steer(right);
        } else
        if (
            (GetRadarForwardMeasurements(LEFT1)+GetRadarForwardMeasurements(DEAD_AHEAD))
            >
            (GetRadarForwardMeasurements(DEAD_AHEAD)+GetRadarForwardMeasurements(RIGHT1))
           )
        {
          Serial.println("Plan Route; decision: steer none->left, drive forward\n");
          Steer(left);
        }
      }
    }
    else if (!(ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT)))
    {
      Serial.println("Plan Route; obstacle ahead, no obstacle behind");
      Serial.println("Plan Route; decision: steer none, drive reverse");
      Steer(none);
      Drive(reverse);
    }
    else
    {
      Serial.println("Plan Route; obstacle ahead & behind");
      Serial.println("Plan Route; decision: no option to change course");
      Steer(none);
    }
  }
  else if (DirectionIsReverse())
  {
    Serial.println("Plan Route; current direction is reverse\n");
    if (!ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT))
    {
      Serial.println("Plan Route; no obstacle behind\n");
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
            Serial.println("Plan Route; decision: steer left->left, drive reverse\n");
            Steer(left);
          }
          else
          {
            Serial.println("Plan Route; decision: steer left->none, drive reverse\n");
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
            Serial.println("Plan Route; decision: steer right->right, drive reverse\n");
            Steer(right);
          }
          else
          {
            Serial.println("Plan Route; decision: steer right->none, drive reverse\n");
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
          Serial.println("Plan Route; decision: steer none->left, drive reverse\n");
          Steer(left);
        } else
        if (
            (GetRadarRearwardMeasurements(LEFT1)+GetRadarRearwardMeasurements(DEAD_AHEAD))
            >
            (GetRadarRearwardMeasurements(DEAD_AHEAD)+GetRadarRearwardMeasurements(RIGHT1))
           )
        {
          Serial.println("Plan Route; decision: steer none->right, drive reverse\n");
          Steer(right);
        }
      }
    }
    else if (!ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      Serial.println("Plan Route; obstacle behind, no obstacle ahead");
      Serial.println("Plan Route; decision: steer none, drive forward");
      Steer(none);
      Drive(forward);
    }
    else
    {
      Serial.println("Plan Route; obstacle ahead & behind");
      Serial.println("Plan Route; decision: no option to change course");
      Steer(none);
      Drive(reverse);
    }
  }

}
void Car::PlanSpeed()
{
  Serial.println("Plan Speed");

  if (DirectionIsForward())
  {
    if (ObstacleDetectedAhead(STEERING_FORWARD_RANGE_LIMIT))
    {
      Serial.println("Forward Stop");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        Serial.println("Forward Low Speed");
        SetDriveMotorSpeed(LOW_SPEED);
      }
      else
      {
        Serial.println("Forward High Speed");
        SetDriveMotorSpeed(HIGH_SPEED);
      }
    }
  }
  else if (DirectionIsReverse())
  {
    if (ObstacleDetectedBehind(STEERING_REAR_RANGE_LIMIT))
    {
      Serial.println("Reverse Stop");
      SetDriveMotorSpeed(0);
    }
    else
    {
      if (Turning())
      {
        Serial.println("Reverse Low Speed");
        SetDriveMotorSpeed(LOW_SPEED);
      }
      else
      {
        Serial.println("Reverse High Speed");
        SetDriveMotorSpeed(HIGH_SPEED);
      }
    }
  }
}
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
  if (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE_LIMIT))
  {
    Serial.println("Vehicle direction is forward, but obstacle too close to front of vehicle!");
  }
  else if (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE_LIMIT))
  {
    Serial.println("Vehicle direction is reverse, but obstacle too close to rear of vehicle!");
  }

  return (
           (DirectionIsForward() && ObstacleDetectedAhead(MINIMUM_FORWARD_RANGE_LIMIT))
           ||
           (DirectionIsReverse() && ObstacleDetectedBehind(MINIMUM_REAR_RANGE_LIMIT))
         );
}
void Car::SetDriveMotorSpeed(uint8_t s)
{
  Serial.println("Set drive speed");

  if (s == last_drive_motor_speed)
  {
    Serial.println("No speed change");
    return;
  }
  
  Serial.print("Changing speed from ");
  Serial.print(last_drive_motor_speed);
  Serial.print(" to ");
  Serial.println( s);

  if ( s > last_drive_motor_speed )
  {
    Serial.println("Accelerating");

    for (int i = last_drive_motor_speed; (i <= s) && !(CollisionAvoidance()); i++) {
      pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, i);
      last_drive_motor_speed = i;
    }
  } else if ( s < last_drive_motor_speed )
  {
    Serial.println("Decelerating");

    for (int i = last_drive_motor_speed; (i >= s) && !(CollisionAvoidance()); i--) {
      pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, i);
      last_drive_motor_speed = i;
    }
  }
}
void Car::EmergencyStop()
{
  Serial.println("<<< EMERGENCY STOP >>>");

  // Power down drive motor
  pwm_set_gpio_level(DRIVE_MOTOR_SPEED_PIN, 0);
  // Power down steering motor
  gpio_put(STEERING_MOTOR_SPEED_PIN, false );
  // Power down steering relay
  gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
  // Power down steering motor
  gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );

  Drive(forward);
  SetDriveMotorSpeed(0);
  Steer(none);
  SendStatus();
  
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
  // Change direction
  switch (direction) {
    case forward:
      if (!DirectionIsForward())
      {
        Serial.println("Changing direction to forward");

        // Slow to a stop
        SetDriveMotorSpeed( 0 );
        gpio_put(DRIVE_MOTOR_DIRECTION_PIN, false );
      } else
      {
        Serial.println("Maintaining forward direction");
      }
      last_drive_motor_direction = forward;
      break;
    case reverse:
      if (!DirectionIsReverse())
      {
        Serial.println("Changing direction to reverse");

        // Slow to a stop
        SetDriveMotorSpeed( 0 );
        gpio_put(DRIVE_MOTOR_DIRECTION_PIN, true );
      } else
      {
        Serial.println("Maintaining reverse direction");
      }
      last_drive_motor_direction = reverse;
      break;
  }

}
void Car::Steer( steering_motor_direction direction )
{
  // Change direction
  switch (direction) {
    case left:
      last_steering_motor_direction = left;
      Serial.println("Steering left");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, true );
      gpio_put(STEERING_MOTOR_SPEED_PIN, true );
      break;
    case right:
      last_steering_motor_direction = right;
      Serial.println("Steering to right");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
      gpio_put(STEERING_MOTOR_SPEED_PIN, true );
      break;
    default:
      last_steering_motor_direction = none;
      Serial.println("Steering cancel");

      gpio_put(STEERING_MOTOR_DIRECTION_PIN, false );
      gpio_put(STEERING_MOTOR_SPEED_PIN, false );
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
    Serial.println("Radar ready");
    SendStatus();
    return;
  }

  for(uint8_t i=0; i<5; i++) decoded_message[i]=0;
  decoded_message[0] = (fifo_message & 0xFF000000) >> 24;
  decoded_message[1] = (fifo_message & 0x00FF0000) >> 16;

  switch (decoded_message[0])
  {
    case 'R': // radar measurement
      Serial.print("Radar  measurement ");
      angle = ((fifo_message & 0x0000FF00) >> 8);
      range = (fifo_message & 0x000000FF);
      index = (angle / RADAR_MEASUREMENTS_GRANULARITY);
      switch (decoded_message[1])
      {
        case 'F': // forward
          Serial.print("forward ");
          radar_forward_measurements[index] = range;
          break;
        case 'R': // rearward
          Serial.print("rearward ");
          radar_rearward_measurements[index] = range;
          break;
        default:
          break;
      }
      Serial.print(angle);
      Serial.print(" degs, range ");
      Serial.println(range);
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

void Car::SendStatus()
{
  uint32_t message = CAR_FIFO_MESSAGE;
  message = message & 0xFF000000; // C___
  
  switch (last_steering_motor_direction) {
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

  switch (last_drive_motor_direction) {
    case forward:
      message = message | 'F' << 16;
      break;
    case reverse:
      message = message | 'R' << 16;
      break;
  }

  message = message | last_drive_motor_speed;

  if (multicore_fifo_wready())
    multicore_fifo_push_blocking( message ); 
}
void Car::DumpRadarMetrics()
{
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
void Car::PlotRadarMetrics(int d)
{
  float o,a,h,ft,rt,m,s;
  int i,r, starty, endy;
  r = round( d/2 );
  s = 255/((float) r);

  starty = (Stopped()||DirectionIsForward())?r:0;
  endy   = (Stopped()||DirectionIsReverse())?-r:0;

  for (int y=starty; y>=endy; y--)
  {
    for (int x=-r; x<=r; x++)
    {
      a = x*s;
      o = abs(y*s);
      h = sqrt(pow(a,2)+pow(o,2));
      rt = 180 - (atan2(o,a)*57.295779513);
      ft = (atan2(o,a)*57.295779513);

      if (y>0)
      {
        i = round (ft / RADAR_MEASUREMENTS_GRANULARITY);
        m = radar_forward_measurements[i];
      }
      else
      {
        i = round (rt / RADAR_MEASUREMENTS_GRANULARITY);
        m = radar_rearward_measurements[i];
      }

      if (y==0)
      {
        Serial.print( (x==0)? "+":"-");
      }
      else if (x==0)
      {
        if (h<m)
        {
          Serial.print("#");
        }
        else
        {
          Serial.print( "|");
        }
      }
      else if (h<m)
      {
        Serial.print("#");
      }
      else if ((h/255)>1)
      {
        Serial.print(" ");
      }
      else
      {
        if(h<STEERING_FORWARD_RANGE_LIMIT)
        {
          Serial.print((h<MINIMUM_FORWARD_RANGE_LIMIT)?".":"??");
        }
        else
        {
          Serial.print(".");
        }
      }
    }      
    Serial.println();   
  }
 
}
