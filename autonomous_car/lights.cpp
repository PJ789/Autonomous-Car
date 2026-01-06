#include "Lights.h"
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <pico/time.h>

Lights::Lights()
{
  _gpio_init(BRAKE_RIGHT_LIGHT);
  gpio_set_dir(BRAKE_RIGHT_LIGHT, GPIO_OUT);
  _gpio_init(BRAKE_LEFT_LIGHT);
  gpio_set_dir(BRAKE_LEFT_LIGHT, GPIO_OUT);
  _gpio_init(REAR_RIGHT_LIGHT);
  gpio_set_dir(REAR_RIGHT_LIGHT, GPIO_OUT);
  _gpio_init(REAR_LEFT_LIGHT);
  gpio_set_dir(REAR_LEFT_LIGHT, GPIO_OUT);
  _gpio_init(FRONT_RIGHT_LIGHT);
  gpio_set_dir(FRONT_RIGHT_LIGHT, GPIO_OUT);
  _gpio_init(FRONT_LEFT_LIGHT);
  gpio_set_dir(FRONT_LEFT_LIGHT, GPIO_OUT);
}

void Lights::On()              { FrontOn();         RearOn();         }
void Lights::Off()             { FrontOff();        RearOff();        }
void Lights::Flash()           { FrontFlash();      RearFlash();      }

void Lights::FrontOn()         { FrontRightOn();    FrontLeftOn();    }
void Lights::FrontOff()        { FrontRightOff();   FrontLeftOff();   }
void Lights::FrontFlash()      { FrontRightFlash(); FrontLeftFlash(); }

void Lights::RearOn()          { RearRightOn();     RearLeftOn();     }
void Lights::RearOff()         { RearRightOff();    RearLeftOff();    }
void Lights::RearFlash()       { RearRightFlash();  RearLeftFlash();  }

void Lights::BrakeOn()          { BrakeRightOn();    BrakeLeftOn();   }
void Lights::BrakeOff()         { BrakeRightOff();   BrakeLeftOff();  }
void Lights::BrakeFlash()       { BrakeRightFlash(); BrakeLeftFlash();}

void Lights::LeftOn()          { FrontLeftOn();     RearLeftOn();     }
void Lights::LeftOff()         { FrontLeftOff();    RearLeftOff();    }
void Lights::LeftFlash()       { FrontLeftFlash();  RearLeftFlash();  }

void Lights::RightOn()         { FrontRightOn();    RearRightOn();    }
void Lights::RightOff()        { FrontRightOff();   RearRightOff();   }
void Lights::RightFlash()      { FrontRightFlash(); RearRightFlash(); }

void Lights::FrontLeftOn()     { gpio_put(FRONT_LEFT_LIGHT,                                           true); }
void Lights::FrontLeftOff()    { gpio_put(FRONT_LEFT_LIGHT,                                          false); }
void Lights::FrontLeftFlash()  { gpio_put(FRONT_LEFT_LIGHT,  ((( time_us_32()/1000) % 500)>250)?false:true); }

void Lights::FrontRightOn()    { gpio_put(FRONT_RIGHT_LIGHT,                                          true); }
void Lights::FrontRightOff()   { gpio_put(FRONT_RIGHT_LIGHT,                                         false); }
void Lights::FrontRightFlash() { gpio_put(FRONT_RIGHT_LIGHT, ((( time_us_32()/1000) % 500)>250)?false:true); }

void Lights::RearLeftOn()      { gpio_put(REAR_LEFT_LIGHT,                                           false); }
void Lights::RearLeftOff()     { gpio_put(REAR_LEFT_LIGHT,                                            true); }
void Lights::RearLeftFlash()   { gpio_put(REAR_LEFT_LIGHT,   ((( time_us_32()/1000) % 500)>250)?true:false); }

void Lights::RearRightOn()     { gpio_put(REAR_RIGHT_LIGHT,                                          false); }
void Lights::RearRightOff()    { gpio_put(REAR_RIGHT_LIGHT,                                           true); }
void Lights::RearRightFlash()  { gpio_put(REAR_RIGHT_LIGHT,  ((( time_us_32()/1000) % 500)>250)?true:false); }

void Lights::BrakeLeftOn()      { gpio_put(BRAKE_LEFT_LIGHT,                                         false); }
void Lights::BrakeLeftOff()     { gpio_put(BRAKE_LEFT_LIGHT,                                          true); }
void Lights::BrakeLeftFlash()   { gpio_put(BRAKE_LEFT_LIGHT, ((( time_us_32()/1000) % 500)>250)?false:true); }//out of phase with rear light

void Lights::BrakeRightOn()     { gpio_put(BRAKE_RIGHT_LIGHT,                                        false); }
void Lights::BrakeRightOff()    { gpio_put(BRAKE_RIGHT_LIGHT,                                         true); }
void Lights::BrakeRightFlash()  { gpio_put(BRAKE_RIGHT_LIGHT,((( time_us_32()/1000) % 500)>250)?false:true); }//out of phase with rear light
