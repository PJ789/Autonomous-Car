#ifndef Lights_h

#define Lights_h

#define FRONT_RIGHT_LIGHT            18
#define FRONT_LEFT_LIGHT             19
#define REAR_RIGHT_LIGHT             20
#define REAR_LEFT_LIGHT              21


class Lights
{

  public:

    Lights();

    void On();
    void Off();
    void Flash();
    
    void FrontOn();
    void FrontOff();
    void FrontFlash();
    
    void RearOn();
    void RearOff();
    void RearFlash();
    
    void LeftOn();
    void LeftOff();
    void LeftFlash();
    
    void RightOn();
    void RightOff();
    void RightFlash(); 
    
    void FrontLeftOn();
    void FrontLeftOff();
    void FrontLeftFlash();
    
    void FrontRightOn();
    void FrontRightOff();
    void FrontRightFlash();
    
    void RearLeftOn();
    void RearLeftOff();
    void RearLeftFlash();

    void RearRightOn();
    void RearRightOff();
    void RearRightFlash();


  private:



};



#endif
