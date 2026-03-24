#include "DualTB9051FTGMotorShieldUnoMega.h"



// Constructors ////////////////////////////////////////////////////////////////

//Default Constructor
DualTB9051FTGMotorShieldUnoMega::DualTB9051FTGMotorShieldUnoMega()
{
  //Pin map
  _M1EN = 2;
  _M1DIR = 7;
  _M1PWM = 9;
  _M1DIAG = 6;
  _M1OCM = A0;

  _M2EN = 4;
  _M2DIR = 8;
  _M2PWM = 10;
  _M2DIAG = 12;
  _M2OCM = A1;

  _flipM1 = false;
  _flipM2 = false;
}

//Constructor with User-Defined Pins
DualTB9051FTGMotorShieldUnoMega::DualTB9051FTGMotorShieldUnoMega(unsigned char M1EN,
                                                   unsigned char M1DIR,
                                                   unsigned char M1PWM,
                                                   unsigned char M1DIAG,
                                                   unsigned char M1OCM,
                                                   unsigned char M2EN,
                                                   unsigned char M2DIR,
                                                   unsigned char M2PWM,
                                                   unsigned char M2DIAG,
                                                   unsigned char M2OCM)
{
  _M1EN = M1EN;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  _M1DIAG = M1DIAG;
  _M1OCM = M1OCM;

  _M2EN = M2EN;
  _M2DIR = M2DIR;
  _M2PWM = M2PWM;
  _M2DIAG = M2DIAG;
  _M2OCM = M2OCM;

  _flipM1 = false;
  _flipM2 = false;

}

// Public Methods //////////////////////////////////////////////////////////////

void DualTB9051FTGMotorShieldUnoMega::init()
{
// Define pinMode for the pins and set the frequency for timer2.

  pinMode(_M1EN, OUTPUT);
  pinMode(_M2EN, OUTPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1DIAG, INPUT_PULLUP);
  pinMode(_M1OCM, INPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2DIAG, INPUT_PULLUP);
  pinMode(_M2OCM, INPUT);

#ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    // Timer 1 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
  }
#endif
#ifdef DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE
	if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
	{
		// Timer 2 configuration
		// prescaler: clockI/O / 8
		// outputs enabled
		// fast PWM
		// top of 255
		//
		// PWM frequency calculation
		// 16MHz / 8 (prescaler) / 1 (fast PWM) / 255 (top) = 7.8kHz
		TCCR2A = 0b10100011; //11 in first two bits = fast PWM (could switch to phase correct PWM by making first two bits = 01, PWM frequency would be 3.9kHz)
		TCCR2B = 0b00000010; //first 3 bits are prescaler
	}
  #endif

}

// Set speed for motor 1, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldUnoMega::setM1Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
    {
      OCR1A = speed;
    }
    else
    {
      analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #elif defined(DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE)
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2B = speed*51/80; // map 400 to 255
    }
    else
    {
      analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
  #endif
 
  if (forward ^ _flipM1) // DIR high if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M1DIR, HIGH);
  }
  else
  {
    digitalWrite(_M1DIR, LOW);
  }
}

// Set speed for motor 2, speed is a number between -400 and 400
void DualTB9051FTGMotorShieldUnoMega::setM2Speed(int speed)
{
  unsigned char forward = 1;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    forward = 0;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  #ifdef DUALTB9051FTGMOTORSHIELD_TIMER1_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
    {
      OCR1B = speed;
    }
    else
    {
      analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #elif defined(DUALTB9051FTGMOTORSHIELD_TIMER2_AVAILABLE)
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2A = speed*51/80; // map 400 to 255
    }
    else
    {
      analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
  #endif

  if (forward ^ _flipM2) // DIR high if speed was negative or _flipM2 setting is active, but not both
  {
    digitalWrite(_M2DIR, HIGH);

  }
  else
  {
    digitalWrite(_M2DIR, LOW);

  }
}


// Set speed for both motors
void DualTB9051FTGMotorShieldUnoMega::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Return error status for motor 1
unsigned char DualTB9051FTGMotorShieldUnoMega::getM1Fault()
{
  return !digitalRead(_M1DIAG);
}

// Return error status for motor 2
unsigned char DualTB9051FTGMotorShieldUnoMega::getM2Fault()
{
  return !digitalRead(_M2DIAG);
}

// Flip direction for motor 1
void DualTB9051FTGMotorShieldUnoMega::flipM1(boolean flip)
{
  DualTB9051FTGMotorShieldUnoMega::_flipM1 = flip;
}

// Flip direction for motor 2
void DualTB9051FTGMotorShieldUnoMega::flipM2(boolean flip)
{
  DualTB9051FTGMotorShieldUnoMega::_flipM2 = flip;
}

// Enable the driver for motor 1
void DualTB9051FTGMotorShieldUnoMega::enableM1Driver()
{
  digitalWrite(_M1EN, HIGH);
}

// Enable the driver for motor 2
void DualTB9051FTGMotorShieldUnoMega::enableM2Driver()
{
  digitalWrite(_M2EN, HIGH);
}

// Enable the drivers for both motors
void DualTB9051FTGMotorShieldUnoMega::enableDrivers()
{
  digitalWrite(_M1EN, HIGH);
  digitalWrite(_M2EN, HIGH);
}

// Disable the driver for motor 1
void DualTB9051FTGMotorShieldUnoMega::disableM1Driver()
{
  digitalWrite(_M1EN, LOW);
}

// Disable the driver for motor 2
void DualTB9051FTGMotorShieldUnoMega::disableM2Driver()
{
  digitalWrite(_M2EN, LOW);
}

// Disable the drivers for both motors
void DualTB9051FTGMotorShieldUnoMega::disableDrivers()
{
  digitalWrite(_M1EN, LOW);
  digitalWrite(_M2EN, LOW);
}

// Return motor 1 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldUnoMega::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M1OCM) * 10;
}

// Return motor 2 current value in milliamps.
unsigned int DualTB9051FTGMotorShieldUnoMega::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M2OCM) * 10;
}
