/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */
/////////////////////////////
#define ThrottleControlPin 12 // Also acts as X when in mixed mode and throttle when in RC Mode
#define SteeringControlPin 9 // Also acts as Y when in mixed mode and steering when in RC Mode
/////////////////////////////
#define MAX_SPEED 36       // %
/////////////////////////////
#define LoopTime 100 // 10 Hz
///////////////

#include <Servo.h>

int current_throttle_setting = 0;
int current_steering_angle = 0;

int parseIntFast(int numberOfDigits)
{
  /*
  This function returns the converted integral number as an int value.
  If no valid conversion could be performed, it returns zero.*/
  char theNumberString[numberOfDigits + 1];
  int theNumber;
  for (int i = 0; i < numberOfDigits; theNumberString[i++] = Serial.read())
  {
    delay(5);
  };
  theNumberString[numberOfDigits] = 0x00;
  theNumber = atoi(theNumberString);
  return theNumber;
}

//Globals
unsigned long lastMilli = 0; // time at the end of the last loop
boolean moving_forward = false;

Servo Throttle;
Servo Steering;

void Brake()
{                         // Disengages the brake on the ESC
  moving_forward = false; // Set first to prevent recurion
  Throttle.write(60);
  delay(100);
  Throttle.write(90);
  delay(100);
}

void set_Throttle(int _speed)
{
  if ((_speed < 0) && moving_forward)
  {
    Brake();
  }
  if (_speed > 0)
  {
    moving_forward = true;
  }
  current_throttle_setting  = _speed;
  _speed = map(_speed, -100, 100, 55, 145);
  Throttle.write(_speed);
}

void set_Steering(int _angle){
  Steering.write(_angle);
  current_steering_angle = _angle; 
}
  
void ReadSerialCommands()
{ // Plot PID Values using Serial Plotter
  if (Serial.available())
  {
    char cmdByte = Serial.read();
    int setting;
    switch (cmdByte)
    {
    case 'T': // Change Proportianal Gain
      setting = parseIntFast(4);
      set_Throttle(setting);
      Serial.println(current_throttle_setting);
      break;
    case 'S': // Change Integral Gain
      setting = parseIntFast(4);
      set_Steering(setting);
      Serial.println(current_steering_angle);
      break;
    }
    Serial.flush();
  }
}

void setup()
{
  pinMode(SteeringControlPin, OUTPUT);
  pinMode(ThrottleControlPin, OUTPUT);

  Throttle.attach(ThrottleControlPin);
  Steering.attach(SteeringControlPin);

  // Throttle.write(90);
  // delay(2000);
  // Throttle.write(0);
  // delay(2000);
  // Throttle.write(180);
  // delay(2000);
  Throttle.write(90);
  delay(2000);
}

void loop()
{
  ReadSerialCommands();
}
//////////////////////
