/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ 
*/

#define THROTTLE_ENABLE_PIN 2
#define STEERING_ENABLE_PIN 4

#define THROTTLE_CONTROL_PIN A4
#define STEERING_CONTROL_PIN A5

#define STEERING_FEEDBACK_PIN A3

#define THROTTLE_DIR_PIN 10
#define STEERING_DIR_PIN 12

#define STEERING_SPEED 25

#define MAX_STEERING_ANGLE 15

#define MAX_STEERING_READING 843
#define MIN_STEERING_READING 0

#define TIMEOUT_SECONDS 600

#include <Servo.h>

Servo Throttle;  // create servo object to control a servo
Servo Steering;  // create servo object to control a servo

// Globals
int target_steering_angle = 0;
unsigned long lastCmdMilli = 0; // time at the end of the last command

int smoothAnalogRead(int pin)
{
  int i;
  int value = 0;
  int numReadings = 10;

  for (i = 0; i < numReadings; i++)
  {
    // Read light sensor data.
    value = value + analogRead(pin);

    // 1ms pause adds more stability between reads.
    delay(1);
  }

  // Take an average of all the readings.
  value = value / numReadings;

  return value;
}

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

void set_Throttle(int _speed)
{
  digitalWrite(THROTTLE_DIR_PIN, (_speed > 0) ? HIGH : LOW);
  int cmd = map(abs(_speed), 0, 100, 0, 180);
  Throttle.write(cmd);
}

void set_Steering(int _angle)
{
  target_steering_angle = constrain(_angle, 90 - MAX_STEERING_ANGLE, 90 + MAX_STEERING_ANGLE);
}

int reading2degree(int _reading)
{
  return map(_reading, MIN_STEERING_READING, MAX_STEERING_READING, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

void ReadSerialCommands()
{ // Plot PID Values using Serial Plotter
  if (Serial.available())
  {
    char cmdByte = Serial.read();
    int setting;
    switch (cmdByte)
    {
    case 'T':
      setting = parseIntFast(4);
      set_Throttle(setting);
      Serial.println(setting);
      lastCmdMilli = millis();
      break;
    case 'S':
      setting = parseIntFast(4);
      set_Steering(setting);
      Serial.println(setting);
      break;
    case 'X':
      while (true) {
        Serial.println(reading2degree(smoothAnalogRead(STEERING_FEEDBACK_PIN)));
      }
      break;
    }
    Serial.flush();
  }
}

void UpdatePowerSteering()
{
  int reading = smoothAnalogRead(STEERING_FEEDBACK_PIN);
  if (reading < 10){ // Feedback Cable Has Come undone
    digitalWrite(STEERING_ENABLE_PIN, LOW);
    Steering.write(0);
  }
  int angle = reading2degree(reading);

  if (abs(target_steering_angle - angle) > 5) // Deadzone
  {
    digitalWrite(STEERING_DIR_PIN, (target_steering_angle > angle) ? HIGH : LOW);
    int cmd = map(abs(STEERING_SPEED), 0, 100, 0, 180);
    Steering.write(cmd);
    Serial.println(cmd);
    digitalWrite(STEERING_ENABLE_PIN, HIGH);
  }
  else
  {
    digitalWrite(STEERING_ENABLE_PIN, LOW);
    Steering.write(0);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(THROTTLE_CONTROL_PIN, OUTPUT);
  pinMode(STEERING_CONTROL_PIN, OUTPUT);
  Steering.attach(STEERING_CONTROL_PIN);
  Throttle.attach(THROTTLE_CONTROL_PIN);

  pinMode(THROTTLE_DIR_PIN, OUTPUT);
  pinMode(STEERING_DIR_PIN, OUTPUT);
  pinMode(STEERING_FEEDBACK_PIN, INPUT);

  pinMode(THROTTLE_ENABLE_PIN, OUTPUT);
  pinMode(STEERING_ENABLE_PIN, OUTPUT);

  set_Throttle(0);
  set_Steering(0);
}

void loop()
{

ReadSerialCommands();
// UpdatePowerSteering();

 if ((millis() - lastCmdMilli) > (TIMEOUT_SECONDS * 1000))
 { // Cut-off for safety if no commands read
   set_Throttle(0);
   digitalWrite(THROTTLE_ENABLE_PIN, LOW);
 }
 else
 {
   digitalWrite(THROTTLE_ENABLE_PIN, HIGH);
 }
}
