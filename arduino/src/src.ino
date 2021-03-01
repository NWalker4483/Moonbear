/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */

#define SAFETY_PIN 3
#define THROTTLE_PIN 6
#define STEERING_PIN 11

#define THROTTLE_DIR_PIN 6
#define STEERING_DIR_PIN 11

#define STEERING_FEEDBACK_PIN 2
#define STEERING_SPEED 25

#define MAX_STEERING_ANGLE 45

#define MAX_STEERING_READING 1023
#define MIN_STEERING_READING 0

#define TIMEOUT_SECONDS 2

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

  // Scale to 8 bits (0 - 255).
  value = value / 4;

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
// Globals
int target_steering_angle = 0;
unsigned long lastCmdMilli = 0; // time at the end of the last command

void set_Throttle(int _speed)
{
  digitalWrite(THROTTLE_DIR_PIN, (_speed > 0) ? HIGH : LOW);
  int cmd = map(abs(_speed), 0, 100, 1023, 0);
  analogWrite(THROTTLE_PIN, cmd);
}

void set_Steering(int _angle)
{
  target_steering_angle = constrain(_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

int reading2degree(int reading)
{
  return map(MIN_STEERING_READING, MAX_STEERING_READING, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
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
    }
    Serial.flush();
  }
}

void CheckPowerSteering()
{
  int reading = smoothAnalogRead(STEERING_FEEDBACK_PIN);
  int angle = reading2degree(reading);

  digitalWrite(STEERING_DIR_PIN, (target_steering_angle > angle) ? HIGH : LOW);

  if (abs(target_steering_angle - angle) > 2)
  {
    int cmd = map(abs(STEERING_SPEED), 0, 100, 1023, 0);
    analogWrite(STEERING_PIN, cmd);
  }
  else
  {
    digitalWrite(STEERING_PIN, HIGH);
  }
}

void setup()
{
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(STEERING_PIN, OUTPUT);

  digitalWrite(THROTTLE_PIN, HIGH);
  digitalWrite(STEERING_PIN, HIGH);

  pinMode(SAFETY_PIN, OUTPUT);
  digitalWrite(SAFETY_PIN, HIGH); // Disable Safety
}

void loop()
{
  ReadSerialCommands();
  CheckPowerSteering();
  if ((millis() - lastCmdMilli) > (TIMEOUT_SECONDS * 1000))
  { // Cut-off for safety if no commands read
    digitalWrite(THROTTLE_PIN, HIGH);
    digitalWrite(SAFETY_PIN, LOW);
  }
  else
  {
    digitalWrite(SAFETY_PIN, HIGH);
  }
}
