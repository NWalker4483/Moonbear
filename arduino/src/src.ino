//////////////////////
/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */
/////////////////////////////
#define StatusModePin 8        // Input Pin acting as the dev jumper
#define BluetoothControlModePin 10    // Input Pin acting as the dev jumper
#define SerialControlModePin 11 // Input Pin acting as the dev jumper
/////////////////////////////////////////////////
#define EncoderPin 2 // Digital Interrupt Pin
/////////////////////////////
#define ThrottleControlPin 12 // Also acts as X when in mixed mode and throttle when in RC Mode
#define SteeringControlPin 9 // Also acts as Y when in mixed mode and steering when in RC Mode
/////////////////////////////
#define CRAWL_SPEED 20     // %
#define MAX_SPEED 36       // %
/////////////////////////////
// Physical Properties of the robot
#define WHEEL_BASE 10        //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION 9 // pulses per revolution
#define WHEEL_DIAMETER .12   // m
#define PI 3.1415926535897932384626433832795
/////////////////////////////
#define LoopTime 100 // 10 Hz
///////////////
#define    STX          0x02
#define    ETX          0x03   
//////////////

#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial BT_Serial(11,10); // RX | TX

int current_throttle_setting = 0;
int current_steering_angle = 0;
/////// Bluetooth stuff I aint write this LOL
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received

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

void getJoystickState(byte data[8])
{
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200; // Offset to avoid
  joyY = joyY - 200; // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)
    return; // commmunication error

  set_Throttle(joyY);
  set_Steering(map(-joyX, -100, 100, 0, 180));
}

//Globals
unsigned long lastMilli = 0; // time at the end of the last loop
byte mode = 'R';             // ROS Mode
byte ledstatus = LOW;
boolean moving_forward = false;

volatile unsigned long pulses = 0;     // rev counter
volatile unsigned long last_pulse = 0; // time of the last pulse
unsigned long old_pulses = 0;          // pulses since turning on
volatile bool moved = false;

void Blink(int times)
{
  while (times < 0)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    times -= 1;
  }
}

void CheckModeJumpers()
{
  pinMode(StatusModePin, INPUT);
  pinMode(BluetoothControlModePin, INPUT);
  pinMode(SerialControlModePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (digitalRead(StatusModePin) == HIGH)
  {                     // If the dev jumper is connected set dev_mode to true
    Serial.begin(9600); // For Serial Status Info
    Blink(1);
    mode = 'D';
  }
  else if (digitalRead(BluetoothControlModePin) == HIGH)
  {
    BT_Serial.begin(9600); // HC-06 default serial speed is 9600
    Blink(2);
    mode = 'B';
  }
  else if (digitalRead(SerialControlModePin) == HIGH)
  {
    Serial.begin(9600); // Arduino default serial speed is 9600
    Serial.println("Beginning Simple Serial Mode.");
    Blink(3);
    mode = 'S';
  }
  else
  {
    BT_Serial.begin(9600); // HC-06 default serial speed is 9600
    Blink(2);
    mode = 'B';
  }
}

void getMotorData(unsigned long time)
{
  // double delta_time = double(time) / 1000; // must be in seconds for these formulas
  // rpm = (60*(double(pulses - old_pulses)/ENCODER_RESOLUTION))/delta_time; // 60 is to convert from seconds to minutes
  // old_pulses = pulses
  // state_msg.drive.speed = 1;
}
void EncoderEvent()
{ // Counts pulses on the Encoder
  if (digitalRead(EncoderPin) == LOW)
  {
    pulses++;
    moved = true;
    last_pulse = millis();
  }
}

Servo Throttle;
Servo Steering;

void set_Steering(int _angle){
  Steering.write(_angle);
  current_steering_angle = _angle; 
}
  
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
  _speed = map(_speed, -100, 100, 55, 145);
  Throttle.write(_speed);
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
      Serial.print("Set Throttle to ");
      Serial.println(current_throttle_setting);
      break;
    case 'S': // Change Integral Gain
      setting = parseIntFast(4);
      set_Steering(setting);
      Serial.print("Set Steering to ");
      Serial.println(current_steering_angle);
      break;
    }
    Serial.flush();
  }
}

void ReadBluetoothCommands()
{ 
  if (BT_Serial.available() > 0)
  { // data received from smartphone
    delay(2);
    cmd[0] = BT_Serial.read();
    if (cmd[0] == STX)
    {
      int i = 1;
      while (BT_Serial.available() > 0)
      {
        delay(1);
        cmd[i] = BT_Serial.read();
        if (cmd[i] > 127 || i > 7)
          break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))
          break; // Button or Joystick data
        i++;
      }
      //if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      if (i == 7)
        getJoystickState(cmd); // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}

void setup()
{
  CheckModeJumpers();

  pinMode(SteeringControlPin, OUTPUT);
  pinMode(ThrottleControlPin, OUTPUT);
  pinMode(EncoderPin, INPUT);
  // digitalPinToInterrupt(RightEncoderPinA) == 0
  attachInterrupt(0, EncoderEvent, FALLING); // Trigger right_rpmcounter whenever hall sensor pulses

  Throttle.attach(ThrottleControlPin);
  Steering.attach(SteeringControlPin);

  Throttle.write(90);
  delay(2000);
  Throttle.write(0);
  delay(2000);
  Throttle.write(180);
  delay(2000);
  Throttle.write(90);
  delay(2000);
  mode = 'B';
}

void loop()
{
  unsigned long time = millis(); // time - lastMilli == time passed
  //OUPUTS
  if (time - lastMilli >= LoopTime)
  {
    getMotorData(time - lastMilli);
    switch (mode)
    {
    case 'D': // Send Status Info
      SendDebugInfo();
      break;
    }
    lastMilli = time;
  }
  // INPUTS
  switch (mode)
  {
  case 'B': // Handle Bluetooth
    ReadBluetoothCommands();
    break;
  case 'A':
    ReadSerialCommands();
    break;
  }
}
//////////////////////

void SendDebugInfo()
{ // TODO: Printing all this data causes a large delay in the speed updater
  Serial.print("THROTTLE SETTING: ");
  Serial.println(current_throttle_setting);
  Serial.print("STEERING SETTING: ");
  Serial.println(current_steering_angle);
  Serial.print("\n");
}
