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
#define STILL_ALLOWANCE 10 // Loop I
#define REVERSE_ALLOWANCE 10
/////////////////////////////
// Physical Properties of the robot
#define WHEEL_BASE 10        //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION 9 // pulses per revolution
#define WHEEL_DIAMETER .12   // m
#define PI 3.1415926535897932384626433832795
/////////////////////////////
//SimpleKalmanFilter simpleKalmanFilter(.25, .25, 0.001);
/////////////////////////////
#define LoopTime 100 // 10 Hz
///////////////
#define    STX          0x02
#define    ETX          0x03   
//////////////
#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//#include <SimpleKalmanFilter.h>
#include <Servo.h>

#include <SoftwareSerial.h>
SoftwareSerial BT_Serial(11,10); // RX | TX

volatile float velocity_estimate = 0;
//float goal_velocity = 0;
//float velocityError = 0;
//float lastVelocityError = 0;
float current_throttle_setting = 0;

ros::NodeHandle nh;

std_msgs::Int16 ticks_msg;
ackermann_msgs::AckermannDriveStamped state_msg;

/////// Bluetooth stuff I aint write this LOL
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

void getJoystickState(byte data[8])
{
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200; // Offset to avoid
  joyY = joyY - 200; // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)
    return; // commmunication error

  set_Throttle(joyY);
  set_Steering(map(-joyX, -100, 100, -90, 90));
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

ros::Publisher pub_state("/rc_state", &state_msg);

void DriverCallback(const ackermann_msgs::AckermannDriveStamped &);
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> drive("ackermann_cmd", &DriverCallback);

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
    Serial.begin(9600); // HC-06 default serial speed is 9600
    Serial.println("Beginning Simple Serial Mode.");
    Blink(3);
    mode = 'S';
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    nh.initNode();
    nh.getHardware()->setBaud(57600); // Ros Node uses 57600 by default
    nh.subscribe(drive);
    nh.advertise(pub_state);
    mode = 'R';
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
{ // Counts pulses on the  Encoder
  if (digitalRead(EncoderPin) == LOW)
  {
    moved = true;
    last_pulse = millis();
  }
}
void set_Steering(int _speed){}
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
  _speed = map(_speed, -100, 100, 55, 145);
  Throttle.write(_speed);
}

void DriverCallback(const ackermann_msgs::AckermannDriveStamped &cmd_msg)
{
  // Lin -.5:.5  Ang -1.5:1.5
  set_Throttle(cmd_msg.drive.speed * 200);
  Steering.write(mapf(cmd_msg.drive.steering_angle, -.5, .5, 0, 180));
  state_msg.drive.steering_angle = cmd_msg.drive.steering_angle;
}

void PublishState(unsigned long time)
{
  state_msg.header.stamp = nh.now();
  //state_msg.drive.speed = simpleKalmanFilter.updateEstimate(velocity_estimate);
  if (moved == false)
  {
    state_msg.drive.speed = 0;
  }
  pub_state.publish(&state_msg);
  moved = false;
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
     // Serial.println(current_steering_angle);
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
}

void loop()
{
  unsigned long time = millis(); // time - lastMilli == time passed
  //OUPUTS
  if (time - lastMilli >= LoopTime)
  { // Enter Timed Loop
    getMotorData(time - lastMilli);
    switch (mode)
    {
    case 'D': // Send Status Info
      SendDebugInfo();
      break;
    case 'R': // ROS Mode
      //PublishTransform();
      PublishState(time - lastMilli); // Publish and Restart Loop
      break; // Publish and Restart Loop
    }
    lastMilli = time;
  }
  //INPUTS
  switch (mode)
  {
  case 'R':
    nh.spinOnce();
    break;
  case 'B': // Handle Bluetooth
    ReadBluetoothCommands();
    break;
  case 'A':
    ReadSerialCommands();
    break;
  }
}
//https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/
//http://andrewjkramer.net/motor-encoders-arduino/
//////////////////////
/*
void getMotorData(unsigned long time)
{
  double delta_time = double(time) / 1000;                                     // must be in seconds for these formulas
  rpm = (60 * (double(pulses - oldpulses) / ENCODER_RESOLUTION)) / delta_time; // 60 is to convert from seconds to minutes
  oldpulses = pulses;
  // store your measured speed setting in this variable
  actual_throttle = map(rpm, 0, PERCENT_100_RPM, 0, 100); // 0 :-: 100
  actual_throttle = actual_throttle * (rpm / abs(rpm));   // -100 :-: 100
  //basic velocity inputs
  linear_velocity = (rpm * (WHEEL_DIAMETER / 100.0) * PI) / delta_time; // m/s from rear wheels
  //current_steering_angle;                                               // must be in radians ... I think
  //first get the theta update
  //angular_velocity = linear_velocity * (tan(current_steering_angle) / WHEEL_BASE);
}*/
/*
void PublishTransform()
{
  geometry_msgs::TransformStamped t;

  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
  t.transform.translation.x = current_x;
  t.transform.translation.y = current_y;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = sin(current_yaw / 2.0);
  t.transform.rotation.w = cos(current_yaw / 2.0);

  t.header.stamp = current_time;

  broadcaster.sendTransform(t);
}*/

void SendDebugInfo()
{ // TODO: Printing all this data causes a large delay in the speed updater
  Serial.print("THROTTLE SETTING: ");
  Serial.println(current_throttle_setting);
  Serial.print("STEERING SETTING: ");
  //Serial.println(current_steering_angle);
  Serial.print("RPM: ");
 // Serial.println(rpm);
  /*
  Serial.print("LINEAR VELOCITY: ");
  Serial.print(linear_velocity);
  Serial.println(" m/s");
  Serial.print("ANGULAR VELOCITY: ");
  Serial.print(angular_velocity);
  Serial.println(" m/s");  
  Serial.print("PULSES SINCE LAST UPDATE: ");
  Serial.println(pulses);*/
  Serial.print("TOTAL PULSES: ");
//  Serial.println(abs(oldpulses));
  Serial.print("\n");
}
