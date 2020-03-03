//////////////////////
/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */
/////////////////////////////
#define EncoderPin           2 // Digital Interrupt Pin
/////////////////////////////
#define UsingMixedMode       false
#define ThrottleControlPin   6 // Also acts as X when in mixed mode and throttle when in RC Mode
#define SteeringControlPin   5 // Also acts as Y when in mixed mode and steering when in RC Mode
/////////////////////////////
#define CRAWL_SPEED          20  // %
#define MAX_SPEED            36 // %
/////////////////////////////
// Physical Properties of the robot
#define WHEEL_BASE           10 //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION   2.5 // right_pulses per revolution
#define WHEEL_DIAMETER       12 // cm
#define PI                   3.1415926535897932384626433832795
/////////////////////////////
#define LoopTime             100
/////////////////////////////
#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <Servo.h> 

void Blink(int times){
  while(times<0){
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    times-=1; 
  }
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int parseIntFast(int numberOfDigits){
  /*
  This function returns the converted integral number as an int value.
  If no valid conversion could be performed, it returns zero.*/
  char theNumberString[numberOfDigits + 1];
  int theNumber;
  for (int i = 0; i < numberOfDigits; theNumberString[i++] = Serial.read()){
    delay(5);
    };
  theNumberString[numberOfDigits] = 0x00;
  theNumber = atoi(theNumberString);
  return theNumber;
}

//Globals
unsigned long lastMilli = 0; // time at the end of the last loop 
byte mode = 'R'; // ROS Mode
byte ledstatus = LOW;
boolean moving_forward = false;

int rpm= 0;
volatile unsigned long pulses = 0; // rev counter
unsigned long old_pulses = 0;

ros::NodeHandle nh;

std_msgs::Int16 ticks_msg;
std_msgs::Int16 right_ticks_msg;

ros::Publisher pub_ticks("/ticks",&ticks_msg);
ros::Publisher pub_right_ticks("/right_ticks",&right_ticks_msg);

void DriverCallback(const geometry_msgs::Twist&);
ros::Subscriber<geometry_msgs::Twist> drive("cmd_vel", &DriverCallback); 

void getMotorData(unsigned long time) {
  double delta_time = double(time)/1000; // must be in seconds for these formulas 
  
  rpm = (60*(double(pulses - old_pulses)/ENCODER_RESOLUTION))/delta_time; // 60 is to convert from seconds to minutes
  ticks_msg.data += pulses - old_pulses;
  old_left_pulses = pulses;
}
void EncoderEvent() { // Counts right_pulses on the  Encoder
  if (digitalRead(EncoderPin) == LOW) {
    if (moving_forward){
      right_pulses++;
    } else {
      right_pulses--;
    }
  }
}
 Servo Throttle;
Servo Steering;
void Brake(){ // Disengages the brake on the ESC
  moving_forward = false; // Set first to prevent recurion 
  Throttle.write(60);
  delay(100);    
  Throttle.write(90); 
  delay(100);
}
void set_Throttle(int _speed) { // -100 :-: 100
  _speed = constrain(_speed, -MAX_SPEED, MAX_SPEED); // Speed Limit
  if ((_speed < 0) && moving_forward){
    Brake(); 
  }   
  if (_speed > 0){
    moving_forward = true;      
  }
  _speed = map(_speed,-100,100,55,145);
  Throttle.write(_speed);
}
void DriverCallback(const geometry_msgs::Twist& cmd_msg) {
  // Lin -.5:.5  Ang -1.5:1.5
  
    set_Throttle(mapf(cmd_msg.linear.x,-.5,.5,-100,100));
    Steering.write(mapf(cmd_msg.angular.z,-.5,.5,0,180));
}
void PublishTICKS(unsigned long time) {
  pub_ticks.publish(&ticks_msg);
  ticks_msg.data = 0;
}
void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600); // Ros Node uses 57600 by default
  nh.subscribe(drive);
  nh.advertise(pub_ticks);
  ticks_msg.data = 0;
  
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

void loop() {
  nh.spinOnce();
  unsigned long time = millis(); // time - lastMilli == time passed
  if(time - lastMilli >= LoopTime)   { // Enter Timed Loop 
    getMotorData(time - lastMilli);
    PublishTICKS(time - lastMilli);// Publish and Restart Loop 
    lastMilli = time;
  }
}
//https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/
//http://andrewjkramer.net/motor-encoders-arduino/
