//////////////////////
/* 
~~~~~~~~~~~~~~~~~~~~~/\{ } 
~~~~~~~  __________,' |\---
~~~~~~,/                   \
~~~~~,'      
~~~~~|   '\      
~~~~~|    |~~~~~~|   |""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
//////////////////////
#define RightEncoderPin      2 // Interrupt Pin
#define LeftEncoderPin      2 // Interrupt Pin
#define StatusModePin   8 // Input Pin acting as the dev jumper
#define PidPlotModePin  9 // Input Pin acting as the dev jumper
#define SimpleSerialModePin 11 // Input Pin acting as the dev jumper
/////////////////////////////////////////////////
#define LoopTime        100   // PID loop time(ms)
////////////////////////
float P_GAIN=           0.7;
float I_GAIN=           0.3;
float D_GAIN=           0.4;
/////////////////////////
#define ThrottlePin     14
#define SteeringPin     0
/////////////////////////
#define CRAWL_SPEED     20 // %
#define MAX_SPEED       30 // %
/////////////////////
#define PERCENT_100_RPM 25600 // Top Possible RPM of the motor for PID
#define MIN_PULSE_WIDTH 410 // 410 820 // 10% to 20% Duty Cycle
#define MAX_PULSE_WIDTH 820
#define FREQUENCY 106 // Hz
//////////////////////
#define    STX          0x02
#define    ETX          0x03   

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>

// Physical Properties of the robot
#define WHEEL_BASE 10 //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION 2.5 // right_pulses per revolution
#define WHEEL_DIAMETER     12 // cm
#define PI 3.1415926535897932384626433832795

//Globals
unsigned long lastMilli = 0; // time at the end of the last loop 
byte mode = 'R'; // ROS Mode
byte ledstatus = LOW;

double current_x = 0;
double current_y = 0;
double current_yaw = 0;
bool publish_tf = false;

double delta_x = 0.0;
double delta_y = 0.0;
double linear_velocity = 0.0;
double angular_velocity = 0.0;
bool right_moving_forward = false;
bool left_moving_forward = false;

int current_throttle_setting = 0;
int actual_throttle = 0;
int goal_throttle = 0;
int current_steering_angle = 0;

int rpm = 0;
volatile long right_pulses = 0; // rev counter
long oldright_pulses = 0;

bool Clamped = false;
int IntegralTerm = 0;
int DerivativeTerm = 0;
int PID_Output = 0; 
int throttleError = 0;
int lastThrottleError = 0;

////
void DriverCallback(const ackermann_msgs::AckermannDriveStamped&);
////
ros::NodeHandle nh;
ros::Time current_time;
nav_msgs::Odometry odom_msg;

ros::Subscriber<geometry_msgs::Twist> drive("cmd_vel", &DriverCallback); 
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

void getMotorData(unsigned long time) {
  double delta_time = double(time)/1000; // must be in seconds for these formulas 
  rpm = (60*(double(right_pulses-oldright_pulses)/ENCODER_RESOLUTION))/delta_time; // 60 is to convert from seconds to minutes
  oldright_pulses = right_pulses;
  // store your measured speed setting in this variable
  actual_throttle = map(rpm,0,PERCENT_100_RPM,0,100); // 0 :-: 100
  actual_throttle = actual_throttle * (rpm/abs(rpm)); // -100 :-: 100
  //basic velocity inputs
  linear_velocity = (rpm * (WHEEL_DIAMETER/100.0)*PI)/delta_time; // m/s from rear wheels
  current_steering_angle; // must be in radians ... I think 
  //first get the theta update
  angular_velocity = linear_velocity*(tan(current_steering_angle)/WHEEL_BASE);
}

void RightMotorEncoder() { // Counts right_pulses on the  Encoder
  if (right_moving_forward) {
    right_pulses++;
  } else {
    right_pulses--;
  }
}

void LeftMotorEncoder() { // Counts right_pulses on the  Encoder
  if (left_moving_forward) {
    left_pulses++;
  } else {
    left_pulses--;
  }
}

void set_Throttle_Goal(int speed) { // -100 :-: 100
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED); // Speed Limit
  goal_throttle = speed;
}

void set_Throttle(int speed) { // -100 :-: 100
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED); // Speed Limit
  if ((speed < 0) && moving_forward){
    Brake(); 
  }   
  if (speed > 0){
    moving_forward = true;      
  }
  current_throttle_setting = speed;
  speed = map(speed,-100,100,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
  pwm.setPWM(ThrottlePin, 0, speed); 
}

void Brake(){ // Disengages the brake on the ESC
  moving_forward = false; // Set first to prevent recurion 
  set_Throttle(-20);
  delay(100);    
  set_Throttle(0); 
  delay(100);
}

void set_Steering(int angle) {
  angle = constrain(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE); // Limit angle for safety
  current_steering_angle = angle;
  angle = map(angle, -90, 90, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pwm.setPWM(SteeringPin, 0, angle);
}

void DriverCallback(const ackermann_msgs::AckermannDriveStamped& cmd_msg) {
  // Lin -.5:.5  Ang -1.5:1.5
  double linear = cmd_msg.drive.speed;
  double steering_radians = cmd_msg.drive.steering_angle;

  int speed = map(linear,-.5,.5,-100,100); 
  int steering_angle = steering_radians * (180/PI);

  set_Steering(steering_angle);
  set_Throttle(speed); 
}

// Bound the input value between x_min and x_max. Also works in anti-windup 
int CheckClamp(int x) {
  int speed = constrain(x, -MAX_SPEED, MAX_SPEED); // Speed Limit
  Clamped = not (speed == x);
  return speed;
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

void ReadSerialCommands(){ // Plot PID Values using Serial Plotter
  if (Serial.available()){
    char cmdByte = Serial.read();
    int setting;
    switch (cmdByte) {
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

void UpdatePIDController(){
  // compute the error between the measurement and the desired value
  throttleError = goal_throttle - actual_throttle;
  DerivativeTerm = throttleError - lastThrottleError;
  
  // If the actuator is saturating ignore the integral term
  // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
  if (Clamped and ((PID_Output/abs(PID_Output))==(IntegralTerm/abs(IntegralTerm)))){ 
    IntegralTerm += 0;
  } else {
    IntegralTerm += throttleError;
  }
  // compute the control effort by multiplying the error by Kp
  PID_Output = (throttleError * P_GAIN) + (IntegralTerm * I_GAIN) + (DerivativeTerm * D_GAIN);
  current_throttle_setting += PID_Output; 

  // make sure the output value is bounded to 0 to 100 using the bound function defined below
  current_throttle_setting = CheckClamp(current_throttle_setting);
  set_Throttle(current_throttle_setting); // then write it to the LED pin to change control voltage to LED
}

void PublishODOM(unsigned long time) {
  double delta_time = double(time)/1000; // must be in seconds
  //compute odometry update values
  delta_x = linear_velocity*cos(current_yaw);
  delta_y = linear_velocity*sin(current_yaw);
  //Update the pose estimate
  current_x += delta_x * delta_time;
  current_y += delta_y * delta_time;
  current_yaw += angular_velocity * delta_time;
  if (publish_tf){PublishTransform();}

  odom_msg.header.stamp = current_time; // nh.now();
  odom_msg.header.frame_id = "/odom";
  odom_msg.child_frame_id = "/base_link";
  odom_msg.pose.pose.position.x = current_x;
  odom_msg.pose.pose.position.y = current_y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(current_yaw/2.0);
  odom_msg.pose.pose.orientation.w = cos(current_yaw/2.0);
  
  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_velocity;

  // Position uncertainty
  odom_msg.pose.covariance[0]  = 0.2; // x
  odom_msg.pose.covariance[7]  = 0.2; // y
  odom_msg.pose.covariance[35] = 0.4; // yaw

  odom_pub.publish(&odom_msg);
  nh.spinOnce();
}

void PublishTransform(){
  geometry_msgs::TransformStamped t;
  
  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
  t.transform.translation.x = current_x;
  t.transform.translation.y = current_y;
  t.transform.translation.z = 0.0;
  
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = sin(current_yaw/2.0);
  t.transform.rotation.w = cos(current_yaw/2.0);
  
  t.header.stamp = current_time;

  broadcaster.sendTransform(t);
}

void SendPlotInfo(){ // Plot PID Values using Serial Plotter
  if (Serial.available()){
    byte inByte = Serial.read();
    switch (inByte) {
      case 'P': // Change Proportianal Gain
        P_GAIN = Serial.parseFloat(); break;
      case 'I': // Change Integral Gain
        I_GAIN = Serial.parseFloat(); break;
      case 'D': // Change Derivative Gain
        D_GAIN = Serial.parseFloat(); break;
      default:
        break;
    }
  }
  Serial.print(actual_throttle);
  Serial.print('\t');
  // plot the desired output
  Serial.print(throttleError);
  Serial.print('\t');
  // plot the error
  Serial.println(goal_throttle);
}

void SendStatusInfo() {// TODO: Printing all this data causes a large delay in the speed updater 
  Serial.print("THROTTLE SETTING: ");
  Serial.println(current_throttle_setting);
  Serial.print("STEERING SETTING: ");
  Serial.println(current_steering_angle);
  Serial.print("RPM: ");
  Serial.println(rpm);
  /*
  Serial.print("LINEAR VELOCITY: ");
  Serial.print(linear_velocity);
  Serial.println(" m/s");
  Serial.print("ANGULAR VELOCITY: ");
  Serial.print(angular_velocity);
  Serial.println(" m/s");  
  Serial.print("right_pulses SINCE LAST UPDATE: ");
  Serial.println(right_pulses);*/
  Serial.print("TOTAL right_pulses: ");
  Serial.println(abs(oldright_pulses));  
  Serial.print("\n");  
}

void Blink(int times){
  while(times<0){
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    times-=1; 
  }
}
  
void CheckModeJumpers(){
  pinMode(StatusModePin, INPUT);
  pinMode(PidPlotModePin, INPUT);
  pinMode(SimpleSerialModePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (digitalRead(StatusModePin) == HIGH){ // If the dev jumper is connected set dev_mode to true
    Serial.begin(9600);  // For Serial Status Info
    Blink(1);
    mode = 'S';
  } else if (digitalRead(PidPlotModePin) == HIGH){
    Serial.begin(9600);  // For Serial Status Info
    Blink(2);
    mode = 'P';
  } else if (digitalRead(SimpleSerialModePin) == HIGH){
    Serial.begin(9600); 
    Serial.println("Beginning Simple Serial Mode.");
    Blink(4);
    mode = 'A'; 
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    nh.initNode();
    nh.getHardware()->setBaud(57600); // Ros Node uses 57600 by default
    nh.subscribe(drive);
    nh.advertise(odom_pub);
    mode = 'R';
  }
}

void setup() {
  CheckModeJumpers();

  pinMode(RightEncoderPin, INPUT); 
  //digitalPinToInterrupt(EncoderPin)
  attachInterrupt(0, RightMotorEncoder, RISING); // Trigger RPM counter whenever hall sensor right_pulses
  
  pinMode(LeftEncoderPin, INPUT); 
  //digitalPinToInterrupt(EncoderPin)
  attachInterrupt(0, LeftMotorEncoder, RISING); // Trigger RPM counter whenever hall sensor right_pulses
}

void loop() {
  if(mode=='R'){nh.spinOnce();}
  unsigned long time = millis(); // time - lastMilli == time passed
  //OUPUTS
  if(time - lastMilli >= LoopTime)   { // Enter Timed Loop 
    getMotorData(time - lastMilli); //
    // UpdatePIDController();
    switch (mode) {
      case 'S': // Send Status Info
        SendStatusInfo(); break;
      case 'P': // Plot/Tune  PID Info
        SendPlotInfo(); break;
      case 'R': // ROS Mode
        PublishODOM(time - lastMilli); break; // Publish and Restart Loop 
    }
    lastMilli = time;
  }  //INPUTS 
  switch (mode) {
    case 'A':
      ReadSerialCommands(); break;
  }
}
