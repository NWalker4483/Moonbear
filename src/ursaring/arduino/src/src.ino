//////////////////////
/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */
/////////////////////////////
#define RightEncoderPinA     2 // Digital Interrupt Pin
#define RightEncoderPinB     4 // Digital Pin
#define LeftEncoderPinA      3 // Digital Interrupt Pin
#define LeftEncoderPinB      5// Digital Pin
#define StatusModePin        8 // Input Pin acting as a jumper
#define SimpleSerialModePin  11 // Input Pin acting as a jumper
/////////////////////////////
#define RightTreadControlPin 14
#define LeftTreadControlPin  0
// For Motor drivers with mixed mode 
#define XPin                 0
#define YPin                 0
/////////////////////////////
#define CRAWL_SPEED          20  // %
#define MAX_SPEED            30 // %
/////////////////////////////
// Physical Properties of the robot
#define WHEEL_BASE           10 //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION   2.5 // right_pulses per revolution
#define WHEEL_DIAMETER       12 // cm
#define PI                   3.1415926535897932384626433832795
/////////////////////////////
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

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

int current_throttle_setting = 0;
int actual_throttle = 0;
int goal_throttle = 0;
int current_steering_angle = 0;

int right_rpm = 0;
volatile unsigned long right_pulses = 0; // rev counter
unsigned long old_right_pulses = 0;

int left_rpm = 0;
volatile unsigned long left_pulses = 0; // rev counter
unsigned long old_left_pulses = 0;

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
  if (digitalRead(RightEncoderPinA) == HIGH) {
    if (digitalRead(RightEncoderPinB) == LOW) {
      right_pulses++;
    } else {
      right_pulses--;
    }
  } else {
    if (digitalRead(RightEncoderPinB) == LOW) {
      right_pulses--;
    } else {
      right_pulses++;
    }
  }
}

void LeftMotorEncoder() { // Counts right_pulses on the  Encoder
 if (digitalRead(LeftEncoderPinA) == HIGH) {
    if (digitalRead(LeftEncoderPinB) == LOW) {
      left_pulses++;
    } else {
      left_pulses--;
    }
  } else {
    if (digitalRead(LeftEncoderPinB) == LOW) {
      left_pulses--;
    } else {
      left_pulses++;
    }
  }
}


void set_RightTread(int speed){
}
void set_LeftTread(int speed){
}

void DriverCallback(const geometry_msgs::Twist& cmd_msg) {
  // Lin -.5:.5  Ang -1.5:1.5
  double linear = cmd_msg.drive.speed;
  double steering_radians = cmd_msg.drive.steering_angle;

  int speed = map(linear,-.5,.5,-100,100); 
  int steering_angle = steering_radians * (180/PI);

  set_Steering(steering_angle);
  set_Throttle(speed); 
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
      case 'T':
        setting = parseIntFast(4);
        set_Throttle(setting); 
        Serial.print("Set Throttle to ");
        Serial.println(current_throttle_setting); 
        break;
      case 'S':
        setting = parseIntFast(4);
        set_Steering(setting); 
        Serial.print("Set Steering to ");
        Serial.println(current_steering_angle); 
        break;
    }
  Serial.flush();
  }
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
  pinMode(SimpleSerialModePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (digitalRead(StatusModePin) == HIGH){ // If the dev jumper is connected set dev_mode to true
    Serial.begin(9600);  // For Serial Status Info
    Blink(1);
    mode = 'S';
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

  pinMode(RightEncoderPinA, INPUT); 
  pinMode(RightEncoderPinB, INPUT); 
  attachInterrupt(digitalPinToInterrupt(RightEncoderPinA), RightMotorEncoder, CHANGE); // Trigger RPM counter whenever hall sensor right_pulses
  
  pinMode(LeftEncoderPinA, INPUT); 
  pinMode(LeftEncoderPinB, INPUT); 
  attachInterrupt(digitalPinToInterrupt(LeftEncoderPinA),  LeftMotorEncoder, CHANGE); // Trigger RPM counter whenever hall sensor right_pulses
}

void loop() {
  if(mode=='R'){nh.spinOnce();}
  unsigned long time = millis(); // time - lastMilli == time passed
  //OUPUTS
  if(time - lastMilli >= LoopTime)   { // Enter Timed Loop 
    getMotorData(time - lastMilli);
    switch (mode) {
      case 'S': // Send Status Info
        SendStatusInfo(); break;
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
///http://andrewjkramer.net/motor-encoders-arduino/
