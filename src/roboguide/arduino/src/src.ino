//////////////////////
/* 
  _      _      _
>(.)__ <(.)__ =(.)__
 (___/  (___/  (___/ */
/////////////////////////////
#define EncoderPin 2 // Digital Interrupt Pin
/////////////////////////////
#define ThrottleControlPin 6 // Also acts as X when in mixed mode and throttle when in RC Mode
#define SteeringControlPin 5 // Also acts as Y when in mixed mode and steering when in RC Mode
/////////////////////////////
#define CRAWL_SPEED 20 // %
#define MAX_SPEED 36   // %
#define STILL_ALLOWANCE 10 // Loop I
#define REVERSE_ALLOWANCE 10
/////////////////////////////
// Physical Properties of the robot
#define WHEEL_BASE 10        //inches // I forgot Y I did this in inches
#define ENCODER_RESOLUTION 9 // pulses per revolution
#define WHEEL_DIAMETER .12    // m
#define PI 3.1415926535897932384626433832795
/////////////////////////////
float P_GAIN = .002;
float I_GAIN = 0.1;
float D_GAIN = 6;
SimpleKalmanFilter simpleKalmanFilter(.25, .25, 0.001);
/////////////////////////////
#define LoopTime 100 // 10 Hz
/////////////////////////////
#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>

#include <SimpleKalmanFilter.h>
#include <Servo.h>

bool Clamped = false;
float IntegralTerm = 0;
float DerivativeTerm = 0;
float PID_Output = 0;
volatile float velocity_estimate = 0;
float goal_velocity = 0;
float velocityError = 0;
float lastVelocityError = 0;
float current_throttle_setting = 0;

ros::NodeHandle nh;

std_msgs::Int16 ticks_msg;
ackermann_msgs::AckermannDriveStamped state_msg;

// Bound the input value between x_min and x_max. Also works in anti-windup
float CheckClamp(float x)
{
  float speed = constrain(x, -MAX_SPEED, MAX_SPEED); // Speed Limit
  Clamped = not(speed == x);
  return speed;
}

void UpdatePIDController()
{
  // compute the error between the measurement and the desired value
  velocityError = goal_velocity - velocity_estimate;
  DerivativeTerm = velocityError - lastVelocityError;

  // If the actuator is saturating ignore the integral term
  // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
  if (Clamped and ((PID_Output / abs(PID_Output)) == (IntegralTerm / abs(IntegralTerm))))
  {
    IntegralTerm += 0;
  }
  else
  {
    IntegralTerm += velocityError;
  }
  // compute the control effort by multiplying the error by Kp
  PID_Output = (velocityError * P_GAIN) + (IntegralTerm * I_GAIN) + (DerivativeTerm * D_GAIN);
  current_throttle_setting = PID_Output;
  state_msg.drive.acceleration = PID_Output;
  // make sure the output value is bounded to 0 to 100 using the bound function defined below
  current_throttle_setting = CheckClamp(current_throttle_setting);
  set_Throttle(current_throttle_setting); // then write it to the LED pin to change control voltage to LED
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Globals
unsigned long lastMilli = 0; // time at the end of the last loop
byte mode = 'R';             // ROS Mode
byte ledstatus = LOW;
boolean moving_forward = false;

int rpm = 0;
volatile unsigned long pulses = 0; // rev counter
volatile unsigned long last_pulse = 0; // time of the last pulse 
unsigned long old_pulses = 0; // pulses since turning on 
volatile bool moved = false;


ros::Publisher pub_state("/rc_state", &state_msg);

void DriverCallback(const ackermann_msgs::AckermannDriveStamped &);
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> drive("ackermann_cmd", &DriverCallback);

void getMotorData(unsigned long time)
{
  // double delta_time = double(time) / 1000; // must be in seconds for these formulas
  // rpm = (60*(double(pulses - old_pulses)/ENCODER_RESOLUTION))/delta_time; // 60 is to convert from seconds to minutes
  // old_pulses = pulses;
  // state_msg.drive.speed = 1;
}
void EncoderEvent()
{ // Counts pulses on the  Encoder
  if (digitalRead(EncoderPin) == LOW)
  {
    if (last_pulse != 0){
      
    velocity_estimate = ((WHEEL_DIAMETER * PI) / ENCODER_RESOLUTION) / (((millis() - last_pulse))/1000.f);
    velocity_estimate *= moving_forward ? 1 : 1;
    simpleKalmanFilter.updateEstimate(velocity_estimate);
  }
    moved = true;
    last_pulse = millis();
  }
}
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
  goal_velocity = cmd_msg.drive.speed * 5;
  state_msg.drive.jerk = goal_velocity;
  //set_Throttle_Goal();
  Steering.write(mapf(cmd_msg.drive.steering_angle, -.5, .5, 0, 180));
  state_msg.drive.steering_angle = cmd_msg.drive.steering_angle;
}
void PublishState(unsigned long time)
{
  state_msg.header.stamp = nh.now();
  state_msg.drive.speed = simpleKalmanFilter.updateEstimate(velocity_estimate);;
  if (moved == false){state_msg.drive.speed = 0;}
  pub_state.publish(&state_msg);
  moved = false;
}

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600); // Ros Node uses 57600 by default
  nh.subscribe(drive);
  nh.advertise(pub_state);

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
  //while(true){}
}

void loop()
{
  nh.spinOnce();
  unsigned long time = millis(); // time - lastMilli == time passed
  if (time - lastMilli >= LoopTime)
  { // Enter Timed Loop
    UpdatePIDController();
    PublishState(time - lastMilli); // Publish and Restart Loop
    lastMilli = time;
  }
}
//https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/
//http://andrewjkramer.net/motor-encoders-arduino/
