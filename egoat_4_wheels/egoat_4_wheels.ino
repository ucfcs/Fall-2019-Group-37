/* 
 *  ROS subscriber for motor speed commands
 *  
 *  To test (typically 3 terminals):
 *  
 *  (1) roscore
 *  
 *  (2) rosrun rosserial_python serial_node.py /dev/ttyACM0
 *
 *  (3) rostopic pub motors egoat/SetMotorSpeed '{left_motor: 100, right_motor: 100}' --once
 *  
 *  Replacing /dev/ttyACM0 with the serial port of the Arduino device
 *  and using the desired values for the left and right motor speeds
 */

#include <ros.h>
#include "egoat/SetMotorSpeed.h"

//Left Motor Pins
const int LEFT_EN_A = 52;
const int LEFT_IN1 = 50;
const int LEFT_IN2 = 48;
const int LEFT_IN3 = 46;
const int LEFT_IN4 = 44;
const int LEFT_EN_B = 42;

//Right Motor Pins
const int RIGHT_EN_A = 53;
const int RIGHT_IN1 = 51;
const int RIGHT_IN2 = 49;
const int RIGHT_IN3 = 47;
const int RIGHT_IN4 = 45;
const int RIGHT_EN_B = 43;

//Ultrasonic Sensor Pins
const int FRONT_TRIG_PIN = 1;
const int FRONT_ECHO_PIN = 2;
const int BACK_TRIG_PIN = 3;
const int BACK_ECHO_PIN = 4;

//Variables for calculating distance from ultrasonic sensors
long duration;
int frontDistance;
int backDistance;
int nearDistance = 10; //Distance to compare to for ultrasonic sensors


ros::NodeHandle  nh;

/*
 * Handle the message received.
 */
void messageCb(const egoat::SetMotorSpeed& msg){
  int left = msg.left_motor;
  int right = msg.right_motor;

  // Convert message to values the motor controller understands
  writeMotorSpeedAndDirection(left, LEFT_EN_A, LEFT_IN1, LEFT_IN2);
  writeMotorSpeedAndDirection(left, LEFT_EN_B, LEFT_IN3, LEFT_IN4);

  writeMotorSpeedAndDirection(right, RIGHT_EN_A, RIGHT_IN1, RIGHT_IN2);
  writeMotorSpeedAndDirection(right, RIGHT_EN_B, RIGHT_IN3, RIGHT_IN4);

  // Use NodeHandle to output debug info to rostopic
  char buf[5]; // loginfo is picky and wants a char*, so we have to use itoa
  nh.loginfo("Received: ");
  nh.loginfo(itoa(left, buf, 10));
  nh.loginfo(itoa(right,buf, 10));
}

/*
 * Convert the message value
 * 
 * Currently, the ROS message SetMotorSpeed receives 2 int8, which means
 * the range for each motor is [-128, 127].
 * 
 * The intention is to set the motor speed as a percentage (in int)
 * of maximum motor speed. This will have to be mapped to
 * [0, 255] for the L298N motor controller on the prototype's
 * speed, and the sign indicates the direction
 * 
 * TODO: Check the range of values for the final version's
 * motor controllers.
 */
void writeMotorSpeedAndDirection(int messageValue, int enablePin, int input1, int input2) {
  int powerOut = map(abs(messageValue), 0, 100, 0, 255);
  analogWrite(enablePin, powerOut);
  char buf[10];
    nh.loginfo("analogWrite:");
    nh.loginfo("enablePin:");
    nh.loginfo(itoa(enablePin, buf, 10));
    nh.loginfo("powerOut:");
    nh.loginfo(itoa(powerOut, buf, 10));
  if(messageValue > 0) {
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    nh.loginfo("digitalWrite HIGH LOW");
  } else if(messageValue < 0) {
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
    nh.loginfo("digitalWrite LOW HIGH");
  } else {
    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);
    
    nh.loginfo("digitalWrite LOW LOW");
  }
}

bool checkDistanceFrontAndBack() {
  //Clear trigger pins
  digitalWrite(FRONT_TRIGGER_PIN, LOW);
  digitalWrite(BACK_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  
  //Set trigger pins on HIGH for 10ms
  digitalWrite(FRONT_TRIGGER_PIN, HIGH);
  digitalWrite(BACK_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);

  //Return trigger pins to LOW
  digitalWrite(FRONT_TRIGGER_PIN, LOW);
  digitalWrite(BACK_TRIGGER_PIN, LOW);

  //Read echo pins, returns sound wave travel time
  duration = pulseIn(FRONT_ECHO_PIN, HIGH);
  frontDistance = duration * 0.034 / 2;
  duration = pulseIn(BACK_ECHO_PIN, HIGH);
  backDistance = duration * 0.034 / 2;

  //Check front and back distances.
  if(backDistance <= nearDistance || frontDistance <= nearDistance)
    return true;
  return false;
}

void stopRoverMessage(bool stop)
{
  if(stop)
  {
    stopRover();
    //TODO publish stop rover message with true
    return;
  }
  //TODO publish stop rover message with false
    
}

//TODO stop rover
void stopRover()
{
  analogWrite(LEFT_EN_A, 0);
  analogWrite(LEFT_EN_B, 0);
  analogWrite(RIGHT_EN_A, 0);
  analogWrite(RIGHT_EN_B, 0);
}

ros::Subscriber<egoat::SetMotorSpeed> sub("motors", &messageCb);
ros::Publisher stop_pub = nh.advertise<egoat::StopRover>("stop", 100);

void setup()
{   
  // Set up left motor controller pins
  pinMode(LEFT_EN_A, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);  
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);  
  pinMode(LEFT_IN4, OUTPUT);
  pinMode(LEFT_EN_B, OUTPUT);

  //Set up right motor controller pins
  pinMode(RIGHT_EN_A, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);  
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);  
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_EN_B, OUTPUT);

  //Setup front and back ultrasonic sensor pins
  pinMode(FRONT_TRIGGER_PIN, OUTPUT); 
  pinMode(FRONT_ECHO_PIN, INPUT); 
  pinMode(BACK_TRIGGER_PIN, OUTPUT); 
  pinMode(BACK_ECHO_PIN, INPUT); 

  // Set up the NodeHandler for ROS
  nh.initNode();
  nh.subscribe(sub);
}

/*
 * TODO: Add code to stop the bot regardless of message if the ultrasonic
 * sensors detect an obstacle.
 */
void loop()
{ 
  stopRoverMessage(checkDistanceFrontAndBack);
  
  nh.spinOnce();
  delay(1);
}
