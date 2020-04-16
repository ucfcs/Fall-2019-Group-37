/* 
 *  ROS subscriber for Twist messages
 */
 
#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

/*
 * Currently:
 * Arduino:
 *  purple, blue, green, yellow, white, grey
 *  13,     12,   11,    10,     9,     8
 * Motor controller:
 *  white, yellow, green, blue, purpple, grey
 *  (9,    10,     11,    12,   13,      8)
 */

//Motor Pins
const int EN_R = 8;      //Enable pin for right motor
const int IN1_R = 12;    //control pin for right motor
const int IN2_R = 13;    //control pin for right motor

const int IN1_L = 10;    //control pin for left motor
const int IN2_L = 11;    //control pin for left motor
const int EN_L = 9;      //Enable pin for left motor

//Bot dimensions
//TODO: Correct for final bot
const double wheel_rad = 0.0325, wheel_sep = 0.140;
const double max_rpm = 60;


const double max_wheel_vel = TWO_PI * wheel_rad * (max_rpm / 60.0);

//Wheel velocities
double wheel_r = 0.0, wheel_l = 0.0;

ros::NodeHandle  nh;

void stopAllMotors();

/*
 * Receive the Twist message and convert it into motor controller
 * commands.
 */
void messageCbTwist( const geometry_msgs::Twist& msg){
  double speed_ang = msg.angular.z;
  double speed_lin = msg.linear.x;
  
  
  if(speed_lin > max_wheel_vel)
    speed_lin = max_wheel_vel;
  
  wheel_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  wheel_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

void wheel_l_pulse(int pulse) {
  char msg[80]="LEFT wheel pulse: ";
  nh.logdebug(msg);
  char buf[7]="";
  dtostrf(pulse, 7, 0, buf);
  nh.logdebug(buf);
  strcat(msg, buf);
  nh.logdebug(msg);
  
  if (pulse > 0) {
    analogWrite(EN_L, pulse);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else if (pulse < 0) {
    pulse = abs(pulse);
    analogWrite(EN_L, pulse);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    wheel_l_stop();
  }
}

void wheel_r_pulse(int pulse) {
  
  if (pulse > 0) {
    analogWrite(EN_R, pulse);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  } else if (pulse < 0) {
    pulse = abs(pulse);
    analogWrite(EN_R, pulse);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else {
    wheel_r_stop();
  }
}



void wheel_l_stop() {
  digitalWrite(EN_L, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
}

void wheel_r_stop() {
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCbTwist);

void setup()
{   
  // Set up motor controller pins
  pinMode(EN_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);  
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);  
  pinMode(IN2_R, OUTPUT);
  pinMode(EN_R, OUTPUT);

  // Set up the NodeHandler for ROS
  nh.initNode();
  nh.subscribe(sub);

  nh.loginfo("Node handler initialized.");
  
  stopAllMotors();
}

/*
 * TODO: Add code to stop the bot regardless of message if the ultrasonic
 * sensors detect an obstacle.
 */
void loop()
{
  while(!nh.connected()) {  
    nh.spinOnce();
  }
  
  wheel_l_pulse(wheel_l*10);
  wheel_r_pulse(wheel_r*10);
  
  delay(1);
  nh.spinOnce();

}

void stopAllMotors() {
  wheel_l_stop();
  wheel_r_stop();
}
