// Cooper Ducharme(12/4/2024) 

// Caleb Williams (02/07/2020)
// Actuates the Actuonix PQ12-100-6-R linear actuator
// A 2ms pulse is used to retract the arm while a 1ms pulse extends the arm
// The actuator takes ~2s to extend/retract when there is no load.  When loaded,
// the travel time increases.  Ensure the delay between when the arm extends
// and retracts is appropriate to avoid damaging the component.
// This model features a three wire output:
// Connect black to GND
// Connect red to VCC (5-6V)
// Connect white to an Arduino PWM pin (here I use pin 9)

#include <Servo.h>
#include <ros.h>
#include <stf_msgs/Bool.h>

ros::NodeHandle nh;


Servo actuator; // create a servo object named "actuator"

#define PIN 9

void pinControlCallback(const std_msgs::Bool &msg){
    digitalWrite(PIN, msg.data ? HIGH : LOW);
}

ros::Subscriber<std_msgs::Bool> pin_control_sub("fin_ray_gripper_publisher", &pinControlCallback);

void setup() {
  pinMode(PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(pin_control_sub);
    //old:
  actuator.attach(9); // attach the actuator to Arduino pin 9 (PWM)
  actuator.writeMicroseconds(1000); // give the actuator a 2ms pulse to retract the arm (1000us = 1ms)
  delay(10); // delay a few seconds to give the arm time to retract
}

void loop() {
  nh.spinOnce();
  delay(10);//momentary pause
  //old
  // Extend and retract the actuator arm on a 5 second interval
  msg_data=msg.data
  print(msg_data)
  if(msg_data){
    actuator.writeMicroseconds(1000); // 1ms pulse to extend the arm
    delay(1); // the actuator takes >2s to extend/retract when loaded - give it plenty of time
  }
  else{
    actuator.writeMicroseconds(2000); // 2ms pulse to retract the arm
    delay(1);
  }
}

//void open_gripper(){
//actuator.writeMicroseconds(1000); // 1ms pulse to extend the arm
//  delay(3000); // the actuator takes >2s to extend/retract when loaded - give it plenty of time
//}

//void close_gripper(){
//actuator.writeMicroseconds(2000); // 2ms pulse to retract the arm
//  delay(3000);
//}