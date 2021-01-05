#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define USE_USBCON

const int buttonPin = 2;

int buttonState = LOW;

ros::NodeHandle nh;
std_msgs::Bool bumper_state;

ros::Publisher pub_bumper("bumper_state", &bumper_state);

void setup() {
  nh.initNode();
  nh.advertise(pub_bumper);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    bumper_state.data = true;
  }
  else if (buttonState == HIGH) {
    bumper_state.data = false;
  }
  pub_bumper.publish(&bumper_state);
  nh.spinOnce();
}


// when switch pressed, LOW
// when switch non pressed, HIGH
