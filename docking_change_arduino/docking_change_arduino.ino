#include <ros.h>
#include <std_msgs/String.h>
#include <IRremote.h>
#include <CppList.h>
#include <james_docking/sensor_state.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>

#define USE_USBCON


int RECEIVERS = 3;


const int buttonPin = 8;                          // for bumpercase
const int LeftIr = 4;                             //Variable
const int CenterIr = 5;                           //for
const int RightIr = 6;                            //GPIO

long range_time, range_time_bumper;


unsigned long bit_L[3];
unsigned long bit_C[3];
unsigned long bit_R[3];

unsigned long bit_T[3];


bool buttonState = LOW;

IRrecv **irrecvs;
decode_results results;

ros::NodeHandle nh;
std_msgs::Int16 dockdata_T;
std_msgs::Bool bumper_state;

ros::Publisher pubdockdata_T("dockdata_T", &dockdata_T);
ros::Publisher pub_bumper("bumper_state", &bumper_state);


void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pubdockdata_T);
  nh.advertise(pub_bumper);
  pinMode(buttonPin, INPUT_PULLUP);

  irrecvs = (IRrecv **)malloc(RECEIVERS * sizeof(int));
  irrecvs[0] = new IRrecv(LeftIr); // Receiver #0: pin 4
  irrecvs[1] = new IRrecv(CenterIr); // Receiver #1: pin 5
  irrecvs[2] = new IRrecv(RightIr); // Receiver #2: pin 6

  for (int i = 0; i < RECEIVERS; i++)
  {
    irrecvs[i]->enableIRIn();
  }
}

void loop() {

  buttonState = digitalRead(buttonPin);

  for (int i = 0 ; i < RECEIVERS ; i++) {
    if (irrecvs[i]->decode(&results)) {

      switch (results.value) {
        case 0xFFE21D :
          bit_L[i] = B001;
          break;
        case 0xFFA25D :
          bit_C[i] = B010;
          break;
        case 0xFF629D :
          bit_R[i] = B100;
          break;
        default:
          bit_L[i] = B000;
          bit_C[i] = B000;
          bit_R[i] = B000;
      }
      irrecvs[i]->resume();
    }
  }


  if (millis() >= range_time) {
    for (int i = 0; i < 3; i++) {
      bit_T[i] = bit_L[i] + bit_C[i] + bit_R[i];
    }

    dockdata_T.data =  bit_T[0] << 6 | bit_T[1] << 3 | bit_T[2] ;

    pubdockdata_T.publish(&dockdata_T);




    range_time = millis() + 50;
  }


  if (millis() >= range_time_bumper) {

    if (buttonState == LOW) {
      bumper_state.data = true;
    }
    else if (buttonState == HIGH) {
      bumper_state.data = false;
    }
    pub_bumper.publish(&bumper_state);
    // when switch pressed, LOW
    // when switch non pressed, HIGH


    range_time_bumper = millis() + 10;
  }



  nh.spinOnce();

}
