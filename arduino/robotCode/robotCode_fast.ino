#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Timer.h"

// everything in this file is calculated in tenths of mms

// robot parameters
#define MAXSPEED 1500
#define TANKRADIUS 700
#define PRECISIONFACTOR 10000

// global variables for speed and correction
int robotSpeed = 0;
double correction = 0;
// also initialize the soft limit that will be used for object stopping and robot following
int speedLimit = MAXSPEED;
// also initialize a last updated long
unsigned long lastUpdated = 0;

class NewHardware: public ArduinoHardware {
  public: NewHardware():ArduinoHardware(&Serial1, 57600){};
};

ros::NodeHandle_<NewHardware> nh;

/* uncomment to go back to cable instead of bluetooth
// create a nodehandle
ros::NodeHandle nh;
*/

// declare the update timer
Timer t;

// list the pins
int rupsFwdR = 2;
int rupsFwdL = 6;
int rupsRevR = 3;
int rupsRevL = 7;
int rupsEnR = 24;
int rupsEnL = 25;

int ultrasonicTrigger = 23;
int ultrasonicResponse = 22;

int ledPin = 13;

// returns average
int avg(int a, int b){
  return ((a + b)/2);
}

// function that calculates and adjusts the speed
void updateDrive(){
  // calculate the correction that needs to be made and apply to the tracks
  int trackCorrection = int(TANKRADIUS * correction);
  int rightTrackSpeed = robotSpeed + trackCorrection;
  int leftTrackSpeed = robotSpeed - trackCorrection;

  // check if the corrections didn't make the tracks exceed their maximum speed
  // if yes: correct
  int speedMax = max(leftTrackSpeed, rightTrackSpeed);
  int speedMin = min(leftTrackSpeed, rightTrackSpeed);

  if (speedMax > MAXSPEED){
    int diff = speedMax - MAXSPEED;
    leftTrackSpeed -= diff;
    rightTrackSpeed -= diff;
  }

  if (speedMin < -MAXSPEED){
    int diff = speedMin + MAXSPEED;
    leftTrackSpeed -= diff;
    rightTrackSpeed -= diff;
  }
  
  // now apply the soft speedlimit to the track by making sure the mean stays below the soft limit
  int speedAvg = avg(leftTrackSpeed, rightTrackSpeed);
  
  if (speedAvg > speedLimit){
    leftTrackSpeed = int((double(speedLimit) / speedAvg) * leftTrackSpeed);
    rightTrackSpeed *= int((double(speedLimit) / speedAvg) * rightTrackSpeed);
  } else if (speedAvg < -speedLimit){
    leftTrackSpeed *= int((double(-speedLimit) / speedAvg) * leftTrackSpeed);
    rightTrackSpeed *= int((double(-speedLimit) / speedAvg) * rightTrackSpeed)
  }
  
  // check if the speeds are inbound
  leftTrackSpeed = constrain(leftTrackSpeed, -MAXSPEED, MAXSPEED);
  rightTrackSpeed = constrain(rightTrackSpeed, -MAXSPEED, MAXSPEED);
  
  // finally write the correct values to the pin out
  if (rightTrackSpeed >= 0){
    analogWrite(rupsFwdR, round(double(rightTrackSpeed) / MAXSPEED * 255));
    analogWrite(rupsRevR, 0);
  } else{
    analogWrite(rupsRevR, round(double(-rightTrackSpeed) / MAXSPEED * 255));
    analogWrite(rupsFwdR, 0);
  }
  
  if (leftTrackSpeed >= 0){
    analogWrite(rupsFwdL, round(double(leftTrackSpeed) / MAXSPEED * 255));
    analogWrite(rupsRevL, 0);
  } else{
    analogWrite(rupsRevL, round(double(-leftTrackSpeed) / MAXSPEED * 255));
    analogWrite(rupsFwdL, 0);
  }
}

// function that updates the soft speed limit from the depth sensor
void updateSpeedLimit(){
  // calculate time since last update
  unsigned long timeSinceLastUpdate = millis() - lastUpdated;
  
  // if the last update is less than 1.5 seconds in the past, update according to ultrasonic sensor
  if (timeSinceLastUpdate < 1000){
    // initiate the ultrasonic sensor
    digitalWrite(ultrasonicTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicTrigger, LOW);

    // read the result pulse
    unsigned int pulseLength = pulseIn(ultrasonicResponse, HIGH);

    // calculate the distance
    float distance = pulseLength / 58;

    // linearly decrease the soft speed limit from maxspeed to zero between 30 and 20 cms
    if (distance < 20){
      speedLimit = 0;
    } else if (distance > 30){
      speedLimit = MAXSPEED;
    } else{
      speedLimit = int((distance - 20) / 10 * MAXSPEED);
    }
  }
  // if the last update was long ago, set speedlimit to 0 automatically
  else{
    speedLimit = 0;
  }
}

// callback that handles the twist message
// note that turning has priority over linear speed
void driveCallback(const geometry_msgs::Twist& msg){
  // extract the linear speed and the turn that needs to be made
  // make sure that the input does not exceed the maximum spec
  robotSpeed = int(constrain(msg.linear.x * PRECISIONFACTOR, -MAXSPEED, MAXSPEED));
  correction = constrain(msg.angular.z, -MAXSPEED/TANKRADIUS, MAXSPEED/TANKRADIUS);

  // update the last updated time
  lastUpdated = millis();
  
  // call updateDrive for minimum latency
  updateDrive();
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &driveCallback);

void setup(){
  pinMode(rupsFwdR, OUTPUT);
  pinMode(rupsFwdL, OUTPUT);
  pinMode(rupsRevR, OUTPUT);
  pinMode(rupsRevL, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(ultrasonicTrigger, OUTPUT);
  pinMode(ultrasonicResponse, INPUT);
  pinMode(rupsEnR, OUTPUT);
  pinMode(rupsEnL, OUTPUT);
  
  digitalWrite(rupsEnR, HIGH);
  digitalWrite(rupsEnL, HIGH);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.spinOnce();
  
  // set up the timer that contiuously updates the speed limit and driving speed
  t.every(400, updateSpeedLimit);
  t.every(250, updateDrive);
}

void loop(){
  t.update();
  nh.spinOnce();
}
