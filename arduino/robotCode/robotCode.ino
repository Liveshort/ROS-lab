#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Timer.h"

// robot parameters
#define MAXSPEED 0.15
#define TANKRADIUS 0.07

// global variables for speed and correction
float robotSpeed = 0;
float correction = 0;
// also initialize the soft limit that will be used for object stopping and robot following
float speedLimit = MAXSPEED;
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
float avg(float a, float b){
  return ((a + b)/2);
}

// function that calculates and adjusts the speed
void updateDrive(){
  // calculate the correction that needs to be made and apply to the tracks
  float trackCorrection = TANKRADIUS * correction;
  float rightTrackSpeed = robotSpeed + trackCorrection;
  float leftTrackSpeed = robotSpeed - trackCorrection;

  // check if the corrections didn't make the tracks exceed their maximum speed
  // if yes: correct
  float speedMax = max(leftTrackSpeed, rightTrackSpeed);
  float speedMin = min(leftTrackSpeed, rightTrackSpeed);

  if (speedMax > MAXSPEED){
    float diff = speedMax - MAXSPEED;
    leftTrackSpeed -= diff;
    rightTrackSpeed -= diff;
  }

  if (speedMin < -MAXSPEED){
    float diff = speedMin + MAXSPEED;
    leftTrackSpeed -= diff;
    rightTrackSpeed -= diff;
  }
  
  // now apply the soft speedlimit to the track by making sure the mean stays below the soft limit
  float speedAvg = avg(leftTrackSpeed, rightTrackSpeed);
  
  if (speedAvg > speedLimit){
    leftTrackSpeed *= (speedLimit / speedAvg);
    rightTrackSpeed *= (speedLimit / speedAvg);
  } else if (speedAvg < -speedLimit){
    leftTrackSpeed *= (-speedLimit / speedAvg);
    rightTrackSpeed *= (-speedLimit / speedAvg);
  }
  
  // finally write the correct values to the pin out
  int tempWrite;
  if (rightTrackSpeed >= 0){
    analogWrite(rupsFwdR, round(rightTrackSpeed / MAXSPEED * 255));
    analogWrite(rupsRevR, 0);
  } else{
    analogWrite(rupsRevR, round(-rightTrackSpeed / MAXSPEED * 255));
    analogWrite(rupsFwdR, 0);
  }
  
  if (leftTrackSpeed >= 0){
    analogWrite(rupsFwdL, round(leftTrackSpeed / MAXSPEED * 255));
    analogWrite(rupsRevL, 0);
  } else{
    analogWrite(rupsRevL, round(-leftTrackSpeed / MAXSPEED * 255));
    analogWrite(rupsFwdL, 0);
  }
}

// function that updates the soft speed limit from the depth sensor
void updateSpeedLimit(){
  // calculate time since last update
  unsigned long timeSinceLastUpdate = millis() - lastUpdated;
  
  // if the last update is less than 1.5 seconds in the past, update according to ultrasonic sensor
  if (timeSinceLastUpdate < 1500){
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
      speedLimit = (distance - 20) / 10 * MAXSPEED;
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
  robotSpeed = constrain(msg.linear.x, -MAXSPEED, MAXSPEED);
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
