#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Timer.h"

// robot parameters
#define MAXSPEED 0.15
#define TANKRADIUS 0.07

// initialize timercycles, for micros and millis
volatile unsigned long timerCycles = 0;

// global variables for speed and correction
float robotSpeed = 0;
float correction = 0;
// also initialize the soft limit that will be used for object stopping and robot following
float speedLimit = MAXSPEED;
// also initialize a last updated long
unsigned long lastUpdated = 0;
// declare two variables that indicate that the drive or the speedlimit need to be updated
int updateSL = 0;
int updateDV = 0;

class NewHardware: public ArduinoHardware {
  public: NewHardware():ArduinoHardware(&Serial1, 57600){};
};

ros::NodeHandle_<NewHardware> nh;

/* uncomment to go back to cable instead of bluetooth
// create a nodehandle
ros::NodeHandle nh;
*/

/*
// list the pins
int rupsFwdR = 2;
int rupsFwdL = 6;
int rupsRevR = 3;
int rupsRevL = 7;
int rupsEnR = 24;
int rupsEnL = 25;
*/

// list the pins
int rupsFwdR = 7;
int rupsFwdL = 3;
int rupsRevR = 6;
int rupsRevL = 2;
int rupsEnR = 25;
int rupsEnL = 24;

int ultrasonicTrigger = 23;
int ultrasonicResponse = 22;

int ledPin = 13;

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
  
  // set up timer1. This timer will run updateSpeedLimit with COMPA and updateDrive with COMPB
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 25000; // 2.5 Hz
  OCR1B = 15625; // 4 Hz
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12); // 256 prescaler
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // enable timer compare interrupt
  
  // initialize timer2 for millis and micros
  // set control registers and counter to zero
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  
  TCCR2B = (1 << CS22);
  TIMSK2 = (1 << TOIE2);
  
  // initialize timer3 for driving the left tyre
  // set control registers and counter to zero
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCNT3 = 0;
  
  OCR3B = 0;
  OCR3C = 0;
  TCCR3A |= (1 << COM3B1) | (1 << COM3C1) | (1 << WGM30);
  TCCR3B = (1 << CS32);
  
  // initialize timer4 for driving the left tyre
  // set control registers and counter to zero
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  TCNT4 = 0;
  
  OCR4A = 0;
  OCR4B = 0;
  TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << WGM40);
  TCCR4B = (1 << CS42);
  interrupts(); // enable interrupts
}

// timer compare interrupt service routines
ISR(TIMER1_COMPA_vect){
  updateSL = 1;
}

ISR(TIMER1_COMPB_vect){
  updateDV = 1;
}

ISR(TIMER2_OVF_vect){
  timerCycles++;
}

unsigned long microsTR(){
  unsigned long m, t;
  
  cli();
  t = TCNT0;
  m = timerCycles;
  sei();
  
  return (1024 * m + 4 * t);
}

unsigned long millisTR(){
  return (microsTR() / 1000);
}

void delayMicrosTR(unsigned long usecs){
  unsigned long current = microsTR();
  while (microsTR() - current < usecs){
    // do nothing, just block until the delay has passed
  }
}

unsigned long pulseInTR(int pin){
  unsigned long pulseBegin = microsTR();
  // wait for the pin to go low
  while(digitalRead(pin) == HIGH){
    // do nothing
  }
  // continuously update the pulse begin time
  while(digitalRead(pin) == LOW){
    pulseBegin = microsTR();
  }
  // pulse begins, wait till end
  while(digitalRead(pin) == HIGH){
    // just wait
  }
  unsigned long res = microsTR() - pulseBegin;
  if (res > 40000) return 40000;
  return res;
}

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
  if (rightTrackSpeed >= 0){
    OCR4B = round(rightTrackSpeed / MAXSPEED * 255);
    OCR4A = 0;
  } else{
    OCR4A = round(-rightTrackSpeed / MAXSPEED * 255);
    OCR4B = 0;
  }
  
  if (leftTrackSpeed >= 0){
    OCR3C = round(rightTrackSpeed / MAXSPEED * 255 * 1.3);
    OCR3B = 0;
  } else{
    OCR3B = round(-rightTrackSpeed / MAXSPEED * 255 * 1.3);
    OCR3C = 0;
  }
}

// function that updates the soft speed limit from the depth sensor
void updateSpeedLimit(){
  // calculate time since last update
  unsigned long timeSinceLastUpdate = millisTR() - lastUpdated;
  
  // if the last update is less than 1.5 seconds in the past, update according to ultrasonic sensor
  if (timeSinceLastUpdate < 1000){
    // initiate the ultrasonic sensor
    digitalWrite(ultrasonicTrigger, HIGH);
    delayMicrosTR(10);
    digitalWrite(ultrasonicTrigger, LOW);

    // read the result pulse
    unsigned int pulseLength = pulseInTR(ultrasonicResponse);

    // calculate the distance
    float distance = pulseLength / 58;

    // linearly decrease the soft speed limit from maxspeed to zero between 30 and 20 cms
    if (distance < 20){
      speedLimit = 0;
    } else if (distance > 40){
      speedLimit = MAXSPEED;
    } else{
      speedLimit = (distance - 20) / 20 * MAXSPEED;
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
  lastUpdated = millisTR();
  
  // call updateDrive for minimum latency
  updateDrive();
}


void loop(){
  if (updateSL == 1){
    updateSpeedLimit();
    updateSL = 0;
  }
  if (updateDV == 1){
    updateDrive();
    updateDV = 0;
  }
  nh.spinOnce();
}
