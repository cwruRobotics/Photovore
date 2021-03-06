/*Photovore Robot Control Code
  Peyton Turner and Mike Schafer and Nikil Pancha and Marcus Daly*/


#include <Servo.h>

Servo Head;  // create servo object to control the head
Servo Panels;  // create servo object to control the orientation of the solar panels
int posHead = 0;    // variable to store the Head servo position
int posPanels = 0;    // variable to store the Panel servo position
int sensorPinLeft = A0;    // select the input pin for the left light sensor
int sensorPinRight = A1;    // select the input pin for the right light sensor
int sensorValueLeft = 0;    // variable to store the value coming from the left light sensor
int sensorValueRight = 0;   // variable to store the value coming from the right light sensor
int stablePosition = 0;     // variable to determine if the position of the head is not changing
int lengthOfStability = 0;    // variable to store time since the head last moved
int panelLB = 45; //lower bound on rotation of Panel
int panelUB = 90; //upper bound on rotation of Panel
int panelTurnTime = 15; // in ms per degree amount -- test for this!
int headTurnTime = 15; // in ms per degree amount -- test for this!
int panelVoltagePin = -1; //what should this be (analog)
int panelVoltage;
int someVoltage = 10; // variable to store voltage threshold -- test for this!
bool stable = false;

void setup() {
  // put your setup code here, to run once:
  Head.attach(7);  // attaches the servo on pin 9 to the servo Head object
  Panels.attach(6); // attaches the servo on pin 10 to the servo Panels object
  pinMode(4, OUTPUT); // left motor forward
  pinMode(5, OUTPUT); // left motor backward
  pinMode(2, OUTPUT); // right motor forward
  pinMode(3, OUTPUT); // right motor backward
  Panels.write(posPanels);
  posPanels = panelLB;

  Serial.begin(9600);
}

void loop() {
/*
  // put your main code here, to run repeatedly:
  sensorValueLeft = analogRead(sensorPinLeft);    // read off the voltage from the left sensor (increased light intensity -> smaller values)
  sensorValueRight = analogRead(sensorPinRight);    // read off the voltage from the rgiht sensor (increased light intensity -> smaller values)
  findOptimalRotation(50);
  //head is now in optimal position (sort of not really but hopefully)
  if (approximatelyEqual(stablePosition, posHead, 1)) {    // if the position of the head has not changed, increase the length of stability
    lengthOfStability++;
  } else {    // if the position of the head has changed, reset the length of stability and set the stable position to the current position
    lengthOfStability = 0;
    stablePosition = posHead;
  }
  if (lengthOfStability == 20) {    // if the position of the head has been stable for 20 cycles, align the wheels with the head
    lengthOfStability = 0;
    if (posHead >= 90) {  // if the head is turned right, turn the robot right
      turnRobotRight(500);
    } else {  // head is turned left, turn the robot left
      turnRobotLeft(500);
    }
  }
*/

/*
  while(true) {
    
    sensorValueLeft = analogRead(sensorPinLeft);
    sensorValueRight = analogRead(sensorPinRight);
    
    Serial.println(sensorValueLeft);
    Serial.println(sensorValueRight);
    
    delay(1000);
  }
*/

// my attempt at rewriting stuff
  if (!stable) {
    
    findOptimalRotation(30);
    moveRobot(30);
    /*panelVoltage = analogRead(panelVoltagePin);
    panelVoltage = 5;
    /*if (panelVoltage < someVoltage) moveRobotAntiBackward(1000L);//move forward for a bit i think
    if (panelVoltage >= someVoltage) {
      posPanels = optimalInclination();
      Panels.write(posPanels);
      stable = true;
  } else if (panelVoltage >= someVoltage){
    //wait and sit and do nothing
    delay(123456789L);
  } else {
    //Reorient to find voltage
    stable = false;
    }*/
  }
}

boolean approximatelyEqual(int val1, int val2, int threshold) {
  return abs(val1 - val2) <= threshold;
}

void turnRobotRight(long milliseconds) { // turns on the left motor forward for the given number of milliseconds
  /*first ensure all motors are off to avoid shorting*/
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);

  analogWrite(4, 150);
  analogWrite(3, 150);
  delay(milliseconds);
  analogWrite(4, 0);
  analogWrite(3, 0);
}

void turnRobotLeft(long milliseconds) { // turns on the right motor forward for the given number of milliseconds
  /*first ensure all motors are off to avoid shorting*/
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);

  analogWrite(2, 150);
  analogWrite(5, 150);
  delay(milliseconds);
  analogWrite(2,  0);
  analogWrite(5, 0);
}

void moveRobotAntiBackward(long milliseconds){
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);

  analogWrite(3, 100);
  analogWrite(5, 100);
  delay(milliseconds);
  analogWrite(3, 0);
  analogWrite(5, 0);
}

//find optimal direction to travel in and rotate head to that position
void findOptimalRotation(int threshold) { 
  sensorValueLeft = analogRead(sensorPinLeft);    // read off the voltage from the left sensor (increased light intensity -> smaller values)
  sensorValueRight = analogRead(sensorPinRight);    // read off the voltage from the right sensor (increased light intensity -> smaller values)
 
  
  while (!approximatelyEqual(sensorValueLeft, sensorValueRight, threshold)) {
    if (sensorValueLeft < sensorValueRight) {   // if there is stronger light intensity to the left of the head rotate left 1 degree
      posHead -= 1;
      if (posHead < 0) { // the angle does not underrun 0 degrees
        posHead = 0;
        turnRobotRight(10);
      }
      Head.write(posHead);
      delay(headTurnTime);    // waits 15ms for the servo to reach the position
    } else if (sensorValueLeft > sensorValueRight) {   // if there is stronger light intensity to the right of the head rotate right 1 degree
      posHead += 1;
      if (posHead >= 180) { // the angle does not overrun 180 degrees
        posHead = 179;
        turnRobotLeft(10);
      }
      Head.write(posHead);
      delay(headTurnTime);    // waits 15ms for the servo to reach the position
    }
    sensorValueLeft = analogRead(sensorPinLeft);    // read off the voltage from the left sensor (increased light intensity -> smaller values)
    sensorValueRight = analogRead(sensorPinRight);    // read off the voltage from the rgiht sensor (increased light intensity -> smaller values)
  }
}

int optimalInclination() {
  int bestAngle = posPanels;
  int bestVoltage = analogRead(panelVoltagePin);
  Panels.write(panelLB);
  delay(panelTurnTime * (posPanels - panelLB));
  for (int i = panelLB; i <= panelUB; i++) {
    Panels.write(i);
    delay(panelTurnTime);
    int nowVoltage = analogRead(panelVoltagePin);
    if (nowVoltage > bestVoltage) {
      bestAngle = i; //current maximum
      bestVoltage = nowVoltage;
    }
    
  }
  return bestAngle;
}

void moveRobot(int threshold) {
//  int goalSensors = (sensorValueLeft + sensorValueRight); 
//  int goalPos = posHead;
  int robotTurnTime = 10; // placeholder for how long to turn
  
  if (abs(posHead - 90) > 3) {
    if (posHead > 90) {
      turnRobotLeft(robotTurnTime); //we should make this time a function of abs(posHead-90) but need to experiment
    } else if (posHead < 90) {
      turnRobotRight(robotTurnTime);
    }
    delay(150);
  }
  else {
    moveRobotAntiBackward(200);
  }
}


