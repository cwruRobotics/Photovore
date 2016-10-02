/*Photovore Robot Control Code
  Peyton Turner and Mike Schafer*/


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


void setup() {
  // put your setup code here, to run once:
  Head.attach(9);  // attaches the servo on pin 9 to the servo Head object
  Panels.attach(10); // attaches the servo on pin 10 to the servo Panels object
  pinMode(1, OUTPUT); // left motor forward
  pinMode(2, OUTPUT); // left motor backward
  pinMode(3, OUTPUT); // right motor forward
  pinMode(4, OUTPUT); // right motor backward


}

void loop() {
  // put your main code here, to run repeatedly:

  sensorValueLeft = analogRead(sensorPinLeft);    // read off the voltage from the left sensor (increased light intensity -> smaller values)
  sensorValueRight = analogRead(sensorPinRight);    // read off the voltage from the rgiht sensor (increased light intensity -> smaller values)

  if (approximatelyEqual(sensorValueLeft, sensorValueRight)) {

    delay(15);    // waits 15ms to avoid evaluating 20 times very quickly and moving the wheels more often than necessary

  }

  else if (sensorValueLeft < sensorValueRight) {   // if there is stronger light intensity to the left of the head rotate left 1 degree

    posHead -= 1;
    if (posHead < 0) { // the angle does not underrun 0 degrees
      posHead = 0;
    }

    Head.write(posHead);
    delay(15);    // waits 15ms for the servo to reach the position

  }

  else if (sensorValueLeft > sensorValueRight) {   // if there is stronger light intensity to the right of the head rotate right 1 degree

    posHead += 1;
    if (posHead > 180) { // the angle does not overrun 180 degrees
      posHead = 180;
    }

    Head.write(posHead);
    delay(15);    // waits 15ms for the servo to reach the position

  }

  if (posHead == stablePosition) {    // if the position of the head has not changed, increase the length of stability
    lengthOfStability += 1;
  }
  else {    // if the position of the head has changed, reset the length of stability and set the stable position to the current position
    lengthOfStability = 0;
    stablePosition = posHead;
  }

  if (lengthOfStability == 20) {    // if the position of the head has been stable for 20 cycles, align the wheels with the head

    lengthOfStability = 0;

    if (posHead >= 90) {  // if the head is turned right, turn the robot right
      turnRobotRight(500);
    }

    if (posHead < 90) {  // if the head is turned left, turn the robot left
      turnRobotLeft(500);
    }




  }
}

  boolean approximatelyEqual(int val1, int val2) {

    return ((val1 == val2) || (val1 == val2 + 1) || (val1 == val2 - 1));

  }

  void turnRobotRight(int milliseconds) { // turns on the left motor forward for the given number of milliseconds
    /*first ensure all motors are off to avoid shorting*/
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);

    digitalWrite(1, HIGH);
    delay(milliseconds);
    digitalWrite(1, LOW);

  }

  void turnRobotLeft(int milliseconds) { // turns on the right motor forward for the given number of milliseconds
    /*first ensure all motors are off to avoid shorting*/
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);

    digitalWrite(3, HIGH);
    delay(milliseconds);
    digitalWrite(3, LOW);

  }
