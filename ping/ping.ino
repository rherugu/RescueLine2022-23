/*
  Ping))) Sensor

  This sketch reads a PING))) ultrasonic rangefinder and returns the distance
  to the closest object in range. To do this, it sends a pulse to the sensor to
  initiate a reading, then listens for a pulse to return. The length of the
  returning pulse is proportional to the distance of the object from the sensor.

  The circuit:
  - +V connection of the PING))) attached to +5V
  - GND connection of the PING))) attached to ground
  - SIG connection of the PING))) attached to digital pin 7

  created 3 Nov 2008
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Ping
*/

// this constant won't change. It's the pin number of the sensor's output:
#include <QTRSensors.h>
#include "MeMegaPi.h"
MeMegaPiDCMotor rightmotor(PORT1B);

MeMegaPiDCMotor leftmotor(PORT2B);
QTRSensors qtr;
const int pingPin = A13;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
}

void loop() {
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(100);
  
  if (cm <= 8) {
    int x = 0;
    float distratio = 57.14;
    while (x == 0)
    {
//      rightmotor.run(-100);
//      leftmotor.run(-100);
//      delay(975);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//      
//      rightmotor.run(-100);
//      leftmotor.run(100);
//      delay(800);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//
//      rightmotor.run(100);
//      leftmotor.run(100);
//      delay(975);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//
//      float delayval = distratio * (cm+40);
//      Serial.print(delayval);
//      rightmotor.run(-100);
//      leftmotor.run(100);
//      delay(delayval);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//
//      rightmotor.run(100);
//      leftmotor.run(100);
//      delay(975);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//
//      rightmotor.run(-100);
//      leftmotor.run(100);
//      delay(800);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);
//
//      rightmotor.run(-100);
//      leftmotor.run(-100);
//      delay(975);
//      rightmotor.run(0);
//      leftmotor.run(0);
//      delay(200);





      // turn 45 degrees to the left
      rightmotor.run(-100);
      leftmotor.run(-100);
      delay(715);
      rightmotor.run(0);
      leftmotor.run(0);
      delay(200);

      // go straight
      rightmotor.run(-100);
      leftmotor.run(100);
      delay(1900);
      rightmotor.run(0);
      leftmotor.run(0);
      delay(200);

      // turn 90 degrees to the right
      rightmotor.run(100);
      leftmotor.run(100);
      delay(1100);
      rightmotor.run(0);
      leftmotor.run(0);
      delay(200);

      // go straight
      rightmotor.run(-100);
      leftmotor.run(100);
      delay(2500);
      rightmotor.run(0);
      leftmotor.run(0);
      delay(200);
      x=1;
    }
  }
  
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
