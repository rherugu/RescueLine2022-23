#include "code.h"
//THIS IS OUR GREEN SQUARE CODE///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void greenSquare(int red1, int green1, int blue1) {
  
  I2CMultiplexer.selectPort(7);  // left
  tcs.getRGB(&red, &green, &blue);

  //  Serial.print("L:\t"); Serial.print(int(red));
  //  Serial.print("\tG:\t"); Serial.print(int(green));
  //  Serial.print("\tB:\t\n"); Serial.print(int(blue));
  float ratio = green / red;
  //Serial.print("RATIO: "); Serial.println(ratio); //int(green) > 90 && int(red) < 80 && int(blue) < 100
  if (ratio > 1.5) {
    Serial.println("Left sensor seeing green!");
    greenBool[0] = true;
  }
  I2CMultiplexer.selectPort(6);  // right

  tcs.getRGB(&red, &green, &blue);
  float ratio1 = green / red;  // int(green) > 90 && int(red) < 80 && int(blue) < 100
  // Serial.print("RATIO1: "); Serial.println(ratio1);
  if (ratio1 > 1.5) {  // red = 100
    Serial.println("Right sensor seeing green!");
    greenBool[1] = true;
  }
}



//THIS IS OUR CODE TO TURN 180 DEGREES////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnAround() {
  sensors_event_t orientationData, linearAccelData;

  I2CMultiplexer.selectPort(0);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  int value = (int)euler.x();
  if (value >= 180) {
    value -= 360;
  }
  int target = value + 180;
  int i = value;



  rightmotor.run(-105);

  leftmotor.run(105);

  delay(600);


  while (i < target) {
    rightmotor.run(105);
    leftmotor.run(105);
    Serial.println("DOING EULER TURN");

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    i = (int)euler.x();
  }

  rightmotor.run(-90);
  leftmotor.run(90);
  delay(400);
}


//This is our code to turn left///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnLeft() {
  Serial.println("LEFT_FUNCITON");
  // Part 1
  sensors_event_t orientationData, linearAccelData;
  I2CMultiplexer.selectPort(0);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Part 2
  int value = (int)euler.x();
  if (value <= 50) {
    value += 360;
  }
  int target = value - 50;
  int i = value;

  // Part 3
  rightmotor.run(0);
  leftmotor.run(0);
  delay(2000);
  rightmotor.run(-75);
  leftmotor.run(75);
  Serial.println("CHECKING...");
  delay(50);  //250

  // Part 4
  I2CMultiplexer.selectPort(6);
  tcs.getRGB(&red, &green, &blue);
  float ratio1 = green / red;
  if (ratio1 > 1.25) {  // red = 100
    Serial.println("RRIGHT!!");
    turnAround();
  } else {
    // Part 5
    I2CMultiplexer.selectPort(0);
    rightmotor.run(0);
    leftmotor.run(0);
    delay(2000);
    rightmotor.run(-85); /* 1. These lines of codes may not be needed 2. May need to switch positive/negative */
    leftmotor.run(85);
    delay(700);  // this is where we go straight

    // Part 6
    while (i > target) {
      rightmotor.run(-75);
      leftmotor.run(-100);  // changed this
      Serial.println("DOING EULER TURN");
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      i = (int)euler.x();
    }


    // Part 7
    double positionGreen = qtr.readLineBlack(sensorValues);
    while (positionGreen < 3200 || positionGreen > 3800) {
      Serial.println("IN LOOP");
      rightmotor.run(-100);
      leftmotor.run(-100);
      positionGreen = qtr.readLineBlack(sensorValues);
    }
  }
}


//This is our code to turn Right//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnRight() {
  Serial.println("Right[ FUnciton");
  // Part 1
  sensors_event_t orientationData, linearAccelData;
  I2CMultiplexer.selectPort(0);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Part 2
  int value = (int)euler.x();
  if (value >= 310) {
    value -= 360;
  }
  int target = value + 50;
  int i = value;

  // Part 3
  rightmotor.run(0);
  leftmotor.run(0);
  delay(2000);
  leftmotor.run(75);
  rightmotor.run(-75);
  Serial.println("CHECKING...");
  delay(50);

  // Part 4
  I2CMultiplexer.selectPort(7);
  tcs.getRGB(&red, &green, &blue);
  float ratio = green / red;
  if (ratio > 1.25) {  // red = 100
    Serial.println("LLEFT!!");
    turnAround();
  } else {
    // Part 5
    I2CMultiplexer.selectPort(0);
    rightmotor.run(0);
    leftmotor.run(0);
    delay(2000);
    rightmotor.run(-85);
    leftmotor.run(85);  // this is where we go straight
    delay(700);

    // Part 6
    while (i < target) {
      rightmotor.run(100);  // changed this
      leftmotor.run(75);
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      i = (int)euler.x();
    }

    // Part 7
    double positionGreen = qtr.readLineBlack(sensorValues);
    while (positionGreen < 3200 || positionGreen > 3800) {
      Serial.println("IN LOOP");
      rightmotor.run(100);
      leftmotor.run(100);
      positionGreen = qtr.readLineBlack(sensorValues);
    }
  }
}


//This is both of our ultrasonic codes////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
