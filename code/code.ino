#include <QTRSensors.h>
#include "MeMegaPi.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include "DFRobot_SSD1306_I2C.h"
#include "DFRobot_I2CMultiplexer.h"
#include "DFRobot_Character.h"
#include "DFRobot_GT30L.h"
#include <SPI.h>

DFRobot_SSD1306_I2C COLOR(0x29);

DFRobot_I2CMultiplexer I2CMulti(0x70);

#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true

//0.04 // 0.09 //0.11
#define Kp 0.35 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
// experiment to 8determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot
#define rightBaseSpeed 80 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 80

MeMegaPiDCMotor rightmotor(PORT1B);

MeMegaPiDCMotor leftmotor(PORT2B);
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const int pingPin = A13;
const int rightPing = A11;
const int leftPing = A12;

int colorports[] = {6, 7};
// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

float red, green, blue;
int x = 0;
bool greenBool[2] = {false, false};
void greenSquare(int red1, int green1, int blue1) {
  I2CMulti.selectPort(7); // left
  tcs.getRGB(&red, &green, &blue);

  //  Serial.print("L:\t"); Serial.print(int(red));
  //  Serial.print("\tG:\t"); Serial.print(int(green));
  //  Serial.print("\tB:\t\n"); Serial.print(int(blue));
  float ratio = green/red;
  Serial.print("RATIO: "); Serial.println(ratio); //int(green) > 90 && int(red) < 80 && int(blue) < 100
  if (ratio > 1.25) {
    Serial.println("Left sensor seeing green!");
    greenBool[0] = true;
  }
  I2CMulti.selectPort(6); // right
  
  tcs.getRGB(&red, &green, &blue);
  float ratio1 = green/red; // int(green) > 90 && int(red) < 80 && int(blue) < 100
  Serial.print("RATIO1: "); Serial.println(ratio1);
  if (ratio1 > 1.25) { // red = 100
    Serial.println("Right sensor seeing green!");
    greenBool[1] = true;
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

void setup()
{

  Serial.begin(9600);

  for (int port : colorports) {
    I2CMulti.selectPort(port);

    if (tcs.begin()) {
      //Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1); // halt!
    }
  }
  Wire.setClock(400000);

#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif


  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }


  delay(1000);
  Serial.println("Starting...");
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    28, 26, 24, 22, 29, 27, 25, 23
  }, SensorCount);
  qtr.setEmitterPin(2);

  // calibration
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 150; i++)
  {
    qtr.calibrate();
    Serial.print('.');
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}
double lasterror = 0;
int countbacks = 0;

bool getout = false;
int timecount = 0;
void loop()
{
  long duration, inches, cm, durationleftPing, durationrightPing, cmleftPing, cmrightPing;
  int y = 1;
  bool right;
  unsigned long startTime;
  unsigned long totalTime;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  // ping
  if (cm <= 8) {
    int x = 0;
    //    float distratio = 57.14;
    while (x == 0)
    {
      pinMode(leftPing, OUTPUT);
      digitalWrite(leftPing, LOW);
      delayMicroseconds(2);
      digitalWrite(leftPing, HIGH);
      delayMicroseconds(5);
      digitalWrite(leftPing, LOW);


      pinMode(leftPing, INPUT);
      durationleftPing = pulseIn(leftPing, HIGH);
      cmleftPing = microsecondsToCentimeters(durationleftPing);

      pinMode(rightPing, OUTPUT);
      digitalWrite(rightPing, LOW);
      delayMicroseconds(2);
      digitalWrite(rightPing, HIGH);
      delayMicroseconds(5);
      digitalWrite(rightPing, LOW);


      pinMode(rightPing, INPUT);
      durationrightPing = pulseIn(rightPing, HIGH);
      cmrightPing = microsecondsToCentimeters(durationrightPing);

      if (cmleftPing < 30) {
        right = true;
      }
      else {
        right = false;
      }

      if (right == true) {
        //needs to turn right

        rightmotor.run(100);
        leftmotor.run(100);
        delay(1300);
        rightmotor.run(0);
        leftmotor.run(0);
        delay(200);

        timecount = 0;
        while (timecount < 5) {
          pinMode(leftPing, OUTPUT);
          digitalWrite(leftPing, LOW);
          delayMicroseconds(2);
          digitalWrite(leftPing, HIGH);
          delayMicroseconds(5);
          digitalWrite(leftPing, LOW);


          pinMode(leftPing, INPUT);
          durationleftPing = pulseIn(leftPing, HIGH);
          cmleftPing = microsecondsToCentimeters(durationleftPing);



          int setPointVal = 10;
          int pingError = cmleftPing - setPointVal;
          int pingP = 25;
          int motorSpeedPing = pingP * pingError;
          int baseSpeedPing = 50;

          int rightMotorSpeedPing = baseSpeedPing + motorSpeedPing;
          int leftMotorSpeedPing = baseSpeedPing - motorSpeedPing;

          leftMotorSpeedPing = constrain(leftMotorSpeedPing, 80, 255);
          rightMotorSpeedPing = constrain(rightMotorSpeedPing, 80, 255);

          //          Serial.print("\n\n\t\t");
          //          Serial.println(pingError);
          //          Serial.print("\t\t Right:");
          //          Serial.print(rightMotorSpeedPing);
          //          Serial.print("\t\t Left:");
          //          Serial.print(leftMotorSpeedPing);

          if (rightMotorSpeedPing < leftMotorSpeedPing) {
            int oldright = rightMotorSpeedPing;
            rightMotorSpeedPing = leftMotorSpeedPing;
            leftMotorSpeedPing = oldright;
          }

          rightmotor.run(-200);
          leftmotor.run(70);
          
//          rightmotor.run(-rightMotorSpeedPing);
//          leftmotor.run(leftMotorSpeedPing);
          timecount++;
          Serial.println(timecount);


          pinMode(leftPing, OUTPUT);
          digitalWrite(leftPing, LOW);
          delayMicroseconds(2);
          digitalWrite(leftPing, HIGH);
          delayMicroseconds(5);
          digitalWrite(leftPing, LOW);


          pinMode(leftPing, INPUT);
          durationleftPing = pulseIn(leftPing, HIGH);
          cmleftPing = microsecondsToCentimeters(durationleftPing);

        }

        rightmotor.run(100);
        leftmotor.run(100);
        delay(1200);
        rightmotor.run(0);
        leftmotor.run(0);
        delay(200);

      }
      if (right == false) {
        //needs to turn left

        rightmotor.run(-100);
        leftmotor.run(-100);
        delay(1200);
        rightmotor.run(0);
        leftmotor.run(0);
        delay(200);
        timecount = 0;
        while (timecount < 150) {
          pinMode(rightPing, OUTPUT);
          digitalWrite(rightPing, LOW);
          delayMicroseconds(2);
          digitalWrite(rightPing, HIGH);
          delayMicroseconds(5);
          digitalWrite(rightPing, LOW);


          pinMode(rightPing, INPUT);
          durationrightPing = pulseIn(rightPing, HIGH);
          cmrightPing = microsecondsToCentimeters(durationrightPing);



          int setPointVal1 = 10;
          int pingError1 = cmrightPing - setPointVal1;
          int pingP1 = 25;
          int motorSpeedPing1 = pingP1 * pingError1;
          int baseSpeedPing1 = 50;

          int rightMotorSpeedPing1 = baseSpeedPing1 - motorSpeedPing1;
          int leftMotorSpeedPing1 = baseSpeedPing1 + motorSpeedPing1;

          leftMotorSpeedPing1 = constrain(leftMotorSpeedPing1, 80, 255);
          rightMotorSpeedPing1 = constrain(rightMotorSpeedPing1, 80, 255);

          //          Serial.print("\n\n\t\t");
          //          Serial.println(pingError);
          //          Serial.print("\t\t Right:");
          //          Serial.print(rightMotorSpeedPing);
          //          Serial.print("\t\t Left:");
          //          Serial.print(leftMotorSpeedPing);

          if (leftMotorSpeedPing1 < rightMotorSpeedPing1) {
            int oldleft = leftMotorSpeedPing1;
            leftMotorSpeedPing1 = rightMotorSpeedPing1;
            rightMotorSpeedPing1 = oldleft;
          }



          rightmotor.run(-95);
          leftmotor.run(225);
          timecount++;
          Serial.println(timecount);


          pinMode(leftPing, OUTPUT);
          digitalWrite(leftPing, LOW);
          delayMicroseconds(2);
          digitalWrite(leftPing, HIGH);
          delayMicroseconds(5);
          digitalWrite(leftPing, LOW);


          pinMode(leftPing, INPUT);
          durationleftPing = pulseIn(leftPing, HIGH);
          cmleftPing = microsecondsToCentimeters(durationleftPing);

        }

        rightmotor.run(-100);
        leftmotor.run(-100);
        delay(1200);
        rightmotor.run(0);
        leftmotor.run(0);
        delay(200);

      }

      x = 1;
    }
  }
  // cycle through color sensors
  for (int port : colorports) {
    I2CMulti.selectPort(port);
    Serial.print("\t");
    tcs.getRGB(&red, &green, &blue);
    greenSquare(int(red), int(green), int(blue));


    tcs.setInterrupt(true);  // turn off LED

    Serial.print("\t");
    Serial.print("\n");
    // port 7 is left, port 6 is right

  }
  // intersections
  if (greenBool[0] == true && greenBool[1] == false) { // left
    Serial.print("left green");
    rightmotor.run(-100);
    leftmotor.run(-50);
    delay(850);

    greenBool[0] = false;
    getout = true;
  }
  if (greenBool[1] == true && greenBool[0] == false) { // right
    Serial.print("right green");
    rightmotor.run(70);
    leftmotor.run(100);
    delay(850);

    greenBool[1] = false;
    getout = true;
  }
  if (greenBool[0] == true && greenBool[1] == true) { // both
    Serial.print("both green");
    rightmotor.run(-100);
    leftmotor.run(-100);
    delay(1700);

    greenBool[0] = false;
    greenBool[1] = false;

    getout = true;
  }
  double position2222 = qtr.readLineBlack(sensorValues);
  if (position2222 == 0 || position2222 == 7000 && getout == false) {


    rightmotor.run(-60);
    leftmotor.run(60);
    //    countbacks++;
    //    position2222 = qtr.readLineBlack(sensorValues);
    //    if (countbacks > 5) {
    //      if (position2222 > 3500) {
    //
    //        rightmotor.run(-150);
    //        leftmotor.run(190);
    //      } else {
    //        rightmotor.run(-190);
    //        leftmotor.run(150);
    //      }
    //      delay(250);
    //      countbacks = 0;
    //    }
    //    delay(150);
    //    double positionleft = qtr.readLineBlack(sensorValues);
    //    if (positionleft != 0 || positionleft != 7000) {
    //      getout = true;
    //    }

    getout = false;
  }

  else {    // Proportional line tracing
    countbacks = 0;
    double position = qtr.readLineBlack(sensorValues);
    unsigned int sensor_values[8];
    double error = position - 3500.0;
    if (error < 500 && error > -500) {
      error = error / 2;
    }

    double motorSpeed = Kp * error;
    int diffError = error - lasterror;



    lasterror = error;

    double Kd = diffError;
    // Serial.println(Kd);
    //Serial.print("KKKKKDDDDD");
    //    while (Kd == 0)
    //    {
    //      rightmotor.run(-100);
    //      leftmotor.run(100);
    //      position = qtr.readLineBlack(sensorValues);
    //      error = position - 3500.0;
    //      diffError = error - lasterror;
    //      lasterror = error;
    //      Kd = diffError;
    //    }

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    leftMotorSpeed = constrain(leftMotorSpeed, -175, 175);
    rightMotorSpeed = constrain(rightMotorSpeed, -175, 175);


    char buf[60] = {0};
    //    Serial.println(">>>>>");
    //    Serial.print(position);
    //    Serial.print("\t");
    //    Serial.print(motorSpeed);
    //
    //    sprintf(buf, ">> \t(%d)\t(%d)\n",  rightMotorSpeed, leftMotorSpeed);
    //    Serial.print(buf);
    //    Serial.println("\n");
    //    qtr.read(sensor_values);
    //    for (int i = 0; i < 8; i++) {
    //      if (sensor_values[i] < 700) {
    //        Serial.print("o");
    //      }
    //      else {
    //        Serial.print("x");
    //      }
    //    } 

     rightmotor.run(-rightMotorSpeed);
     leftmotor.run(leftMotorSpeed);
    //
    //    if (error >=3300 || error <=-3300) {
    //      rightmotor.run(-100);
    //    leftmotor.run(100);
    //    }

#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(1, gammatable[(int)red]);
    ledcWrite(2, gammatable[(int)green]);
    ledcWrite(3, gammatable[(int)blue]);
#else
    analogWrite(redpin, gammatable[(int)red]);
    analogWrite(greenpin, gammatable[(int)green]);
    analogWrite(bluepin, gammatable[(int)blue]);
#endif
  }

}
