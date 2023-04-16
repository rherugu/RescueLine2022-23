#include <QTRSensors.h>
#include "MeMegaPi.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include "DFRobot_SSD1306_I2C.h"
#include "DFRobot_I2C_Multiplexer.h"
#include "DFRobot_Character.h"
#include "DFRobot_GT30L.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "LineTracing.h"
extern QTRSensors qtr;
extern DFRobot_SSD1306_I2C COLOR(0x29);

extern DFRobot_I2C_Multiplexer I2CMultiplexer(&Wire, 0x70);


extern uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;


extern Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true

//0.04 // 0.09 //0.11
//0.47
#define Kp 0.35  // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
// experiment to 8determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define rightMaxSpeed 255  // max speed of the robot
#define leftMaxSpeed 255   // max speed of the robot
#define rightBaseSpeed 60  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 60
#define ellipseDirection 1

extern MeMegaPiDCMotor rightmotor(PORT1B);

extern MeMegaPiDCMotor leftmotor(PORT2B);

const uint8_t SensorCount = 8;
extern uint16_t sensorValues[SensorCount];
const int pingPin = A13;
const int rightPing = A11;
const int leftPing = A12;

extern int colorports[] = { 6, 7 };
// our RGB -> eye-recognized gamma color
extern byte gammatable[256];


extern Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

extern float red, green, blue;
extern int x = 0;
extern bool greenBool[2] = { false, false };
