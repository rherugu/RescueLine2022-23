#include <DFRobot_I2C_Multiplexer.h>

#include <DFRobot_I2C_Multiplexer.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include "DFRobot_SSD1306_I2C.h"
#include "DFRobot_I2C_Multiplexer.h"
#include "DFRobot_Character.h"
#include "DFRobot_GT30L.h"
#include <SPI.h>

DFRobot_SSD1306_I2C COLOR(0x29);

DFRobot_I2C_Multiplexer I2CMulti(&Wire, 0x70);



// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

int colorports[] = {6, 7};
// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

float red, green, blue;
bool greenBool[2] = {false, false};


void greenSquare(int red1, int green1, int blue1) {
  I2CMulti.selectPort(7); 
  tcs.getRGB(&red, &green, &blue);
  float ratio = green/red;
  if (ratio>1.5) {
    
    Serial.print("RL:\t"); Serial.print(int(red));
    Serial.print("\tGL:\t"); Serial.print(int(green));
    Serial.print("\tBL:\t"); Serial.print(int(blue));
    
    Serial.println("Left sensor seeing green!");
    greenBool[0] = true;
  }
  I2CMulti.selectPort(6); 
  tcs.getRGB(&red, &green, &blue);
  float ratio1 = green / red;  
  if (ratio1>1.5) {
    
    Serial.print("RR:\t"); Serial.print(int(red));
    Serial.print("\tGR:\t"); Serial.print(int(green));
    Serial.print("\tBR:\t"); Serial.print(int(blue));
    
    Serial.println("Right sensor seeing green!");
    greenBool[1] = true;
  }
}

void setup() {
  Serial.begin(9600);
  //Serial.println("Color View Test!");
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

  // use these three pins to drive an LED
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

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
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
    //Serial.println(gammatable[i]);
  }
}

// The commented out code in loop is example of getRawData with clear value.
// Processing example colorview.pde can work with this kind of data too, but It requires manual conversion to
// [0-255] RGB value. You can still uncomments parts of colorview.pde and play with clear value.
void loop() {


  //tcs.setInterrupt(false);  // turn on LED

  //delay(60);  // takes 50ms to read
  for (int port : colorports) {
    I2CMulti.selectPort(port);
    //Serial.print(port);

    Serial.print("\t");
    tcs.getRGB(&red, &green, &blue);
    greenSquare(int(red), int(green), int(blue));
    // tcs.setInterrupt(true);  // turn off LED

      
    Serial.print("RR:\t"); Serial.print(int(red));
    Serial.print("\tGR:\t"); Serial.print(int(green));
    Serial.print("\tBR:\t"); Serial.print(int(blue));
      Serial.print("\t");
//    Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
      Serial.print("\n");
    // port 7 is left, port 6 is right

  }



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