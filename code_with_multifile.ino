void setup() {

  Serial.begin(9600);
  I2CMultiplexer.begin();

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  I2CMultiplexer.selectPort(0);
  if (!bno.begin())  //Adafruit_BNO055::OPERATION_MODE_IMUPLUS
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }


  delay(1000);

  for (int port : colorports) {
    I2CMultiplexer.selectPort(port);

    if (tcs.begin()) {
      //Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1)
        ;  // halt!
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
  qtr.setSensorPins((const uint8_t[]){
                      28, 26, 24, 22, 29, 27, 25, 23 },
                    SensorCount);
  qtr.setEmitterPin(2);

  // calibration
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 150; i++) {
    qtr.calibrate();
    Serial.print('.');
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
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

void loop() {
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


  
    Serial.print("Centimeters: ");
    Serial.println(cm);
    if (cm < 6) {
      //rightmotor.run(100);
     // leftmotor.run(-100);
      pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);


      pinMode(pingPin, INPUT);
      long duration2 = pulseIn(pingPin, HIGH);
      cm = microsecondsToCentimeters(duration2);
      Serial.println(cm);
      Serial.println("Going backwards");
      // Part 1
      sensors_event_t orientationData, linearAccelData;
      I2CMultiplexer.selectPort(0);
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      // Part 2
      int value = (int)euler.x();
      Serial.print("Value = ");
      Serial.println(value);
      if (value >= 270) {
        value -= 360;
      }
      int target = value + 90;
      int i = value;
      while (i < target) {
        rightmotor.run(100);
        leftmotor.run(100);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        i = (int)euler.x();
      
      }

      pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);


      pinMode(pingPin, INPUT);
      long duration121 = pulseIn(pingPin, HIGH);
      long cmRIGHT = microsecondsToCentimeters(duration121);
      if (cmRIGHT <= 30) {
        // you have to turn left around the obstacle
        sensors_event_t orientationData, linearAccelData;
        I2CMultiplexer.selectPort(0);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        // Part 2
        int value = (int)euler.x();
        // Serial.print("Value = "); Serial.println(value);
        if (value >= 180) {
          value -= 360;
        }
        int target = value + 180;
        int i = value;
        bool seeWall = true;
        while (i < target) {
          rightmotor.run(100);
          leftmotor.run(100);
          imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
          i = (int)euler.x();
          //  Serial.print("i="); Serial.println(i);
        }
        //RUN ELLIPSE CODE

      } else {
        /*int value = (int)euler.x();
        Serial.print("Value324342324234234234234234234234234324gsgffgdfdfg = "); Serial.println(value);
        if (value >= 225) {
          value -= 360;
        }
        rightmotor.run(0);
        leftmotor.run(0);
        delay(1000);
        int target = value + 135;
        int i = value;
     //   while (i <= (target-3) || i >= (target+3)) {
       while(i < target){
          Serial.println("FASFDFDFDF");
          Serial.println(i);
          rightmotor.run(-118); //-88
          leftmotor.run(36); // 35
          imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
          i = (int)euler.x();

        }
        */
          rightmotor.run(-118); //-88
          leftmotor.run(36); // 35


        delay(3850);

        rightmotor.run(0); //-88
          leftmotor.run(0); // 35
          delay(5000);

          double position22 = qtr.readLineBlack(sensorValues);
          while (position22 > 4500 || position22 < 3500)
          {
            rightmotor.run(-50); 
            leftmotor.run(50);
            position22 = qtr.readLineBlack(sensorValues);

          }
          rightmotor.run(0); //-88
          leftmotor.run(0); // 35
          delay(5000);

           pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);


      pinMode(pingPin, INPUT);
      long duration1211 = pulseIn(pingPin, HIGH);
      long cmRIGHT1 = microsecondsToCentimeters(duration121);

       /*
        int value1 = (int)euler.x();
        Serial.print("VALUEE = ");
        Serial.println(value);
        if (value1 >= 270) {
          value1 -= 360;
        }
        int target1 = value1 + 90;
        int i1 = value1;
        while (i1 < target1) {
          rightmotor.run(100);
          leftmotor.run(100);
          imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
          i1 = (int)euler.x();
        }
        delay(8000);
       */
        
      }
    }
  

  // cycle through color sensors
  for (int port : colorports) {
    I2CMultiplexer.selectPort(port);
    Serial.print("\t");
    tcs.getRGB(&red, &green, &blue);
    greenSquare(int(red), int(green), int(blue));


    tcs.setInterrupt(true);  // turn off LED

    Serial.print("\t");
    Serial.print("\n");
    // port 7 is left, port 6 is right
  }
  // intersections
  if (greenBool[0] == true && greenBool[1] == false) {  // left
    Serial.print("left green");
    // rightmotor.run(-75);
    // leftmotor.run(-75);
    //delay(1377);
    turnLeft();
    greenBool[0] = false;
    getout = true;
  }
  if (greenBool[1] == true && greenBool[0] == false) {  // right
    Serial.print("right green");
    // rightmotor.run(75);
    //leftmotor.run(75);
    // delay(1377);
    turnRight();
    greenBool[1] = false;
    getout = true;
  }
  if (greenBool[0] == true && greenBool[1] == true) {  // both
    Serial.print("both green");
    //rightmotor.run(-100);
    //leftmotor.run(-100);
    //delay(1700);
    turnAround();
    greenBool[0] = false;
    greenBool[1] = false;

    getout = true;
  }
  double position2222 = qtr.readLineBlack(sensorValues);
  if (position2222 == 0 || position2222 == 7000 && getout == false) {


    rightmotor.run(-100);
    leftmotor.run(100);
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

  else {  // Proportional line tracing
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


    char buf[60] = { 0 };
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
