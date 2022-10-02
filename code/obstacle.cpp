void obstacle() {
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
}
