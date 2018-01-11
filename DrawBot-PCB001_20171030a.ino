// this sketch is for a custom PCB: i/o must be matched
// this is evolving from 20160425a... and 20161107b...

//=======================LIBRARIES============================
//============================================================
//============================================================
#include <Servo.h>
//============================================================
//=======================VARIABLES============================
//============================================================
//============================================================
// unused digital pins: 0, 1.
// unused analog pins: A3.
// analog pin A4 & A5 used in setup for servo control output.

// actuators - servo1
Servo servo1;  // create servo object to control a servo.
int servoPosition1 = 90; // position of servo motor.
Servo servo2;  // create servo object to control a servo.
int servoPosition2 = 70; // position of servo motor.
int servoUp2 = 30;  // position of servo2 for pen raised.
int servoDown2 = 47;  // position of servo2 for pen down. This adjusted later by pot1.
int delayServo = 100;  // millis to allow ~10-degree servo travel
int servoLeft = 0;  // left most stop (degrees)(limit approx.0).
int servoRight = 185;  // right most stop (degrees)(limit approx.190).
int servoCenter = ((servoRight - servoLeft) / 2);  // find center of set right/left extremes.
const int servoCycles = 11;  // number of divisions (of 180-deg) to make scans. (was 11)
int servoIncrement = ((servoRight - servoLeft) / (servoCycles - 1)); // degrees to change between scan samples
int sweepIncrement = 6;  // positive or negative for sweep steering. (was 2, 4).
int delaySweep = 30;  // delay between each sweep increment. (was 30).
int sweepBuffer = 6;  // degrees on either side of center to consider "centered" (to ensure stopping at center). (was 2, 7).

// actuators - motor1
const int motorEnable1 = 11; // PWM-equipped (speed control)
const int motorClockwise1 = 12;
const int motorCounterClockwise1 = 13;
int motorSpeed1 = 100;  // percentage speed for matching 1&2
int motorDirection1 = 111;

// actuators - motor2
const int motorEnable2 = 9; // PWM-equipped (speed control)
const int motorClockwise2 = 6;
const int motorCounterClockwise2 = 8;
int motorSpeed2 = 100;  // percentage speed for matching 1&2
int motorDirection2 = 111;

// inputs
const int button1 = 7; // on-board, button closest to digital pins
const int button2 = 4; // on-board closest to analog pins
const int button3 = 2; //
const int button4 = 5; //
int buttonRead1 = 111;
int buttonRead2 = 111;
int buttonRead3 = 111;
int buttonRead4 = 111;
const int pot1 = A0; // on-board pot
int potRead1 = 111;
const int distanceSensor1 = A1; // Sharp "GP2Y0A21YK0F" is on A1...low value is far away.
int delaySensor = 50;  // millis to allow voltage stabilization after dist read. (was 50)
int sensorAveraging1 = 3;  // consecutive reads to average for noise reduction. (!Careful, it multiplies delaySensor and others!). (was 3)
int sensorAverage1 = 0;  // for averaging readings.
int distanceReads1[servoCycles + 1];
const int lightSensor1 = A2;
int lightRead1 = 111;

// indicators
const int light1 = 10;  // LED by USB jack; PWM-equipped pin allows brightness control.
const int light2 = 3;  // LED by D0; PWM-equipped pin allows brightness control.
bool lightOn1 = false;
bool lightOn2 = false;
bool lightState1 = LOW;  // for blinking light1.

// administrative variables
int distanceScores1[(servoCycles + 1)];
int lowestValue = 32000;  // for remembering low (every group should be lower, cuz this number is high).
int lowestValueI = (servoCycles / 2);  // for remembering which direction was low.
int lowestReverse = 150;  // avg. dist of "lowest/best" direction to trigger reverse maneuver (lower=sensitive).
int lowestDriving = 200;  // how close to drive before scanning again (100=24"; 120=18"; 190=12"; 230=approx.10"; 280=approx.8"; 350=approx.6"; 500=approx.4"). ((was 180, 270, 250, 190))
int allClearValue = 90;  // avg distance to determine "allClear" and then "celebrate." ((was 95))
int allScores = (allClearValue * servoCycles);  // for tallying the average of all scans (and deciding whether to celebrate).
int decide = 0;  // for analyze/act functions to communicate: 0=none; 1=backUp; 2=celebrate; 3=steering.
int hardLeftTrigger = (servoCenter - 75);  // under this constitues a "hard left" turn, for steering.
int hardRightTrigger = (servoCenter + 75);  // over this constitutes a "hard right" turn, for steering.
bool turnLeftNext = false;  // for alternating direction turning for reverse &or celebrate.
bool movementOn = false;  // turns on physical operations (movement&scanning).
bool justCelebrated = false;  // shortens the straightening drivetime after celebrating.
bool printCycle = false;  // if things print to serial monitor.
int delayPrint = 1000;  // delay to print to serial monitor.
int delayBlink = 111;  // for blinking light.
int delayMovementShort = 200;  // interval to drive short (when making big turns).
int delayMovement = 400;  // interval to drive medium.
int delayMovementLong = 1000;  // interval to drive long (when going straight)
int delayMovementReverse = 700;  // interval to drive in reverse (to get unstuck).
int delayMovementCelebrate = 1300;  // interval to drive in circle to "celebrate."
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long currentMillisPrint = 0;
unsigned long previousMillisPrint = 0;
unsigned long currentMillisBlink = 0;
unsigned long previousMillisBlink = 0;
unsigned long currentMillisMovement = 0;
unsigned long previousMillisMovement = 0;
//============================================================
//=========================SETUP==============================
//============================================================
//============================================================
void setup() {
  Serial.begin(9600);
  // Input pins:
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(pot1, INPUT);
  pinMode(distanceSensor1, INPUT);
  pinMode(lightSensor1, INPUT);
  // Output pins:
  pinMode(light1, OUTPUT);
  pinMode(light2, OUTPUT);
  // H-Bridge motor1 pins:
  pinMode(motorEnable1, OUTPUT);
  digitalWrite(motorEnable1, LOW);  // turn off just in case
  pinMode(motorClockwise1, OUTPUT);
  digitalWrite(motorClockwise1, LOW);  // turn off just in case
  pinMode(motorCounterClockwise1, OUTPUT);
  digitalWrite(motorCounterClockwise1, LOW);  // turn off just in case
  // H-Bridge motor2 pins:
  pinMode(motorEnable2, OUTPUT);
  digitalWrite(motorEnable2, LOW);  // turn off just in case
  pinMode(motorClockwise2, OUTPUT);
  digitalWrite(motorClockwise2, LOW);  // turn off just in case
  pinMode(motorCounterClockwise2, OUTPUT);
  digitalWrite(motorCounterClockwise2, LOW);  // turn off just in case
  // servo motor
  servo1.attach(A5);  // attach servo on pin A5 to servo1 object.
  servo1.write(servoPosition1);  // center the servo (twitches on startup, so may as well set it).
  servo2.attach(A4);  // attach servo on pin A4 to servo1 object.
  servo2.write(servoUp2);  // center the servo (twitches on startup, so may as well set it).
  // conversions
  motorSpeed1 = map (motorSpeed1, 0, 100, 0, 255);
  motorSpeed2 = map (motorSpeed2, 0, 100, 0, 255);
  // set time variables
  currentMillis = millis();
  previousMillis = millis();
  currentMillisPrint = millis();
  previousMillisPrint = millis();
  currentMillisBlink = millis();
  previousMillisBlink = millis();
  currentMillisMovement = millis();
  previousMillisMovement = millis();
}  // close setup.
//============================================================
//=========================LOOP===============================
//============================================================
//============================================================
void loop() {
  declarations();  // declare values for arrays, cuz arduino seems to want that.
  readInputs();  // read button and sensor inputs and writes to serial.
  safety();  // stop movement if bumper switches engaged.
  buttons();  // act on the button pushes.
  if (movementOn == true) {  // if operating...
    scanning();  // sweep the servo and record values.
    analyze();  // decide what action to perform.
    act();  // perform an action.
  }  // close "if(operating...)"
}  // close loop.
//============================================================
//=======================FUNCTIONS============================
//============================================================
//============================================================
//.................Declarations function......................
//============================================================
void declarations() {  // declare values for arrays, cuz arduino seems to want that.
  for (int i = 0; i < (servoCycles + 1); i++) {
    distanceReads1[i] = 1023;
    distanceScores1[i] = 1023;
  }  // close "for(int...)"
}  // close function "declarations"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//...................readInputs function......................
//============================================================
void readInputs() {
  printCycle = false;
  // read inputs
  buttonRead1 = digitalRead(button1);
  buttonRead2 = digitalRead(button2);
  buttonRead3 = digitalRead(button3);
  buttonRead4 = digitalRead(button4);
  potRead1 = analogRead(pot1);
  delay(2);  // delay for analog read stability
  servoDown2 = map(potRead1, 0, 1023, 35, 60);  // set pen down angle.
  sensorAverage1 = 0;  // reset variable.
  for (int j = 0 ; j < sensorAveraging1 ; j++) {
    sensorAverage1 += analogRead(distanceSensor1);  // check a few times...
    delay(delaySensor);
  }  // close "for(i=0...)"
  distanceReads1[0] = (sensorAverage1 / sensorAveraging1);  //average out the multiple readings.
  // write results
  currentMillisPrint = millis();
  if (currentMillisPrint > (previousMillisPrint + delayPrint)) {
    printCycle = false;
    Serial.println("");
    Serial.println("new cycle: ");
    Serial.print("( servoPosition1 = ");
    Serial.print(servoPosition1);
    Serial.print(" )(  buttonRead1 = ");
    Serial.print(buttonRead1);
    Serial.print("  )(  buttonRead2 = ");
    Serial.print(buttonRead2);
    Serial.print("  )(  buttonRead3 = ");
    Serial.print(buttonRead3);
    Serial.print("  )(  buttonRead4 = ");
    Serial.print(buttonRead4);
    Serial.println("  )");
    Serial.print("(  potRead1 = ");
    Serial.print(potRead1);
    Serial.print("  )(  distanceReads1[0] = ");
    Serial.print(distanceReads1[0]);
    Serial.print("  )(  lightRead1 = ");
    Serial.print(lightRead1);
    Serial.print("  )(  motorSpeed1 = ");
    Serial.print(map (motorSpeed1, 0, 255, 0, 100));
    Serial.print("  )(  motorSpeed2 = ");
    Serial.print(map (motorSpeed2, 0, 255, 0, 100));
    Serial.println("  )");
    previousMillisPrint = currentMillisPrint;
  }  // close "if(currentMillisPrint...)"
  else {
    printCycle = false;
  }  // close "else"
  servo2.write(servoUp2);  // position down according to pot.
  delay(delayServo);  // wait for it to move.
}  // close function "readInputs"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................safety function.........................
//============================================================
void safety() {
  if (buttonRead3 == 0 || buttonRead4 == 0) {
    motorDirection1 = 0;
    motorDirection2 = 0;
    Serial.println("safety bumper activated");
  }  // close "if(buttonRead3...)"
}  // close function "safety()"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................buttons function........................
//============================================================
void buttons() {
  if (buttonRead1 == 0 && buttonRead2 == 0) {
    lightOn1 = !lightOn1;
    lightOn2 = !lightOn2;
    Serial.print("toggle lightOn1 = ");
    Serial.println(lightOn1);
    Serial.print("toggle lightOn2 = ");
    Serial.println(lightOn2);
  } // close first "if"
  else {
    if (buttonRead1 == 0 && buttonRead2 == 1) {
      movementOn = false;
      Serial.print("movementOn = ");
      Serial.println(movementOn);
    } // close second "if"
    else {
      if (buttonRead1 == 1 && buttonRead2 == 0) {
        movementOn = true;
        Serial.print("movementOn = ");
        Serial.println(movementOn);
      } // close "if (buttonRead1...)"
    }  // close second "else"
  }  // close first "else"
}  // close function "buttons"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................motors function.......................
//============================================================
void motors() {
  // motor1 //////////////////////////////////////
  if (motorDirection1 == 1) {
    if (printCycle = true) {
      Serial.print("motorDirection1 = 1 @");
      Serial.println(motorSpeed1);
    }  // close "if"
    analogWrite(motorEnable1, motorSpeed1);
    digitalWrite(motorClockwise1, HIGH);
    digitalWrite(motorCounterClockwise1, LOW);
  } // close first "if"
  else {
    if (motorDirection1 == 2) {
      if (printCycle = true) {
        Serial.print("motorDirection1 = 2 @");
        Serial.println(motorSpeed1);
      }  // close "if"
      analogWrite(motorEnable1, motorSpeed1);
      digitalWrite(motorClockwise1, LOW);
      digitalWrite(motorCounterClockwise1, HIGH);
    } // close second "if"
    else {
      if (motorDirection1 == 0) {
        if (printCycle = true) {
          Serial.println("motor1 STOPPING");
        }  // close "if"
        analogWrite(motorEnable1, LOW);
        digitalWrite(motorClockwise1, LOW);
        digitalWrite(motorCounterClockwise1, LOW);
      } // close "if (stopButtonState)..."
    }  //close second "else"
  }  //close first "else"
  // motor2 ///////////////////////////////////////
  if (motorDirection2 == 1) {
    if (printCycle = true) {
      Serial.print("motorDirection2 = 1 @");
      Serial.println(motorSpeed2);
    }  // close "if"
    analogWrite(motorEnable2, motorSpeed2);
    digitalWrite(motorClockwise2, HIGH);
    digitalWrite(motorCounterClockwise2, LOW);
  } // close first "if"
  else {
    if (motorDirection2 == 2) {
      if (printCycle = true) {
        Serial.print("motorDirection2 = 2 @ ");
        Serial.println(motorSpeed2);
      }  // close "if"
      analogWrite(motorEnable2, motorSpeed2);
      digitalWrite(motorClockwise2, LOW);
      digitalWrite(motorCounterClockwise2, HIGH);
    } // close second "if"
    else {
      if (motorDirection2 == 0) {
        if (printCycle = true) {
          Serial.println("motor2 STOPPING");
        }  // close "if"
        analogWrite(motorEnable2, LOW);
        digitalWrite(motorClockwise2, LOW);
        digitalWrite(motorCounterClockwise2, LOW);
      }  // close "if (stopButtonState)..."
    }  //close second "else"
  }  //close first "else"
}  // close function "movement"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................scanning function.......................
//============================================================
void scanning() {
  int i = 0;
  Serial.println("new scan: ");
  servo2.write(servoUp2);  // raise pen before scanning.
  servoPosition1 = servoLeft;
  servo1.write(servoPosition1);
  delay( delayServo * 5 );  // wait for servo to move

  //__ scanning/panning here __//
  for (i = 0; i < (servoCycles); i++) {
    if (movementOn == true) {
      servo1.write(servoPosition1);
      delay(delayServo);  // wait for servo to move
      delay(delaySensor);  // wait for voltage stabilization before sensor reading
      sensorAverage1 = 0;  // reset variable.
      for (int j = 0 ; j < sensorAveraging1 ; j++) {
        sensorAverage1 += analogRead(distanceSensor1);  // check a few times...
        delay(delaySensor);
      }  // close "for(i=0...)"
      distanceReads1[i] = (sensorAverage1 / sensorAveraging1);  //average out the multiple readings.
      Serial.print("( distanceReads1[");
      Serial.print(i);
      Serial.print("] @ ");
      Serial.print(i * servoIncrement, DEC);
      Serial.print(" degrees = ");
      Serial.print(distanceReads1[i]);
      Serial.println(" )");
      buttonRead1 = digitalRead(button1);
      buttonRead2 = digitalRead(button2);
      buttons();
      servoPosition1 += servoIncrement;
    }  // close "if(movementOn...)"
    delay(delayServo);
  }  // close "for(servoPosition1...)"

  //__ tally scores by groups of three __//
  allScores = 0;  // reset allScores.
  for (i = 1; i < (servoCycles - 2); i++) {
    allScores += distanceReads1[i]; // while we're cycling, total all readings for later averaging.
    distanceScores1[i] = ( distanceReads1[(i - 1)] + distanceReads1[i] + distanceReads1[(i + 1)] );  // record in 3-group sets.
  }  // close "for(i=0...)"

  //__ find farthest (lowest-scoring) group/direction __//
  lowestValue = 32000;  // for remembering low (every group should be lower, cuz this number is high).
  for (int k = 1; k < (servoCycles - 2); k++) {
    int value = distanceScores1[k];
    if (lowestValue > value) {
      lowestValue = value;  // record [possible] new lowest value
      lowestValueI = k;  // record [possible] new lowest direction
    }  // close "if(lowestValue...)"
  }  // close "for(i...)"

  //__ report the chosen score __//
  Serial.print("lowestValueI (aka Chosen Direction) = ");
  Serial.print(lowestValueI);
  Serial.print("...    degrees= ");
  Serial.println(lowestValueI * servoIncrement);
  servoPosition1 = (lowestValueI * servoIncrement); // convert to degrees.
}  // close function "scanning."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//.....................analyze function.......................
//============================================================
void analyze() {
  if (lowestValue > (lowestReverse * 3)) { // if even the best direction is too close...
    decide = 1;  // ...record decision to backUp.
  }  // close "if(tooClose)..."
  else {
    if (allScores < (allClearValue * servoCycles)) {  // if all clear...
      decide = 2;  // ...record decision to celebrate.
    }  // close "if(allClear...)"
    else {  // ...or...
      decide = 3;  // ...record decision for regular steering.
    }  // close else2.
  }  // close else1.
}  // close function "analyze."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//.......................act function.........................
//============================================================
// note: yes, this could probably be combined with "analyze" function...
// ...to shorten and simplify.
void act() {
  servo2.write( servoUp2 );  // raise pen.
  if (decide == 1) {
    Serial.println("decided to back-up");
    backUp();  // do that function.
  }  // close "if(...backUp...)"
  else {  // else1.
    if (decide == 2) {
      Serial.println("decided to celebrate");
      celebrate();  // do that function.
    }  // close "if(...celebrate...)"
    else {  // else2.
      if (decide == 3) {
        Serial.println("decided to steer normal");
        steering();  // do that function.
      }  // close "if(...steering...)"
    }  // close else2.
  }  // close else1.
}  // close function "act"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//......................backUp function.......................
//============================================================
void backUp() {
  servo1.write (servoCenter); // straighten out.
  delay(delayServo * 4);  // wait for servo to move.
  motorDirection2 = 1;
  motors();  // back up.
  delay(delayMovementReverse);
  motorDirection2 = 0;
  motors();  // stop.
  if (turnLeftNext == true) { // pick alternating hard turn direction.
    servoPosition1 = servoLeft;  // set left.
  }  // close "if(setHardTurnDir...)"
  else {  // ...otherwise
    servoPosition1 = servoRight;  // ... set right.
  }  // close "elseRight"
  turnLeftNext = !turnLeftNext;  // alternate the direction for next time.
  servo1.write (servoPosition1);  // write servo.
  delay(delayServo * 3);  // allow time for servo movement.
  servo2.write(servoDown2);  // lower pen before driving forward.
  delay(delayServo);  // wait for servo.
  motorDirection2 = 2;  // choose forward direction.
  motors();  // engage motors to go.
  servo2.write(servoUp2);  // raise pen.
  delay(delayMovementReverse);  // wait just long enough to get a new view from turning.
  motorDirection2 = 0; // choose stop motor.
  motors();  // turn off motor (stop!).
  delay(delayServo);  // wait for servo.
}  // close function "backUp."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................celebrate function......................
//============================================================
void celebrate() {
  if (turnLeftNext == true) { // pick alternating hard turn direction.
    servoPosition1 = servoLeft;  // set left.
  }  // close "if(setHardTurnDir...)"
  else {  // ...otherwise
    servoPosition1 = servoRight;  // ... set right.
  }  // close "elseRight"
  turnLeftNext = !turnLeftNext;  // alternate the direction for next time.
  servo1.write (servoPosition1);  // write servo.
  delay(delayServo);  // wait for servo.
  servo2.write(servoDown2);  // lower pen before driving forward.
  motorDirection2 = 2;  // choose forward direction.
  motors();  // engage motors (go!).
  delay(delayMovementCelebrate);  // wait a satisfying moment before returning to normal.
  justCelebrated = true;  // this will shorten drivetime after straightening.
  Serial.println("supposed to steer normal now");
  steering();  // finish by "steering" until close to an obstacle.
}  // close function "celebrate."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//....................steering function.......................
//============================================================
void steering() {
  //__ decide how much to steer __//
  Serial.println("begin normal steering");
  sweepIncrement = abs(sweepIncrement);  // set sweep increment as positive.
  servo1.write(servoPosition1);  // (whatever the previous choices) move the servo.
  if (servoPosition1 < servoCenter) {  // set sweep increment according to direction of turn.
    sweepIncrement = (sweepIncrement * 1);  // set positive for count up 0-90 (left-center).
    Serial.print("sweeping from right to center. sweepIncrement = ");
    Serial.println(sweepIncrement);
  }  // close "if(servo1<(servoRight - servoLeft )...)"
  if (servoPosition1 > servoCenter) {
    sweepIncrement = (sweepIncrement * -1);  // set negative for count down 180-90 (right-center).
    Serial.print("sweeping from left to center. sweepIncrement = ");
    Serial.println(sweepIncrement);
  }  // close "if(servo1>(servoRight - servoLeft )...)"
  if (servoPosition1 <= hardLeftTrigger) {  // if it's a hard left turn...
    servoPosition1 = servoLeft;  // ...set for hard left turn.
  }  // close "if(servo1<hardLeftTrigger...)"
  if (servoPosition1 >= hardRightTrigger) {  // if it's a hard right turn...
    servoPosition1 = servoRight;  // ...set for hard right turn.
  }  // close "if(servo1>hardRightTrigger...)"
  servo1.write(servoPosition1);  // (whatever the previous choices) move the servo.
  delay(delayServo * 5); // wait for servo.
  servo2.write(servoDown2);  // lower pen before driving forward.
  delay(delayServo * 2);  // wait for servo.
  motorDirection2 = 2;  // choose forward direction.
  motors();  // engage motors (go!).

  //__ sweep steering to center__//
  Serial.println("begin sweep steering");
  if (servoPosition1 < servoCenter) { // if we're right of center...
    for ( ; servoPosition1 < 40 ; servoPosition1 += sweepIncrement) {
      Serial.print("sweeping from right to 2/3. servoPosition1 = ");
      Serial.println(servoPosition1);
      servo1.write(servoPosition1);  // move the servo.
      //     delay(delaySweep);  // wait for servo.
    }  // close "for(...sweep steering...)"
    for ( ; servoPosition1 < (servoCenter - sweepBuffer) ; servoPosition1 += sweepIncrement) {
      checkObstacle();  // quick check if we're getting close to something.
      Serial.print("sweeping from right to center. servoPosition1 = ");
      Serial.println(servoPosition1);
      servo1.write(servoPosition1);  // move the servo.
      //     delay(delaySweep);  // wait for servo.
    }  // close "for(...sweep steering...)"
  }  // close "if(servoPosition1<servoCenter...)"
  if (servoPosition1 >= servoCenter) { // if we're left of center...
    for ( ; servoPosition1 > 140 ; servoPosition1 += sweepIncrement) {
      Serial.print("sweeping from left to 4/3. servoPosition1 = ");
      Serial.println(servoPosition1);
      servo1.write(servoPosition1);  // move the servo.
      //     delay(delaySweep);  // wait for servo.
    }  // close "for(...sweep steering...)"
    for ( ; servoPosition1 > (servoCenter + sweepBuffer) ; servoPosition1 += sweepIncrement) {
      checkObstacle();  // quick check if we're getting close to something.
      Serial.print("sweeping from left to center. servoPosition1 = ");
      Serial.println(servoPosition1);
      servo1.write(servoPosition1);  // move the servo.
      //     delay(delaySweep);  // wait for servo.
    }  // close "for(...sweep steering...)"
  }  // close "if(servoPosition1<servoCenter...)"

  //__ drive straight until near obstacle __//
  Serial.println("drive straight after sweep");
  if (justCelebrated == false) {  // check if we are doing normal steering/driving.
    checkObstacle();
    for ( ; distanceReads1[0] < lowestDriving ;  ) {  // as long as far-enough away...
      checkObstacle();  // ...keep driving/checking.
    }  // close "if(distanceReads1[0]<lowestDrive...)"
  }  // close "if(justCelebrated...)"
  else {  // if we had just celebrated, don't drive far.
    //    checkObstacle();  // quick check if we're getting close to something.
    delay (delayMovement);  // medium delay.
  }  // close else delay.
  justCelebrated = false;  // reset the celebration checker.
  servo2.write(servoUp2);  // raise pen.
  motorDirection2 = 0; // choose stop motor.
  motors();  // turn off motor (stop!).
  delay(delayServo);  // wait for servo.
}  // close function "steering."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//......................checkObstacle function................
//============================================================
void checkObstacle() {
  sensorAverage1 = 0;  // reset variable.
  for (int i = 0 ; i < sensorAveraging1 ; i++) {
    sensorAverage1 += analogRead(distanceSensor1);  // quick check if we're getting close to something.
    delay(delaySensor);
    //    Serial.println(sensorAverage1);
  }  // close "for(i=0...)"
  distanceReads1[0] = (sensorAverage1 / sensorAveraging1);  //average out the multiple readings.
  Serial.print("checkObstacles(): distanceReads1[0] = ");
  Serial.println(distanceReads1[0]);
  if (distanceReads1[0] > lowestDriving) {  // check the average.
    servo2.write(servoUp2);  // raise pen.
    motorDirection2 = 0; // choose stop motor.
    motors();  // turn off motor (stop!).
    servoPosition1 = servoCenter;  // set center to stop the sweep.
  }  // close "if(distanceReads1[0]...)"
}  // close function "checkObstacle."
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
