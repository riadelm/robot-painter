//#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "armPositionsAdj.h"  //262 => missing
#include <string.h>
//#include <VarSpeedServo.h>


#define OFF 1
#define ON 0
#define TURN_ON -1
#define TURN_OFF 0
#define NUM_SERVOS 5
#define SERVO_BASE 0
#define SERVO_A 1
#define SERVO_B 2
#define SERVO_C 3

#define buttonPin 3
#define led0Pin 4             // 0
#define led1Pin 5             // 1
#define led2Pin 6             // 2
#define led3Pin 7             // 3
#define ledPaintingPin 8      // painting
#define ledRecordingPin 9     // recording
#define ledProcessingPin 10   // processing
#define ledSystemReadyPin 11  // system ready
#define ledTakingPhotoPin 12  // taking photo

#define numLEDs 9
#define ledOffset 4

// ----Global Variables----- bluetooth
int canvasMap[576] = { 0 };
byte buf[3] = { 0 };
int index = 0;
int canvasIndex = -1;
boolean readingData = true;

// the center degree pulse length for each motor varies, since the horns cannot be perfectly aligned, this will act as a frame of reference for the motor shaft's position
float SERVO_BASE_FORWARD_DEGREES = 0.0;  // 365 // rotating Clockwise = larger pulses
float SERVO_A_UP_DEGREES = 90.0;         // 357 // moving right = larger pulses
float SERVO_B_UP_DEGREES = 0.0;          // 360 // moving right smaller pulses
float SERVO_C_UP_DEGREES = 0.0;          // 363 // moving right = smaller pulses
int SERVO_BASE_FORWARD_PULSE = 365;      // 0 degrees // rotating Clockwise = larger pulses
int SERVO_A_UP_PULSE = 357;              // 90 degrees // moving right = larger pulses
int SERVO_B_UP_PULSE = 360;              // 0 degrees // moving right smaller pulses
int SERVO_C_UP_PULSE = 360;              // 0 degrees // moving right = smaller pulses


//testing
int loopCtr = 5;

#define ANALOG_PIN A0

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // use the default address of 0x40
//VarSpeedServo myServo;

int pulse = 0;
float angBaseCur = 0.0;
float angACur = 0.0;
float angBCur = 0.0;
float angCCur = 0.0;
float angBaseNext = 0.0;
float angANext = 0.0;
float angBNext = 0.0;
float angCNext = 0.0;
int angColumn = 0;


char imagePacketFinal[48] = {
  // 02 "blocks" of 5 by 7 pixel shapes with a 0 pixel right and bottom border
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  0,
};
int finalArrayPos = 0;
int row = 0;
int col = 0;

float motionQueue[50][4] = {};  // up to twelve hundred possible arm actions per run! The queue holds the theta values

boolean buttonState = true;

void setup() {
  // put your setup code here, to run once:

  pwm.begin();
  pwm.setPWMFreq(60);  // in Hertz
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(2000);
  //while(!Serial);
  Serial.println("in setup");
  //setupToEndPosition();
  //reset();
  //setupToZeroFromEndPosition();
  //testRefillFromZero();
  //setupToZeroFromFlat();
  //delay(1000);
  Serial.println(setMotor(SERVO_BASE, SERVO_BASE_FORWARD_DEGREES));  // it's important that the robot goes to sleep and wakes up in the same position, in this case, with the brush in the water cup
  Serial.println(setMotor(SERVO_A, SERVO_A_UP_DEGREES));
  Serial.println(setMotor(SERVO_B, SERVO_B_UP_DEGREES));
  Serial.println(setMotor(SERVO_C, SERVO_C_UP_DEGREES));

  //setupToEndPosition();
  delay(1000);
}


void loop() {
  static unsigned long SpamTimer;
  if ((unsigned long)(millis() - SpamTimer) >= (100)) {
    SpamTimer = millis();
    if(readingData){
        if (Serial1.available() > 0) {  // ble sending stuff to arduino
          byte b = Serial1.read();

          char c = (char)b;

          if (c == '/') {
            String indexStr = String((char*)buf);
            canvasIndex = indexStr.toInt();
            if (canvasIndex == -1) readingData = !readingData;
            if (canvasIndex < 48 && canvasIndex >= 0){
              imagePacketFinal[canvasIndex] = 1;
            } 
            Serial.print("Index: ");
            Serial.println(canvasIndex);       
            memset(buf, 0, sizeof(buf));
            index = 0;
          } else {
            buf[index] = b;
            index++;
          }
        }
    }else{
      emptyMotionQueue();  // clear any data that might remain
      Serial.println("emptied.    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
      loadMotionQueue();
      Serial.println("loaded.   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
      runMotionQueue();
      while (1);
    }
    //Serial.println("ran.  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
    //loopCtr = loopCtr - 1;
  }
}


void setupToEndPosition() {
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES + 0.0, SERVO_A_UP_DEGREES + 0.0, SERVO_B_UP_DEGREES + 0.0, SERVO_C_UP_DEGREES + 0.0,       // zero
                    SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES - 20.0, SERVO_B_UP_DEGREES - 45.0, SERVO_C_UP_DEGREES - 90.0);  //paint midpoint
  delay(500);
}

void setupToZeroFromEndPosition() {
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES - 5.0, SERVO_B_UP_DEGREES - 65.0, SERVO_C_UP_DEGREES - 90.0,  // zero
                    SERVO_BASE_FORWARD_DEGREES + 0.0, SERVO_A_UP_DEGREES + 0.0, SERVO_B_UP_DEGREES + 0.0, SERVO_C_UP_DEGREES + 0.0);    //paint midpoint
  delay(900);
}

void setupToZeroFromFlat() {
  moveBetweenShapes(0.0, 0.0, 0.0, 0.0,                                                                                               // zero
                    SERVO_BASE_FORWARD_DEGREES + 0.0, SERVO_A_UP_DEGREES + 0.0, SERVO_B_UP_DEGREES + 0.0, SERVO_C_UP_DEGREES + 0.0);  //paint midpoint
  delay(900);
}

void testRefillFromZero() {
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES, SERVO_C_UP_DEGREES,                               // zero
                    SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES - 25.0, SERVO_B_UP_DEGREES - 45.0, SERVO_C_UP_DEGREES - 90.0);  //paint midpoint
  delay(900);
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES - 25.0, SERVO_B_UP_DEGREES - 45.0, SERVO_C_UP_DEGREES - 90.0,   // zero
                    SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES + 35.0, SERVO_B_UP_DEGREES - 45.0, SERVO_C_UP_DEGREES - 90.0);  //paint midpoint
  delay(900);
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES + 35.0, SERVO_B_UP_DEGREES - 45.0, SERVO_C_UP_DEGREES - 90.0,  // midpoint
                    SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES - 25.0, SERVO_C_UP_DEGREES);               // rotation
  delay(900);
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES - 90.0, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES, SERVO_C_UP_DEGREES,  // rotation
                    SERVO_BASE_FORWARD_DEGREES, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES, SERVO_C_UP_DEGREES);        // zero
  delay(900);
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES, SERVO_C_UP_DEGREES,                 // zero
                    SERVO_BASE_FORWARD_DEGREES + 65.2, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES + 35.0, SERVO_C_UP_DEGREES);  //paint midpoint
  delay(900);
  moveBetweenShapes(SERVO_BASE_FORWARD_DEGREES + 65.2, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES + 35.0, SERVO_C_UP_DEGREES,                //paint midpoint
                    SERVO_BASE_FORWARD_DEGREES + 65.2, SERVO_A_UP_DEGREES + 2.8, SERVO_B_UP_DEGREES + 62.0, SERVO_C_UP_DEGREES + 90.4);  //paint endpoint
  delay(900);
  slowReset(SERVO_BASE_FORWARD_DEGREES + 65.2, SERVO_A_UP_DEGREES + 2.8, SERVO_B_UP_DEGREES + 62.0, SERVO_C_UP_DEGREES + 90.4);

  delay(900);
}

void slowReset(float in_BASE, float in_A, float in_B, float in_C) {

  moveBetweenShapes(in_BASE, in_A, in_B, in_C,
                    SERVO_BASE_FORWARD_DEGREES, SERVO_A_UP_DEGREES, SERVO_B_UP_DEGREES, SERVO_C_UP_DEGREES);  //zero

}  // END FUNCTION slowReset()

void reset() {
  // put your main code here, to run repeatedly:
  Serial.println(setMotor(SERVO_BASE, 0.0));  // it's important that the robot goes to sleep and wakes up in the same position, in this case, with the brush in the water cup
  Serial.println(setMotor(SERVO_A, 90.0));
  Serial.println(setMotor(SERVO_B, 0.0));
  Serial.println(setMotor(SERVO_C, 0.0));
}  // END FUNCTION reset()



void loadMotionQueue() {

  int queueCount = 0;
  int lastQueueCount = 0;
  float tempPosAng = 0.0;

  // wake up
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; //endpoint in water cup
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 5.0;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0;
  //   queueCount++;
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; // cup midpoint
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 35.0;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0;
  //   queueCount++;
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; //rotated
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  //   queueCount++;

  // //   //get some paint
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.0; // paint midpoint
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  //   queueCount++;
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.0; // endpoint in paint
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 0.8;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES + 62.0;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES + 90.4;
  //   queueCount++;
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.0; // paint midpoint
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  //   queueCount++;
  //   motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES; // zero
  //   motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  //   motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  //   motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  //   queueCount++;
  motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES;  // zero
  motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 10;
  motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  queueCount++;
  // see what, if anything, is in the image array
  for (int qq = 0; qq < 48; qq++) {  //cycle through the final pixel array
    if (imagePacketFinal[qq]) {      // if the pixel is "on" then load that position from the .h file into the queue
      motionQueue[queueCount][0] = armPositions[qq][0];
      motionQueue[queueCount][1] = 180 - armPositions[qq][1];
      motionQueue[queueCount][2] = -armPositions[qq][2];
      motionQueue[queueCount][3] = -armPositions[qq][3];
      queueCount++;
      tempPosAng = armPositions[qq][2];  // get the value for thetaB
      tempPosAng -= 20.0;                // move back 20 degrees
      motionQueue[queueCount][0] = armPositions[qq][0];
      motionQueue[queueCount][1] = 180 - armPositions[qq][1];
      motionQueue[queueCount][2] = -tempPosAng;
      motionQueue[queueCount][3] = -armPositions[qq][3];
      queueCount++;
      //Serial.print("queue count "); Serial.println(queueCount);
      if (abs(queueCount - lastQueueCount) >= 30) {  // every 30 queue settings (which means 15 pixels have been painted) load a reach paint and return to zero sequence
        Serial.println("added paint!");
        lastQueueCount = queueCount;  // adding paint!

        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES;  // load painting sequence (zero, midpoint, endpoint, midpoint, zero)
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
        queueCount++;
        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.2;  // midpoint
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
        queueCount++;
        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.2;  // endpoint in paint
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 0.8;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES + 62.0;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES + 90.4;
        queueCount++;
        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES + 65.2;  // midpoint
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
        queueCount++;
        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES;  // zero
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
        queueCount++;
        motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES;  // tip back
        motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 35;
        motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
        motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
        queueCount++;
      }  // END IF queueCount

    }  // END IF imagePacketFinal[]
  }    // END FOR LOOP cycle
  Serial.println("added sleep");
  // once pixel count is done, add go to sleep sequence
  // motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES; // start with zero postion
  // motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  // motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  // motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  // queueCount++;
  // motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; //rotated
  // motionQueue[queueCount][1] = SERVO_A_UP_DEGREES;
  // motionQueue[queueCount][2] = SERVO_B_UP_DEGREES;
  // motionQueue[queueCount][3] = SERVO_C_UP_DEGREES;
  // queueCount++;
  // motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; //midpoint
  // motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 35.0;
  // motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0;
  // motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0;
  // queueCount++;
  //  motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0; //endpoint in water cup
  // motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 5.0;
  // motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0;
  // motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0;
  // queueCount++;

  Serial.print("Added ");
  Serial.print(queueCount);
  Serial.println(" actions to the queue");

}  // END FUNCTION loadMotionQueue()

void emptyMotionQueue() {  // clear the queue of any data

  for (int rq = 0; rq < 48; rq++) {
    motionQueue[rq][0] = 0.0;
    motionQueue[rq][1] = 90.0;
    motionQueue[rq][2] = 0.0;
    motionQueue[rq][3] = 0.0;
  }

}  // END FUNCTION emptyMotionQueue

void runMotionQueue() {
  boolean queueHasData = true;
  //int curRow = 0;
  Serial.println("Beginning run ");
  for (int curRow = 0; curRow < 48; curRow++) {
    delay(100);
    if (motionQueue[curRow + 1][1] >= 40.0) {  // check if angle A is ever too low or zero, which means the next value is an empty one
      moveBetweenShapes(motionQueue[curRow][0], motionQueue[curRow][1], motionQueue[curRow][2], motionQueue[curRow][3],
                        motionQueue[curRow + 1][0], motionQueue[curRow + 1][1], motionQueue[curRow + 1][2], motionQueue[curRow + 1][3]);
      //curRow++; // move to the next shape in the queue
      Serial.print("Changed to ");
      Serial.print(curRow);
      Serial.println(" row");
    } else {  // we've finished the run
      queueHasData = false;
    }
  }  // END WHILE LOOP

}  // END FUNCTION runMotionQueque()



//WORKS LIKE A CHARM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int setMotor(byte whichMotor, float whatAngle) {

  float pulsesPerDegree = 1.5;
  int tempPulse = 0;
  switch (whichMotor) {  // map from degrees to pulse length
    case (SERVO_BASE):
      tempPulse = int(map(long(whatAngle), 90, -90, 120, 590));  // CCW angluar max (45 degrees) to CW angular max (90 degrees) to pulses (252.5 microseconds) to (590 microseconds)
      break;
    case (SERVO_A):
      tempPulse = int(map(long(whatAngle), 135, 45, 244.5, 469.50));
      break;
    case (SERVO_B):
      tempPulse = int(map(long(whatAngle), 22.50, -112.50, 416.25, 78.75));
      break;
    case (SERVO_C):
      tempPulse = int(map(long(whatAngle), 22.50, -112.50, 419.25, 70.75));
      break;
  }
  Serial.println(tempPulse);
  pwm.setPWM(whichMotor, 0, tempPulse);
  return tempPulse;  // just to see how well the function converts the angles

}  // END FUNCTION setMotor()

void moveBetweenShapes(float thetaBaseStart, float thetaAStart, float thetaBStart, float thetaCStart, float thetaBaseStop, float thetaAStop, float thetaBStop, float thetaCStop) {
  Serial.println("inside of moveBetweenShapes");
  // smoothly move from one arm shape to the next, in the quickest time possible, and with all motors arriving at the desired shape at the same time
  boolean doneMoving = false;
  boolean baseAtTarget = false;  // need to begin movement at theta start
  boolean motAAtTarget = false;
  boolean motBAtTarget = false;
  boolean motCAtTarget = false;

  float tempBaseAngle = thetaBaseStart;
  float tempMotAAngle = thetaAStart;
  float tempMotBAngle = thetaBStart;
  float tempMotCAngle = thetaCStart;

  int deltaThetaBaseDir = 0;  // the sign of a given deltaTheta value
  int deltaThetaADir = 0;
  int deltaThetaBDir = 0;
  int deltaThetaCDir = 0;

  int deltaBaseInterval = 0;
  int deltaMotAInterval = 0;
  int deltaMotBInterval = 0;
  int deltaMotCInterval = 0;
  int totalTime = 0;

  float deltaThetaBase = abs(thetaBaseStart - thetaBaseStop);
  float deltaThetaA = abs(thetaAStart - thetaAStop);
  float deltaThetaB = abs(thetaBStart - thetaBStop);
  float deltaThetaC = abs(thetaCStart - thetaCStop);
  float maxAngularResoltion = 2.0;  // move no greater than one degree increments (this value cannot be changed without modifying other math
  int maxDelay = 32;                // move no faster that one degree per every 20 milliseconds in either direction

  if (thetaBaseStart - thetaBaseStop < 0) { deltaThetaBaseDir = 1; }
  if (thetaBaseStart - thetaBaseStop > 0) { deltaThetaBaseDir = -1; }
  if (thetaAStart - thetaAStop < 0) { deltaThetaADir = 1; }
  if (thetaAStart - thetaAStop > 0) { deltaThetaADir = -1; }
  if (thetaBStart - thetaBStop < 0) { deltaThetaBDir = 1; }
  if (thetaBStart - thetaBStop > 0) { deltaThetaBDir = -1; }
  if (thetaCStart - thetaCStop < 0) { deltaThetaCDir = 1; }
  if (thetaCStart - thetaCStop > 0) { deltaThetaCDir = -1; }

  float greaterResult = 0.0;

  if (deltaThetaBase > deltaThetaA) { greaterResult = deltaThetaBase; }  // lets find the maximum angular change
  else {
    greaterResult = deltaThetaA;
  }  // we don't need to worry if they're equal, since we ultimately just want the largest deltaTheta value
  if (greaterResult > deltaThetaB) {
  }  // greaterResult = greaterResult;
  else { greaterResult = deltaThetaB; }
  if (greaterResult > deltaThetaC) {
  }  // greaterResult = greaterResult;
  else { greaterResult = deltaThetaC; }

  totalTime = int(greaterResult) * maxDelay;  // now set our total time based upon the largest change in motor postion, this way the arm moves as fast as possible
                                              //Serial.print("Total time:");Serial.println(totalTime);
                                              //Serial.println("got the time!" ); // LAST successful print, then the system hangs...
                                              //Serial.print("deltaThetaBase "); Serial.println(int(deltaThetaBase));
  if (deltaThetaBase > 0.0) {
    deltaBaseInterval = int(float(totalTime) / deltaThetaBase);
  } else {
    deltaBaseInterval = 0;
  }  // the inteval between updating motor positions, don't devide by zero
  //NOTE: the galileo decides int(1.0) is equal to 0...
  if (deltaThetaA > 0.0) {
    deltaMotAInterval = int(float(totalTime) / deltaThetaA);
  } else {
    deltaMotAInterval = 0;
  }  // The Galileo doesn't seem to reliably convert between types...
  if (deltaThetaB > 0.0) {
    deltaMotBInterval = int(float(totalTime) / deltaThetaB);
  } else {
    deltaMotBInterval = 0;
  }
  if (deltaThetaC > 0.0) {
    deltaMotCInterval = int(float(totalTime) / deltaThetaC);
  } else {
    deltaMotCInterval = 0;
  }

  long motorTime = 0;  // a marker to keep track of when we need to increment the motor position
  long lastBaseTime = 0;
  long lastMotATime = 0;
  long lastMotBTime = 0;
  long lastMotCTime = 0;

  //Serial.print("Largest change: "); Serial.print(greaterResult); Serial.print(" So the total time is: "); Serial.print(totalTime); Serial.println(" milliseconds\n");
  //Serial.print("Delta Base is : "); Serial.print(deltaThetaBase); Serial.print(" and the direction is: "); Serial.print(deltaThetaBaseDir); Serial.print(" and it's time interval is "); Serial.print(deltaBaseInterval); Serial.println(" milliseconds\n");
  //Serial.print("Delta A is : "); Serial.print(deltaThetaA); Serial.print(" and the direction is: "); Serial.print(deltaThetaADir); Serial.print(" and it's time interval is "); Serial.print(deltaMotAInterval); Serial.println(" milliseconds\n");
  //Serial.print("Delta B is : "); Serial.print(deltaThetaB); Serial.print(" and the direction is: "); Serial.print(deltaThetaBDir); Serial.print(" and it's time interval is "); Serial.print(deltaMotBInterval); Serial.println(" milliseconds\n");
  //Serial.print("Delta C is : "); Serial.print(deltaThetaC); Serial.print(" and the direction is: "); Serial.print(deltaThetaCDir); Serial.print(" and it's time interval is "); Serial.print(deltaMotCInterval); Serial.println(" milliseconds\n");

  while (doneMoving == false) {  //update the motor angles until we've reached the desired position for all motors

    motorTime = millis();

    if (deltaThetaBase == 0.0) {  // don't send any position data to the motor if it is already at the desired angle
      baseAtTarget = true;
    } else {                                                              //increase or decrease the motor angle until is is greater than or equal to the target position, then limit it to the thetaStop value
      if (motorTime - lastBaseTime > deltaBaseInterval) {                 // has it been long enough since we last updated this motor postion?
        lastBaseTime = motorTime;                                         // reset our current motor timer
        tempBaseAngle += float(deltaThetaBaseDir) * maxAngularResoltion;  // degree should be positive or negative depending on deltaThetaDir
        if (tempBaseAngle > thetaBaseStop && deltaThetaBaseDir == 1) {    // don't set the motor beyond the desired angles
          tempBaseAngle = thetaBaseStop;
          baseAtTarget = true;
        }
        if (tempBaseAngle < thetaBaseStop && deltaThetaBaseDir == -1) {
          tempBaseAngle = thetaBaseStop;
          baseAtTarget = true;
        }
        setMotor(SERVO_BASE, tempBaseAngle);  // convert the angle to a pulse length for the PWM driver board
      }
    }  // END motor BASE update

    if (deltaThetaA == 0) {  // update motor A
      motAAtTarget = true;
    } else {
      if (motorTime - lastMotATime > deltaMotAInterval) {
        lastMotATime = motorTime;
        tempMotAAngle += float(deltaThetaADir) * maxAngularResoltion;
        if (tempMotAAngle > thetaAStop && deltaThetaADir == 1) {
          tempMotAAngle = thetaAStop;
          motAAtTarget = true;
        }
        if (tempMotAAngle < thetaAStop && deltaThetaADir == -1) {
          tempMotAAngle = thetaAStop;
          motAAtTarget = true;
        }
        Serial.println("check");
        Serial.println(tempBaseAngle);
        setMotor(SERVO_A, tempMotAAngle);
      }
    }  // END motor A update

    if (deltaThetaB == 0) {  // update motor B
      motBAtTarget = true;
    } else {
      if (motorTime - lastMotBTime > deltaMotBInterval) {
        lastMotBTime = motorTime;
        tempMotBAngle += float(deltaThetaBDir) * maxAngularResoltion;
        if (tempMotBAngle > thetaBStop && deltaThetaBDir == 1) {
          tempMotBAngle = thetaBStop;
          motBAtTarget = true;
        }
        if (tempMotBAngle < thetaBStop && deltaThetaBDir == -1) {
          tempMotBAngle = thetaBStop;
          motBAtTarget = true;
        }
        Serial.println("check");
        Serial.println(tempBaseAngle);
        setMotor(SERVO_B, tempMotBAngle);
      }
    }  // END motor B update

    if (deltaThetaC == 0) {  // update motor C
      motCAtTarget = true;
    } else {
      if (motorTime - lastMotCTime > deltaMotCInterval) {
        lastMotCTime = motorTime;
        tempMotCAngle += float(deltaThetaCDir) * maxAngularResoltion;
        if (tempMotCAngle > thetaCStop && deltaThetaCDir == 1) {
          tempMotCAngle = thetaCStop;
          motCAtTarget = true;
        }
        if (tempMotCAngle < thetaCStop && deltaThetaCDir == -1) {
          tempMotCAngle = thetaCStop;
          motCAtTarget = true;
        }
        Serial.println("check");
        Serial.println(tempBaseAngle);
        setMotor(SERVO_C, tempMotCAngle);
      }
    }  // END motor C update

    delay(maxDelay);  // wait some time for the motors to physically catch up to the set position
    Serial.print("Base: ");
    if (baseAtTarget == false) {
      Serial.print(tempBaseAngle);
    } else {
      Serial.print("done!");
    }
    Serial.print(", motA: ");
    if (motAAtTarget == false) {
      Serial.print(tempMotAAngle);
    } else {
      Serial.print("done!");
    }
    Serial.print(", motB: ");
    if (motBAtTarget == false) {
      Serial.print(tempMotBAngle);
    } else {
      Serial.print("done!");
    }
    Serial.print(", motC: ");
    if (motCAtTarget == false) {
      Serial.println(tempMotCAngle);
    } else {
      Serial.println("done!");
    }

    if (baseAtTarget && motAAtTarget && motBAtTarget && motCAtTarget) { doneMoving = true; }

  }  // END WHILE LOOP

}  // END FUNCTION moveBetweenShapes
