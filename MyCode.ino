//RECIEVER CODE!! GOES IN YOUR ROBOT
int RobotChannel = 10;
//LOOK FOR RF = 110 in Serial OUTPUT

//Drive Straight Code
int A_power = 255; //Motor A tweaking
int B_power = 255; //Motor B tweaking


//PWM outputs
int led1 = 3; //General Debugging LED
int led0 = 2; //Transmitter Debugging LED


int PWMA = 9;
int PWMB = 10;
int PWMC = 5;
int PWMD = 6;

int sLeftPin = A5;
int sRightPin = A6;

int mapA, mapB;

int mapAold, mapBold;

int state;

int oldH;

int MotorA, MotorB; //Full Speed = 255 ||| SET ZERO WHEN USING WIRELESS
long resetTimer = 0;

// the setup routine runs once when you press reset:
void setup() {
  //Debugging! Uncomment Both PrintData(); & delay(50);
  //**********************
  //PrintData();
  //delay(50);
  //**********************

  //Shift Register Initialize
  shiftSetup();

 

  pinMode(sLeftPin, INPUT_PULLUP);
  pinMode(sRightPin, INPUT_PULLUP);

  //Transmitter Initialize
  ReceiverSetup(RobotChannel); //ReceiverSetup(<CHANNEL>) CHANGE CHANNEL******
}

// the loop routine runs over and over again forever:
void loop() {

  //Debugging! Uncomment Both PrintData(); & delay(50);
  //**********************
  //PrintData();
  //delay(50);
  //**********************

  GetData(); // Get Data fron Transmitter
  processData(); //Process Data

  beat(); // Heartbeat Detect


//DRIVING CODE
  //a - forward
  //b - backward
  //c - left
  //d - right
  if (a > 2) {
    driveForward();
  } else if (b > 2) {
    driveBackward();
  } else if (c > 40 || d > 40) { //if c > 40 OR d > 40 then...
    driveForward();
  } else {
    driveStop();
  }

  delay(1);
  autoReset(); // Leave this is for


  int val0 = map(e, 0, 180, 744, 2200);     // scale it to use it with the servo (value between 0 and 180)
  myservo0.write(val0);
  delay(5);
  int val1 = map(f, 0, 180, 744, 2200);     // scale it to use it with the servo (value between 0 and 180)
  myservo1.write(val1);
  delay(5);

}

void mapMotors() {

  mapA = map(MotorA, 0, 255, 0, A_power);
  mapB = map(MotorB, 0, 255, 0, B_power);

  mapA = constrain(mapA, 0, 255);
  mapB = constrain(mapB, 0, 255);

}

//Robot Drive Functions
void driveForward() {
  mapMotors();
  motorDirection(2, 2);
  analogWrite(PWMA, mapA);
  analogWrite(PWMB, mapB);

  timeOut();
}

void driveForward(int aLeft, int bRight) { //Manual control of the robot.
  aLeft = map(aLeft, 0, 255, 0, A_power);
  bRight = map(bRight, 0, 255, 0, B_power);
  motorDirection(1, 1);
  analogWrite(PWMA, aLeft);
  analogWrite(PWMB, bRight);

  timeOut();
}

void driveBackward() {
  mapMotors();
  motorDirection(1, 1);
  analogWrite(PWMA, mapA);
  analogWrite(PWMB, mapB);

  timeOut();


}
void driveStop() {
  mapMotors();
  motorDirection(0, 0);
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);

}

void timeOut() {

  if (mapAold != mapA || mapBold != mapB) {
    resetTimer = millis() + 2000;
    mapAold = mapA;
    mapBold = mapB;
  }
}



void SensorDetect() {
  int sLeft = digitalRead(sLeftPin);
  int sRight = digitalRead(sRightPin);

  if (!sLeft && !sRight) {
    // a = Forward direction a = 0;
    a = 0;
    Serial.println("S3");
  }

  if (!sLeft) {
    MotorA = 0;
    Serial.println("SL");
  }
  else if (!sRight) {
    MotorB = 0;
    Serial.println("SR ");
  }
}

void beat() {
  if (oldH != h) {

    resetTimer = millis() + 2000;
    Serial.print(h);
    Serial.print(oldH);
    Serial.println("Beat");
    oldH = h;
  }
}

void autoReset() {
  int timeRemain =  millis() - resetTimer;
  if (timeRemain > 0) {
    pinMode(led0, OUTPUT);
    digitalWrite(led0, HIGH);
    ReceiverSetup(RobotChannel); //*************************************
    resetTimer = millis() + 5000;

    digitalWrite(led0, LOW);
  }
}

void processData() {

  //Filter for DATA that is OUT OF BOUNDS
  if (a > 255 || a < 30) {
    a = 0;
  }
  if (b > 255 || b < 30) {
    b = 0;
  }
  if (c > 255 || c < 30) {
    c = 0;
  }
  if (d > 255 || d < 30) {
    d = 0;
  }

//REDUCE MOTOR SPEED IF IN REVERSE  
if(a < b){
  MotorA = (b/1.5) + (d/3);
  MotorB = (b/1.5) + (c/3);
}else{
  MotorA = a + d;
  MotorB = a + c;
}

  constrain(MotorA, 0, 255);
  constrain(MotorB, 0, 255);
}


//Servo Info
//#include <ServoTimer2.h>
//ServoTimer2 myservo0;
//ServoTimer2 myservo1;

//Inside void setup(){
 //myservo0.attach(A3);
 //myservo1.attach(A4);
 }


