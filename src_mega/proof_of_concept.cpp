//Adding Libraries
  #include <Arduino.h>
  #include <Encoder.h>
  #include <SoftwareSerial.h>
  #include <math.h>
  #include <string.h>
  #include <DualVNH5019MotorShield.h> // New Library (Larger Motor Driver)

//Pin declarations
  int driveMotorEnablePin = 2; //Pin for Drive Motor Enable !This may be 6 but probably 5.
  int driveMotorDirectionPin = 7; //Pin for Drive Motor Direction !Dummy Value

  int driveEncoderPinA = 19; //Pin for drive motor encoder channel A !Dummy Value
  int driveEncoderPinB = 18
  ; //Pin for drive motor encoder channel B !Dummy Value

  

//Motor Driver Object Initialization
  DualVNH5019MotorShield mDrive; //creating an object for the motor sheild

//Encoder Object Initialization
  Encoder eDrive(driveEncoderPinA,driveEncoderPinB); //creating a drive encoder object

//Misc variable declaration
  int cmdissued;
  int driveMotorSpeed;
  int driveMotorMoveThresh = 200; //this is the command at which the drive motor is actually able to start moving.
  int driveMotorSpeedSteady; //this is the speed adjusting variable

//Variables for calculating drive position from motor encoders !Check types!
  double CountsPerRev = 1200; //Number of encoder counts per a single drive motor revolution !Check value!
  double wheelRadius = 0.6; //the radius of the drive wheel in inches.
  double gearRatio = 0.5; //The drive motor to drive wheel gear ratio

//Setting distances for ramp up, steady speed, and ramp down
  double setDistRamp = 0.5; //distance for ramp up/down in feet
  double setDistSteady = 5; //distance for steady speed in feet

//Serial Communication variables
  bool newData = false;
  const byte numChars = 32; //This may need to be changed to 4 if we want to include a negative sign !Check! (the '\0' char should do this)
  char charArrayCommand[numChars];

//Initializine speed command variables
  unsigned long startTime;
  double robotSpeed;
  double startPosition;

//Initializing motor testing commands
  const int motorCurrentPin = A0;
  double I = 0.0;
  double I_old = 0.0;
  double I_sum = 0.0;
  double I_avg = 0.0;
  double number_I = 0.0;

void setup() {
//Opening Serial Communication
  Serial.begin(115200);
  Serial.println("Hello Computer!");

//Initializing Drive motor driver object
  mDrive.init(); // initializing drive wheels motor driver object.
  // mDrive.enableDrivers(); //uncomment for the other drive library

  pinMode(driveMotorEnablePin, OUTPUT);
  pinMode(driveMotorDirectionPin, OUTPUT);

  mDrive.setSpeeds(0, 0); //ensuring starting speed command is 0.
  driveMotorSpeed = 0;

//Initializing Drive encoder object.
  pinMode(driveEncoderPinA, INPUT);
  pinMode(driveEncoderPinB, INPUT);

  //!Likely need some more variables initalized here


//Setting initial cmdissued
  cmdissued = 1;
//motor current set up
pinMode(motorCurrentPin,INPUT);

}

void loop() {

switch (cmdissued){
  case 1 : //Standby (Do nothing, await commands)
    receiveCommand();
    processCommand();

    // Serial.println(robotPosition(eDrive.read()));

    break;

  case 2 : //Ramp up motor speed
    //this if statement is included to ensure robot is able to move as the ramp up requires he robot moving to get started
    if (driveMotorSpeed < driveMotorMoveThresh){
      mDrive.setM2Speed(driveMotorMoveThresh);
    }
    else {
      mDrive.setM2Speed(driveMotorSpeed); //!change back to 1
    }
    //update speed command value
    driveMotorSpeed = (driveMotorSpeedSteady/setDistRamp)*robotPosition(eDrive.read());
    if (robotPosition(eDrive.read()) >= setDistRamp){
      mDrive.setM2Speed(driveMotorSpeedSteady); //ensure robot is driving at steady state !check if necessary
      cmdissued = 3; //transition to case 3, steady motor speed
      startSpeedTest();
    }

    I_old = getStallCurrent(I_old);
    
    // Serial.println("In ramp up");
    checkSerial();
    break;

  case 3 : //steady motor speed
    if (robotPosition(eDrive.read()) >= (setDistRamp + setDistSteady)){
      robotSpeed = getSpeed();
      cmdissued = 4;
      I_avg = getAverageCurrent(number_I, I_sum);
    }

    I_sum = getSummedCurrent(I_sum);
    number_I++;

    // Serial.println("In Steady Speed");
    // Serial.println(robotPosition(eDrive.read()));
    checkSerial();
    break;

  case 4 : //ramp down motor speed
    driveMotorSpeed = driveMotorSpeedSteady - (driveMotorSpeedSteady/setDistRamp)*(robotPosition(eDrive.read())- (setDistRamp + setDistSteady));
    if(driveMotorSpeed < driveMotorMoveThresh){
      mDrive.setM2Speed(driveMotorMoveThresh);
    } //may not need this when already moving? !Check!
    else{
      mDrive.setM2Speed(driveMotorSpeed);    }
    if (robotPosition(eDrive.read()) >= (2*setDistRamp + setDistSteady)){
      mDrive.setM2Speed(0);
      cmdissued = 1; //return to standby.
      int t_ref = millis();
      delay(500);
      Serial.println("Test Finished");
      Serial.print("Encoder Distance: ");
      Serial.println(robotPosition(eDrive.read()));
      Serial.print("Steady-State Speed: ");
      Serial.println(robotSpeed);
      Serial.print("Stall Current: ");
      Serial.println(I_old);
      Serial.print("Average Current: ");
      Serial.println(I_avg);

      I_old = 0.0;
      I = 0.0;
      I_avg = 0.0;
      number_I = 0.0;
      I_sum = 0.0;

    }


    // Serial.println("In ramp down");
    // Serial.println(robotPosition(eDrive.read()));
    checkSerial();
    break;

  case 5 : //emergency stop !check to make sure this stops everything!
    mDrive.setM2Speed(0);

      I_old = 0.0;
      I = 0.0;
      I_avg = 0.0;
      number_I = 0.0;
      I_sum = 0.0;


    cmdissued = 1; //!Remove this when testing!
    break;

}
}

double robotPosition(long encoderCounts){ //calculate robot position from encoderCounts !CHeck types!!
  double thetaWheel = (2.0*PI*encoderCounts)/(gearRatio*CountsPerRev);
  double robotPosition = wheelRadius*thetaWheel/12.0; //robot position along track in feet
  return robotPosition;
}

void receiveCommand(){
  static bool receiveInProgress = false;
  static byte ndx = 0;
  char startFlag = '<';
  char endFlag = '>';
  char receivedChar;

  // Serial.println("In receive command");

  while (Serial.available() > 0 && newData == false){
    receivedChar = Serial.read();

    if (receiveInProgress == true){
      if (receivedChar != endFlag){
        charArrayCommand[ndx] = receivedChar;
        ndx++;
        if (ndx >= numChars){
          ndx = numChars - 1;
        }
      }
      else {
        charArrayCommand[ndx] = '\0'; //terminate the character array
        receiveInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (receivedChar == startFlag){
      receiveInProgress = true;
    }
  }
}

void processCommand(){
  if (newData ==true){
    Serial.print("Just received ");
    Serial.println(charArrayCommand);

    if (cmdissued != 1){
      cmdissued = 5;
      newData = false;
      Serial.println("Emergency Stop Command Issued");
      return;
    }

    int receivedInt = atoi(charArrayCommand);

    if (receivedInt >= 1 && receivedInt <= 400){
      driveMotorSpeedSteady = receivedInt;
      
      Serial.print("as an integer this is: ");
      Serial.println(driveMotorSpeedSteady);
      Serial.println("Starting Test");
      eDrive.write(0); //resetting encoder before each run !check for final!

      cmdissued = 2;
    }
    else{
      Serial.print("as an integer this is: ");
      Serial.println(receivedInt);
      Serial.println("Invalid entery, please enter again from 0-400");
    }

    newData = false;
  }
}

void checkSerial(){
  receiveCommand();
  processCommand();
}

void startSpeedTest(){
  startTime = millis();
  startPosition = robotPosition(eDrive.read());
}
// void updateSpeed(){
  //   unsigned long currentTime = millis();
  //   double dt = (durrentTime - oldTime) / 1000.0; // in seconds
  //   if (dt <= 0) {
  //     return;
  //   }

  //   double currentPosition = robotPosition(eDrive.read());
  //   double instantaneousSpeed = )currentPosition - oldPosition) / dt; // in ft/s

  //   double alpha = dt / (tau + dt);
  //   robotSpeed = robotSpeed + alpha * (instantaneousSpeed - robotSpeed);

  //   oldTime = currentTime;
  //   oldPosition = currentPosition;

  // }

double getSpeed(){
  double totalTime = (millis() - startTime)/1000.0; 
  double distance = robotPosition(eDrive.read()) - startPosition;
  return distance / totalTime;
}

double getStallCurrent(double I_old){
  double I_new = double(analogRead(motorCurrentPin))*0.0349;
  if (I_new > I_old){
    I_old = I_new;
  }
  return I_old;
}

double getSummedCurrent(double I){
  return I = I + double(analogRead(motorCurrentPin))*0.0349;
}

double getAverageCurrent(double number, double I_sum){
  I_avg = I_sum/double(number);
  return I_avg;
}