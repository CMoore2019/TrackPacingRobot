//This script is for controlling a track pacing robot.
//Caleb Moore 7/1/2023
  //States:
    //0 Booting Up
    //1 Running
    //2 Recently Stopped
    //3 Awaiting Input
    //4 About to Start

#include <LiquidCrystal.h>
#include <Pixy2.h>
#include <math.h>

LiquidCrystal lcd(3,2,6,7,4,5);
Pixy2 pixy;

//Pin Designations
int led = 8; //LED
int JoyStick_X = 0; //joystick x
int JoyStick_Y = 1; //joystick y
int JoyStick_Z = 9; //joystick button press
int DefaultTurnAngle = 0; //Pixy parameter

//Variables for each run
//Value of -1 designates that it is, for all intents and purposes, currently empty (No NAN option)
int Distance = -1; //length of rep in meters
int Pace = -1; //time per 400m lap in seconds
unsigned long RepDuration;
unsigned long StartTime; //stores start time of rep by calling micros(), will overflow after 70 minutes
unsigned long PreviousTime; //stores last time micros() was called to find delta T
int offlinecounter = 0; //stores number of times camera doesn't detect the line in a row
float previousangle = 0.0;
int STATE = 0;

//Values adjustable, however: MaxPace*MaxDist*2500 must not exceed UL limit of 4,294,967,295
int MinDist = 100; //minimum allowable distance in meters (must be between 3-5 digits long)
int MaxDist = 10000; //maximum allowable distance in meters (must be between 3-5 digits long)
int MinPace = 120; //minimum allowable time per 400m lap in seconds (must be between 2-3 digits long)
int MaxPace = 170; //maximum allowable time per 400m lap in seconds (must be between 2-3 digits long)

//Physical camera distances and measurements
int framewidth = 79;
int frameheight = 52;
float worldx1 = -0.22; //TODO
float worldx2 = -3.66; //TODO
float worldx3 = 3.66; //TODO
float worldx4 = 0.22; //TODO
float worldy1 = 0.13; //TODO
float worldy2 = 4.29; //TODO
float worldy3 = 4.29; //TODO
float worldy4 = 0.13; //TODO
float h = 0.23; //TODO height of camera from ground in meters
float topofframe = 0.5*(worldy2+worldy3);
float bottomofframe = 0.5*(worldy1+worldy4);

struct features {
  float angle;
};

float getydistance(int ypixel) { //STATUS: COMPLETE BUT UNTESTED
  // Converts a vertical pixel location (0 corresponds to top of frame) into a physical distance in front of the robot
  float thetamin = atan(bottomofframe/h);
  float thetamax = atan(topofframe/h);
  float theta = thetamin + (float((frameheight-ypixel)/frameheight)*(thetamax-thetamin));
  float ydistance = h*tan(theta);
  return ydistance;
}

float getxdistance(int pixelx, float disty) { //STATUS: COMPLETE BUT UNTESTED
  // Converts a horizontal pixel location (0 corresponds to left edge of frame) into a physical distance from the robot laterally
  float xrange = (((disty-bottomofframe)/(topofframe-bottomofframe))*((worldx3-worldx2)-(worldx4-worldx1))) + (worldx4-worldx1);
  float xdistance = float(float(pixelx*xrange)/framewidth)-(0.5*xrange);
  return xdistance;
}

void SetSteeringAngle(control) { //STATUS: INCOMPLETE
  //TODO
}

void SetMotorSpeed(control) { //STATUS: INCOMPLETE
  //TODO
}

void Brake() { //STATUS: COMPLETE BUT UNTESTED
  SetMotorSpeed(0);
}

float SpeedController() { //STATUS: COMPLETE BUT UNTESTED
  //Naive approach, this is a temporary fix until robot has speed control or lookup table
  float ScalingFactor = 1.0;
  float control = Pace*ScalingFactor;
  return control
}

float SteeringController(float PV, unsigned long deltaT) { //STATUS: INCOMPLETE
  //PID Controller for Steering
  //deltaT is in microseconds, control should be adjusted accordingly
  //TODO
}

struct features getfeatures(){ //STATUS: COMPLETE BUT UNTESTED
  //Get Vector Information from Pixy
  struct features feats;
  int8_t res;
  res = pixy.line.getMainFeatures();
  if (res<=0) { //if error or nothing detected
    // Use the previous value for linetracking, and if the line hasn't been picked up, record that.
    offlinecounter = offlinecounter + 1;
    feats.angle = previousangle;
  }
  else { //if no errors and something is detected
    if (res&LINE_VECTOR) { //if a line is detected
      int tailx = pixy.line.vectors->m_x0;
      int taily = pixy.line.vectors->m_y0;
      int headx = (pixy.line.vectors->m_x1) + 0; //add offset as necessary to line up camera with world
      int heady = pixy.line.vectors->m_y1;
      float hy = getydistance(int((taily+heady)*.5)); //get forward distance from robot in meters
      float wy = getxdistance(int((tailx+headx)*.5),hy); //get lateral distance from the robot in meters
      feats.angle = atan2(wy,hy)*(180/M_PI); //get angle to vector center in degrees
      previousangle = feats.angle;
      offlinecounter = 0; //reset offline counter
    }
    else { //if a line is not detected
      offlinecounter = offlinecounter + 1;
      feats.angle = previousangle;
    }
  }
  if (offlinecounter >= 10) { //if the camera has not picked up a line in 10 iterations, stop
    Serial.println("Cannot Find Line");
    // Change State to Finished
    STATE = 2;
  }
  return feats;
}

void Running() { //STATUS: COMPLETE BUT UNTESTED
  unsigned long CurrentTime = micros();
  struct features LineData;
  LineData = getfeatures(); //get processed camera data
  //Get control measures from controllers
  unsigned long deltaT;
  deltaT = CurrentTime - PreviousTime;
  float steeringcontrol = SteeringController(LineData.angle, deltaT);
  float speedcontrol = SpeedController();
  //Send control measure to drivers
  SetSteeringAngle(steeringcontrol);
  SetMotorSpeed(speedcontrol);
  //Check if robot has finished the run
  PreviousTime = CurrentTime;
  if (CurrentTime - StartTime >= RepDuration) { //check if rep is over in a micros() overflow-proof method
    STATE = 2;
  }
}

void Finished() { //STATUS: COMPLETE BUT UNTESTED
  Brake();
  digitalWrite(led, LOW);

  //Reset all variables from previous run
  Distance = -1;
  Pace = -1;
  offlinecounter = 0;
  previousangle = 0;
  //Handle STATE Change
  STATE = 3;
}

void Inputs() { //STATUS: COMPLETE BUT UNTESTED
  int y,z;
  int TempDist = 400; //temporary distance in meters, this is the default option displayed
  int TempPace = 130; //temporary time per 400m lap in seconds, this is the default option displayed
  
  while (Distance == -1) {
    delay(200);
    y = analogRead(JoyStick_Y);
    z = digitalRead(Joystick_Z);
    if (z) {
      Distance = TempDist;
      digitalWrite(led, HIGH); //turn on LED
      delay(1000);
      digialWrite(led, LOW); //turn off LED
    }
    if ((y > 600) && (TempDist < MaxDist)) {
      TempDist = TempDist + 100;
    }
    else if ((y < 400) && (TempDist > MinDist)) {
      TempDist = TempDist - 100;
    }
    // Need length of 'TempDist' in characters for LCD display
    int stringlength = 0;
    if (TempDist < 1000) {
      stringlength = 3;
      lcd.setCursor(0,0);
      lcd.print("     ");
      lcd.setCursor(5, 0);
      lcd.print(TempDist);
    }
    else if (TempDist < 10000) {
      stringlength = 4;
      lcd.setCursor(0,0);
      lcd.print("    ");
      lcd.setCursor(4,0);
      lcd.print(TempDist);
    }
    else {
      stringlength = 5;
      lcd.setCursor(0,0);
      lcd.print("   ");
      lcd.setCursor(3,0);
      lcd.print(TempDist);
    }
    lcd.setCursor(0, 1);
    lcd.print("  meters");
  }

  while (Pace == -1) {
    delay(200);
    y = analogRead(JoyStick_Y);
    z = digitalRead(Joystick_Z);
    if (z) {
      Pace = TempPace;
      digitalWrite(led, HIGH); //turn on LED
      delay(1000);
      digialWrite(led, LOW); //turn off LED
    }
    if ((y > 600) && (TempPace < MaxPace)) {
      TempPace = TempPace + 1;
    }
    else if ((y < 400) && (TempPace > MinPace)) {
      TempDist = TempDist - 1;
    }
    // Need length of 'TempPace' in characters for LCD display
    int stringlength = 0;
    if (TempPace < 100) {
      stringlength = 2;
      lcd.setCursor(0,0);
      lcd.print("      ");
      lcd.setCursor(6, 0);
      lcd.print(TempPace);
    }
    else {
      stringlength = 3;
      lcd.setCursor(0,0);
      lcd.print("     ");
      lcd.setCursor(5,0);
      lcd.print(TempPace);
    }
    lcd.setCursor(0, 1);
    lcd.print("sec/400m");
  }


  //Handle STATE Change
  STATE = 4;
}

void Starting() { //STATUS: COMPLETE BUT UNTESTED
  lcd.setCursor(0, 0);
  lcd.print("Starting");
  lcd.setCursor(0, 1);
  lcd.print("in 5 sec");
  delay(2000); //give user 2 seconds before starting the 3 second countdown
  digitalWrite(led, HIGH);//turn on LED
  lcd.setCursor(0, 0);
  lcd.print("       3");
  lcd.setCursor(0, 1);
  lcd.print("        ");
  delay(500);
  digitalWrite(led, LOW); //turn off LED
  delay(500);
  digitalWrite(led, HIGH);//turn on LED
  lcd.setCursor(0, 0);
  lcd.print("       2");
  lcd.setCursor(0, 1);
  lcd.print("        ");
  delay(500);
  digitalWrite(led, LOW); //turn off LED
  delay(500);
  digitalWrite(led, HIGH);//turn on LED
  lcd.setCursor(0, 0);
  lcd.print("       1");
  lcd.setCursor(0, 1);
  lcd.print("        ");
  delay(500);
  digitalWrite(led, LOW); //turn off LED
  delay(500);
  digitalWrite(led, HIGH);//turn on LED
  lcd.setCursor(0, 0);
  lcd.print("     GO!");
  lcd.setCursor(0, 1);
  lcd.print("        ");
  StartTime = micros();
  PreviousTime = StartTime;
  unsigned long PaceUL = Pace;
  unsigned long DistUL = Distance;
  RepDuration = PaceUL*DistUL*2500UL; //total time of run in microseconds

  //Handle STATE Change
  STATE = 1;
}

void setup() { //STATUS: COMPLETE
  //STATE 0
  Serial.begin(115200);
  //Initialize the following items:
  //Pins
  pinMode(led, OUTPUT);
  pinMode(JoyStick_Z, INPUT);
  //PixyCam
  pixy.init();
  pixy.changeProg("line");
  pixy.line.setDefaultTurn(DefaultTurnAngle);
  pixy.line.setMode(LINE_MODE_WHITE_LINE);
  pixy.setServos(500,0);
  //LCD
  lcd.begin(8, 2);
  delay(1000);
  //Print opening message on LCD and display for 3 seconds
  lcd.setCursor(0, 0);
  lcd.print("Its Time");
  lcd.setCursor(0, 1);
  lcd.print(" To Run ");
  delay(3000);

  //Handle STATE Change
  STATE = 3;
}

void loop() { //STATUS: COMPLETE
  //If STATE is 1, do Running()
  if (STATE == 1) {
    Running();
  }
  //If STATE is 2, do Finished()
  if (STATE == 2) {
    Finished();
  }
  //If STATE is 3, do Inputs()
  if (STATE == 3) {
    Inputs();
  }
  //If STATE is 4, do Starting()
  if (STATE == 4) {
    Starting();
  }
  //If STATE is not one of the options, print error message
  else {
    Serial.println("STATE ERROR");
    delay(1000);
  }
}
