#include <LiquidCrystal.h>
#include <Pixy2.h>
#include <math.h>

LiquidCrystal lcd(3,2,6,7,4,5);
Pixy2 pixy;

// Sensor/Display information
int led = 8; //initialize digital pin 8 as LED
int JoyStick_X = 0; //joystick x
int JoyStick_Y = 1; //joystick y
int JoyStick_Z = 9; //joystick button press
// Useful Variables to reset after each rep
bool DistanceLock = false; //flag to set desired distance, is true when distance is locked in
bool TimeLock = false; //flag to set desired time, is true when time is locked in
unsigned long Distance = 100; //length of rep in meters (max: 10000, min: 100)
unsigned long Time = 200; //time per 400m lap in seconds (max: 188, min: 80)
bool RepInProgress = false; //flag is true when robot is currently running a rep
bool FinishedRep = false; //flag is true when robot completes a rep
unsigned long StartTime = 0; //captures start time of rep, will overflow after 70 minutes
unsigned long LastTime = 0; //captures last time that micros() was called to measure deltaT
float SteeringServoPos = 0;
float SpeedServoPos = 0;
float accumulatedspeederror = 0;
float previousspeederror = 0;
int previousangle = 0;
int previousintersectiony = -1; //start with a value that cannot be returned by the camera
int offlinecounter = 0;
unsigned long previousintersectiontime = 0; //captures time of last intersection detection
int IntersectionLineArray[6]; //place to store intersection line indices
// Servo, Pixy, and World mapping information
int DefaultTurnAngle = 0;
float MaxSteeringServoPos = 30;
float MinSteeringServoPos = -30;
float MaxSpeedServoPos = 5; //TODO maximum speed of robot in m/s
float MinSpeedServoPos = 0.1; //2.12; //minimum speed of robot in m/s
int MinSpeedinPWM = 120; //PWM signal for Pixy to drive the robot at 2.12m/s
int MaxSpeedinPWM = 250; //TODO //PWM signal for Pixy to drive the robot at 5m/s
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
  float speed;
  float vectortilt;
};

float getydistance(int ypixel) {
  // Converts a vertical pixel location (0 corresponds to top of frame) into a physical distance in front of the robot
  float thetamin = atan(bottomofframe/h);
  float thetamax = atan(topofframe/h);
  float theta = thetamin + (float((frameheight-ypixel)/frameheight)*(thetamax-thetamin));
  float ydistance = h*tan(theta);
  return ydistance;
}

float getxdistance(int pixelx, float disty) {
  // Converts a horizontal pixel location (0 corresponds to left edge of frame) into a physical distance from the robot laterally
  float xrange = (((disty-bottomofframe)/(topofframe-bottomofframe))*((worldx3-worldx2)-(worldx4-worldx1))) + (worldx4-worldx1);
  float xdistance = float(float(pixelx*xrange)/framewidth)-(0.5*xrange);
  return xdistance;
}

void motordriver(float angle, float velocity) { //INCOMPLETE
  // Takes an angle in degrees and drives a motor to this angle with PWM
  // Takes a velocity in meters/second and drives a motor at this speed through an ESC
  int steeringservo = 500 + round((300/MaxSteeringServoPos)*angle); //300 is the positive range of the servo in PWM
  //int speedservo = MinSpeedinPWM + round(((velocity-MinSpeedServoPos)/(MaxSpeedServoPos-MinSpeedServoPos))*float(MaxSpeedinPWM-MinSpeedinPWM));
  int speedservo = Time;
  if (velocity < MinSpeedServoPos-.05) {
    velocity = 0;
    speedservo = 0;
  }
  pixy.setServos(steeringservo,speedservo); //speed should be controlled through S1 pins, which are further from the center of the robot
  SteeringServoPos = angle; //program needs memory of current heading
  SpeedServoPos = velocity; //program needs memory of current speed
}

float steeringPID(float PV, float tilt, float deltaT) {
  // PID Controller for steering
  // Inputs:
  //  PV: float from -90 to 90 [angle between heading and vector center]
  //  tilt: float from -framewidth to framewidth [represents slope of vector]
  // deltaT: float [time in seconds since last iteration]
  // Returns:
  //  control: float [angle from heading that the robot should move toward]
  float Kp = 0.4;
  float Kd = 0.35;
  float sp = 0;
  float error = PV;
  float control = (Kp*error) + (Kd*tilt);

  if (control > MaxSteeringServoPos) { //set maximum control values, cutoff if higher
    control = MaxSteeringServoPos;
  }
  else if (control < MinSteeringServoPos) { //set minimum control values, cutoff if higher
    control = MinSteeringServoPos;
  }
  return control;
}

float speedPID(float PV, float deltaT) { //INCOMPLETE
  float Kp = 1; //TODO
  float Ki = 0; //TODO
  float Kd = 0; //TODO
  float sp = (400.0/float(Time)); //setpoint variable in meters/second
  float error = sp-PV;
  accumulatedspeederror = accumulatedspeederror + error;
  float control = SpeedServoPos + float(Kp*(error + (((deltaT*Ki)/Kp)*accumulatedspeederror) + ((Kd/(Kp*deltaT))*(error-previousspeederror))));
  previousspeederror = error;
  if (control > MaxSpeedServoPos) { //set maximum control values, cutoff if higher
    control = MaxSpeedServoPos;
  }
  else if (control < MinSpeedServoPos) { //set minimum control values, cutoff if lower
    control = MinSpeedServoPos;
  }
  return control;
}

struct features getfeatures(){
  //get vector and intersections
  struct features feats;
  int8_t res;
  res = pixy.line.getMainFeatures();
  if (res<=0) { //if error or nothing detected
    // Use the previous value for linetracking, and if the line hasn't been picked up, record that.  Record absence of intersection
    offlinecounter = offlinecounter + 1;
    feats.angle = previousangle;
    feats.speed = -1;
    feats.vectortilt = 0;
  }
  else { //if no errors, and something is detected
    if (res&LINE_VECTOR) { //if a line is detected
      int tailx = pixy.line.vectors->m_x0;
      int taily = pixy.line.vectors->m_y0;
      int headx = (pixy.line.vectors->m_x1)+3; //offset is due to the camera's non-perfect installation (3 works when m_x0 is 0)
      int heady = pixy.line.vectors->m_y1;
      float hy = getydistance(int((taily+heady)*.5)); //get forward distance from robot in meters
      float wy = getxdistance(int((tailx+headx)*.5),hy); //get lateral distance from the robot in meters
      feats.angle = atan2(wy,hy)*(180/M_PI); //get angle to vector center in degrees
      feats.vectortilt = (headx-tailx);
      previousangle = feats.angle; //save the last angle we detected in case we lose it next time
      offlinecounter = 0; //reset the offline counter
    }
    else { //if a line is not detected
      feats.angle = previousangle; //use the previous data
      feats.vectortilt = 0;
      offlinecounter = offlinecounter + 1; //record that we didn't find a line
    }
    if (res&LINE_INTERSECTION) { //if an intersection is detected (only one can be detected at a time)
      // Check if this line was seen previously
      bool match = false;
      for (int i=0; i<pixy.line.intersections->m_n; i++) {
        if (pixy.line.intersections->m_intLines[i].m_index != -1) { //Unneccesary but thorough in case one line's index is -1
          if (pixy.line.intersections->m_intLines[i].m_index != pixy.line.vectors->m_index) {
            for (int j=0; j<6; j++) {
              if (pixy.line.intersections->m_intLines[i].m_index == IntersectionLineArray[j]) { //if the line index is in the previous lines array
                match = true;
                break;
              }
            }
          }
        }
      }
      if (match) { //if this intersection was seen previously
        //calculate velocity
        float IntersectionTime;
        IntersectionTime = float(micros() - previousintersectiontime)/1000000;
        float currentintersectiondist = getydistance(pixy.line.intersections->m_y);
        float pastintersectiondist = getydistance(previousintersectiony);
        float IntersectionDist = pastintersectiondist - currentintersectiondist;
        float IntersectionSpeed = IntersectionDist/IntersectionTime;
        feats.speed = IntersectionSpeed;
      }
      else { //if this intersection has not been seen previously
        feats.speed = -1;
      }
      // Store all the details about this intersection
      previousintersectiony = pixy.line.intersections->m_y; //record the PV
      previousintersectiontime = micros(); //record the time in microseconds
      // Remember up to 6 lines in the intersection
      if (pixy.line.intersections->m_n >= 6) { //if there are at least 6 lines in the intersection, store 6
        for (int i=0; i<6; i++) {
          IntersectionLineArray[i] = pixy.line.intersections->m_intLines[i].m_index;
        }
      }
      else { //if there are not at least 6 lines in the intersection, store them all and put -1 in the others
        for (int i=0; i<pixy.line.intersections->m_n; i++) {
          IntersectionLineArray[i] = pixy.line.intersections->m_intLines[i].m_index;
        }
        for (int i=pixy.line.intersections->m_n; i<6; i++) {
          IntersectionLineArray[i] = -1; //-1 indicates a lack of a line in the array (it isn't full)
        }
      }
    }
    else { //if no intersection was detected
      IntersectionLineArray[0] = -1;
      IntersectionLineArray[1] = -1;
      IntersectionLineArray[2] = -1;
      IntersectionLineArray[3] = -1;
      IntersectionLineArray[4] = -1;
      IntersectionLineArray[5] = -1;
      feats.speed = -1;
    }
  }
  if (offlinecounter >= 10) { //if the camera has not picked up a line in 10 iterations, end the rep
    Serial.println("Cannot Find Line");
    // Kill the motor
    motordriver(0,0);
    delay(100);
    // Flip Flags
    FinishedRep = true;
    RepInProgress = false;
  }
  return feats;
}

void magic() {
  // Only enter function once 'Distance' has been set and 'Time' has been set
  // Will not stay in function- will constantly enter and exit as program loops
  if (RepInProgress) { //during a rep, do PID control and at the end, set the FinishedRep and RepInProgress flags
    // Get Pixy data, but have backup plan if it doesn't capture a line
    struct features LineandIntersection;
    LineandIntersection = getfeatures(); //get processed camera data
    float angle = LineandIntersection.angle; //this angle is the current angle from the heading to the tip of the vector
    float vel = LineandIntersection.speed; //this speed is the current speed, or -1 if no intersections were available
    float tilt = LineandIntersection.vectortilt; //this represents the sideways tilt of the vector
    if (vel < 0) { //if no intersection was detected, don't change the speed
      // We have no data on speed
      vel = SpeedServoPos; //keep the same speed in the absence of new data
    }
    // Implement PID controllers ///////////////////////////////////////////////////////////
    float deltaT;
    deltaT = float(micros() - LastTime)/1000000;
    LastTime = micros();
    float steeringcontrol = steeringPID(angle, tilt, deltaT); //steering control is an angle in degrees
    float speedcontrol = speedPID(vel, deltaT); //speed control is a velocity in m/s
    // Change motors
    if (RepInProgress) {
      motordriver(steeringcontrol,speedcontrol);
    }
    ////////////////////////////////////////////////////////////////////////////////////////

    // Check if rep is over based on time
    unsigned long CurrentTime = millis();
    unsigned long RepTime = Time*Distance*2.5;
    unsigned long TotalTime = round(StartTime*0.001)+RepTime;
    if (CurrentTime >= TotalTime) { //if the rep is over
      // Kill the motor
      motordriver(steeringcontrol,0);
      delay(100);
      // Flip Flags
      FinishedRep = true;
      RepInProgress = false;
    }
  }
  else if (FinishedRep) { //reset the robot and clear the FinishedRep flag
    digitalWrite(led, LOW); //turn off LED
    Distance = 100; //reset to starting values, not necessary
    Time = 200; //reset to starting values, not necessary
    accumulatedspeederror = 0; //reset to starting value
    previousspeederror = 0; //reset to starting value
    FinishedRep = false;
    DistanceLock = false;
    TimeLock = false;
    offlinecounter = 0;
    previousangle = 0;
    previousintersectiony = -1;
    previousintersectiontime = 0;
  }
  else { //before a rep, when robot needs to countdown to the start and flip the RepInProgress flag
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
    RepInProgress = true;
    StartTime = micros();
    LastTime = StartTime;
  }
}

void setup() {
  Serial.begin(115200);
  // Initialize pins
  pinMode(led, OUTPUT); //set LED pin
  pinMode(JoyStick_Z, INPUT); //set joystick button pin
  // Initialize Pixy
  pixy.init();
  pixy.changeProg("line");
  pixy.line.setDefaultTurn(DefaultTurnAngle);
  pixy.line.setMode(LINE_MODE_WHITE_LINE);
  pixy.setServos(500,0);
  // Initialize LCD
  lcd.begin(8, 2);
  delay(1000);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Its Time");
  lcd.setCursor(0, 1);
  lcd.print(" To Run ");
  delay(1000);
}

void loop() {
  
  if (DistanceLock) { //if the user has selected their distance
    if (TimeLock) { //if the user has selected their pace
      magic(); //everything happens here once the user has given their inputs
    }
    else { //if the user has not selected their pace
      int y,z;
      delay(200);
      y = analogRead(JoyStick_Y);
      z = digitalRead(JoyStick_Z);
      if (z) {
        TimeLock = true;
        digitalWrite(led, HIGH);//turn on LED
        delay(1000);
        digitalWrite(led, LOW); //turn off LED
      }
      if ((y > 600) && (Time < 250)) {
        Time = Time + 1;
      }
      else if ((y < 400) && (Time > 120)) {
        Time = Time - 1;
      }
      // Need length of 'Time' in characters for LCD display
      int stringlength = 0;
      if (Time < 100) {
        stringlength = 2;
        lcd.setCursor(0,0);
        lcd.print("      ");
        lcd.setCursor(6, 0);
        lcd.print(Time);
      }
      else {
        stringlength = 3;
        lcd.setCursor(0,0);
        lcd.print("     ");
        lcd.setCursor(5,0);
        lcd.print(Time);
      }
      lcd.setCursor(0, 1);
      lcd.print("sec/400m");
    }
  }
  else { //if the user has not selected their distance
    int x,z;
    delay(200);
    x = analogRead(JoyStick_X);
    z = digitalRead(JoyStick_Z);
    if (z) {
      DistanceLock = true;
      digitalWrite(led, HIGH);//turn on LED
      delay(1000);
      digitalWrite(led, LOW); //turn off LED
    }
    if ((x > 600) && (Distance < 10000)) {
      Distance = Distance + 100;
    }
    else if ((x < 400) && (Distance > 100)) {
      Distance = Distance - 100;
    }
    // Need length of 'Distance' in characters for LCD display
    int stringlength = 0;
    if (Distance < 1000) {
      stringlength = 3;
      lcd.setCursor(0,0);
      lcd.print("     ");
      lcd.setCursor(5, 0);
      lcd.print(Distance);
    }
    else if (Distance < 10000) {
      stringlength = 4;
      lcd.setCursor(0,0);
      lcd.print("    ");
      lcd.setCursor(4,0);
      lcd.print(Distance);
    }
    else {
      stringlength = 5;
      lcd.setCursor(0,0);
      lcd.print("   ");
      lcd.setCursor(3,0);
      lcd.print(Distance);
    }
    lcd.setCursor(0, 1);
    lcd.print("  meters");
  }
}