/* 
 *  last updated: Alina  5/5/22 @ 2:56 PM
 *  added axleFront() and axleStop() functions and incorporated it into loop() 
 *  need to adjust time that axleFront() runs
 *  have fake loop() version 
 *  Vincent adjusted code for distance traveled to obstacle. 
 */

/* Imported Libraries */
#include <Enes100.h>
#include "NewPing.h" // include NewPing library for ult. sensor

/* Motor Pins */
// left side
#define in1a 6
#define in1b 7
// right side
#define in2a 8
#define in2b 9
// axle
#define axleA A5
#define axleB A4

/* Sensor Pins */
// Ultrasonic
#define trigPinF 12 // define Arduino pin #12 for both trig & echo of FRONT ult. sensor
#define echoPinF 12 
#define trigPinR 11 // define Arduino pin #11 for both trig & echo of RIGHT ult. sensor
#define echoPinR 11
#define trigPinL 10 // define Arduino pin #10 for both trig & echo of LEFT ult. sensor
#define echoPinL 10
#define MAX_DISTANCE 400 // HCSR04 - maximum distance detected is 400 cm
// Fire
#define firePinR 13 // Arduino Pin #13 attaches to D0 of LEFT fire sensor
#define firePinL 2 // Arduino pin #2 attaches to D0 of RIGHT fire sensor
// Button
const int buttonLPin = 10;
const int buttonRPin = 11;
const int buttonLApin = 12;
const int buttonRApin = A0; 

/* Ultrasonic Sensor Setup */
NewPing sonarF(trigPinF, echoPinF, MAX_DISTANCE);
NewPing sonarR(trigPinR, echoPinR, MAX_DISTANCE);
NewPing sonarL(trigPinL, echoPinL, MAX_DISTANCE); 

/* Navigation Variables */
int rotDirection; // counterclickwise is 0, clockwise is 1
boolean checkStart = false; // checks if we checked the starting zone
char start = 'x'; // x is a placeholder, basically means nothing

/*Mission Variables*/
int totalFlames = 1;



/* Sensor Variables */
// Ultrasonic
float durationF;
float durationR;
float durationL;
float distanceF; 
float distanceR;
float distanceL;
float soundcm = 34.3; // speed of sound = 34.3 cm/ms
// Fire
int fireValR = 1; // holds either a 0 (fire) or 1 (no fire). instantiates default values to 1 (no fire present)
int fireValL = 1;
boolean detectFlames = true; // turns false once we are done measuring the # of flames present (prevents numFlames() from repeating more than once)
// Buttons
int buttonL = 0; // 1 = button not pressed, 0 = button pressed
int buttonR = 0;
int buttonLA = 0;
int buttonRA = 0;
// other stuff
boolean aligned = false; 
int iterations = 5;
float distToGround = 50; // sets maximum distance to ground as 50 cm (fine tune)
boolean isLinedUp = false; // declares if platform is lined up w/ OSV (false = not lined up)
int minObstacleDist = 33;
; //fine tune: If front sensor detects something less than this distance, we turn right.
boolean pastObstacles = false; // this changes based on our location. Once we are past obstacles, there is no need to detect for them anymore.

/* Arena "Checklist" */
boolean atMission = false; // check if we are at mission site
boolean missionComplete = false; // check if we completed mission goals
boolean readyForObsZone = false; // check if the OTV is "ready" to go through obstacle zone
boolean nearGoal = false; // check if we are in open zone
boolean epicVictory = false; // check if we got the goal zone

void setup() {
  // Team Name, Mission Type, Marker ID (hopefully 9 all the time), TX Pin, RX Pin
  Enes100.begin("Blazin Mike n' Ikes", FIRE, 9, 4, 5);
  // Transmit = 4
  // Recieve = 5
  pinMode(in1a, OUTPUT);
  pinMode(in1b, OUTPUT);
  pinMode(in2a, OUTPUT);
  pinMode(in2b, OUTPUT);

  pinMode(firePinL, INPUT);
  pinMode(firePinR, INPUT); 

  pinMode(buttonLPin, INPUT_PULLUP);
  pinMode(buttonRPin, INPUT_PULLUP);
  pinMode(buttonLA, INPUT_PULLUP);
  pinMode(buttonRA, INPUT_PULLUP);
  
  Serial.begin(9600);
}

void loop() {
  while(!Enes100.updateLocation()) { // if the vision system cant see marker
    Enes100.println("Where da Aruco Marker.");
  }
  // this logic statement will (hopefully) run once, identifies which zone we at
  Enes100.updateLocation();
  if (Enes100.location.y > 1 && checkStart == false) { // if the OTV is on top half
    checkStart = true;
    start = 'a';
    Enes100.println("We are starting at A zone"); // just for visual reference
  } else if (Enes100.location.y < 1 && checkStart == false) { // if the OTV is on the bot half
    checkStart = true;
    start = 'b';
    Enes100.println("We are starting at B zone"); // just for visual reference
  }

  if (checkStart = true && atMission == false) { // basicallly run this stuff until we are at the zone
    if (start == 'a') { // if we are at A
      fullTurn(-1.5, 0.15);
      // WIP for reallignment
      
      fullStop();
      // now we are facing down
      stopAt(16.5);
      // read first row of flames
      numFlames();
      delay(2000);
      stopAt(2);
      atMission = true;
      Enes100.println("We are at mission site");
    } else if (start == 'b') { // if we are at B
      fullTurn(1.5, 0.15);
      // WIP for reallignment
      fullStop();
      // now we are facing up
      stopAt(16.5);
      // Read first row of flames
      numFlames();
      delay(2000);
      stopAt(2);
      atMission = true;
      Enes100.println("We are at mission site");
    }
  } 
  
  if (atMission == true && missionComplete == false) { // now we are at mission site: measure num flames, and figure out topography. Trigger servo motor to extinguish flames
    // stop a certain distance away for detecting first row of fire flames (to be measured)
    Enes100.println("Starting Mission");
    numFlames(); // after this is finished executing we should be flush with the platform
    
    Enes100.mission(NUM_CANDLES, totalFlames);
    Enes100.println("Done with flames"); 
    topography(); // identifies topography 
    
    // call topography(); //detects topography type once flush with platform 
    // trigger axle motor for certain period of time (to be measured)
  axleForward();
  delay(26000);
  axleStop();
    
    delay(1000);
    missionComplete = true;
    Enes100.print("Finished Mission");
  }

  if (atMission == true && missionComplete == true && readyForObsZone == false) { // once we finish mission site prepare for obstacle zone
    backward(); // will change this up later
    delay(2000);
    fullStop();
    fullTurn(0, 0.15);
    driveRightByX(0.5);
    fullTurn(1.5, 0.15);
    driveUpToY(1.4); // OTV will always start at the top of the obstacle zone
    fullTurn(0, 0.15);
    readyForObsZone = true;
    Enes100.println("Ready for obstacle zone!");
  }

  if (atMission == true && missionComplete == true && readyForObsZone == true && nearGoal == false) {
    // obstacle zone will worry about this in class
    senseObstacle();
    Enes100.println("Done obstacle zone");
  }

  if (atMission == true && missionComplete == true && readyForObsZone == true && nearGoal == true && epicVictory == false) {
    // assuming we are at the goal zone
    Enes100.updateLocation();
    Enes100.println("We are in open zone");
    fullTurn(-1.5, 0.15);
    getPosition();
    Enes100.println("Driving down to log");
    driveDownToY(0.75);
    fullTurn(0, 0.15);
    getPosition();
    Enes100.println("Going over log");
    // fuck it we going over the log
    driveRightToX(3.7);
    getPosition();

    epicVictory = true;
    Enes100.print("We have arrived at the victory zone!");
  }

  if (atMission == true && missionComplete == true && readyForObsZone == true && nearGoal == true && epicVictory == true) {
    Enes100.println("EPIC VICTORY. LEMON BARS DRAWS NEAR!");
    victoryDance();
  }
}

/* Beginning of the Navigation Related Methods */

void leftBackward() {
  digitalWrite(in1a, HIGH);
  digitalWrite(in1b, LOW);
  rotDirection = 1;
}

void leftForward() {
  digitalWrite(in1a, LOW);
  digitalWrite(in1b, HIGH);
  rotDirection = 0;
}

void rightForward() {
  digitalWrite(in2a, LOW);
  digitalWrite(in2b, HIGH);
  rotDirection = 0;
}

void rightBackward() {
  digitalWrite(in2a, HIGH);
  digitalWrite(in2b, LOW);
  rotDirection = 1;
}

void forward() {
  rightForward();
  leftForward();
}

void backward() {
  rightBackward();
  leftBackward();
}

void leftTurn() {
  rightForward();
  leftBackward();
}

void rightTurn() {
  rightBackward();
  leftForward();
}

void fullStop(){
  digitalWrite(in1a, LOW);
  digitalWrite(in1b, LOW);
  digitalWrite(in2a, LOW);
  digitalWrite(in2b, LOW);  
}

void getPosition() {
  Enes100.updateLocation();
  Enes100.print("OSV is at ( x = ");
    Enes100.print(Enes100.location.x);
    Enes100.print(", y = ");
    Enes100.print(Enes100.location.y);
    Enes100.print(", theta = ");
    Enes100.print(Enes100.location.theta);
    Enes100.println(")");
}

void stopAt(float cm) {
  distanceF = sonarF.ping_cm();
  while (distanceF > cm) {
    distanceF = sonarF.ping_cm();
    forward();
  }
  fullStop();
}

// Turns the OTV to a specified theta heading, called goal
// with a given error amount, called error
void fullTurn(float goal, float error) {
  
  // Loop runs until the return statement inside
  while(true) {
       Enes100.updateLocation();
      // Checks if the OTV heading is less than the desired heading
      // This would mean it is pointing to the right of the desired direction
    if (Enes100.location.theta < (goal - error)) {
      leftTurn();

        // Checks if the OTV heading is greater than the desired heading
        // This would mean it is pointing to the left of the desired direction
    } else if (Enes100.location.theta > (goal + error)) {
      rightTurn();

      
    } else {

        //If neither of these conditions are satisfied, then it must be within the tolerance
      fullStop();
        //returns from the function and the OTV is now at the correct heading
      return;  
    }
    
  }
}

// Drives down to a certain y position
//Must start oriented down
void driveDownToY(float Y) {
  while(true) {
    Enes100.updateLocation();

    if (Enes100.location.y > Y) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

// Drives Up by a to a certain y position
//Must start oriented up
void driveUpToY(float Y) {
  while(true) {
    Enes100.updateLocation();
    Enes100.println(Enes100.location.y);
    if (Enes100.location.y <= Y) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

//Drives down by a certain distance in meters
//Must start oriented down
void driveDownByY(float distance) {
  Enes100.updateLocation();
  float current = Enes100.location.y;
  while(true) {
    Enes100.updateLocation();
    
    if (Enes100.location.y > current - distance ) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

//Drives up by a certain distance in meters
//Must start oriented up
void driveUpByY(float distance) {
  Enes100.updateLocation();
  float current = Enes100.location.y;
  while(true) {
    Enes100.updateLocation();
    
    if (Enes100.location.y < current + distance ) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

// Drives left to a certain x position
//Must start oriented left
void driveLeftToX(float X) {
  while(true) {
    Enes100.updateLocation();

    if (Enes100.location.x > X) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

// Drives right by a to a certain x position
//Must start oriented right
void driveRightToX(float X) {
  while(true) {
    Enes100.updateLocation();

    if (Enes100.location.x < X) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

//Drives left by a certain distance in meters
//Must start oriented left
void driveLeftByX(float distance) {
  Enes100.updateLocation();
  float current = Enes100.location.x;
  while(true) {
    Enes100.updateLocation();
    
    if (Enes100.location.x > current - distance ) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

//Drives right by a certain distance in meters
//Must start oriented right
void driveRightByX(float distance) {
  Enes100.updateLocation();
  float current = Enes100.location.x;
  while(true) {
    Enes100.updateLocation();
    
    if (Enes100.location.x < current + distance ) {
    
        forward();
    } else {
        fullStop();
        return;
    }
  }
}

/* Beginning of the Sensor Related code */

/*
 * detects the number of flames lit on the platform. Runs only once (after the platform is lined up and we are a certain distance
 * away from it. 
 * currently edited for MS6. Old version can be found in file.
 */
void numFlames() {

  // detect first row of flames
  fireValR = digitalRead(firePinR);
  delay(2000); // delay 2 seconds
  fireValL = digitalRead(firePinL);
  delay(2000); // delay 2 seconds

  // add right and left fire sensor vals to totalFlames
  if(fireValR == 0) {
    Serial.println("Row 1 right side fire detected");
    totalFlames += 1;
  } 
  if(fireValL == 0) {
    Serial.println("Row 1 left side fire detected");
    totalFlames += 1;
  }
  
  /*
  // detect for second row of flames
  fireValR = 1; 
  fireValL = 1; 
  fireValR = digitalRead(firePinR);
  delay(2000); // delay 2 seconds
  fireValL = digitalRead(firePinL);
  delay(2000); // delay 2 seconds

  // add right and left fire sensor vals to totalFlames
  if(fireValR == 0) {
    Serial.println("Row 2 right side fire detected");
    totalFlames += 1;
  }
  if(fireValL == 0) {
    Serial.println("Row 2 left side fire detected");
    totalFlames += 1;
  }

   */
  
  Serial.print("total # flames: ");
  Serial.println(totalFlames);

  Enes100.print("total # flames: ");
  Enes100.println(totalFlames);
}

/*
 * detects obstacles using front ultrasonic sensor and directs vehicle to turn right. 
 * Repeats while we are in portion of arena where there are obstacles. 
 * Must be called once we are aligned in the top left corner of the arena
 * not tested & not finished - unimplemented areas 
 */
void senseObstacle() {
   float beforeLog = 2.6; //X position where we will be past all obstacles

   
   // get distance for front ultrasonic distance sensor
   distanceF = sonarF.ping_cm();
   Enes100.updateLocation();
   Enes100.println(distanceF);
   // if front < certain distance, turn right
   if (distanceF < minObstacleDist && (distanceF != 0)) {
     Enes100.updateLocation();
     if (Enes100.location.y < 0.75) {
      Enes100.println("Obstacle detected within min. distance! Turning left ..."); 
      Serial.println("obstacle detected within min. distance!");
      fullTurn(1.4, 0.15);
      getPosition();
      driveUpToY(1.4);
      fullTurn(0, 0.15);

      forward();
      Enes100.println("Back on track, moving forwards"); 
      Enes100.updateLocation();
      getPosition();
     } else {
      Enes100.updateLocation();
      if (Enes100.location.x > 1.4) {
        Enes100.println("Obstacle detected within min. distance! Turning right ...");
        Serial.println("obstacle detected within min. distance!");
        fullTurn(-1.4, 0.15);
        driveDownByY(0.6);
        fullTurn(0, 0.15);
      } else {
        Enes100.println("Obstacle detected within min. distance! Turning right ...");
        Serial.println("obstacle detected within min. distance!");
        fullTurn(-1.5, 0.15);
        driveDownByY(0.6);
        fullTurn(0, 0.15);
      }

      forward();
      Enes100.println("Back on track, moving forwards"); 
      getPosition();
     }
     
   } 

   // if no obstacle is detected, keep moving forward in small increments 
   else {
     forward();
     // check curr position and last position w/ aruco marker
     // maybe add some code here that compares current position to last position, and if curr = last then back up and turn right again because we've hit an obstacle and the sensors failed
     
     Enes100.updateLocation();
     getPosition();
     if (Enes100.location.x > beforeLog) {
      fullStop();
      getPosition();
      nearGoal = true;
      return;
     }
   } 
}

/*
 * Identifies topography based on button (L, M, R) states. Runs only once.  
 * not tested
 */
void topography() {
  buttonL = digitalRead(buttonLPin);
  buttonR = digitalRead(buttonRPin);

  Serial.print(buttonL);
  Serial.print("      ");
  Serial.println(buttonR); 
  
  // if L = off & R = off: platform C
  if (buttonL == 1 && buttonR == 1) {
    Serial.println("Platform C");
    Enes100.mission(TOPOGRAPHY, TOP_C);
    Enes100.println("Topography C");
    // Enes100.println("Platform C"); 
  } else if (buttonL == 0 && buttonR == 1) { // if L = on & right = off: Platform B
    Serial.println("Platform B");
    Enes100.mission(TOPOGRAPHY, TOP_B);
        Enes100.println("Topography B");

    // Enes100.println("Platform B"); 
  } else if (buttonL == 1 and buttonR == 0) { // if L = off & right = on: Platform A
    Serial.println("Platform A"); 
    Enes100.mission(TOPOGRAPHY, TOP_A);
    Enes100.println("Topography A");

    // Enes100.println("Platform A"); 
  } else { // if both buttons are pressed, randomly guess platform A
    Serial.println("Random: Platform A");
    // Enes100.println("Platform A");
    Enes100.println("Topography A"); 
  }
}

/*
 * Checks if the OSV is aligned based on bottom buttons.
 * not tested
 */
void alignOSV() {
  while (!aligned) {
    buttonLA = digitalRead(buttonLApin);
    buttonRA = digitalRead(buttonRApin);
  
    // if both buttons aligned, OSV is aligned
    // if left button pressed, right button not pressed, we need to shift more left 
    // if right button pressed, left button not pressed, we need to shift more right
    // continue until we are aligned

    // if both sides are aligned, we are 100% aligned 
    if (buttonLA == 0 && buttonRA == 0) {
      aligned = true;
    } else if (buttonLA == 0 && buttonRA == 1) { // right side not aligned (right button not pressed)
      Serial.println("Right side not aligned - shifting left");
      // Vincent - shift left
      backward();
      delay(500);
      leftTurn();
      delay(200);
      forward();
      delay(500);
      fullStop();
      
    } else if (buttonLA == 1 && buttonRA == 0) { // left side not aligned (left button not pressed)
      Serial.println("Left side not aligned - shifting right");
      // Vincent - shift right
    } else { // if neither side is aligned...try again?
      Serial.println("Neither side aligned, try again?");
      // Vincent - maybe back up and just turn left OR right
    }
  }
}

/* triggers and calculates distance for all 3 ultrasonic sensors 
 * potentially may ignore outliers 
 */
void trigAllUltra() {
  distanceF = sonarF.ping_cm(); // meausres front dist. in cm
  delay(1000); // delay 1 second
  distanceR = sonarR.ping_cm(); // measures right dist. in cm
  delay(1000);
  distanceL = sonarL.ping_cm(); // measures left dist. in cm
  delay(1000);

  // print distances for F, R, L
  Serial.print("distanceF: ");
  Serial.print(distanceF);
  Serial.println(" cm"); 
  Serial.print("distanceR: ");
  Serial.print(distanceR); 
  Serial.println(" cm"); 
  Serial.print("distanceL: ");
  Serial.print(distanceL);
  Serial.println(" cm");  
}

// turn axle on 
void axleForward() {
  digitalWrite(axleA, LOW);
  digitalWrite(axleB, HIGH);
  rotDirection = 0;
}

// turn axle motor off
void axleStop()
{
  digitalWrite(axleA, LOW);
  digitalWrite(axleB, LOW); 
}


/* For fun stuff */
void victoryDance() {
  


}
