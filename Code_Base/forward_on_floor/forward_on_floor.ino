/*
  Day 5: Full Omni-Motion Test
  Sequence: Forward -> Backward -> Strafe Right -> Strafe Left -> Spin Left -> Spin Right
*/

// --- Pin Definitions ---
// Front Left
int FL_PWM = 3;   int FL_DIR = 2;   
// Front Right
int FR_PWM = 11;  int FR_DIR = 12;
// Back Right
int BR_PWM = 9;   int BR_DIR = 8;
// Back Left
int BL_PWM = 10;  int BL_DIR = 7;

void setup() {
  // Set all pins as OUTPUT
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);

  Serial.begin(9600);
  Serial.println("--- STARTING OMNI TEST ---");
  // SAFETY DELAY: 3 Seconds to step back
  delay(3000); 
}

void loop() {
  int runTime = 1000; // Move for 1 second
  int stopTime = 500; // Rest for 0.5 second

  // 1. Move Forward
  moveRobot(1, 1, 1, 1); 
  delay(runTime);
  stopRobot();
  delay(stopTime);

  // 2. Move Backward
  moveRobot(-1, -1, -1, -1);
  delay(runTime);
  stopRobot();
  delay(stopTime);

  // 3. Strafe Right (Crab Walk Right)
  // FL & BR = Forward, FR & BL = Backward
  moveRobot(1, -1, 1, -1);
  delay(runTime);
  stopRobot();
  delay(stopTime);

  // 4. Strafe Left (Crab Walk Left)
  // FL & BR = Backward, FR & BL = Forward
  moveRobot(-1, 1, -1, 1);
  delay(runTime);
  stopRobot();
  delay(stopTime);

  // 5. Spin Left
  moveRobot(-1, 1, 1, -1); // Left side back, Right side fwd
  delay(runTime);
  stopRobot();
  delay(stopTime);

  // 6. Spin Right
  moveRobot(1, -1, -1, 1); // Left side fwd, Right side back
  delay(runTime);
  stopRobot();
  
  // Long pause before repeating
  delay(3000);
}

// --- Universal Motor Helper ---
// Inputs: 1 (Fwd), -1 (Back), 0 (Stop)
void moveRobot(int fl, int fr, int br, int bl) {
  int speed = 130; // Increased speed slightly to overcome floor friction
  
  setOneMotor(FL_PWM, FL_DIR, fl, speed);
  setOneMotor(FR_PWM, FR_DIR, fr, speed);
  setOneMotor(BR_PWM, BR_DIR, br, speed);
  setOneMotor(BL_PWM, BL_DIR, bl, speed);
}

void setOneMotor(int pwm, int dir, int input, int spd) {
  if (input == 1) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, spd);
  }
  else if (input == -1) {
    digitalWrite(dir, LOW);
    analogWrite(pwm, spd);
  }
  else {
    analogWrite(pwm, 0);
  }
}

void stopRobot() {
  moveRobot(0, 0, 0, 0);
}
