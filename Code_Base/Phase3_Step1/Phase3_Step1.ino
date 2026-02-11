// --- SERIAL CONTROLLED ROBOT (Arduino Monitor Version) ---
// Type a number (cm) in Serial Monitor -> Robot moves that distance.

#include <Arduino.h>

// *** PIN DEFINITIONS ***
const int FL_PWM = 3;    const int FL_DIR = 2; 
const int FL_ENC_A = A2; const int FL_ENC_B = A3;

const int FR_PWM = 11;   const int FR_DIR = 12;
const int FR_ENC_A = A5; const int FR_ENC_B = A4;

const int BL_PWM = 10;   const int BL_DIR = 7; 
const int BL_ENC_A = A0; const int BL_ENC_B = A1;

const int BR_PWM = 9;    const int BR_DIR = 8;
const int BR_ENC_A = 5;  const int BR_ENC_B = 4;

// *** YOUR CALIBRATION ***
// REPLACE THIS NUMBER with your calculated value (e.g., 0.07033)
const double CM_PER_TICK = 0.07033;  

// PID SETTINGS
float Kp = 6.0; 
float Ki = 2.0;

// VARIABLES
volatile long fl_ticks=0, fr_ticks=0, bl_ticks=0, br_ticks=0;
long prev_fl=0, prev_fr=0, prev_bl=0, prev_br=0;
double int_fl=0, int_fr=0, int_bl=0, int_br=0; 
int last_fl=0, last_fr=0, last_bl=0, last_br=0;

// FUNCTION DECLARATIONS
void pollEncoders();
void driveDistance(float distance_cm);
void computePID(double target_spd, double current_spd, double &integral, int pwmPin, int dirPin, double dt, bool isLeft);

void setup() {
  Serial.begin(115200); // Make sure Monitor is set to 115200
  Serial.setTimeout(50); 

  // MOTOR PINS
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  // ENCODER PINS
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);

  Serial.println("--- ROBOT READY ---");
  Serial.println("Type a distance (e.g., 60) and press Enter:");
}

void loop() {
  // Listen for user input from Serial Monitor
  if (Serial.available() > 0) {
    // Read the number typed by user
    float target_cm = Serial.parseFloat(); 
    
    // Clear buffer (remove "Enter" key characters)
    while(Serial.available()) { Serial.read(); }

    if (target_cm > 0) {
      Serial.print("Command Received: GO "); 
      Serial.print(target_cm); 
      Serial.println(" cm");
      
      driveDistance(target_cm);
      
      Serial.println("--- ARRIVED. Ready for next command. ---");
    }
  }
  
  pollEncoders(); // Keep counting in background
}

// --- DRIVING LOGIC ---
void driveDistance(float target_cm) {
  // 1. Reset counters for new move
  fl_ticks=0; fr_ticks=0; bl_ticks=0; br_ticks=0;
  prev_fl=0; prev_fr=0; prev_bl=0; prev_br=0;
  int_fl=0; int_fr=0; int_bl=0; int_br=0;
  
  unsigned long last_time = millis();
  bool reached = false;
  double target_speed = 30.0; // Speed in cm/s

  // 2. Drive Loop
  while (!reached) {
    pollEncoders();

    unsigned long curr_time = millis();
    if (curr_time - last_time >= 50) { // PID every 50ms
      double dt = (curr_time - last_time) / 1000.0;
      last_time = curr_time;

      // Calculate Distance
      long avg_ticks = (abs(fl_ticks) + abs(fr_ticks) + abs(bl_ticks) + abs(br_ticks)) / 4;
      double current_dist = avg_ticks * CM_PER_TICK;

      // Stop Condition
      if (current_dist >= target_cm) {
        reached = true;
        break; 
      }

      // Run PID for all 4 motors
      long diff_fl = fl_ticks - prev_fl; prev_fl = fl_ticks;
      double speed_fl = (diff_fl * CM_PER_TICK) / dt;
      computePID(target_speed, speed_fl, int_fl, FL_PWM, FL_DIR, dt, true);

      long diff_fr = fr_ticks - prev_fr; prev_fr = fr_ticks;
      double speed_fr = (diff_fr * CM_PER_TICK) / dt;
      computePID(target_speed, speed_fr, int_fr, FR_PWM, FR_DIR, dt, false);

      long diff_bl = bl_ticks - prev_bl; prev_bl = bl_ticks;
      double speed_bl = (diff_bl * CM_PER_TICK) / dt;
      computePID(target_speed, speed_bl, int_bl, BL_PWM, BL_DIR, dt, true);

      long diff_br = br_ticks - prev_br; prev_br = br_ticks;
      double speed_br = (diff_br * CM_PER_TICK) / dt;
      computePID(target_speed, speed_br, int_br, BR_PWM, BR_DIR, dt, false);
      
      // Optional: Print progress
      // Serial.print("Dist: "); Serial.println(current_dist);
    }
  }

  // 3. Stop Motors
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
  analogWrite(BL_PWM, 0); analogWrite(BR_PWM, 0);
  delay(500); // Pause before accepting new commands
}

// --- HELPER FUNCTIONS ---
void pollEncoders() {
  int curr_fl = digitalRead(FL_ENC_A);
  if (curr_fl != last_fl && curr_fl == HIGH) { 
    if(digitalRead(FL_ENC_B)) fl_ticks--; else fl_ticks++; 
  }
  last_fl = curr_fl;

  int curr_fr = digitalRead(FR_ENC_A);
  if (curr_fr != last_fr && curr_fr == HIGH) {
    if(digitalRead(FR_ENC_B)) fr_ticks++; else fr_ticks--;
  }
  last_fr = curr_fr;

  int curr_bl = digitalRead(BL_ENC_A);
  if (curr_bl != last_bl && curr_bl == HIGH) {
    if(digitalRead(BL_ENC_B)) bl_ticks--; else bl_ticks++;
  }
  last_bl = curr_bl;

  int curr_br = digitalRead(BR_ENC_A);
  if (curr_br != last_br && curr_br == HIGH) {
    if(digitalRead(BR_ENC_B)) br_ticks++; else br_ticks--;
  }
  last_br = curr_br;
}

void computePID(double target_spd, double current_spd, double &integral, int pwmPin, int dirPin, double dt, bool isLeft) {
  double error = target_spd - current_spd;
  integral += (error * dt);
  integral = constrain(integral, -100, 100);
  double output = (Kp * error) + (Ki * integral);
  
  if (output > 0 && output < 60) output = 60;
  if (output < 0 && output > -60) output = -60;
  
  int pwm = constrain((int)output, -255, 255);
  
  if (pwm > 0) {
    if (isLeft) digitalWrite(dirPin, LOW); else digitalWrite(dirPin, HIGH);       
    analogWrite(pwmPin, pwm);
  } else {
    if (isLeft) digitalWrite(dirPin, HIGH); else digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, abs(pwm));
  }
}
