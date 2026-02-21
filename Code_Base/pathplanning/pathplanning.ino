// --- MECANUM PID FIRMWARE WITH SERIAL INPUT ---

#include <Arduino.h>

// *** PIN DEFINITIONS (MATCHING YOUR SETUP) ***
const int FL_PWM = 3;   const int FL_DIR = 2; 
const int FL_ENC_A = A2; const int FL_ENC_B = A3;

const int FR_PWM = 11;  const int FR_DIR = 12;
const int FR_ENC_A = A5; const int FR_ENC_B = A4;

const int BL_PWM = 10;   const int BL_DIR = 7; 
const int BL_ENC_A = A0; const int BL_ENC_B = A1;

const int BR_PWM = 9;   const int BR_DIR = 8;
const int BR_ENC_A = 5;  const int BR_ENC_B = 4;

// *** TUNING PARAMETERS ***
// Calibrate this value exactly using your 60cm test!
const double CM_PER_TICK = 0.07033;  
float Kp = 6.0; 
float Ki = 2.0;
float Kd = 0.0; // Added D term just in case, usually 0 is fine for velocity

// *** ROBOT GEOMETRY (IN METERS) ***
const double LX = 0.235; // Half Length
const double LY = 0.15;  // Half Width
const double WHEEL_RADIUS = 0.05; 

// *** VARIABLES ***
volatile long fl_ticks=0, fr_ticks=0, bl_ticks=0, br_ticks=0;
long prev_fl=0, prev_fr=0, prev_bl=0, prev_br=0;
double int_fl=0, int_fr=0, int_bl=0, int_br=0; 
int last_fl=0, last_fr=0, last_bl=0, last_br=0;
unsigned long last_pid_time = 0;

// Target Speeds (cm/s) calculated from Serial Input
double target_fl = 0;
double target_fr = 0;
double target_bl = 0;
double target_br = 0;

// Serial Parsing
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200); // Fast communication
  
  // Motor Pins
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  // Encoder Pins
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);

  Serial.println("READY");
}

void loop() {
  pollEncoders();
  recvWithStartEndMarkers();
  if (newData) {
      parseData();
  }

  unsigned long curr_time = millis();
  if (curr_time - last_pid_time >= 50) { // 20Hz PID Loop
    double dt = (curr_time - last_pid_time) / 1000.0;
    last_pid_time = curr_time;

    // Calculate current speeds
    double speed_fl = ((fl_ticks - prev_fl) * CM_PER_TICK) / dt;
    double speed_fr = ((fr_ticks - prev_fr) * CM_PER_TICK) / dt;
    double speed_bl = ((bl_ticks - prev_bl) * CM_PER_TICK) / dt;
    double speed_br = ((br_ticks - prev_br) * CM_PER_TICK) / dt;

    prev_fl = fl_ticks; prev_fr = fr_ticks;
    prev_bl = bl_ticks; prev_br = br_ticks;

    // Run PID (Note: Left side inverted logic passed as boolean)
    computePID(speed_fl, target_fl, int_fl, FL_PWM, FL_DIR, dt, true);
    computePID(speed_fr, target_fr, int_fr, FR_PWM, FR_DIR, dt, false);
    computePID(speed_bl, target_bl, int_bl, BL_PWM, BL_DIR, dt, true);
    computePID(speed_br, target_br, int_br, BR_PWM, BR_DIR, dt, false);
  }
}

// --- SERIAL PARSING (<Vx, Vy, Omega>) ---
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) { ndx = numChars - 1; }
            } else {
                receivedChars[ndx] = '\0'; // terminate string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx; 
    
    // Expecting: <Vx, Vy, Omega> in m/s and rad/s
    strtokIndx = strtok(receivedChars, ",");
    float vx = atof(strtokIndx); 
    
    strtokIndx = strtok(NULL, ",");
    float vy = atof(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    float omega = atof(strtokIndx);

    // INVERSE KINEMATICS (Convert m/s to cm/s for PID)
    // Formula: Wheel = Vx - Vy - (Lx+Ly)*Omega
    float geom = LX + LY;
    
    // Convert to cm/s because your PID is tuned for cm/s
    target_fl = (vx - vy - (geom * omega)) * 100.0;
    target_fr = (vx + vy + (geom * omega)) * 100.0;
    target_bl = (vx + vy - (geom * omega)) * 100.0;
    target_br = (vx - vy + (geom * omega)) * 100.0;

    newData = false;
}

// --- PID CONTROLLER ---
void computePID(double current, double target, double &integral, int pwmPin, int dirPin, double dt, bool isLeft) {
  double error = target - current;
  
  // Anti-windup
  if(abs(target) < 0.1) { integral = 0; error = 0; } // Stop condition
  else { integral += (error * dt); }
  integral = constrain(integral, -100, 100);

  double output = (Kp * error) + (Ki * integral);

  // Deadzone / Stiction Fix (Adjust 60 if needed)
  if (abs(target) > 0.1) {
     if (output > 0 && output < 60) output = 60;
     if (output < 0 && output > -60) output = -60;
  } else {
     output = 0; // Hard stop
  }

  int pwm = constrain((int)output, -255, 255);

  // Direction Logic
  if (pwm >= 0) {
    digitalWrite(dirPin, isLeft ? LOW : HIGH); // Forward
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(dirPin, isLeft ? HIGH : LOW); // Backward
    analogWrite(pwmPin, abs(pwm));
  }
}

// --- ENCODER POLLING (Keep this fast) ---
void pollEncoders() {
  // Same as your original code
  int c;
  // FL
  c = digitalRead(FL_ENC_A); if(c!=last_fl && c==HIGH) { if(digitalRead(FL_ENC_B)) fl_ticks--; else fl_ticks++; } last_fl=c;
  // FR
  c = digitalRead(FR_ENC_A); if(c!=last_fr && c==HIGH) { if(digitalRead(FR_ENC_B)) fr_ticks++; else fr_ticks--; } last_fr=c;
  // BL
  c = digitalRead(BL_ENC_A); if(c!=last_bl && c==HIGH) { if(digitalRead(BL_ENC_B)) bl_ticks--; else bl_ticks++; } last_bl=c;
  // BR
  c = digitalRead(BR_ENC_A); if(c!=last_br && c==HIGH) { if(digitalRead(BR_ENC_B)) br_ticks++; else br_ticks--; } last_br=c;
}
