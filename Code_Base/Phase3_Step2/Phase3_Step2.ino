// --- PHASE 3: FINAL FIRMWARE (MATCHED TO YOUR WORKING DIRECTION) ---
// Features: Vector Control <Vx,Vy,Omega>, Odometry, Fixed Directions

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

// *** CALIBRATION ***
const double CM_PER_TICK = 0.07033;  // <--- YOUR VERIFIED VALUE
const double ROBOT_RADIUS_CM = 26.0; // Nexus Standard Radius

// *** VARIABLES ***
volatile long fl_ticks=0, fr_ticks=0, bl_ticks=0, br_ticks=0;
long prev_fl=0, prev_fr=0, prev_bl=0, prev_br=0;
int last_fl=0, last_fr=0, last_bl=0, last_br=0;

unsigned long last_time = 0;
double posX = 0.0, posY = 0.0, theta = 0.0;
float target_vx = 0, target_vy = 0, target_omega = 0;

// FUNCTION DECLARATIONS
void pollEncoders();
void parseCommand(String data);
void updateOdometry(long d_fl, long d_fr, long d_bl, long d_br);
void setMotorSpeeds(float vx, float vy, float omega);
void runMotor(int pwmPin, int dirPin, float speed, bool isRightSide);

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(10); 

  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);

  last_fl = digitalRead(FL_ENC_A); last_fr = digitalRead(FR_ENC_A);
  last_bl = digitalRead(BL_ENC_A); last_br = digitalRead(BR_ENC_A);
}

void loop() {
  // 1. READ SERIAL COMMANDS <Vx,Vy,Omega>
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      String data = Serial.readStringUntil('>');
      parseCommand(data);
    }
  }

  // 2. CONTROL LOOP (50ms)
  unsigned long curr_time = millis();
  if (curr_time - last_time >= 50) {
    long d_fl = fl_ticks - prev_fl; prev_fl = fl_ticks;
    long d_fr = fr_ticks - prev_fr; prev_fr = fr_ticks;
    long d_bl = bl_ticks - prev_bl; prev_bl = bl_ticks;
    long d_br = br_ticks - prev_br; prev_br = br_ticks;

    updateOdometry(d_fl, d_fr, d_bl, d_br);

    // FEEDBACK: <X, Y, Theta>
    Serial.print("<");
    Serial.print(posX); Serial.print(",");
    Serial.print(posY); Serial.print(",");
    Serial.print(theta);
    Serial.println(">");

    setMotorSpeeds(target_vx, target_vy, target_omega);
    last_time = curr_time;
  }
  
  pollEncoders();
}

// --- HELPERS ---
void parseCommand(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.lastIndexOf(',');
  if (firstComma > 0 && secondComma > 0) {
    target_vx = data.substring(0, firstComma).toFloat();
    target_vy = data.substring(firstComma + 1, secondComma).toFloat();
    target_omega = data.substring(secondComma + 1).toFloat();
  }
}

void setMotorSpeeds(float vx, float vy, float omega) {
  float rot = omega * ROBOT_RADIUS_CM; 
  float fl = vx - vy - rot;
  float fr = vx + vy + rot;
  float bl = vx + vy - rot;
  float br = vx - vy + rot;

  // Pass "true" for Right side motors to handle direction correctly
  runMotor(FL_PWM, FL_DIR, fl, false); // Left
  runMotor(FR_PWM, FR_DIR, fr, true);  // Right
  runMotor(BL_PWM, BL_DIR, bl, false); // Left
  runMotor(BR_PWM, BR_DIR, br, true);  // Right
}

void runMotor(int pwmPin, int dirPin, float speed, bool isRightSide) {
  if (abs(speed) < 2.0) speed = 0; 
  int pwm = constrain((int)(speed * 5), -255, 255); 
  
  // Logic from your Working Code:
  // Left Side: Positive Speed -> LOW
  // Right Side: Positive Speed -> HIGH
  
  if (isRightSide) {
     if (pwm > 0) digitalWrite(dirPin, HIGH); 
     else digitalWrite(dirPin, LOW);
  } else {
     if (pwm > 0) digitalWrite(dirPin, LOW); 
     else digitalWrite(dirPin, HIGH);
  }
  
  analogWrite(pwmPin, abs(pwm));
}

void updateOdometry(long fl, long fr, long bl, long br) {
  double dist_fl = fl * CM_PER_TICK;
  double dist_fr = fr * CM_PER_TICK;
  double dist_bl = bl * CM_PER_TICK;
  double dist_br = br * CM_PER_TICK;

  double delta_x = (dist_fl + dist_fr + dist_bl + dist_br) / 4.0;
  double delta_y = (-dist_fl + dist_fr + dist_bl - dist_br) / 4.0; 
  double delta_theta = (-dist_fl + dist_fr - dist_bl + dist_br) / (4.0 * ROBOT_RADIUS_CM);

  posX += delta_x * cos(theta) - delta_y * sin(theta);
  posY += delta_x * sin(theta) + delta_y * cos(theta);
  theta += delta_theta;
}

void pollEncoders() {
  // FL
  int curr_fl = digitalRead(FL_ENC_A);
  if (curr_fl != last_fl && curr_fl == HIGH) { 
    if(digitalRead(FL_ENC_B)) fl_ticks++; else fl_ticks--; 
  }
  last_fl = curr_fl;

  // FR
  int curr_fr = digitalRead(FR_ENC_A);
  if (curr_fr != last_fr && curr_fr == HIGH) {
    if(digitalRead(FR_ENC_B)) fr_ticks++; else fr_ticks--; 
  }
  last_fr = curr_fr;

  // BL
  int curr_bl = digitalRead(BL_ENC_A);
  if (curr_bl != last_bl && curr_bl == HIGH) {
    if(digitalRead(BL_ENC_B)) bl_ticks++; else bl_ticks--; 
  }
  last_bl = curr_bl;

  // BR
  int curr_br = digitalRead(BR_ENC_A);
  if (curr_br != last_br && curr_br == HIGH) {
    if(digitalRead(BR_ENC_B)) br_ticks++; else br_ticks--; 
  }
  last_br = curr_br;
}
