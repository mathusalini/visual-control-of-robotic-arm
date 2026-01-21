// --- STEP 3.5: INVERTED LEFT SIDE PID ---

// *** CHECK YOUR PINS ***
// Make sure FL_PWM is on a PWM pin (3, 5, 6, 9, 10, 11). Pin 2 is usually NOT PWM on Uno.
const int FL_PWM = 3;   const int FL_DIR = 2; // I swapped 2 & 3 back for safety
const int FL_ENC_A = A2; const int FL_ENC_B = A3;

const int FR_PWM = 11;  const int FR_DIR = 12;
const int FR_ENC_A = A5; const int FR_ENC_B = A4;

const int BL_PWM = 10;   const int BL_DIR = 7; // I verified this is PWM capable
const int BL_ENC_A = A0; const int BL_ENC_B = A1;

const int BR_PWM = 9;   const int BR_DIR = 8;
const int BR_ENC_A = 5;  const int BR_ENC_B = 4;

// TUNING
const double CM_PER_TICK = 31.4 / 980.0;
double target_speed = 30.0; 
float Kp = 6.0; 
float Ki = 2.0;

// VARIABLES
volatile long fl_ticks=0, fr_ticks=0, bl_ticks=0, br_ticks=0;
long prev_fl=0, prev_fr=0, prev_bl=0, prev_br=0;
double int_fl=0, int_fr=0, int_bl=0, int_br=0; 
int last_fl=0, last_fr=0, last_bl=0, last_br=0;
unsigned long last_pid_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  // ENCODERS
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);

  Serial.println("--- PID START (LEFT INVERTED) ---");
  delay(1000);
}

void loop() {
  pollEncoders();

  unsigned long curr_time = millis();
  if (curr_time - last_pid_time >= 50) { 
    double dt = (curr_time - last_pid_time) / 1000.0;
    last_pid_time = curr_time;

    // --- 1. FRONT LEFT (INVERTED = TRUE) ---
    long diff_fl = fl_ticks - prev_fl; prev_fl = fl_ticks;
    double speed_fl = (diff_fl * CM_PER_TICK) / dt;
    // Note: We pass 'true' because this is a Left wheel
    computePID(speed_fl, int_fl, FL_PWM, FL_DIR, dt, true); 

    // --- 2. FRONT RIGHT (INVERTED = FALSE) ---
    long diff_fr = fr_ticks - prev_fr; prev_fr = fr_ticks;
    double speed_fr = (diff_fr * CM_PER_TICK) / dt;
    computePID(speed_fr, int_fr, FR_PWM, FR_DIR, dt, false);

    // --- 3. BACK LEFT (INVERTED = TRUE) ---
    long diff_bl = bl_ticks - prev_bl; prev_bl = bl_ticks;
    double speed_bl = (diff_bl * CM_PER_TICK) / dt;
    computePID(speed_bl, int_bl, BL_PWM, BL_DIR, dt, true);

    // --- 4. BACK RIGHT (INVERTED = FALSE) ---
    long diff_br = br_ticks - prev_br; prev_br = br_ticks;
    double speed_br = (diff_br * CM_PER_TICK) / dt;
    computePID(speed_br, int_br, BR_PWM, BR_DIR, dt, false);
    
    // Debug
    Serial.print("Target:30,Actual_FL:");
    Serial.println(speed_fl);
  }
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

// 2. MODIFIED PID FUNCTION (Handles Inversion)
void computePID(double current_speed, double &integral, int pwmPin, int dirPin, double dt, bool isLeft) {
  
  double error = target_speed - current_speed;
  integral += (error * dt);
  integral = constrain(integral, -100, 100);

  double output = (Kp * error) + (Ki * integral);

  // Deadzone Fix
  if (output > 0 && output < 60) output = 60;
  if (output < 0 && output > -60) output = -60;

  int pwm = (int)output;
  pwm = constrain(pwm, -255, 255);

  // --- DIRECTION LOGIC ---
  if (pwm > 0) {
    // STARTING FORWARD
    if (isLeft) {
      digitalWrite(dirPin, LOW); // Left needs LOW to go Forward
    } else {
      digitalWrite(dirPin, HIGH); // Right needs HIGH to go Forward
    }
    analogWrite(pwmPin, pwm);
  } 
  else {
    // STARTING BACKWARD
    if (isLeft) {
      digitalWrite(dirPin, HIGH); // Left needs HIGH to go Backward
    } else {
      digitalWrite(dirPin, LOW); // Right needs LOW to go Backward
    }
    analogWrite(pwmPin, abs(pwm));
  }
}
