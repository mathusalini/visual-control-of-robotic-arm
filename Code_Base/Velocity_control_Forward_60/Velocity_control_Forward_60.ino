// --- STEP 5: DRIVE EXACTLY 60 CM (CORRECTED) ---

// *** PIN DEFINITIONS ***
const int FL_PWM = 3;    const int FL_DIR = 2; 
const int FL_ENC_A = A2; const int FL_ENC_B = A3;

const int FR_PWM = 11;   const int FR_DIR = 12;
const int FR_ENC_A = A5; const int FR_ENC_B = A4;

const int BL_PWM = 10;   const int BL_DIR = 7; 
const int BL_ENC_A = A0; const int BL_ENC_B = A1;

const int BR_PWM = 9;    const int BR_DIR = 8;
const int BR_ENC_A = 5;  const int BR_ENC_B = 4;

// TUNING
// Calibration: Adjusted because 60cm target -> 47.5cm actual
const double CM_PER_TICK = 0.02536;
double target_speed = 30.0;              // Cruise Speed
double target_distance = 60.0;           // GOAL: 60 cm

// PID Gains
float Kp = 6.0; 
float Ki = 2.0;

// VARIABLES
volatile long fl_ticks=0, fr_ticks=0, bl_ticks=0, br_ticks=0;
long prev_fl=0, prev_fr=0, prev_bl=0, prev_br=0;
double int_fl=0, int_fr=0, int_bl=0, int_br=0; 
int last_fl=0, last_fr=0, last_bl=0, last_br=0;
unsigned long last_pid_time = 0;
bool motion_complete = false;

// --- FUNCTION DECLARATIONS (Fixes scope error) ---
void pollEncoders();
void computePID(double current_speed, double &integral, int pwmPin, int dirPin, double dt, bool isLeft);

void setup() {
  Serial.begin(115200);
  
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

  Serial.println("--- DRIVE 60 CM START ---");
  delay(2000); 
}

void loop() {
  pollEncoders();

  // CHECK DISTANCE 
  long avg_ticks = (abs(fl_ticks) + abs(fr_ticks) + abs(bl_ticks) + abs(br_ticks)) / 4;
  double distance_covered = avg_ticks * CM_PER_TICK; // Fixed typo here

  // IF WE REACHED 60 CM -> STOP
  if (distance_covered >= target_distance) {
    if (!motion_complete) {
      analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
      analogWrite(BL_PWM, 0); analogWrite(BR_PWM, 0);
      Serial.println("--- 60 CM REACHED! ---");
      motion_complete = true;
    }
    return; 
  }

  // PID LOOP (Every 50ms)
  unsigned long curr_time = millis();
  if (curr_time - last_pid_time >= 50) { 
    double dt = (curr_time - last_pid_time) / 1000.0;
    last_pid_time = curr_time;

    // 1. FRONT LEFT (INVERTED = TRUE)
    long diff_fl = fl_ticks - prev_fl; prev_fl = fl_ticks;
    double speed_fl = (diff_fl * CM_PER_TICK) / dt;
    computePID(speed_fl, int_fl, FL_PWM, FL_DIR, dt, true); 

    // 2. FRONT RIGHT (INVERTED = FALSE)
    long diff_fr = fr_ticks - prev_fr; prev_fr = fr_ticks;
    double speed_fr = (diff_fr * CM_PER_TICK) / dt;
    computePID(speed_fr, int_fr, FR_PWM, FR_DIR, dt, false);

    // 3. BACK LEFT (INVERTED = TRUE)
    long diff_bl = bl_ticks - prev_bl; prev_bl = bl_ticks;
    double speed_bl = (diff_bl * CM_PER_TICK) / dt;
    computePID(speed_bl, int_bl, BL_PWM, BL_DIR, dt, true);

    // 4. BACK RIGHT (INVERTED = FALSE)
    long diff_br = br_ticks - prev_br; prev_br = br_ticks;
    double speed_br = (diff_br * CM_PER_TICK) / dt;
    computePID(speed_br, int_br, BR_PWM, BR_DIR, dt, false);
    
    Serial.print("Dist:"); Serial.print(distance_covered);
    Serial.print(" / 60 cm | Speed:"); Serial.println(speed_fl);
  }
}

// --- HELPER FUNCTIONS ---

void pollEncoders() {
  // FL (Fixed Logic: --)
  int curr_fl = digitalRead(FL_ENC_A);
  if (curr_fl != last_fl && curr_fl == HIGH) { 
    if(digitalRead(FL_ENC_B)) fl_ticks--; else fl_ticks++; 
  }
  last_fl = curr_fl;

  // FR (Normal: ++)
  int curr_fr = digitalRead(FR_ENC_A);
  if (curr_fr != last_fr && curr_fr == HIGH) {
    if(digitalRead(FR_ENC_B)) fr_ticks++; else fr_ticks--;
  }
  last_fr = curr_fr;

  // BL (Fixed Logic: --)
  int curr_bl = digitalRead(BL_ENC_A);
  if (curr_bl != last_bl && curr_bl == HIGH) {
    if(digitalRead(BL_ENC_B)) bl_ticks--; else bl_ticks++;
  }
  last_bl = curr_bl;

  // BR (Normal: ++)
  int curr_br = digitalRead(BR_ENC_A);
  if (curr_br != last_br && curr_br == HIGH) {
    if(digitalRead(BR_ENC_B)) br_ticks++; else br_ticks--;
  }
  last_br = curr_br;
}

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

  if (pwm > 0) {
    if (isLeft) digitalWrite(dirPin, LOW); 
    else digitalWrite(dirPin, HIGH);       
    analogWrite(pwmPin, pwm);
  } else {
    if (isLeft) digitalWrite(dirPin, HIGH);
    else digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, abs(pwm));
  }
}
