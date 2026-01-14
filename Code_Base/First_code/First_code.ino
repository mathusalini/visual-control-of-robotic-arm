/* NEXUS ROBOT PIN FINDER
   This code tests common Nexus PWM pins one by one.
   Use this to verify your wiring and fill out your pin map.
*/

// These are the possible PWM pins on the Nexus robot
int possiblePins[] = {3, 4, 5, 6, 9, 10, 11}; 

void setup() {
  Serial.begin(9600); // Start communication
  Serial.println("--- STARTING PIN HUNT ---");
  Serial.println("Type a pin number (e.g. 9) and press Enter.");
  Serial.println("Watch the wheels!");
  
  // Set all potential motor pins to OUTPUT and turn them OFF
  for(int i=0; i<7; i++) {
    pinMode(possiblePins[i], OUTPUT);
    analogWrite(possiblePins[i], 0);
  }
}

void loop() {
  // Listen for user input from Serial Monitor
  if(Serial.available() > 0){
    int pin = Serial.parseInt(); // Read the number typed
    
    if(pin > 0){
       Serial.print("Testing Pin: ");
       Serial.println(pin);
       
       // Spin the motor connected to this pin
       analogWrite(pin, 150); // Medium speed (0-255)
       delay(2000);           // Run for 2 seconds
       
       analogWrite(pin, 0);   // Stop
       Serial.println("Stopped.");
    }
  }
}