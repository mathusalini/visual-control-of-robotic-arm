int buttonPin = 3; 

// setup initializes serial and the button pin 
void setup() 
{ 
  // CORRECTED: 'beginSerial' is now 'Serial.begin'
  Serial.begin(9600); 
  pinMode(buttonPin, INPUT); 
} 

// loop checks the button pin each time, 
// and will send serial if it is pressed 
void loop()
{ 
  if (digitalRead(buttonPin) == HIGH) 
    // CORRECTED: 'serialWrite' is now 'Serial.print'
    Serial.print('H'); 
  else 
    // CORRECTED: 'serialWrite' is now 'Serial.print'
    Serial.print('L'); 
    
  delay(1000); 
}
