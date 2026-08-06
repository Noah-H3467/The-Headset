const int IN1 = 9;    // Motor direction pin 1
const int IN2 = 10;   // Motor direction pin 2
const int EN = 5;     // Enable pin (PWM)
 
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
  analogWrite(EN, 255); // Full speed (0-255 range)
  Serial.begin(9600);
  Serial.println("Test");
}
 
void loop() {
  // Forward direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  Serial.println("Forward");
  delay(250);
 
  // Stop motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
 
  // Reverse direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(250);
 
  // Stop motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
}
