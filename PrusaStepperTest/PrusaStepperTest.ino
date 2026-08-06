// Basic NEMA 17 Stepper + TMC2209 test - does not implement AccelStepper yet

// Pin Definitions
#define EN_PIN 8    // LOW: Driver enabled, HIGH: Driver disabled
#define STEP_PIN 9  // Step on the rising edge
#define DIR_PIN 10  // Set stepping direction
int noOfSteps = 1500;           // Number of steps to move in each direction
int microSecondsDelay = 1000; // at 12V, 575 us (nearly 2x speed) at 16V; // Delay in microseconds between each step

void setup() {
  // Configure pin modes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  // Initialize pin states
  digitalWrite(EN_PIN, LOW);   // Enable the driver
  digitalWrite(DIR_PIN, LOW);  // Set initial direction
  Serial.begin(9600);
}
void loop() {
    // Move motor in one direction
    digitalWrite(DIR_PIN, LOW);  // Set direction to LOW
    Serial.println("LOW");
    moveSteps(noOfSteps);
    // Move motor in the opposite direction
    digitalWrite(DIR_PIN, HIGH); // Set direction to HIGH
    Serial.println("HIGH");
    moveSteps(noOfSteps);
}

void moveSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(microSecondsDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(microSecondsDelay);
  }
}