#include <OneWire.h>
#include <DallasTemperature.h>

// Define Sensors
#define ONE_WIRE_BUS 2 // Pin # of Sensor 1
#define TWO_WIRE_BUS 10 // Pin # of Sensor 2
OneWire oneWire(ONE_WIRE_BUS);
OneWire twoWire(TWO_WIRE_BUS);
DallasTemperature sensor1(&oneWire);
DallasTemperature sensor2(&twoWire);

// Serial
// SoftwareSerial display(3, 2);

// Motor #1
int directionPin = 12;
int pwmPin = 3;
int brakePin = 9;
// Encoder #1
#define ENCODER_1A 4 // Yellow on shield, white on motor
#define ENCODER_1B 5 // White on shield, green on motor

//uncomment if using channel B, and remove above definitions
//int directionPin = 13;
//int pwmPin = 11;
//int brakePin = 8;
// Encoder #2
// #define ENCODER_2A = 6;
// #define ENCODER_2B = 7;

//boolean to switch motor direction
bool directionState = false;

// Position of each motor
int posOne = 0;
int posTwo = 0;
// Target position
int target = 100;

void setup() {
  Serial.begin(9600);
  sensor1.begin();
  sensor2.begin();
  
  //define pins
  // Motor + Encoder #1
  pinMode(directionPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(ENCODER_1A,INPUT);
  pinMode(ENCODER_1B, INPUT);
  // Interrupts whenever ENCODER_1A rises.
  // readEncoder is the function that should be called when interrupted
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A),readEncoder,RISING);
  // analogWrite(pwmPin, 100);
  
}


void loop() {
  // Currently gets thermometer reading and prints it to the serial.
  // updateTemperatures();
  updatePID();

  /*
  //change direction every loop()
  directionState = !directionState;

  //write a low state to the direction pin (13)
  if(directionState == false){
    digitalWrite(directionPin, LOW);
  }

  //write a high state to the direction pin (13)
  else{
    digitalWrite(directionPin, HIGH);
  }

  //release breaks
  digitalWrite(brakePin, LOW);

  //set work duty for the motor
  analogWrite(pwmPin, 100);

  delay(2000);

  //activate breaks
  digitalWrite(brakePin, HIGH);

  //set work duty for the motor to 0 (off)
  analogWrite(pwmPin, 0);

  delay(2000); // */

}

void updateTemperatures() {
  // Get and print temperature
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  delay(750);
  //if (temperatureC != DEVICE_DISCONNECTED_C) {
  Serial.print("Sensor 1: Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensor1.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensor1.getTempFByIndex(0));
  Serial.print("Sensor 2: Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensor2.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensor2.getTempFByIndex(0));
  // } else {
  //  Serial.println("Error: Could not read temperature");
  //}
  delay(1000); // Update every second
}

void updatePID() {

  Serial.print("Position of 1st encoder: ");
  Serial.println(posOne);
  int error = target - posOne;
  if (error > 0) {
    if (error > 100) {
      error = 100;
    }
    setMotor(1, error, pwmPin);
  } else if (error < 0) {
    if (error < -100) {
      error = -100;
    }
    setMotor(-1, error, pwmPin);
  } else {
    setMotor(0, 0, pwmPin);
  }
}

// Called when encoder1A changes state
void readEncoder() {
  Serial.println("readEncoder called");
  delay(500);
  int oneB = digitalRead(ENCODER_1B);
  if (oneB > 0) {
    posOne++;
  } else {
    posOne--;
  }
}

// Called when encoder2A changes state
void readEncoderTwo() {
  // int twoB = digitalRead(ENCODER_2B);
  int twoB = 0;
  if (twoB > 0) {
    posTwo++;
  } else {
    posTwo--;
  }
}

void setMotor(int dir, int pwmVal, int pwm) {
  analogWrite(pwm, pwmVal);
  if (dir==1) {
    digitalWrite(directionPin, HIGH);
  } else if (dir == -1) {
    digitalWrite(directionPin, LOW);
  } else {
    analogWrite(pwm, 0);
  }
}