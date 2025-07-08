// SimplePid class from CurioRes' multiple encoder pid control tutorial
// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

#include <OneWire.h>
#include <DallasTemperature.h>

// Define Sensors
#define ONE_WIRE_BUS 4 // Pin # of Sensor 1
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
#define ENCODER_1A 2 // Yellow on shield, white on motor
#define ENCODER_1B 5 // White on shield, green on motor

//uncomment if using channel B, and remove above definitions
int directionPinTwo = 13;
int pwmPinTwo = 11;
int brakePinTwo = 8;
// Encoder #2
#define ENCODER_2A 3
#define ENCODER_2B 7

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
  pinMode(directionPinTwo, OUTPUT);
  pinMode(pwmPinTwo, OUTPUT);
  pinMode(brakePinTwo, OUTPUT);
  pinMode(ENCODER_1A,INPUT);
  pinMode(ENCODER_1B, INPUT);
  pinMode(ENCODER_2A, INPUT);
  pinMode(ENCODER_2B, INPUT);
  // Interrupts whenever ENCODER_1A rises.
  // readEncoder is the function that should be called when interrupted
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A),readEncoderTwo,RISING);
  analogWrite(pwmPinTwo, 100);
}


void loop() {
  // Currently gets thermometer reading and prints it to the serial.
  // updateTemperatures();
  updatePID();

  int a = digitalRead(ENCODER_2A);
  int b = digitalRead(ENCODER_2B);
  /*
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
  Serial.println(" ");
  delay(500);
  */
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
  Serial.print("Position of 2nd encoder: ");
  Serial.println(posTwo);

  // Basic testing for Motor #1
  int error = target - posOne;
  if (error > 0) {
    if (error > 100) {
      error = 100;
    }
    setMotor(-1, error, pwmPin);
  } else if (error < 0) {
    if (error < -100) {
      error = -100;
    }
    setMotor(1, directionPin, error, pwmPin);
  } else {
    setMotor(0, directionPin, 0, pwmPin);
    Serial.println("ZERO");
  }
}

// Called when encoder1A changes state
void readEncoder() {
  int oneB = digitalRead(ENCODER_1B);
  if (oneB > 0) {
    posOne++;
  } else {
    posOne--;
  }
}

// Called when encoder2A changes state
void readEncoderTwo() {
  //Serial.println("ENCODER 2 CALLED");
  int twoB = digitalRead(ENCODER_2B);
  if (twoB > 0) {
    posTwo++;
  } else {
    posTwo--;
  }
}

void setMotor(int dir, int dirPin, int pwmVal, int pwmPin) {
  if (dir==1) {
    digitalWrite(dirPin, HIGH);
    //set work duty for the motor
    analogWrite(pwmPin, pwmVal);
  } else if (dir == -1) {
    digitalWrite(dirPin, LOW);
    //set work duty for the motor
    analogWrite(pwmPin, pwmVal);
  } else {
    analogWrite(pwmPin, 0);
  }
}