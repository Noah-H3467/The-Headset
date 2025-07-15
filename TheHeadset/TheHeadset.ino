#include <util/atomic.h>
#include <QuadratureEncoder.h>
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
  
    // TODO: make this a parameter in the constructor
    // static constant
    float ks = 50;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    } else if (pwr < ks) {
      pwr = ks;
    }
  
    // motor direction
    dir = 1;
    if(u>0){
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

// Number of motors
#define NMOTORS 2
// TODO: Swap wiring of the two motors and respective encoders
// Power wires already swapped. Will check to see if I need to swap the encoder wires for each.
const int DIRECTION_PINS[] = {12, 13};
const int PWM_PINS[] = {3, 11};
const int BRAKE_PINS[] = {9, 8};
Encoders firstEncoder(2,5);
Encoders secondEncoder(6,7); // the encoder objects could use analog pins

//boolean to switch motor direction
bool directionState = false;

// Initial target position
const int highPos = 1000;
const int lowPos = 0;
int target[] = {highPos, highPos};

// Globals
long prevT = 0;

// PID class instance list length 2
SimplePID pid[NMOTORS];
int STATIC_GAIN[] = {255, 73}; // 255, 73

// Counter for printing position on the serial
int counter = 0;

// Define button pins
// https://forum.arduino.cc/t/using-analog-pins-for-push-buttons/309407/7
const int leftButton = A0;
const int rightButton = A1;
// Records the button state. Either HIGH or LOW.
int leftButtonState = HIGH;
int rightButtonState = HIGH;

void setup() {
  Serial.begin(9600);
  sensor1.begin();
  sensor2.begin();
  
  //define pins
  for (int k = 0; k < NMOTORS; k++) {
    // ENCODER
    // pinMode(ENCODER_ONE[k],INPUT); // TODO: UNCOMMENT
    // pinMode(ENCODER_TWO[k], INPUT);
    // MOTOR
    pinMode(DIRECTION_PINS[k], OUTPUT);
    pinMode(PWM_PINS[k], OUTPUT);
    pinMode(BRAKE_PINS[k], OUTPUT);

    pid[k].setParams(1,0.15,0.0,255);
  }

  resetEncoders();
  // Tell Motor #1 to stop
  setMotor(1, DIRECTION_PINS[0], 0.0, PWM_PINS[0]);

  // Initialize the button pins as inputs:
  pinMode(leftButton, INPUT);   
  pinMode(rightButton, INPUT);
}

unsigned long lastMilli = 0;

void loop() {

  if (millis()-lastMilli > 20) {
    // Currently gets thermometer reading and prints it to the serial.
    // updateTemperatures();
    updateButtonState();
    updatePID();

    lastMilli = millis();  
  }
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

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  long firstPos = firstEncoder.getEncoderCount();
  long secondPos = secondEncoder.getEncoderCount();
  
  lastMilli = millis();  

  long pos[2];
  pos[0] = firstPos;
  pos[1] = secondPos;

  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
    // signal the motor - for now, test static gain instead of PID
    setMotor(dir, DIRECTION_PINS[k], pwr, PWM_PINS[k]);
  }

  if (counter % 10 == 0) {
    for(int k = 0; k < NMOTORS; k++){
      if (k == 1) {
      Serial.print("                                                     ");
      }
      Serial.print("Target of motor ");
      Serial.print(k+1);
      Serial.print(": ");
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print("Position of encoder: ");
      Serial.print(pos[k]);
      Serial.println(" ");
    }
  }
  counter++;
}

void resetEncoders() {
  firstEncoder.setEncoderCount(0.0);
  secondEncoder.setEncoderCount(0.0);
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

void updateButtonState() {
  // Checks to see if the new reading is the same as the existing state
  if (leftButtonState != digitalRead(leftButton)) {
    // If not, Update to the leftButtonState
    leftButtonState = digitalRead(leftButton);
    if (leftButtonState == HIGH) {
      // If the button was just pressed, toggle the target position
      if (target[0] == highPos) {
        target[0] = lowPos;
      } else {
        target[0] = highPos;
      }
    }
  }

  // Checks to see if the new reading is the same as the existing state
  if (rightButtonState != digitalRead(rightButton)) {
    // If not, Update to the rightButtonState
    rightButtonState = digitalRead(rightButton);
    if (rightButtonState == HIGH) {
      // If the button was just pressed, toggle the target position
      if (target[1] == highPos) {
        target[1] = lowPos;
      } else {
        target[1] = highPos;
      }
    }
  }
  // Serial.print("Targets: ");
  // Serial.print(target[0]);
  // Serial.print("   ");
  // Serial.println(target[1]);
}