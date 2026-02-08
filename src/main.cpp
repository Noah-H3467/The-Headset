/* 
 *  Copyright (C) 2026 Noah Haskell
 *  
 *  This program is free software: you can redistribute it and/or modify it under the terms of the
 *  GNU General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or any later version.
 *  
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 *  even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License along with this program. If
 *  not, see <https://www.gnu.org/licenses/>.
 *  
 *  File: main.cpp
 *  Author: Noah Haskell
 *  Decription: "The Headset" is currently a prototype to automate the application of a cold compress on the wearer's eyes. 
 *  This is intended to treat eye itching and pain caused by allergies.
 *  This is the main code file for the Headset. Its key role is to contain the high level logic.
 */

#include <Arduino.h>
#include <util/atomic.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <AccelStepper.h>

// Define Sensors
#define ONE_WIRE_BUS 4 // Pin # of Sensor 1
#define TWO_WIRE_BUS 10 // Pin # of Sensor 2
// OneWire oneWire(ONE_WIRE_BUS);
OneWire twoWire(TWO_WIRE_BUS);
// DallasTemperature sensor1(&oneWire); // Left
DallasTemperature sensor2(&twoWire); // Right

// Define some stepper motors and the pins they will use
AccelStepper stepperLeft(AccelStepper::FULL4WIRE, 2, 3, 5, 6);
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 12, 11, 9, 8);

//boolean to switch motor direction
bool directionState = false;

// Initial target position
const int highPos = 3500;
const int lowPos = 0;
int target[] = {lowPos, lowPos};

// Counter for printing position on the serial
int counter = 0;

// Define button pins
// https://forum.arduino.cc/t/using-analog-pins-for-push-buttons/309407/7
const int leftButton = A0;
const int rightButton = A1;
// Press this to zero the encoders. TODO: Make the press of this button start homing
const int homeButton = A2;
// Records the button state. Either HIGH or LOW.
int leftButtonState = LOW;
int rightButtonState = LOW;
bool homeButtonState = LOW;

unsigned long lastMilli = 0;
unsigned long lastTempMilli = 0; // To be able to run long temperature stuff that takes a second while checking PID
unsigned long lastTempInProgressMilli = 0;

void requestTemps() {
  // Get temperatures
  // sensor1.requestTemperatures();
  sensor2.requestTemperatures();
}

// requestTemps() must be called at least 0.75 seconds before calling this
// Currently gets thermometer reading and prints it to the serial.
void updateTemperatures() {
  /*
  Serial.print("Sensor 1: Celsius temperature: ");
  // Why "byIndex"? One can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensor1.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensor1.getTempFByIndex(0));
  */
  Serial.print("Sensor 2: Celsius temperature: ");
  Serial.print(sensor2.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensor2.getTempFByIndex(0));
}

void updateButtonState() {
  // Checks to see if the new reading is the same as the existing state
  if (leftButtonState != digitalRead(leftButton)) {
    // If not, Update to the leftButtonState
    leftButtonState = digitalRead(leftButton);
    Serial.println("Left Button State:" + leftButtonState);
    if (leftButtonState == HIGH) {
      // If the button was just pressed, toggle the target position
      if (target[0] == highPos) {
        target[0] = lowPos;
        Serial.println("TARGET0 IS LOW");
      } else {
        target[0] = highPos;
        Serial.println("TARGET0 IS HIGH");
      }
      // set new target position of left stepper
      stepperLeft.moveTo(target[0]);
    }
  }

  // Checks to see if the new reading is the same as the existing state
  if (rightButtonState != digitalRead(rightButton)) {
    // If not, Update to the rightButtonState
    rightButtonState = digitalRead(rightButton);
    if (rightButtonState == HIGH) {
      // If the button was just pressed, toggle the target position
      if (target[1] == highPos) {
        Serial.println("TARGET1 IS LOW");
        target[1] = lowPos;
      } else {
        target[1] = highPos;
        Serial.println("TARGET1 IS HIGH");
      }
      // Set new target position of right stepper
      stepperRight.moveTo(target[1]);
    }
  }
  
  // Zero the steppers right when home button is pressed
  if (homeButtonState != digitalRead(homeButton)) {
    homeButtonState = !homeButtonState;
    if (homeButtonState == HIGH) {
      stepperLeft.setCurrentPosition(0.0);
      stepperRight.setCurrentPosition(0.0);
    }
  }
}

// Accepts AccelStepper object by pointer to access its location in memory to get the position info and make it run
void runMotor(AccelStepper* motor, int index) {
  // Tell stepper to go to their target position if not already there.
  if (motor->currentPosition() != target[index]) { // Full speed up to 300
    motor->run();
  } else {
    motor->stop(); // Stop as fast as possible: sets new target
    motor->runToPosition(); 
    // Now stopped after quickstop
  }
}

void setup() 
{
  Serial.begin(9600);

  // TEMPERATURE SENSORS
  // sensor1.begin();
  sensor2.begin();
  
  // STEPPERS
  stepperLeft.setMaxSpeed(500.0);
  stepperLeft.setAcceleration(100.0);
  
  stepperRight.setMaxSpeed(500.0);
  stepperRight.setAcceleration(100.0);

  stepperLeft.moveTo(target[0]);
  stepperLeft.moveTo(target[1]);

  // Initialize the button pins as inputs:
  pinMode(leftButton, INPUT);   
  pinMode(rightButton, INPUT);
  pinMode(homeButton, INPUT);
}

void loop() 
{
  /* Request and report temperature every 3 sec */
  if (millis()-lastTempMilli >= 5000) {
    
    lastTempMilli = millis();
    requestTemps();
    lastTempInProgressMilli = millis();

    lastMilli = millis();
    while (millis()-lastTempInProgressMilli < 750) {
      if (millis()-lastMilli > 20) {
        lastMilli = millis();  

        updateButtonState();

        if (counter % 5 == 0) {
          Serial.println(stepperLeft.currentPosition());
        }
        counter++;
      }

      // In the main loop(), tell steppers to go to their target position if not already there.
      runMotor(&stepperLeft, 0);
      runMotor(&stepperRight, 1);
    }
    updateTemperatures();
    
  }
  if (millis()-lastMilli > 20) {
    lastMilli = millis();  

    updateButtonState();
    if (counter % 5 == 0) {
      Serial.println(stepperLeft.currentPosition());
    }
    counter++;
  }

  // In the main loop(), tell steppers to go to their target position if not already there.
  runMotor(&stepperLeft, 0);
  runMotor(&stepperRight, 1);
}