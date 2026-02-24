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
 *  Description: "The Headset" is currently a prototype to automate the application of a cold compress on the wearer's eyes. 
 *  This is intended to treat eye itching and pain caused by allergies.
 *  This is the main code file for the Headset. Its key role is to contain the high level logic.
 */

#include <Arduino.h>
#include <util/atomic.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <AccelStepper.h>

// Timer Impl from https://github.com/khoih-prog/megaAVR_TimerInterrupt/blob/main/examples/Argument_None/Argument_None.ino which uses an MIT style license
#if !( defined(__AVR_ATmega4809__) || defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_AVR_NANO_EVERY) || \
      defined(ARDUINO_AVR_ATmega4809) || defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209) || \
      defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609) || defined(ARDUINO_AVR_ATmega1608) || \
      defined(ARDUINO_AVR_ATmega809) || defined(ARDUINO_AVR_ATmega808) )
#error This is designed only for Arduino or MegaCoreX megaAVR board! Please check your Tools->Board setting
#endif

// These define's must be placed at the beginning before #include "megaAVR_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select USING_16MHZ     == true for  16MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_8MHZ      == true for   8MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_250KHZ    == true for 250KHz to Timer TCBx => shorter timer, but better accuracy
// Not select for default 250KHz to Timer TCBx => longer timer,  but worse accuracy
#define USING_16MHZ     true
#define USING_8MHZ      false
#define USING_250KHZ    false
// The Headset: use two timers: one for the stepper and button inputs, the other for temperature
#define USE_TIMER_0     false
#define USE_TIMER_1     true
#define USE_TIMER_2     true
#define USE_TIMER_3     false

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "megaAVR_TimerInterrupt.h"

#define TIMER1_INTERVAL_MS    4

#ifndef LED_BUILTIN
	#define LED_BUILTIN   13
#endif

#ifndef STATUS_PIN
  #define STATUS_PIN   5
#endif

// Define Sensors
#define ONE_WIRE_BUS 4 // Pin # of Sensor 1
#define TWO_WIRE_BUS 10 // Pin # of Sensor 2
// OneWire oneWire(ONE_WIRE_BUS);
OneWire twoWire(TWO_WIRE_BUS);
// DallasTemperature sensor1(&oneWire); // Left
DallasTemperature sensor2(&twoWire); // Right

// Initial target position
const int highPos = 3500;
const int lowPos = 0;
int target[] = {lowPos, lowPos};

// Motor Connections to the two ULN2003 unipolar motor drivers
// In order of IN1, IN2, IN3, IN4, matching driver wires to the controller
AccelStepper stepperLeft(AccelStepper::FULL4WIRE, 2, 3, 5, 6);
AccelStepper stepperRight(AccelStepper::FULL4WIRE, 12, 11, 9, 8);

//boolean to switch motor direction
bool INVERT_MOTOR = false;

// Counter for printing position on the serial
int statusCounter = 0;

/* Call periodic methods here */
void periodic(void)
{
  static bool toggle = false;
 
  // Make periodic calls here
  stepperLeft.run();
  stepperRight.run();

  //timer interrupt toggles outputPin - Flash status pin 10% of the time
  if ((statusCounter % 10 == 0) || (statusCounter % 10 == 1)) {
    toggle = !toggle;
    // Print motor info on the serial
    Serial.print("                                                     ");
    
    Serial.print(" ");
    Serial.print("Position of left stepper: ");
    Serial.print(stepperLeft.currentPosition());
    Serial.print("Position of right stepper: ");
    Serial.print(stepperRight.currentPosition());
    Serial.println(" ");
  }
  digitalWrite(STATUS_PIN, toggle);
  statusCounter++;
}

#if USE_TIMER_2

#define TIMER2_INTERVAL_MS    2000

void TimerHandler2(void)
{
	static bool toggle2 = false;
	static bool started = false;

	if (!started)
	{
		started = true;
		pinMode(A0, OUTPUT);
	}

	//timer interrupt toggles outputPin
	digitalWrite(A0, toggle2);
	toggle2 = !toggle2;
}
#endif

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


void setup() 
{
  Serial.begin(9600);

  // TEMPERATURE SENSORS
  // sensor1.begin();
  sensor2.begin();
  
  // STEPPERS
  // Set the maximum speed, acceleration factor, and the target position.

  stepperLeft.setMaxSpeed(800.0);
  stepperLeft.setAcceleration(100.0);
  
  stepperRight.setMaxSpeed(800.0);
  stepperRight.setAcceleration(100.0);

  stepperLeft.moveTo(target[0]);
  stepperRight.moveTo(target[1]);

  // Initialize the button pins as inputs:
  pinMode(leftButton, INPUT);   
  pinMode(rightButton, INPUT);
  pinMode(homeButton, INPUT);

  Serial.print(F("\nStarting THE HEADSET on "));
	Serial.println(BOARD_NAME);
	Serial.println(MEGA_AVR_TIMER_INTERRUPT_VERSION);
	Serial.print(F("CPU Frequency = "));
	Serial.print(F_CPU / 1000000);
	Serial.println(F(" MHz"));

	Serial.print(F("TCB Clock Frequency = "));

#if USING_16MHZ
	Serial.println(F("16MHz for highest accuracy"));
#elif USING_8MHZ
	Serial.println(F("8MHz for very high accuracy"));
#else
	Serial.println(F("250KHz for lower accuracy but longer time"));
#endif

	// Select Timer 1-2 for UNO, 0-5 for MEGA
	// Timer 2 is 8-bit timer, only for higher frequency
	ITimer1.init();

	// Using ATmega328 used in UNO => 16MHz CPU clock ,
	// For 16-bit timer 1, 3, 4 and 5, set frequency from 0.2385 to some KHz
	// For 8-bit timer 2 (prescaler up to 1024, set frequency from 61.5Hz to some KHz

	if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, periodic))
	{
		Serial.print(F("Starting  ITimer1 OK, millis() = "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

// TIMER 2
#if USE_TIMER_2

	// Select Timer 1-2 for UNO, 0-5 for MEGA
	// Timer 2 is 8-bit timer, only for higher frequency
	ITimer2.init();

	if (ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler2))
	{
		Serial.print(F("Starting  ITimer2 OK, millis() = "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set ITimer2. Select another freq. or timer"));

#endif
}

// TODO: Empty loop() when fully switching to a timer periodic
void loop() 
{
  /* Request and report temperature every 3 sec */
  /*
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
      stepperLeft.run();
      stepperRight.run();
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
  */  
}