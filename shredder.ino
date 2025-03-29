/**
 * ESP32 Shredder Control System
 * 
 * Controls a Nema 42 stepper motor for a shredder with safety features:
 * - Safety switch (lid sensor)
 * - On/Off switch
 * - Speed control via rotary encoder
 * - LCD status display
 * 
 * Hardware:
 * - ESP32 microcontroller
 * - Nema 42 stepper motor with driver
 * - LCD display
 * - Safety switch (connected to lid)
 * - On/Off switch
 * - Rotary encoder for speed control
 */

 // is working on arduino ide
#include <Arduino.h>
 #include <Wire.h>
 #include <LiquidCrystal_I2C.h>
 #include <ESP32Encoder.h>
 #include <AccelStepper.h>
 
 // Pin Definitions
 #define SAFETY_SWITCH_PIN   14  // Pin connected to safety switch (lid sensor)
 #define ON_OFF_SWITCH_PIN   12  // Pin connected to on/off switch
 #define STEPPER_DIR_PIN     26  // Stepper motor direction pin
 #define STEPPER_STEP_PIN    27  // Stepper motor step pin
 #define STEPPER_ENABLE_PIN  25  // Stepper motor enable pin
 #define ROTARY_A_PIN        32  // Rotary encoder A pin
 #define ROTARY_B_PIN        33  // Rotary encoder B pin
 
 // Constants
 #define MAX_SPEED           2000    // Maximum speed in steps per second
 #define MIN_SPEED           0       // Minimum speed (off)
 #define ENCODER_STEPS       24      // Encoder steps per revolution
 #define SPEED_INCREMENTS    100     // Number of speed increments
 #define DISPLAY_UPDATE_MS   250     // Display update interval in milliseconds
 #define ERROR_DISPLAY_MS    2000    // Error message display time in milliseconds
 
 // LCD Configuration - Using I2C for simplicity
 LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display
 
 // Stepper motor configuration
 AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
 
 // Encoder setup
 ESP32Encoder encoder;
 
 // Global variables
 bool lidClosed = false;
 bool powerOn = false;
 int currentSpeed = 0;
 unsigned long lastDisplayUpdate = 0;
 bool displayingError = false;
 unsigned long errorStartTime = 0;
 
 // Error message enum
 enum ErrorType {
   NO_ERROR,
   LID_OPEN,
   MOTOR_STALL,  // Additional error type for potential expansion
   SYSTEM_ERROR  // Generic system error
 };
 
 ErrorType currentError = NO_ERROR;
 
 // Function prototypes
 void updateDisplay();
 void displayError(ErrorType error);
 void checkSafety();
 void updateMotorSpeed();
 char* translateDigitToBCD(int digit);
 void displayBCDDigit(int position, int digit);
 void displayASCII(int position, char character);
 
 void setup() {
   Serial.begin(115200);
   
   // Initialize pins
   pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);  // Use pull-up for switch inputs
   pinMode(ON_OFF_SWITCH_PIN, INPUT_PULLUP);
   pinMode(STEPPER_ENABLE_PIN, OUTPUT);
   
   // Initialize stepper motor
   stepper.setEnablePin(STEPPER_ENABLE_PIN);
   stepper.setPinsInverted(false, false, true);  // enable pin is inverted
   stepper.setMaxSpeed(MAX_SPEED);
   stepper.setAcceleration(500);
   stepper.enableOutputs();  // Initially enable outputs but motor won't move until conditions are met
   
   // Initialize encoder
   ESP32Encoder::useInternalWeakPullResistors = UP;
   encoder.attachHalfQuad(ROTARY_A_PIN, ROTARY_B_PIN);
   encoder.setCount(0);
   
   // Initialize LCD
   lcd.init();
   lcd.backlight();
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("Shredder System");
   lcd.setCursor(0, 1);
   lcd.print("Initializing...");
   
   delay(1000);  // Short delay to show startup message
   
   Serial.println("Shredder control system initialized");
 }
 
 void loop() {
   // Check safety conditions
   checkSafety();
   
   // Read switches and encoder
   lidClosed = digitalRead(SAFETY_SWITCH_PIN) == LOW;  // LOW when lid is closed (switch activated)
   powerOn = digitalRead(ON_OFF_SWITCH_PIN) == LOW;    // LOW when switch is in ON position
   
   // Read encoder position and map to speed
   long encoderPosition = encoder.getCount() / 2;  // Divide by 2 for half-quadrature
   int mappedSpeed = map(constrain(encoderPosition, 0, SPEED_INCREMENTS), 0, SPEED_INCREMENTS, MIN_SPEED, MAX_SPEED);
   
   // Only update speed if it has changed to avoid unnecessary operations
   if (mappedSpeed != currentSpeed) {
     currentSpeed = mappedSpeed;
     updateMotorSpeed();
   }
   
   // Update display at regular intervals
   if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_MS) {
     updateDisplay();
     lastDisplayUpdate = millis();
   }
   
   // Clear error message after timeout
   if (displayingError && (millis() - errorStartTime > ERROR_DISPLAY_MS)) {
     displayingError = false;
     currentError = NO_ERROR;
   }
   
   // Run the stepper motor (this needs to be called frequently)
   stepper.run();
   
   // Small delay to avoid overwhelming the processor
   delay(10);
 }
 
 void checkSafety() {
   // Check if lid is open while system is powered on
   if (powerOn && !lidClosed) {
     displayError(LID_OPEN);
   }
 }
 
 void updateMotorSpeed() {
   if (powerOn && lidClosed && currentSpeed > 0) {
     // All safety conditions met, enable motor
     stepper.enableOutputs();
     stepper.setSpeed(currentSpeed);
     Serial.printf("Motor speed set to: %d\n", currentSpeed);
   } else {
     // Stop motor
     stepper.setSpeed(0);
     stepper.disableOutputs();
     Serial.println("Motor stopped");
   }
 }
 
 void updateDisplay() {
   // If we're displaying an error, don't override it until timeout
   if (displayingError) {
     return;
   }
   
   lcd.clear();
   
   // First line: Status information
   lcd.setCursor(0, 0);
   lcd.print("Status: ");
   if (!powerOn) {
     lcd.print("OFF");
   } else if (!lidClosed) {
     lcd.print("LID OPEN");
   } else {
     lcd.print("RUNNING");
   }
   
   // Second line: Speed information
   lcd.setCursor(0, 1);
   lcd.print("Speed: ");
   
   // Display speed as a percentage (0-100%)
   int speedPercent = map(currentSpeed, MIN_SPEED, MAX_SPEED, 0, 100);
   
   // Convert speed to BCD digits and display
   int hundreds = speedPercent / 100;
   int tens = (speedPercent / 10) % 10;
   int ones = speedPercent % 10;
   
   if (hundreds > 0) {
     displayBCDDigit(7, hundreds);
   }
   displayBCDDigit(8, tens);
   displayBCDDigit(9, ones);
   
   displayASCII(10, '%');
 }
 
 void displayError(ErrorType error) {
   // Set error state
   currentError = error;
   displayingError = true;
   errorStartTime = millis();
   
   // Display error message
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("ERROR:");
   
   lcd.setCursor(0, 1);
   switch (error) {
     case LID_OPEN:
       lcd.print("CLOSE LID");
       break;
     case MOTOR_STALL:
       lcd.print("MOTOR STALLED");
       break;
     case SYSTEM_ERROR:
       lcd.print("SYSTEM ERROR");
       break;
     default:
       lcd.print("UNKNOWN ERROR");
   }
   
   // Also output to serial for debugging
   Serial.printf("Error detected: %d\n", error);
 }
 
 // Function to convert a digit to BCD representation
 char* translateDigitToBCD(int digit) {
   static char bcd[8];  // Array to store the BCD representation
   
   switch (digit) {
     case 0:
       strcpy(bcd, "1111110"); // 0: segments a,b,c,d,e,f active
       break;
     case 1:
       strcpy(bcd, "0110000"); // 1: segments b,c active
       break;
     case 2:
       strcpy(bcd, "1011011"); // 2: segments a,b,d,e,g active
       break;
     case 3:
       strcpy(bcd, "1111001"); // 3: segments a,b,c,d,g active
       break;
     case 4:
       strcpy(bcd, "0110101"); // 4: segments b,c,f,g active
       break;
     case 5:
       strcpy(bcd, "1101101"); // 5: segments a,c,d,f,g active
       break;
     case 6:
       strcpy(bcd, "1101111"); // 6: segments a,c,d,e,f,g active
       break;
     case 7:
       strcpy(bcd, "1110000"); // 7: segments a,b,c active
       break;
     case 8:
       strcpy(bcd, "1111111"); // 8: all segments active
       break;
     case 9:
       strcpy(bcd, "1111101"); // 9: segments a,b,c,d,f,g active
       break;
     default:
       strcpy(bcd, "0000000"); // All segments off for invalid input
   }
   
   return bcd;
 }
 
 // Display a BCD digit at a specific position
 void displayBCDDigit(int position, int digit) {
   // This is a simplified representation. In a real implementation,
   // you would need to map this to actual segments on your LCD.
   // Since we're using a character LCD, we just display the digit
   lcd.setCursor(position, 1);
   lcd.print(digit);
 }
 
 // Function to display an ASCII character
 void displayASCII(int position, char character) {
   lcd.setCursor(position, 1);
   lcd.print(character);
 }