/* 
MIT License

Copyright (c) 2025 Webrobotix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 * Webrobotix 16-Channel RC Servo Controller with PIR Motion Detection
 * Version 1.8.0 - PIR + Dual LED Integration (matches Servo16_4.pde)
 * Using Adafruit 16-channel 12-bit servo shield
 * Communicates with Processing for UI control
 * PIR sensor integration for motion-triggered sequences
 * Two independently-pinned LEDs, digital on/off control (blink timing lives in the app)
 * PIR and both LED data pins are configurable at runtime via serial commands
 * NEW: Per-LED brightness control (LED:BRIGHTNESS:x / LED2:BRIGHTNESS:x, 0-255)
 *      Requires a PWM-capable pin (3,5,6,9,10,11 on Uno) for true fading -
 *      on a non-PWM pin the value is just thresholded to on/off by the core. 8/23/2026
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constants
const int NUM_SERVOS = 16;
const int SERVOMIN = 100;
const int SERVOMAX = 600;
const int SERVO_FREQ = 50;

// PIR Sensor Configuration
int pirPin = 2;  // Digital pin for PIR sensor (change via PIR:PIN:x command)
bool pirEnabled = false;
bool pirState = LOW;
bool lastPirState = LOW;
unsigned long pirTriggerTime = 0;
unsigned long pirCooldownPeriod = 5000;  // 5 seconds cooldown between triggers
bool pirSequencePlaying = false;

// LED Configuration (blink timing is handled by the app; brightness sets the "on" level)
int ledPin = 10;  // Digital pin for LED 1 (change via LED:PIN:x command)
bool ledState = false;
int ledBrightness = 255;  // 0-255, applied whenever the LED is ON
int led2Pin = 11;  // Digital pin for LED 2 (change via LED2:PIN:x command)
bool led2State = false;
int led2Brightness = 255;  // 0-255, applied whenever LED 2 is ON

// Servo positions (0-180 degrees)
int servoPositions[NUM_SERVOS];

// Min/Max limits for each servo (0-180 degrees)
int servoMinLimits[NUM_SERVOS];
int servoMaxLimits[NUM_SERVOS];

// Center positions for each servo (0-180 degrees)
int servoCenterPositions[NUM_SERVOS];

// Active/Inactive state for each servo
bool servoActive[NUM_SERVOS];

// Command parsing
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize servo driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize PIR sensor pin
  pinMode(pirPin, INPUT);

  // Initialize LED pins
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(led2Pin, OUTPUT);
  digitalWrite(led2Pin, LOW);

  // Initialize with default values
  initializeDefaults();
  
  // Reserve memory for serial input
  inputString.reserve(50);
  
  Serial.println("Servo Controller with PIR Ready");
  Serial.print("PIR Sensor on Pin: ");
  Serial.println(pirPin);
  Serial.print("LED 1 on Pin: ");
  Serial.println(ledPin);
  Serial.print("LED 2 on Pin: ");
  Serial.println(led2Pin);
  
  // Send all current data to Processing on startup
  delay(1000);
  sendAllData();
}

void loop() {
  // Process any incoming commands
  if (stringComplete) {
    processCommand();
    inputString = "";
    stringComplete = false;
  }
  
  // Check PIR sensor if enabled
  if (pirEnabled) {
    checkPIRSensor();
  }
}

// Check PIR sensor for motion detection
void checkPIRSensor() {
  pirState = digitalRead(pirPin);
  
  // Detect rising edge (motion detected)
  if (pirState == HIGH && lastPirState == LOW) {
    // Check if we're past the cooldown period
    if (millis() - pirTriggerTime > pirCooldownPeriod) {
      pirTriggerTime = millis();
      
      // Notify Processing that motion was detected
      Serial.println("PIR:MOTION_DETECTED");
      
      // Optionally trigger a sequence directly from Arduino
      // (or let Processing handle it via the PIR:MOTION_DETECTED message)
    }
  }
  
  lastPirState = pirState;
}

// Initialize all servos with default values
void initializeDefaults() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoMinLimits[i] = 0;
    servoMaxLimits[i] = 180;
    servoCenterPositions[i] = 90;
    servoActive[i] = false;
    servoPositions[i] = 90;
    
    // Turn off all PWM outputs
    pwm.setPWM(i, 0, 0);
  }
}

// Function to update servo position
void updateServo(int servoNum) {
  if (servoNum < 0 || servoNum >= NUM_SERVOS) return;
  
  if (servoActive[servoNum]) {
    int pulseWidth = map(servoPositions[servoNum], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoNum, 0, pulseWidth);
  } else {
    pwm.setPWM(servoNum, 0, 0);
  }
}

// Function to process incoming commands
void processCommand() {
  inputString.trim();
  
  if (inputString.startsWith("S:")) {
    // Servo position command: S:servoNum:position
    int firstColon = inputString.indexOf(':');
    int secondColon = inputString.indexOf(':', firstColon + 1);
    
    if (firstColon > 0 && secondColon > firstColon) {
      int servoNum = inputString.substring(firstColon + 1, secondColon).toInt();
      int position = inputString.substring(secondColon + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS && position >= 0 && position <= 180) {
        if (servoActive[servoNum]) {
          position = constrain(position, servoMinLimits[servoNum], servoMaxLimits[servoNum]);
          servoPositions[servoNum] = position;
          updateServo(servoNum);
          
          Serial.print("PO:");
          Serial.print(servoNum);
          Serial.print(":");
          Serial.println(position);
        }
      }
    }
  }
  else if (inputString.startsWith("SETMINLIMIT:")) {
    int firstColon = inputString.indexOf(':');
    int secondColon = inputString.indexOf(':', firstColon + 1);
    
    if (firstColon > 0 && secondColon > firstColon) {
      int servoNum = inputString.substring(firstColon + 1, secondColon).toInt();
      int minLimit = inputString.substring(secondColon + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS && minLimit >= 0 && minLimit <= 180) {
        servoMinLimits[servoNum] = minLimit;
        Serial.print("MI:");
        Serial.print(servoNum);
        Serial.print(":");
        Serial.println(minLimit);
      }
    }
  }
  else if (inputString.startsWith("SETMAXLIMIT:")) {
    int firstColon = inputString.indexOf(':');
    int secondColon = inputString.indexOf(':', firstColon + 1);
    
    if (firstColon > 0 && secondColon > firstColon) {
      int servoNum = inputString.substring(firstColon + 1, secondColon).toInt();
      int maxLimit = inputString.substring(secondColon + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS && maxLimit >= 0 && maxLimit <= 180) {
        servoMaxLimits[servoNum] = maxLimit;
        Serial.print("MA:");
        Serial.print(servoNum);
        Serial.print(":");
        Serial.println(maxLimit);
      }
    }
  }
  else if (inputString.startsWith("SETCENTER:")) {
    int firstColon = inputString.indexOf(':');
    int secondColon = inputString.indexOf(':', firstColon + 1);
    
    if (firstColon > 0 && secondColon > firstColon) {
      int servoNum = inputString.substring(firstColon + 1, secondColon).toInt();
      int centerPos = inputString.substring(secondColon + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS && centerPos >= 0 && centerPos <= 180) {
        servoCenterPositions[servoNum] = centerPos;
        Serial.print("CE:");
        Serial.print(servoNum);
        Serial.print(":");
        Serial.println(centerPos);
      }
    }
  }
  else if (inputString.startsWith("RESET:")) {
    int servoNum = inputString.substring(6).toInt();
    if (servoNum >= 0 && servoNum < NUM_SERVOS) {
      servoMinLimits[servoNum] = 0;
      servoMaxLimits[servoNum] = 180;
      servoCenterPositions[servoNum] = 90;
      
      Serial.print("RE:");
      Serial.print(servoNum);
      Serial.print(":");
      Serial.println("RESET");
    }
  }
  else if (inputString.startsWith("ACTIVE:")) {
    int firstColon = inputString.indexOf(':');
    int secondColon = inputString.indexOf(':', firstColon + 1);
    
    if (firstColon > 0 && secondColon > firstColon) {
      int servoNum = inputString.substring(firstColon + 1, secondColon).toInt();
      int state = inputString.substring(secondColon + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS) {
        servoActive[servoNum] = (state == 1);
        updateServo(servoNum);
        
        Serial.print("AC:");
        Serial.print(servoNum);
        Serial.print(":");
        Serial.println(servoActive[servoNum] ? 1 : 0);
      }
    }
  }
  else if (inputString.equals("ALLACTIVE")) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servoActive[i] = true;
      updateServo(i);
      Serial.print("AC:");
      Serial.print(i);
      Serial.print(":");
      Serial.println(1);
    }
  }
  else if (inputString.equals("ALLINACTIVE")) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servoActive[i] = false;
      updateServo(i);
      Serial.print("AC:");
      Serial.print(i);
      Serial.print(":");
      Serial.println(0);
    }
  }
  else if (inputString.startsWith("PIR:")) {
    // PIR control commands
    if (inputString.equals("PIR:ENABLE")) {
      pirEnabled = true;
      Serial.println("PIR:ENABLED");
    }
    else if (inputString.equals("PIR:DISABLE")) {
      pirEnabled = false;
      Serial.println("PIR:DISABLED");
    }
    else if (inputString.startsWith("PIR:COOLDOWN:")) {
      // Set cooldown period: PIR:COOLDOWN:milliseconds
      int cooldown = inputString.substring(13).toInt();
      if (cooldown >= 0 && cooldown <= 60000) {  // Max 60 seconds
        pirCooldownPeriod = cooldown;
        Serial.print("PIR:COOLDOWN:");
        Serial.println(cooldown);
      }
    }
    else if (inputString.startsWith("PIR:PIN:")) {
      // Set PIR data pin: PIR:PIN:pinNumber
      int newPin = inputString.substring(8).toInt();
      if (newPin >= 0 && newPin <= 53) {
        pirPin = newPin;
        pinMode(pirPin, INPUT);
        lastPirState = LOW;  // avoid a false trigger from the pin switch
        Serial.print("PIR:PIN:");
        Serial.println(pirPin);
      }
    }
    else if (inputString.equals("PIR:STATUS")) {
      Serial.print("PIR:STATUS:");
      Serial.print(pirEnabled ? "ENABLED" : "DISABLED");
      Serial.print(":");
      Serial.print(pirCooldownPeriod);
      Serial.print(":");
      Serial.println(digitalRead(pirPin) == HIGH ? "MOTION" : "CLEAR");
    }
  }
  else if (inputString.startsWith("LED:")) {
    // LED 1 control commands
    if (inputString.equals("LED:ON")) {
      ledState = true;
      analogWrite(ledPin, ledBrightness);
    }
    else if (inputString.equals("LED:OFF")) {
      ledState = false;
      analogWrite(ledPin, 0);
    }
    else if (inputString.startsWith("LED:BRIGHTNESS:")) {
      // Set LED 1 brightness: LED:BRIGHTNESS:0-255
      int brightness = inputString.substring(15).toInt();
      if (brightness >= 0 && brightness <= 255) {
        ledBrightness = brightness;
        if (ledState) {
          analogWrite(ledPin, ledBrightness);  // live preview if already on
        }
        Serial.print("LED:BRIGHTNESS:");
        Serial.println(ledBrightness);
      }
    }
    else if (inputString.startsWith("LED:PIN:")) {
      // Set LED 1 data pin: LED:PIN:pinNumber
      int newPin = inputString.substring(8).toInt();
      if (newPin >= 0 && newPin <= 53) {
        analogWrite(ledPin, 0);  // leave the old pin in a known state
        ledPin = newPin;
        pinMode(ledPin, OUTPUT);
        analogWrite(ledPin, ledState ? ledBrightness : 0);
        Serial.print("LED:PIN:");
        Serial.println(ledPin);
      }
    }
  }
  else if (inputString.startsWith("LED2:")) {
    // LED 2 control commands
    if (inputString.equals("LED2:ON")) {
      led2State = true;
      analogWrite(led2Pin, led2Brightness);
    }
    else if (inputString.equals("LED2:OFF")) {
      led2State = false;
      analogWrite(led2Pin, 0);
    }
    else if (inputString.startsWith("LED2:BRIGHTNESS:")) {
      // Set LED 2 brightness: LED2:BRIGHTNESS:0-255
      int brightness = inputString.substring(16).toInt();
      if (brightness >= 0 && brightness <= 255) {
        led2Brightness = brightness;
        if (led2State) {
          analogWrite(led2Pin, led2Brightness);  // live preview if already on
        }
        Serial.print("LED2:BRIGHTNESS:");
        Serial.println(led2Brightness);
      }
    }
    else if (inputString.startsWith("LED2:PIN:")) {
      // Set LED 2 data pin: LED2:PIN:pinNumber
      int newPin = inputString.substring(9).toInt();
      if (newPin >= 0 && newPin <= 53) {
        analogWrite(led2Pin, 0);  // leave the old pin in a known state
        led2Pin = newPin;
        pinMode(led2Pin, OUTPUT);
        analogWrite(led2Pin, led2State ? led2Brightness : 0);
        Serial.print("LED2:PIN:");
        Serial.println(led2Pin);
      }
    }
  }
  else if (inputString.equals("SAVE")) {
    Serial.println("LI:ACKNOWLEDGED");
  }
  else if (inputString.equals("GETALL")) {
    sendAllData();
  }
}

// Function to send all servo data to Processing
void sendAllData() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("PO:");
    Serial.print(i);
    Serial.print(":");
    Serial.println(servoPositions[i]);
    
    Serial.print("MI:");
    Serial.print(i);
    Serial.print(":");
    Serial.println(servoMinLimits[i]);
    
    Serial.print("MA:");
    Serial.print(i);
    Serial.print(":");
    Serial.println(servoMaxLimits[i]);
    
    Serial.print("CE:");
    Serial.print(i);
    Serial.print(":");
    Serial.println(servoCenterPositions[i]);
    
    Serial.print("AC:");
    Serial.print(i);
    Serial.print(":");
    Serial.println(servoActive[i] ? 1 : 0);
    
    delay(10);
  }
  
  // Send PIR status
  Serial.print("PIR:STATUS:");
  Serial.print(pirEnabled ? "ENABLED" : "DISABLED");
  Serial.print(":");
  Serial.print(pirCooldownPeriod);
  Serial.print(":");
  Serial.println(digitalRead(pirPin) == HIGH ? "MOTION" : "CLEAR");

  // Send LED status
  Serial.print("LED:PIN:");
  Serial.println(ledPin);
  Serial.print("LED:BRIGHTNESS:");
  Serial.println(ledBrightness);
  Serial.print("LED2:PIN:");
  Serial.println(led2Pin);
  Serial.print("LED2:BRIGHTNESS:");
  Serial.println(led2Brightness);
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
