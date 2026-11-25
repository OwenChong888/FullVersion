#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// --- OLED Config ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
#define OLED_SDA 11 
#define OLED_SCL 12 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- L298N Motor Driver Pins ---
const int ENA = 4;  
const int IN1 = 5;  
const int IN2 = 6;  
const int IN3 = 7;  
const int IN4 = 15; 
const int ENB = 16; 

// --- 5-Channel IR Sensor Pins ---
const int IR1 = 17; 
const int IR2 = 18; 
const int IR3 = 8;  
const int IR4 = 9;  
const int IR5 = 10; 

// --- Gripper Servo Config ---
Servo gripperServo;
const int servoPin = 13;
const int GRIPPER_OPEN_ANGLE = 0;
const int GRIPPER_CLOSED_ANGLE = 90;

// --- PWM Config ---
const int freq = 20000; 
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; 

const int autoBaseSpeed = 155; 
const int manualDriveSpeed = 200;

float Kp = 40.0; 
float Ki = 0.0;  
float Kd = 40.0; 
float previousError = 0;
float integral = 0;

// --- State Machine & Thread-Safe Variables ---
enum RobotMode { MODE_MANUAL, MODE_AUTO };
volatile RobotMode currentMode = MODE_MANUAL; 

volatile int targetLeftSpeed = 0; 
volatile int targetRightSpeed = 0;
volatile bool isGripperClosed = false; // Toggled by Button 3

// Track states for OLED & Servo
RobotMode lastDisplayedMode = MODE_AUTO; 
bool lastDisplayedBLE = true;            
bool lastGripperState = true; // Force initial sync on boot

// --- BLE Configuration ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// --- OLED Update Function ---
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  if (deviceConnected) display.println("BLE: CONNECTED");
  else display.println("BLE: WAITING...");

  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

  display.setCursor(0, 20);
  display.setTextSize(2);
  if (currentMode == MODE_MANUAL) display.println("MANUAL");
  else display.println("AUTO PID");

  display.setCursor(0, 48);
  display.setTextSize(1);
  if (isGripperClosed) display.println("GRIPPER: CLOSED");
  else display.println("GRIPPER: OPEN");

  display.display();
}

// --- Motor Control Helper ---
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed; 
  }

  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed; 
  }

  ledcWrite(pwmChannelLeft, leftSpeed);
  ledcWrite(pwmChannelRight, rightSpeed);
}

// --- PID Line Following Function ---
void pidLineFollow() {
  int s1 = !digitalRead(IR1);
  int s2 = !digitalRead(IR2);
  int s3 = !digitalRead(IR3);
  int s4 = !digitalRead(IR4);
  int s5 = !digitalRead(IR5);
  int activeSensors = s1 + s2 + s3 + s4 + s5;

  if (activeSensors >= 4) {
    setMotors(0, 0);
    return; 
  }

  float error = 0;
  if (activeSensors == 0) {
    if (previousError <= -1.0) { setMotors(-125, 125); return; } 
    else if (previousError >= 1.0) { setMotors(125, -125); return; } 
    else { error = previousError; }
  } else {
    float sum = (s1 * -2.0) + (s2 * -1.0) + (s3 * 0.0) + (s4 * 1.0) + (s5 * 2.0);
    error = sum / activeSensors;
  }

  float P = error;
  integral += error; 
  integral = constrain(integral, -50.0, 50.0); 
  float D = error - previousError; 
  float pidValue = (Kp * P) + (Ki * integral) + (Kd * D);
  previousError = error;

  int currentBaseSpeed = autoBaseSpeed;
  if (error <= -1.0 || error >= 1.0) currentBaseSpeed = autoBaseSpeed - 35;

  setMotors(currentBaseSpeed + pidValue, currentBaseSpeed - pidValue);
}

// --- BLE Callbacks (Core 0) ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      currentMode = MODE_MANUAL;
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      currentMode = MODE_MANUAL;
      targetLeftSpeed = 0; 
      targetRightSpeed = 0;
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      const uint8_t* rxData = pCharacteristic->getData();
      size_t rxLength = pCharacteristic->getLength();

      if (rxLength >= 4 && rxData[0] == '!' && rxData[1] == 'B') {
        char button = (char)rxData[2]; 
        char action = (char)rxData[3]; 

        // Button press events (action == '1')
        if (action == '1') {
          if (button == '1') {
            currentMode = MODE_MANUAL;
            targetLeftSpeed = 0; targetRightSpeed = 0;
            return;
          } 
          else if (button == '2') {
            currentMode = MODE_AUTO;
            previousError = 0; integral = 0;
            return;
          }
          else if (button == '3') {
            // Toggle gripper open/closed on the same button
            isGripperClosed = !isGripperClosed;
            return;
          }
        }

        // Manual D-Pad Controls
        if (currentMode == MODE_MANUAL) {
          if (action == '0') {
            targetLeftSpeed = 0; targetRightSpeed = 0; 
          } 
          else if (action == '1') {
            switch (button) {
              case '5': targetLeftSpeed = manualDriveSpeed; targetRightSpeed = manualDriveSpeed; break;   
              case '6': targetLeftSpeed = -manualDriveSpeed; targetRightSpeed = -manualDriveSpeed; break; 
              case '7': targetLeftSpeed = -manualDriveSpeed; targetRightSpeed = manualDriveSpeed; break;  
              case '8': targetLeftSpeed = manualDriveSpeed; targetRightSpeed = -manualDriveSpeed; break;  
            }
          }
        }
      } 
    }
};

void setup() {
  // 1. Instantly kill bootloader pins
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  pinMode(ENA, OUTPUT); digitalWrite(ENA, LOW);
  pinMode(ENB, OUTPUT); digitalWrite(ENB, LOW);

  Wire.begin(OLED_SDA, OLED_SCL);
  if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("System Booting...");
    display.display();
  }

  // 2. Motor PWM (Uses Timer 0 via Channels 0 & 1)
  ledcSetup(pwmChannelLeft, freq, pwmResolution);
  ledcAttachPin(ENA, pwmChannelLeft);
  ledcSetup(pwmChannelRight, freq, pwmResolution);
  ledcAttachPin(ENB, pwmChannelRight);

  // 3. Gripper Servo (Use Timer 2 so it never conflicts with Motor Timer 0)
  ESP32PWM::allocateTimer(2);
  gripperServo.setPeriodHertz(50);
  gripperServo.attach(servoPin, 500, 2400);
  gripperServo.write(GRIPPER_OPEN_ANGLE);

  pinMode(IR1, INPUT); pinMode(IR2, INPUT); pinMode(IR3, INPUT);
  pinMode(IR4, INPUT); pinMode(IR5, INPUT);

  setMotors(0, 0);

  BLEDevice::init("ESP32-S3-Robot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}

void loop() {
  // 1. Update Servo only when toggled
  if (isGripperClosed != lastGripperState) {
    if (isGripperClosed) {
      gripperServo.write(GRIPPER_CLOSED_ANGLE);
    } else {
      gripperServo.write(GRIPPER_OPEN_ANGLE);
    }
    lastGripperState = isGripperClosed;
    updateOLED();
  }

  // 2. Update OLED if Mode or BLE status changed
  if (currentMode != lastDisplayedMode || deviceConnected != lastDisplayedBLE) {
    updateOLED();
    lastDisplayedMode = currentMode;
    lastDisplayedBLE = deviceConnected;
  }

  // 3. Run Robot Logic
  if (currentMode == MODE_AUTO) {
    pidLineFollow();
    delay(5); 
  } else {
    setMotors(targetLeftSpeed, targetRightSpeed);
    delay(20); 
  }
}