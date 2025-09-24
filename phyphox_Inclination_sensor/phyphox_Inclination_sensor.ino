/*
  Project: ESP32 Phyphox Kalman Buzzer
  Author: Your Name
  Description:
    - BLE server for Phyphox app
    - Kalman filter for IMU data
    - Buzzer alert based on roll angle
    - Timeout detection for experiment stop
  License: MIT
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const int buzzerPin = 32;   // Example Buzzer pin
unsigned long lastbuzzerToggle = 0;
bool buzzerState = LOW;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

// ---- Kalman filter state variables ----
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

float accelXold, accelYold, accelZold;
float gyroXold, gyroYold, gyroZold;
float magXold, magYold, magZold;

float angle_pitch = 0.0f; // state estimate (rad)
float angle_roll  = 0.0f; // state estimate (rad)
float bias_pitch  = 0.0f; // gyro bias estimate (rad/s)
float bias_roll   = 0.0f;

float P_pitch[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
float P_roll[2][2]  = {{0.0f, 0.0f}, {0.0f, 0.0f}};

// Tuning parameters (ปรับตามสภาพจริง)
const float Q_angle = 0.001f;  // process noise variance for the angle
const float Q_gyro  = 0.003f;  // process noise variance for the gyro bias
const float R_angle = 0.03f;   // measurement noise variance - accelerometer

unsigned long lastTime = 0; 
unsigned long previousMillis = 0; 
const long interval = 500;      

bool allValueisZero = true;

// Timeout detection
unsigned long lastPacketTime = 0;         // Time of last received data
const unsigned long timeoutMs = 500;      // If no packet in 500ms → experiment stopped

// For printing final angles (deg)
float pitch_deg = 0.0f;
float roll_deg  = 0.0f;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length == 36) { // 9 floats × 4 bytes
      memcpy(&accelX, data + 0, 4);
      memcpy(&accelY, data + 4, 4);
      memcpy(&accelZ, data + 8, 4);
      memcpy(&gyroX,  data + 12, 4);
      memcpy(&gyroY,  data + 16, 4);
      memcpy(&gyroZ,  data + 20, 4);
      memcpy(&magX,   data + 24, 4);
      memcpy(&magY,   data + 28, 4);
      memcpy(&magZ,   data + 32, 4);

      // Mark last packet arrival
      lastPacketTime = millis();  

      if ((abs(accelXold - accelX) < 0.01) && (abs(accelYold - accelY) < 0.01) && (abs(accelZold - accelZ) < 0.01) &&
          (abs(gyroXold - gyroX) < 0.01) && (abs(gyroYold - gyroY) < 0.01) && (abs(gyroZold - gyroZ) < 0.01) &&
          (abs(magXold - magX) < 0.01) && (abs(magYold - magY) < 0.01) && (abs(magZold - magZ) < 0.01)) {
        
        accelX = accelY = accelZ = 0;
        gyroX  = gyroY  = gyroZ  = 0;
        magX   = magY   = magZ   = 0;

        allValueisZero = true;
      } else {
        allValueisZero = false;
      }

      unsigned long currentMillis = millis(); 
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        accelXold = accelX;
        accelYold = accelY;
        accelZold = accelZ;
        gyroXold = gyroX;
        gyroYold = gyroY;
        gyroZold = gyroZ;
        magXold = magX;
        magYold = magY;
        magZold = magZ;
      }
    } else {
      // Serial.println("Invalid data length: " + String(length));
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);

  BLEDevice::init("ESP32_Phyphox_Output");
  BLEDevice::setMTU(64); // Support 36-byte writes
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEAdvertisementData advData;
  advData.setName("ESP32_Phyphox_Output");
  pAdvertising->setAdvertisementData(advData);
  BLEDevice::startAdvertising();
  Serial.println("BLE server started, waiting for connections...");

  lastTime = 0; 

  pinMode(buzzerPin, OUTPUT);
}

void KalmanFilter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  unsigned long now = millis();
  float dt = 0.0f;
  if (lastTime == 0) {
    dt = 0.01f; 
  } else {
    dt = (now - lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
  }
  lastTime = now;

  float gx_rad = gx * PI / 180.0f;
  float gy_rad = gy * PI / 180.0f;
  float gz_rad = gz * PI / 180.0f;

  float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));
  float roll_acc  = atan2f(ay, az);

  float pitch_dot = gx_rad - bias_pitch;          
  angle_pitch += dt * pitch_dot;                  
  P_pitch[0][0] += dt * (dt*P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
  P_pitch[0][1] -= dt * P_pitch[1][1];
  P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_gyro * dt;

  float y_pitch = pitch_acc - angle_pitch; 
  float S_pitch = P_pitch[0][0] + R_angle; 
  float K0_pitch = P_pitch[0][0] / S_pitch; 
  float K1_pitch = P_pitch[1][0] / S_pitch; 

  angle_pitch += K0_pitch * y_pitch;
  bias_pitch  += K1_pitch * y_pitch;

  float P00_temp = P_pitch[0][0];
  float P01_temp = P_pitch[0][1];
  P_pitch[0][0] -= K0_pitch * P00_temp;
  P_pitch[0][1] -= K0_pitch * P01_temp;
  P_pitch[1][0] -= K1_pitch * P00_temp;
  P_pitch[1][1] -= K1_pitch * P01_temp;

  float roll_dot = gy_rad - bias_roll; 
  angle_roll += dt * roll_dot;
  P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
  P_roll[0][1] -= dt * P_roll[1][1];
  P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_gyro * dt;

  float y_roll = roll_acc - angle_roll;
  float S_roll = P_roll[0][0] + R_angle;
  float K0_roll = P_roll[0][0] / S_roll;
  float K1_roll = P_roll[1][0] / S_roll;

  angle_roll += K0_roll * y_roll;
  bias_roll  += K1_roll * y_roll;

  float P00r = P_roll[0][0];
  float P01r = P_roll[0][1];
  P_roll[0][0] -= K0_roll * P00r;
  P_roll[0][1] -= K0_roll * P01r;
  P_roll[1][0] -= K1_roll * P00r;
  P_roll[1][1] -= K1_roll * P01r;

  pitch_deg = angle_pitch * 180.0f / PI;
  roll_deg  = angle_roll  * 180.0f / PI;

  // Serial.printf("Roll: %.2f deg, Pitch: %.2f deg\n", roll_deg, pitch_deg);
}

void CheckRollAngle(float roll) {
  unsigned long currentMillis = millis();

  if (abs(90 - roll) >= 12) {
    // If it's time to toggle buzzer (every 1000 ms)
    if (currentMillis - lastbuzzerToggle >= 1000) {
      lastbuzzerToggle = currentMillis;
      buzzerState = !buzzerState;  // Toggle state
      digitalWrite(buzzerPin, buzzerState);
    }
  } else {
    // Condition not met → ensure buzzer is OFF
    buzzerState = LOW;
    digitalWrite(buzzerPin, buzzerState);
  }
}

void loop() {
  if (!deviceConnected) {
    delay(10); 
  } else {
    unsigned long now = millis();

    //Timeout check → force zeros
    if ((now - lastPacketTime) > timeoutMs) {
      accelX = accelY = accelZ = 0;
      gyroX  = gyroY  = gyroZ  = 0;
      magX   = magY   = magZ   = 0;
      allValueisZero = true;
    }

    if (!allValueisZero) {
      KalmanFilter(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
      CheckRollAngle(roll_deg);
    } else {
      // Serial.println("Experiment stopped → sending zeros");

      buzzerState = LOW;
      digitalWrite(buzzerPin, buzzerState);
    }
  }
}
