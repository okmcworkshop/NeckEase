#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <MPU6050_tockn.h>
#include <esp_sleep.h>  // 引入睡眠管理標頭檔
#include <WiFi.h>       // 用於確保 Wi-Fi 模組徹底關閉
#include "esp_bt.h"

MPU6050 mpu6050(Wire);

// ---------- 感測器資料結構 ----------
struct SensorData {
  float roll;
  float pitch;
  int ldr;
  float volt;
};

// ---------- BLE UUID ----------
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define IMU_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9F"
#define SWITCH_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ---------- 全域 BLE 物件 ----------
BLEServer* pServer = NULL;
BLECharacteristic* pImuCharacteristic = NULL;
BLECharacteristic* pSwitchCharacteristic = NULL;

// ---------- 姿勢與濾波變數 ----------
float angleRoll = 0, anglePitch = 0;
float dt = 0;
unsigned long lastTime = 0;
unsigned long lastHighFreqUpdate = 0;
float alpha = 0.05;

// 腳位設定
const int LIGHT_SENSOR_PIN = 3;  // GPIO3
const int V_MOTOR_PIN = 1;       // GPIO1

// 警報與狀態標誌
unsigned long alarmStartTime = 0;
unsigned long lastBlinkTime = 0;
int _BlinkCap = 1000;
bool alarmActive = false;
bool deviceConnected = false;
int postureStartTime = 0;

// 校準與姿態判斷變數
float calRoll_10 = 0, calRoll_15 = 0, calRoll_20 = 0, calRoll_25 = 0;  // 對應前傾的 Gyro Roll
float thresholdRoll = 0;                                               // 使用者選擇的閾值所對應的 Gyro Roll
int durationSeconds = 5;                                               // 持續時間（秒）
bool calibrated = false;                                               // 是否已收到校準資料

//unsigned long postureStartTime = 0;  // 姿勢不良開始計時的時間點
bool alertTriggered = false;         // 是否已觸發振動提醒（持續振動中）

// BLE 計時與儲存
Preferences preferences;         // Preferences 物件
bool bleActive = false;          // 是否處於廣播有效期間（用於計時控制）
bool bleInitialized = false;     // 是否已初始化 BLE（避免重複 deinit）
unsigned long bleStartTime = 0;  // 開始計時的時間點

// ---------- 輔助函式：馬達震動  ----------
void blinkMotor(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(V_MOTOR_PIN, HIGH);
    delay(200);
    digitalWrite(V_MOTOR_PIN, LOW);
    delay(100);
  }
}

// ---------- 用於連線debug  ----------
void _print_gyro_data() {
  Serial.print("Roll: ");
  Serial.print(angleRoll, 2);
  Serial.print("\tPitch: ");
  Serial.println(anglePitch, 2);
}

void shutdownBLECompletely() {
  Serial.println("開始關閉 BLE 與藍牙控制器...");

  if (pServer) {
    pServer->getAdvertising()->stop();
  }

  // 1. 釋放 BLEDevice 物件與內部 Task
  BLEDevice::deinit(true);

  // 2. 徹底關閉並釋放底層 BT Controller 資源
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    esp_bt_controller_disable();
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    esp_bt_controller_deinit();
  }

  // 3. 釋放 BLE 記憶體空間給系統使用
  esp_bt_mem_release(ESP_BT_MODE_BLE);

  bleInitialized = false;
  bleActive = false;
  Serial.println("BLE 已完全釋放，準備安全進入 Light Sleep");
}

// ---------- BLE 回呼類別 ----------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("Client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("Client disconnected");
    if (bleInitialized) {
      pServer->startAdvertising();
    }
  }
};

class SwitchCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    value.trim();

    // --- 處理App警報 ---
    if (value == "alarm") {
      alarmActive = true;
      alarmStartTime = millis();
    }

    // --- 處理校準資料（格式：cal,77.8,70.8,66.5,60.5,66.5,10） ---
    if (value.startsWith("cal,")) {
      String data = value.substring(4);
      int comma1 = data.indexOf(',');
      int comma2 = data.indexOf(',', comma1 + 1);
      int comma3 = data.indexOf(',', comma2 + 1);
      int comma4 = data.indexOf(',', comma3 + 1);
      int comma5 = data.indexOf(',', comma4 + 1);

      if (comma1 != -1 && comma2 != -1 && comma3 != -1 && comma4 != -1 && comma5 != -1) {
        calRoll_10 = data.substring(0, comma1).toFloat();
        calRoll_15 = data.substring(comma1 + 1, comma2).toFloat();
        calRoll_20 = data.substring(comma2 + 1, comma3).toFloat();
        calRoll_25 = data.substring(comma3 + 1, comma4).toFloat();
        thresholdRoll = data.substring(comma4 + 1, comma5).toFloat();
        durationSeconds = data.substring(comma5 + 1).toInt();

        calibrated = true;

        // ----- 儲存參數到 Preferences (用於離線運作)-----
        preferences.begin("neGuard", false);
        preferences.putFloat("threshold", thresholdRoll);
        preferences.putInt("duration", durationSeconds);
        preferences.end();

        postureStartTime = 0;
         alertTriggered = false;

        // ----- 重置 BLE 計時器（若 BLE 仍在運行） -----
        if (bleInitialized) {
          bleStartTime = millis();
          bleActive = true;
        }
      }
    }
  }
};

void setupBLE() {
  BLEDevice::init("neGuard");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pImuCharacteristic = pService->createCharacteristic(
    IMU_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pSwitchCharacteristic = pService->createCharacteristic(
    SWITCH_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSwitchCharacteristic->setCallbacks(new SwitchCharacteristicCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  pServer->startAdvertising();

  bleInitialized = true;
}

void setup() {
  Serial.begin(115200);

  // 關閉 Wi-Fi 模組節省電力
  WiFi.mode(WIFI_OFF);

  // 初始化 I2C （SDA=8, SCL=9）
  Wire.begin(8, 9);
  Wire.setClock(100000);

  // 初始化 MPU6050
  mpu6050.begin();

  // 配置光敏電阻腳位（ADC）
  analogReadResolution(12);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(V_MOTOR_PIN, OUTPUT);
  digitalWrite(V_MOTOR_PIN, LOW);

  // 讀取校準參數
  preferences.begin("neGuard", false);
  if (preferences.isKey("threshold") && preferences.isKey("duration")) {
    thresholdRoll = preferences.getFloat("threshold", 0);
    durationSeconds = preferences.getInt("duration", 5);
    calibrated = true;
    blinkMotor(3);
    //讀取成功震動3次
  } else {
    calibrated = false;
    blinkMotor(2);
    //沒有校準資料震動2次
  }
  preferences.end();

  // 5. 啟動 BLE
  setupBLE();

  if (calibrated) {
    bleStartTime = millis();
  }
  bleActive = true;

  lastTime = micros();
  Serial.println("設定完成");
}

void loop() {

  // 判斷當前採樣模式：BLE 仍在工作或手機已連線時，使用高頻模式 (20ms)
  bool isHighFrequencyMode = bleInitialized || deviceConnected;

  if (isHighFrequencyMode) {

    // ==========================================
    // 階段一：高頻模式 (50Hz / 20ms 週期) - 用於校準與 BLE 傳輸
    // ==========================================
    unsigned long nowMs = millis();
    if (nowMs - lastHighFreqUpdate >= 20) {
      lastHighFreqUpdate = nowMs;

      mpu6050.update();

      // 互補濾波姿態計算
      float accX = mpu6050.getAccX();
      float accY = mpu6050.getAccY();
      float accZ = mpu6050.getAccZ();
      float gyroX = mpu6050.getGyroX();
      float gyroY = mpu6050.getGyroY();

      float accelAnglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
      float accelAngleRoll = atan2(accY, accZ) * 180.0 / PI;

      unsigned long nowUs = micros();
      dt = (nowUs - lastTime) / 1000000.0;
      lastTime = nowUs;
      if (dt > 0.1) dt = 0.1;

      angleRoll = alpha * accelAngleRoll + (1 - alpha) * (angleRoll + gyroX * dt);
      anglePitch = alpha * accelAnglePitch + (1 - alpha) * (anglePitch + gyroY * dt);

      uint16_t lightValue = analogRead(LIGHT_SENSOR_PIN);

      // BLE 通知 App
      if (deviceConnected && bleInitialized) {
        SensorData data;
        data.roll = angleRoll;
        data.pitch = anglePitch;
        data.ldr = lightValue;
        data.volt = 0.0;
        pImuCharacteristic->setValue((uint8_t*)&data, sizeof(data));
        pImuCharacteristic->notify();
        _print_gyro_data();
      }
    }

    // ----- 離線姿態判斷（校準後啟用） -----
    if (calibrated) {
      if (anglePitch < thresholdRoll) {
        if (!alertTriggered) {
          Serial.print("姿勢不良 > ");
          Serial.print((millis() - postureStartTime) / 1000);
          Serial.print(" >> ");
          _print_gyro_data();
          if (postureStartTime == 0) {
            postureStartTime = millis();
          } else {
            if (millis() - postureStartTime >= (unsigned long)durationSeconds * 1000) {
              alertTriggered = true;
              postureStartTime = millis();
              Serial.println("姿勢不良超過設定時間，啟動振動提醒！");
            }
          }
        }
      } else {
        if (alertTriggered) {
          digitalWrite(V_MOTOR_PIN, LOW);
          alertTriggered = false;
          Serial.println("姿勢已恢復，關閉振動");
          _print_gyro_data();
        }
        postureStartTime = 0;
      }
    }

    // ----- 處理手動警報（alarm）超時清除 -----
    if (alarmActive && (millis() - alarmStartTime >= 100)) {
      alarmActive = false;
      Serial.println("Alarm timer expired, flag cleared");
    }

    // ----- 馬達狀態統一控制 -----
    if (alertTriggered && millis() - lastBlinkTime >= _BlinkCap) {
      blinkMotor(1);
      lastBlinkTime = millis();
    } else if (alarmActive) {
      digitalWrite(V_MOTOR_PIN, HIGH);
    } else {
      digitalWrite(V_MOTOR_PIN, LOW);
    }

    // ---------- BLE 30 秒超時關閉判斷 ----------
    if (calibrated && bleActive && !deviceConnected) {
      if (millis() - bleStartTime >= 30000) {
        shutdownBLECompletely();
      }
    }
    delay(1);

  } else {
    Serial.println("isLowFrequencyMode");
    // ==========================================
    // 階段二：離線省電模式  - 30秒關閉 BLE 後切換至此
    // ==========================================
    mpu6050.update();
    float accX = mpu6050.getAccX();
    float accY = mpu6050.getAccY();
    float accZ = mpu6050.getAccZ();

    // 加速度計靜態姿態計算（無需陀螺儀高頻積分）
    anglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    angleRoll = atan2(accY, accZ) * 180.0 / PI;

    if (anglePitch < thresholdRoll) {
      Serial.println("姿勢不良！");
      blinkMotor(1);
    }

    Serial.println("Try Light Sleep");
    uint64_t sleepUs = (uint64_t)(durationSeconds * 1000) * 1000;

    // 設定睡眠定時器喚醒
    esp_sleep_enable_timer_wakeup(sleepUs);
    Serial.flush();  // 確保 Serial 列印完全輸出後才切斷 CPU 時鐘
    esp_light_sleep_start();
  }
}
