#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Preferences.h>  // 用於持久化儲存

#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);

// ---------- 感測器資料結構（與原 App 相容）----------
struct SensorData {
  float roll;
  float pitch;
  int ldr;
  float volt;
};

// ---------- BLE UUID（與之前完全一致）----------
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define IMU_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9F"
#define SWITCH_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define LIGHT_CHAR_UUID "6E400004-B5A3-F393-E0A9-E50E24DCCA9F"

// ---------- 全域 BLE 物件 ----------
BLEServer* pServer = NULL;
BLECharacteristic* pImuCharacteristic = NULL;
BLECharacteristic* pSwitchCharacteristic = NULL;
BLECharacteristic* pLightCharacteristic = NULL;

// ---------- MPU6050 物件 ----------
int16_t ax, ay, az;
int16_t gx, gy, gz;

// 互補濾波變數
float angleRoll = 0, anglePitch = 0;
float dt = 0;
unsigned long lastTime = 0;
float alpha = 0.05;
float accelAngleRoll, accelAnglePitch;

// 陀螺儀零偏校準值
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

// 光敏電阻腳位
const int LIGHT_SENSOR_PIN = 3;  // GPIO3
const int V_MOTOR_PIN = 1;       // GPIO1

// 警報（來自 App 手動觸發）
unsigned long alarmStartTime = 0;
bool alarmActive = false;

// 連線狀態標誌（用於列印）
bool deviceConnected = false;

// ---------- 離線姿態判斷相關變數 ----------
// 校準參數（來自 Flutter 傳送的 "cal,..." 字串）
float calRoll_10 = 0;     // 對應前傾 10 度的 Gyro Roll
float calRoll_15 = 0;     // 對應前傾 15 度的 Gyro Roll
float calRoll_20 = 0;     // 對應前傾 20 度的 Gyro Roll
float calRoll_25 = 0;     // 對應前傾 25 度的 Gyro Roll
float thresholdRoll = 0;  // 使用者選擇的閾值所對應的 Gyro Roll
int durationSeconds = 5;  // 持續時間（秒）
bool calibrated = false;  // 是否已收到校準資料

unsigned long postureStartTime = 0;  // 姿勢不良開始計時的時間點
bool alertTriggered = false;         // 是否已觸發振動提醒（持續振動中）

// ---------- 新增：BLE 計時與持久化儲存 ----------
Preferences preferences;         // Preferences 物件
bool bleActive = false;          // 是否處於廣播有效期間（用於計時控制）
bool bleInitialized = false;     // 是否已初始化 BLE（避免重複 deinit）
unsigned long bleStartTime = 0;  // 開始計時的時間點

// ---------- 輔助函式：馬達閃爍 ----------
void blinkMotor(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(V_MOTOR_PIN, HIGH);
    delay(200);
    digitalWrite(V_MOTOR_PIN, LOW);
    delay(100);
  }
}

// ---------- 回呼類別：處理連線 / 斷線 ----------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
    // 重新開始廣播，讓手機可以再次連線（只在 BLE 未關閉時有效）
    if (bleInitialized) {
      pServer->startAdvertising();
    }
  }
};

// ---------- 回呼類別：處理寫入「開關特徵」 ----------
class SwitchCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    value.trim();
    Serial.print("Switch characteristic written: ");
    Serial.println(value.c_str());

    // --- 處理手動警報（原功能） ---
    if (value == "alarm") {
      Serial.println("Alarm triggered! GPIO0 HIGH");
      alarmActive = true;
      alarmStartTime = millis();
    }

    // --- 處理校準資料（格式：cal,77.8,70.8,66.5,60.5,66.5,10） ---
    if (value.startsWith("cal,")) {
      // 去掉 "cal," 前置字串
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
        Serial.println("校準資料已接收：");
        Serial.printf("10°: %.2f, 15°: %.2f, 20°: %.2f, 25°: %.2f\n", calRoll_10, calRoll_15, calRoll_20, calRoll_25);
        Serial.printf("閾值 Roll: %.2f, 持續時間: %d 秒\n", thresholdRoll, durationSeconds);

        // ----- 新增：儲存參數到 Preferences -----
        preferences.begin("neGuard", false);
        preferences.putFloat("threshold", thresholdRoll);
        preferences.putInt("duration", durationSeconds);
        preferences.end();
        Serial.println("threshold & duration saved to NVS");

        // 重置離線判斷狀態
        postureStartTime = 0;
        alertTriggered = false;

        // ----- 新增：重置 BLE 計時器（若 BLE 仍在運行） -----
        if (bleInitialized) {
          bleStartTime = millis();
          bleActive = true;  // 確保計時生效
          Serial.println("BLE 計時器已重置，將在 30 秒後關閉");
        }
      } else {
        Serial.println("校準資料格式錯誤，應為：cal,val10,val15,val20,val25,threshold,duration");
      }
    }
  }
};

// ---------- 設定 BLE 服務與特徵 ----------
void setupBLE() {
  BLEDevice::init("neGuard");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pImuCharacteristic = pService->createCharacteristic(
    IMU_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pImuCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ);

  pSwitchCharacteristic = pService->createCharacteristic(
    SWITCH_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSwitchCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pSwitchCharacteristic->setCallbacks(new SwitchCharacteristicCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  pServer->startAdvertising();
  Serial.println("BLE advertising started");

  bleInitialized = true;  // 標記 BLE 已初始化
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 初始化 I2C（SDA=8, SCL=9）
  Wire.begin(8, 9);
  Wire.setClock(100000);

  // 初始化 MPU6050
   mpu6050.begin();

  // 配置光敏電阻腳位（ADC）
  analogReadResolution(12);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(V_MOTOR_PIN, OUTPUT);

  // ----- 新增：從 Preferences 讀取校準參數 -----
  preferences.begin("neGuard", false);
  if (preferences.isKey("threshold") && preferences.isKey("duration")) {
    thresholdRoll = preferences.getFloat("threshold", 0);
    durationSeconds = preferences.getInt("duration", 5);
    calibrated = true;
    Serial.println("從 NVS 讀取校準參數成功：");
    Serial.printf("閾值 Roll: %.2f, 持續時間: %d 秒\n", thresholdRoll, durationSeconds);
    //讀取成功震動三次
    blinkMotor(3);
  } else {
    calibrated = false;
    Serial.println("NVS 中無校準參數，將持續廣播等待校準");
    //沒有校準資料震動兩次
    blinkMotor(2);
  }
  preferences.end();

  // ----- 啟動 BLE -----
  setupBLE();  // 內部設置 bleInitialized = true

  // ----- 根據是否有校準參數決定 BLE 計時行為 -----
  if (calibrated) {
    // 已有校準資料：啟動 30 秒計時
    bleStartTime = millis();
    bleActive = true;
    Serial.println("已校準，BLE 將在 30 秒後自動關閉");
  } else {
    // 無校準資料：持續廣播（不啟動計時）
    bleActive = true;  // 永遠 active，但不會觸發關閉
    Serial.println("未校準，BLE 持續廣播等待校準資料");
  }

  lastTime = micros();
}

// ---------- Loop ----------
void loop() {
  // 非阻塞 IMU 更新（50 Hz = 20ms 間隔）
  static unsigned long lastSensorUpdate = 0;
  const unsigned long sensorUpdateInterval = 20;

  if (millis() - lastSensorUpdate >= sensorUpdateInterval) {
    lastSensorUpdate = millis();
    // 讀取 MPU6050
    mpu6050.update();
    
    float accX = mpu6050.getAccX() / 16384.0;
    float accY = mpu6050.getAccY() / 16384.0;
    float accZ = mpu6050.getAccZ() / 16384.0;

    float gyroX = (gx / 131.0) - gyroX_offset;
    float gyroY = (gy / 131.0) - gyroY_offset;
    float gyroZ = (gz / 131.0) - gyroZ_offset;

    accelAnglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    accelAngleRoll = atan2(accY, accZ) * 180.0 / PI;

    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt > 0.1) dt = 0.1;

    angleRoll = alpha * accelAngleRoll + (1 - alpha) * (angleRoll + gyroX * dt);
    anglePitch = alpha * accelAnglePitch + (1 - alpha) * (anglePitch + gyroY * dt);

    uint16_t lightValue = analogRead(LIGHT_SENSOR_PIN);

    // ----- 傳送 IMU 資料（若已連線） -----
    if (deviceConnected && bleInitialized) {
      SensorData data;
      data.roll = angleRoll;
      data.pitch = anglePitch;
      data.ldr = lightValue;
      data.volt = 0.0;
      pImuCharacteristic->setValue((uint8_t*)&data, sizeof(data));
      pImuCharacteristic->notify();

      Serial.print("BLE >  ");

      Serial.print("\tLight: ");
      Serial.print(lightValue);
      _print_gyro_data();
    }

    // ----- 離線姿態判斷（校準後啟用） -----
    if (calibrated) {
      if (anglePitch < thresholdRoll) {
        if (!alertTriggered) {
          Serial.print("姿勢不良 > ");
          Serial.print((millis() - postureStartTime) / 1000 );
          Serial.print(" >> ");
          _print_gyro_data();
          if (postureStartTime == 0) {
            postureStartTime = millis();
          } else {
            if (millis() - postureStartTime >= (unsigned long)durationSeconds * 1000) {
              alertTriggered = true;
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

    // ----- 列印除錯資訊（每 5 秒） -----
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
      lastPrint = millis();
      _print_gyro_data();
      if (calibrated) {
        Serial.printf("Threshold: %.2f, Duration: %d s, Alert: %d\n", thresholdRoll, durationSeconds, alertTriggered);
      }
    }
  }

  // ----- 處理手動警報（alarm）超時清除 -----
  if (alarmActive && (millis() - alarmStartTime >= 100)) {
    alarmActive = false;
    Serial.println("Alarm timer expired, flag cleared");
  }

  // ----- 馬達狀態統一控制 -----
  if (alertTriggered) {
    blinkMotor(1);
  } else if (alarmActive) {
    digitalWrite(V_MOTOR_PIN, HIGH);
  } else {
    digitalWrite(V_MOTOR_PIN, LOW);
  }

  // ----- 新增：BLE 30 秒計時關閉（僅在校準後且 BLE 還活躍時） -----
  if (calibrated && bleActive && bleInitialized && !deviceConnected) {
    if (millis() - bleStartTime >= 30000) {
      Serial.println("30 秒已到，停止 BLE 以節省電力");
      BLEDevice::deinit();
      bleInitialized = false;
      bleActive = false;
      btStop();
      // 可選：停止廣告（deinit 已完全關閉）
      Serial.println("BLE 已完全關閉");
    }
  }
  delay(1);
}

void _print_gyro_data() {
  Serial.print("Roll: ");
  Serial.print(angleRoll, 2);
  Serial.print("\tPitch: ");
  Serial.println(anglePitch, 2);
}
