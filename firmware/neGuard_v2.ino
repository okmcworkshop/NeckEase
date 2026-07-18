#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

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
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// 互補濾波變數
float angleRoll = 0, anglePitch = 0;
float dt = 0;
unsigned long lastTime = 0;
float alpha = 0.05;  // 互補濾波係數（小值更信任陀螺儀）
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
float calRoll_10 = 0;    // 對應前傾 10 度的 Gyro Roll
float calRoll_15 = 0;    // 對應前傾 15 度的 Gyro Roll
float calRoll_20 = 0;    // 對應前傾 20 度的 Gyro Roll
float calRoll_25 = 0;    // 對應前傾 25 度的 Gyro Roll
float thresholdRoll = 0; // 使用者選擇的閾值所對應的 Gyro Roll
int durationSeconds = 5; // 持續時間（秒）
bool calibrated = false; // 是否已收到校準資料

unsigned long postureStartTime = 0; // 姿勢不良開始計時的時間點
bool alertTriggered = false;        // 是否已觸發振動提醒（持續振動中）

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
    // 重新開始廣播，讓手機可以再次連線
    pServer->startAdvertising();
  }
};

// ---------- 回呼類別：處理寫入「開關特徵」 ----------
class SwitchCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    value.trim(); // 移除前後空白或換行
    Serial.print("Switch characteristic written: ");
    Serial.println(value.c_str());

    // --- 處理手動警報（原功能） ---
    if (value == "alarm") {
      Serial.println("Alarm triggered! GPIO0 HIGH");
      alarmActive = true;
      alarmStartTime = millis();
      // 注意：GPIO 實際控制由 loop 末尾統一管理
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

        // 重置離線判斷狀態
        postureStartTime = 0;
        alertTriggered = false;
        // 馬達控制由 loop 末尾統一處理，此處不直接操作
      } else {
        Serial.println("校準資料格式錯誤，應為：cal,val10,val15,val20,val25,threshold,duration");
      }
    }
  }
};

// ---------- 陀螺儀校準函式 ----------
void calibrateGyro() {
  Serial.println("Calibrating gyro... keep sensor still");
  int samples = 200;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_sum += gx / 131.0;
    gy_sum += gy / 131.0;
    gz_sum += gz / 131.0;
    delay(5);
  }
  gyroX_offset = gx_sum / samples;
  gyroY_offset = gy_sum / samples;
  gyroZ_offset = gz_sum / samples;
  Serial.println("Gyro calibration done");
}

// ---------- 設定 BLE 服務與特徵 ----------
void setupBLE() {
  BLEDevice::init("neGuard");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // 1. IMU 資料特徵（唯讀 + 通知）
  pImuCharacteristic = pService->createCharacteristic(
    IMU_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pImuCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ);

  // 2. 開關控制特徵（讀 + 寫）
  pSwitchCharacteristic = pService->createCharacteristic(
    SWITCH_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSwitchCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pSwitchCharacteristic->setCallbacks(new SwitchCharacteristicCallbacks());
  
  pService->start();

  // 開始廣播
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);  // 幫助 iPhone 連線
  pServer->startAdvertising();
  Serial.println("BLE advertising started");
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);


  // 初始化 I2C（SDA=8, SCL=9）
  Wire.begin(8, 9);
  Wire.setClock(100000);

  // 初始化 MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }
  Serial.println("MPU6050 connected.");

  // 設定 MPU6050 參數
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // 校準陀螺儀
  calibrateGyro();

  // 配置光敏電阻腳位（ADC）
  analogReadResolution(12);  // ESP32-C3 為 12 位元（0-4095）
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(V_MOTOR_PIN, OUTPUT);

  Serial.println("啟動振動提醒！");
  blinkMotor(3);                 // 開機提示
  digitalWrite(V_MOTOR_PIN, LOW); // 初始置低

  // 啟動 BLE
  setupBLE();

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
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 加速度轉換為 g（量程 ±2g）
    float accX = ax / 16384.0;
    float accY = ay / 16384.0;
    float accZ = az / 16384.0;

    // 陀螺儀轉換為 度/秒（量程 ±250°/s）
    float gyroX = (gx / 131.0) - gyroX_offset;
    float gyroY = (gy / 131.0) - gyroY_offset;
    float gyroZ = (gz / 131.0) - gyroZ_offset;  // 偏航未使用

    // 加速度計計算靜態角度
    accelAnglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    accelAngleRoll = atan2(accY, accZ) * 180.0 / PI;

    // 時間差 dt
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt > 0.1) dt = 0.1;

    // 互補濾波
    angleRoll = alpha * accelAngleRoll + (1 - alpha) * (angleRoll + gyroX * dt);
    anglePitch = alpha * accelAnglePitch + (1 - alpha) * (anglePitch + gyroY * dt);

    uint16_t lightValue = analogRead(LIGHT_SENSOR_PIN);

    // ----- 傳送 IMU 資料（若已連線） -----
    if (deviceConnected) {
      SensorData data;
      data.roll = angleRoll;
      data.pitch = anglePitch;
      data.ldr = lightValue;
      data.volt = 0.0;  // 後續可替換為真實電池電壓
      pImuCharacteristic->setValue((uint8_t*)&data, sizeof(data));
      pImuCharacteristic->notify();

      Serial.print("BLE >  ");
      Serial.print("Roll: ");
      Serial.print(angleRoll, 2);
      Serial.print("\tPitch: ");
      Serial.print(anglePitch, 2);
      Serial.print("\tLight: ");
      Serial.println(lightValue);
    }

    // ----- 離線姿態判斷（校準後啟用） -----
    if (calibrated) {
      // 當 roll 小於閾值（代表前傾過大）
      if (anglePitch < thresholdRoll) {
        // 姿勢不良
        if (!alertTriggered) {
          if (postureStartTime == 0) {
            postureStartTime = millis(); // 開始計時
          } else {
            // 檢查是否超過持續時間
            if (millis() - postureStartTime >= (unsigned long)durationSeconds * 1000) {
              // 觸發振動提醒
              alertTriggered = true;
              Serial.println("姿勢不良超過設定時間，啟動振動提醒！");
            }
          }
        }
        // 若已觸發，則保持振動（由 loop 末尾控制）
      } else {
        // 姿勢恢復正常
        if (alertTriggered) {
          alertTriggered = false;
          Serial.println("姿勢已恢復，關閉振動");
        }
        postureStartTime = 0; // 重置計時器
      }
    }

    // ----- 列印除錯資訊（每 5 秒） -----
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
      lastPrint = millis();
      Serial.print("Roll: ");
      Serial.print(angleRoll, 2);
      Serial.print("\tPitch: ");
      Serial.print(anglePitch, 2);
      Serial.print("\tLight: ");
      Serial.println(lightValue);
      if (calibrated) {
        Serial.printf("Threshold: %.2f, Duration: %d s, Alert: %d\n", thresholdRoll, durationSeconds, alertTriggered);
      }
    }
  }

  // ----- 處理手動警報（alarm）超時清除 -----
  if (alarmActive && (millis() - alarmStartTime >= 1000)) {
    alarmActive = false;
    Serial.println("Alarm timer expired, flag cleared");
  }

  // ----- 馬達狀態統一控制（離線判斷優先於手動警報） -----
  if (alertTriggered) {
    digitalWrite(V_MOTOR_PIN, HIGH);
  } else if (alarmActive) {
    digitalWrite(V_MOTOR_PIN, HIGH);
  } else {
    digitalWrite(V_MOTOR_PIN, LOW);
  }

  delay(1); // 避免 busy loop 佔用 CPU
}
