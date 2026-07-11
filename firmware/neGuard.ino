#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ---------- 传感器数据结构体（与原 App 兼容）----------
struct SensorData {
  float roll;
  float pitch;
  int ldr;
  float volt;
};

// ---------- BLE UUID（与之前完全一致）----------
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define IMU_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9F"
#define SWITCH_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define LIGHT_CHAR_UUID "6E400004-B5A3-F393-E0A9-E50E24DCCA9F"

// ---------- 全局 BLE 对象 ----------
BLEServer* pServer = NULL;
BLECharacteristic* pImuCharacteristic = NULL;
BLECharacteristic* pSwitchCharacteristic = NULL;
BLECharacteristic* pLightCharacteristic = NULL;

// ---------- MPU6050 对象 ----------
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// 互补滤波变量
float angleRoll = 0, anglePitch = 0;
float dt = 0;
unsigned long lastTime = 0;
float alpha = 0.05;  // 互补滤波系数（小值更信任陀螺仪）
float accelAngleRoll, accelAnglePitch;

// 陀螺仪零偏校准值
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

// 光敏电阻引脚
const int LIGHT_SENSOR_PIN = A3;  // GPIO3
const int V_MOTOR_PIN = 0;        // GPIO0

unsigned long lastToggleTime = 0;
bool motorState = false;

unsigned long alarmStartTime = 0;
bool alarmActive = false;
// 连接状态标志（用于打印）
bool deviceConnected = false;

void blinkMotor(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(V_MOTOR_PIN, HIGH);
    delay(200);
    digitalWrite(V_MOTOR_PIN, LOW);
    delay(100);
  }
}

// ---------- 回调类：处理连接/断开和开关特征的写入 ----------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
    // 重新开始广播，让手机可以再次连接
    pServer->startAdvertising();
  }
};

class SwitchCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    Serial.print("Switch characteristic written: ");
    Serial.println(value.c_str());

    // 判断是否为 "alarm"（长度为5，且内容匹配）
    if (value.length() == 5 && value == "alarm") {
      Serial.println("Alarm triggered! GPIO0 HIGH");
      digitalWrite(V_MOTOR_PIN, HIGH);
      alarmActive = true;
      alarmStartTime = millis();
    }
  }
};


// ---------- 陀螺仪校准函数 ----------
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

// ---------- 设置 BLE 服务和特征 ----------
void setupBLE() {
  BLEDevice::init("neckEase");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // 1. IMU 数据特征 (只读 + 通知)
  pImuCharacteristic = pService->createCharacteristic(
    IMU_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pImuCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ);

  // 2. 开关控制特征 (读 + 写)
  pSwitchCharacteristic = pService->createCharacteristic(
    SWITCH_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSwitchCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pSwitchCharacteristic->setCallbacks(new SwitchCharacteristicCallbacks());

  // 3. 光敏电阻特征 (只读 + 通知)
  pLightCharacteristic = pService->createCharacteristic(
    LIGHT_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pLightCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ);

  pService->start();

  // 开始广播
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);  // 帮助 iPhone 连接
  pServer->startAdvertising();
  Serial.println("BLE advertising started");
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 启动 BLE
  setupBLE();

  // 初始化 I2C (SDA=8, SCL=9)
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

  // 设置 MPU6050 参数
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // 校准陀螺仪
  calibrateGyro();

  // 配置光敏电阻引脚 (ADC)
  analogReadResolution(12);  // ESP32-C3 为 12 位 (0-4095)
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(V_MOTOR_PIN, OUTPUT);

  blinkMotor(3);                 // 开机提示
  digitalWrite(V_MOTOR_PIN, LOW);  // 初始置低
  motorState = false;
  lastToggleTime = millis();  // 记录起始时间

  lastTime = micros();
}

// ---------- Loop ----------
void loop() {
  // 非阻塞 IMU 更新 (50 Hz = 20ms 间隔)
  static unsigned long lastSensorUpdate = 0;
  const unsigned long sensorUpdateInterval = 20;

  if (millis() - lastSensorUpdate >= sensorUpdateInterval) {
    lastSensorUpdate = millis();

    // 读取 MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 加速度转换为 g (量程 ±2g)
    float accX = ax / 16384.0;
    float accY = ay / 16384.0;
    float accZ = az / 16384.0;

    // 陀螺仪转换为 度/秒 (量程 ±250°/s)
    float gyroX = (gx / 131.0) - gyroX_offset;
    float gyroY = (gy / 131.0) - gyroY_offset;
    float gyroZ = (gz / 131.0) - gyroZ_offset;  // 偏航未使用

    // 加速度计计算静态角度
    accelAnglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    accelAngleRoll = atan2(accY, accZ) * 180.0 / PI;

    // 时间差 dt
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt > 0.1) dt = 0.1;

    // 互补滤波
    angleRoll = alpha * accelAngleRoll + (1 - alpha) * (angleRoll + gyroX * dt);
    anglePitch = alpha * accelAnglePitch + (1 - alpha) * (anglePitch + gyroY * dt);

    uint16_t lightValue = analogRead(LIGHT_SENSOR_PIN);
    // ----- 发送 IMU 数据 -----
    if (deviceConnected) {
      SensorData data;
      data.roll = angleRoll;
      data.pitch = anglePitch;
      data.ldr = lightValue;
      data.volt = 0.0;  // 后续替换为真实电池电压
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

    // 串口打印 (每 50ms 一次)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 5000) {
      lastPrint = millis();
      Serial.print("Roll: ");
      Serial.print(angleRoll, 2);
      Serial.print("\tPitch: ");
      Serial.print(anglePitch, 2);
      Serial.print("\tLight: ");
      Serial.println(lightValue);
    }
  }

   // 处理报警超时（1秒后拉低）
  if (alarmActive && (millis() - alarmStartTime >= 1000)) {
    digitalWrite(V_MOTOR_PIN, LOW);
    alarmActive = false;
    Serial.println("Alarm finished, GPIO0 LOW");
  }
  
 
  // 不需要 LED 闪烁，空转即可
  delay(1);  // 避免 busy loop 占用 CPU
}