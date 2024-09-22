// 심박 센서 MAX30105 라이브리리
#include <MAX30105.h>
#include <heartRate.h>

// #include "MAX30105.h"
// #include "heartRate.h"
MAX30105 particleSensor;
long lastBeat = 0;

// 자이로 6축 센서 MPU6050 라이브러리
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

// 가속도계 및 자이로스코프 데이터 저장 변수
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accelX, accelY, accelZ;  // 실제 변환된 가속도 값 (m/s²)
float velocityX = 0, velocityY = 0, velocityZ = 0;  // 속도 (m/s)
float distanceX = 0, distanceY = 0, distanceZ = 0;  // 이동 거리 (km)
float accelTotal;  // 총 가속도 벡터 크기
int steps = 0;  // 걸음 수
float threshold = 12.0;  // 걸음을 감지하기 위한 임계값 (m/s²)
bool stepDetected = false;
// 이동 시간 계산 (실제 시간 추적)
unsigned long currentTime = 0;
float totalDistance = 0;
float totalTimeSeconds = 0;
unsigned long prevTime = 0;  // 이전 시간 저장

// 평균 속도 및 강아지의 체중과 MET 값
float avgSpeedKmh = 0;  // 나중에 속도를 누적한 후 평균을 계산할 수 있음
float dogWeight = 10.0;  // 강아지 체중 10kg로 설정 (kg)
float MET = 3.0;  // 활동 강도에 따른 MET 값 설정 (산책: 3.0, 빠른 달리기: 6.0 등)

// 가속도 노이즈 필터링을 위한 임계값
const float accelThreshold = 0.2;  // m/s² 단위로 노이즈 제거를 위한 임계값

// 블루투스 시리얼 통신 라이브러리
#include <SoftwareSerial.h>
#define BT_RXD 5 // digital 5 bluetooth
#define BT_TXD 4 // digital 4 bluetooth

#include <ArduinoJson.h>

SoftwareSerial bluetooth(BT_TXD, BT_RXD); // 블루투스 설정 BTSerial(TXD, RXD)

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600); // bluetooth 통신
  Serial.println("Bluetooth module begin...");

  // 통신 안정화를 위한 delay
  delay(1000);

  // AT 모드 진입
  bluetooth.println("AT");
  delay(1000);
  if (bluetooth.available()) {
    String response = bluetooth.readString();
    Serial.println(response);
  } else {
    Serial.println("No response from Bluetooth module");
  }

  // HM-10 모듈의 이름을 "Heddy"로 변경
  bluetooth.println("AT+NAME=Heddy");
  delay(1000);
  if (bluetooth.available()) {
    String response = bluetooth.readString();
    Serial.println(response);
  } else {
    Serial.println("No response from Bluetooth module for name change");
  }

  // HM-10 모듈의 이름 확인
  bluetooth.println("AT+NAME?");
  delay(1000);
  if (bluetooth.available()) {
    String response = bluetooth.readString();
    Serial.println(response);
  } else {
    Serial.println("No response from Bluetooth module for name query");
  }

  // I2C 통신 시작
  Wire.begin();

  // MPU6050 초기화
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  // MPU6050 연결 확인
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // MAX30105 센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  // MAX30105 센서 세팅
  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn on the Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED
}

void loop() {
  while (bluetooth.available()){ 
    byte data = bluetooth.read();
    Serial.write(data);
  }  
 
  while (Serial.available()){
    byte data = Serial.read();
    bluetooth.write(data); 
  }

  // 센서 데이터 읽기
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 블루투스를 통해 보낼 JSON 객체 생성
  StaticJsonDocument<200> doc;

  // 가속도 원시 값을 실제 물리적 값으로 변환 (1g = 9.81 m/s²)
  accelX = (ax / 16384.0) * 9.81;
  accelY = (ay / 16384.0) * 9.81;
  accelZ = (az / 16384.0) * 9.81;

  // 총 가속도 벡터 크기 계산
  accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // 노이즈 필터링: 작은 진동 및 센서 노이즈 제거
  if (abs(accelX) < accelThreshold) accelX = 0;
  if (abs(accelY) < accelThreshold) accelY = 0;
  if (abs(accelZ) < accelThreshold) accelZ = 0;

  // 시간 간격 계산
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // deltaTime을 초 단위로 계산
  prevTime = currentTime;

  // 가속도를 적분하여 속도를 계산 (m/s)
  velocityX += accelX * deltaTime;
  velocityY += accelY * deltaTime;
  velocityZ += accelZ * deltaTime;

  // 속도를 km/h로 변환
  float velocityX_kmh = velocityX * 3.6;
  float velocityY_kmh = velocityY * 3.6;
  float velocityZ_kmh = velocityZ * 3.6;

  // 적분된 km/h 데이터
  doc["velocity_kmh"]["x"] = velocityX_kmh;
  doc["velocity_kmh"]["y"] = velocityY_kmh;
  doc["velocity_kmh"]["z"] = velocityZ_kmh;

  // 시간 간격 동안 이동한 거리 계산 (시간은 시간 단위로 변환해야 함)
  float deltaTimeHours = deltaTime / 3600.0;  // 초 단위를 시간 단위로 변환

  // 이동 거리 계산 (km)
  distanceX += velocityX_kmh * deltaTimeHours;
  distanceY += velocityY_kmh * deltaTimeHours;
  distanceZ += velocityZ_kmh * deltaTimeHours;

  // 적분된 km/h 데이터
  doc["distance"]["x"] = distanceX;
  doc["distance"]["y"] = distanceY;
  doc["distance"]["z"] = distanceZ;
  
  // 자이로스코프 데이터
  doc["gyroscope"]["x"] = gx;
  doc["gyroscope"]["y"] = gy;
  doc["gyroscope"]["z"] = gz;

  // 이동 거리 계산 (m/s에서 km 단위로 누적)
  float velocityAvgKmh = (velocityX_kmh + velocityY_kmh + velocityZ_kmh) / 3.0;
  float distanceInThisInterval = velocityAvgKmh * (deltaTime / 3600.0);  // 이번 간격의 이동 거리 (km)
  totalDistance += distanceInThisInterval;

  // 총 이동 시간 누적
  totalTimeSeconds += deltaTime;

  // 평균 속도 계산
  avgSpeedKmh = totalDistance / (totalTimeSeconds / 3600.0);  // km/h

  // 칼로리 소모 계산
  float timeHours = totalTimeSeconds / 3600.0;  // 초 단위로 누적된 시간을 시간으로 변환
  float caloriesBurned = MET * dogWeight * timeHours;

  doc["MET"] = MET;
  doc["dogWeight"] = dogWeight;
  doc["avgSpeedKmh"] = avgSpeedKmh;
  doc["caloriesBurned"] = caloriesBurned;

  // 임계값을 넘는 경우 걸음 감지
  if (accelTotal > threshold && !stepDetected) {
    steps++;
    stepDetected = true;  // 한 번 감지된 후에는 다시 낮아질 때까지 대기
    Serial.println("Step detected!");
  }

  // 가속도가 다시 임계값 이하로 떨어지면 다음 걸음 감지를 준비
  if (accelTotal < threshold - 2) {
    stepDetected = false;
  }

  doc["steps"] = steps;
  
  long irValue = particleSensor.getIR(); // Read IR Value
  
  // if (checkForBeat(irValue)){
  //   long delta = millis() - lastBeat;
  //   lastBeat = millis();
  //   Serial.println("irValue : " + String(irValue));
  //   float beatsPerMinute = 60 / (delta / 1000.0);
    
  //   // BPM Json 데이터에 push
  //   doc["Heart"]["BPM"] = beatsPerMinute;
  // }

  long delta = millis() - lastBeat;
  lastBeat = millis();
  
  float beatsPerMinute = 60 / (delta / 1000.0);
  
  // BPM Json 데이터에 push
  doc["heart"]["bpm"] = beatsPerMinute;

  // JSON 문자열로 변환
  String jsonBuffer;
  serializeJson(doc, jsonBuffer);

  // 시리얼 모니터에 결과 값 출력
  Serial.println(jsonBuffer);

  // JSON 문자열을 블루투스 모듀로 전송
  bluetooth.println("Sent: " + jsonBuffer);

  delay(1000); // 딜레이 1초
}
