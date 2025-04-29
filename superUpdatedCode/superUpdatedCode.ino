#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <TinyGPS++.h>
#include "heartRate.h"
#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h> // ✅ Kalman filter added

// WiFi Credentials
const char* ssidList[] = {
  "JAYSON BUDUAN",
  "shuussh"
};

const char* passwordList[] = {
  "Jaygrace&/04030308",
  "password"
};

// Firebase Details
const char* firebaseHost = "riskband-7551a-default-rtdb.asia-southeast1.firebasedatabase.app";
const String DATABASE_PATH = "/health_data";
const char* firebaseAuth = "fJ0y6TGCa730ewDi3ols8we5DWWGZjHMeeodcOQF";

// WiFi & HTTP Client
WiFiSSLClient wifiClient;
HttpClient client = HttpClient(wifiClient, firebaseHost, 443);

// Sensor Objects
MAX30105 particleSensor;
TinyGPSPlus gps;

// Sensor Data
int32_t heartRate = 0;
int32_t spo2 = 0;
int8_t validHeartRate = 0;
int8_t validSpO2 = 0;

// Kalman filter object for heart rate
SimpleKalmanFilter kalmanHR(2, 2, 0.01); // kalman filter

// GPS Data
double latitude = 0.0;
double longitude = 0.0;

uint32_t irBuffer[100];
uint32_t redBuffer[100];
#define BUFFER_SIZE 100

const int numberOfNetworks = sizeof(ssidList) / sizeof(ssidList[0]);

void connectToWiFi() {
  int status = WL_IDLE_STATUS;
  bool connected = false;

  for (int i = 0; i < numberOfNetworks; i++) {
    Serial.print("Connecting to ");
    Serial.println(ssidList[i]);

    WiFi.begin(ssidList[i], passwordList[i]);
    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected successfully!");
      Serial.print("Connected to SSID: ");
      Serial.println(ssidList[i]);
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      connected = true;
      break;
    } else {
      Serial.println("\nFailed to connect. Trying next network...");
    }
  }

  if (!connected) {
    Serial.println("Failed to connect to any network.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  connectToWiFi();
  clearFirebaseData();

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 sensor not found. Check wiring!");
    while (1);
  }
  particleSensor.setup();
  Serial.println("MAX30102 initialized.");

  Serial1.begin(115200);
  Serial.println("GPS module initialized.");
}

void loop() {
  // Read and parse GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
  }

  // Read MAX30102
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.check()) delay(1);
    irBuffer[i] = particleSensor.getIR();
    redBuffer[i] = particleSensor.getRed();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

  if (validHeartRate == 1 && validSpO2 == 1) {
    // kalman filter para sa heart rate
    float filteredHeartRate = kalmanHR.updateEstimate((float)heartRate);

    Serial.print("Raw HR: ");
    Serial.print(heartRate);
    Serial.print(" BPM, Filtered HR: ");
    Serial.print(filteredHeartRate, 1);
    Serial.print(" BPM, SpO₂: ");
    Serial.print(spo2);
    Serial.println(" %");

    if (gps.location.isValid() && gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.print(latitude, 6);
      Serial.print(", Longitude: ");
      Serial.println(longitude, 6);
    } else {
      Serial.println("Waiting for GPS fix...");
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    }

    sendToFirebase((int)filteredHeartRate, spo2, latitude, longitude);
  } else {
    Serial.println("Invalid data: Heart rate or SpO₂ not valid.");
  }

  delay(5000);
}

void sendToFirebase(int32_t heartRate, int32_t spo2, double latitude, double longitude) {
  StaticJsonDocument<400> jsonDoc;
  jsonDoc["heart_rate"] = heartRate;
  jsonDoc["spo2"] = spo2;
  jsonDoc["latitude"] = latitude;
  jsonDoc["longitude"] = longitude;

  String jsonStr;
  serializeJson(jsonDoc, jsonStr);

  String path = "/health_data.json?auth=" + String(firebaseAuth);

  client.beginRequest();
  client.post(path);
  client.sendHeader("Content-Type", "application/json");
  client.sendHeader("Content-Length", jsonStr.length());
  client.beginBody();
  client.print(jsonStr);
  client.endRequest();

  int statusCode = client.responseStatusCode();
  if (statusCode == 200) {
    Serial.println("Data sent to Firebase successfully!");
  } else {
    Serial.print("Error sending data. HTTP Status Code: ");
    Serial.println(statusCode);
  }
}

void clearFirebaseData() {
  String path = "/health_data.json?auth=" + String(firebaseAuth);

  Serial.println("Sending manual DELETE request to Firebase...");

  client.beginRequest();
  client.del(path);
  client.sendHeader("Connection", "close");
  client.endRequest();

  int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  if (statusCode == 200 || statusCode == 204) {
    Serial.println("✅ Data deleted successfully from Firebase.");
  } else {
    Serial.print("❌ Failed to delete data. HTTP Status Code: ");
    Serial.println(statusCode);
    Serial.println("Check auth token, path, or WiFi connection.");
    Serial.print("Firebase response: ");
    Serial.println(response);
  }
}