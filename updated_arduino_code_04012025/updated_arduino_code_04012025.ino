#include <Wire.h>
#include <WiFiNINA.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"  // SpO2 calculation algorithm
#include <TinyGPS++.h>

// WiFi Credentials
const char* ssid = "JAYSON BUDUAN";
const char* password = "Jaygrace&/04030308";

// Firebase Details
const char* firebaseHost = "riskband-7551a-default-rtdb.asia-southeast1.firebasedatabase.app/sensorData.json";
const char* firebaseAuth = "fJ0y6TGCa730ewDi3ols8we5DWWGZjHMeeodcOQF";
WiFiClient client;

// Sensor Objects
MAX30105 particleSensor;
TinyGPSPlus gps;

// Constants
#define EMERGENCY_HR_LOW 50
#define EMERGENCY_HR_HIGH 120
#define EMERGENCY_SPO2_LOW 90  // Below 90% is considered low oxygen level
#define BATTERY_PIN A0  
#define MAX_BATTERY_VOLTAGE 4.2 
#define MIN_BATTERY_VOLTAGE 3.0 

// SpO2 Algorithm Buffers
const int MY_BUFFER_SIZE = 100;
uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];
int spo2;
int heartRate;
uint8_t validSpO2;
uint8_t validHeartRate;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
        delay(1000);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Wi-Fi!");
    } else {
        Serial.println("\nWi-Fi Connection Failed. Retrying in 10s...");
        delay(10000);
        NVIC_SystemReset(); // Reset the microcontroller if Wi-Fi fails
    }

    // Initialize MAX30102 Sensor
    if (!particleSensor.begin()) {
        Serial.println("MAX30102 sensor not found!");
        while (1);
    } else {
        Serial.println("MAX30102 sensor initialized.");
    }

    // Configure MAX30102 sensor
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeIR(0x0A);

    // GPS Setup
    Serial1.begin(9600);
}

void loop() {
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    Serial.print("IR Value: ");
    Serial.print(irValue);
    Serial.print("  Red Value: ");
    Serial.println(redValue);

    sendSensorData(irValue, redValue);
    delay(5000);
}

void sendSensorData(long irValue, long redValue) {
    if (WiFi.status() == WL_CONNECTED) {
        long timestamp = millis();  // Unique timestamp
        String path = "/sensorData/" + String(timestamp) + ".json?auth=" + String(firebaseAuth);
        client.post("/sensorData.json?auth=" + firebaseAuth); // testing for tomorrow !!!!!!

        String jsonPayload = "{\"irValue\":" + String(irValue) + 
                             ",\"redValue\":" + String(redValue) + 
                             ",\"timestamp\":" + String(timestamp) + "}";
        Serial.println("Sending sensor data to Firebase...");
        
        if (client.connect(firebaseHost, 80)) {  
            client.println("PUT " + path + " HTTP/1.1");
            client.println("Host: " + String(firebaseHost));
            client.println("Content-Type: application/json");
            client.println("Content-Length: " + String(jsonPayload.length()));
            client.println();
            client.println(jsonPayload);
        } else {
            Serial.println("Failed to connect to Firebase.");
            return;
        }

        delay(1000);
        while (client.available()) {
            String response = client.readString();
            Serial.println(response);
        }
        client.stop();
    } else {
        Serial.println("WiFi Disconnected. Cannot send data.");
    }
}

void readHeartRateAndSpO2() {
    for (int i = 0; i < MY_BUFFER_SIZE; i++) {
        while (!particleSensor.check());
        irBuffer[i] = particleSensor.getIR();
        redBuffer[i] = particleSensor.getRed();
    }

   // maxim_heart_rate_and_oxygen_saturation(irBuffer, MY_BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);
    if (!validHeartRate) heartRate = 0;
    if (!validSpO2) spo2 = 0;
}

String getGPSLocation() {
    Serial.println("Reading GPS...");
    long startTime = millis();
    while (millis() - startTime < 5000) {
        while (Serial1.available()) {
            gps.encode(Serial1.read());
        }
    }
    if (gps.location.isValid()) {
        return String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    }
    return "0,0";
}

int getBatteryPercentage() {
    float voltage = analogRead(BATTERY_PIN) * (3.3 / 1023.0);
    int percentage = (int)(((voltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100);
    return constrain(percentage, 0, 100);
}

void sendEmergencyData(int heartRate, int spo2, String location, int batteryPercentage) {
    if (WiFi.status() == WL_CONNECTED) {
        String path = "/emergency.json?auth=" + String(firebaseAuth);
        String jsonPayload = "{\"heartRate\":" + String(heartRate) + 
                             ",\"spo2\":" + String(spo2) +
                             ",\"location\":\"" + location + 
                             "\",\"battery\":" + String(batteryPercentage) + "}";

        Serial.println("Sending data to Firebase...");
        if (client.connect(firebaseHost, 80)) {
            client.println("PUT " + path + " HTTP/1.1");
            client.println("Host: " + String(firebaseHost));
            client.println("Content-Type: application/json");
            client.println("Content-Length: " + String(jsonPayload.length()));
            client.println();
            client.println(jsonPayload);
        } else {
            Serial.println("Failed to connect to Firebase.");
            return;
        }

        delay(1000);
        while (client.available()) {
            String response = client.readString();
            Serial.println(response);
        }
        client.stop();
    } else {
        Serial.println("WiFi Disconnected. Cannot send data.");
    }
}