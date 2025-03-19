#include <Wire.h>
#include <WiFiNINA.h>
#include "MAX30105.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// WiFi Credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Firebase Details
const char* firebaseHost = "https://riskband-7551a-default-rtdb.asia-southeast1.firebasedatabase.app/"; // Your Firebase URL 
const char* firebaseAuth = "fJ0y6TGCa730ewDi3ols8we5DWWGZjHMeeodcOQF"; // Your Firebase Database Secret
WiFiClient client;

// Sensor Objects
MAX30105 particleSensor;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(10, 11); // RX, TX for GPS

#define EMERGENCY_HR_LOW 50
#define EMERGENCY_HR_HIGH 120

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    
    if (!particleSensor.begin()) {
        Serial.println("MAX30102 not found. Check connections.");
        while (1);
    }
    particleSensor.setup();
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi.");
}

void loop() {
    float heartRate = readHeartRate();
    String location = getGPSLocation();

    Serial.print("Heart Rate: ");
    Serial.println(heartRate);

    if (heartRate < EMERGENCY_HR_LOW || heartRate > EMERGENCY_HR_HIGH) {
        Serial.println("Emergency detected! Sending alert...");
        sendEmergencyData(heartRate, location);
    }
    delay(5000);
}

float readHeartRate() {
    long irValue = particleSensor.getIR();
    if (irValue < 50000) return 0;
    return particleSensor.getHeartRate();
}

String getGPSLocation() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()) {
        return String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    }
    return "0,0";
}

void sendEmergencyData(float heartRate, String location) {
    if (WiFi.status() == WL_CONNECTED) {
        String path = "/emergency.json?auth=" + String(firebaseAuth);  // Use .json for REST API
        String jsonPayload = "{\"heartRate\":" + String(heartRate) + ",\"location\":\"" + location + "\"}";

        Serial.println("Sending data to Firebase...");
        
        client.connect(firebaseHost, 443);
        client.println("PUT " + path + " HTTP/1.1");
        client.println("Host: " + String(firebaseHost));
        client.println("Content-Type: application/json");
        client.println("Content-Length: " + String(jsonPayload.length()));
        client.println();
        client.println(jsonPayload);

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
