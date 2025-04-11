#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <TinyGPS++.h>
#include "heartRate.h"
#include <ArduinoJson.h>

// WiFi Credentials
const char* ssid = "shuussh";
const char* password = "password";

// Firebase Details
const char* firebaseHost = "riskband-7551a-default-rtdb.asia-southeast1.firebasedatabase.app";
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

// GPS Data
double latitude = 0.0;
double longitude = 0.0;

uint32_t irBuffer[100];
uint32_t redBuffer[100];
#define BUFFER_SIZE 100

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Start Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 sensor not found. Check wiring!");
    while (1);
  }
  particleSensor.setup();
  Serial.println("MAX30102 initialized.");

  // Initialize GPS
  Serial1.begin(115200);
  Serial.println("GPS module initialized.");
}

void loop() {
  // Read and parse incoming GPS data
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    Serial.print(c); // Optional: comment this out later
  }

  // Read MAX30102 data
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.check()) delay(1);
    irBuffer[i] = particleSensor.getIR();
    redBuffer[i] = particleSensor.getRed();
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

  if (validHeartRate == 1 && validSpO2 == 1) {
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.print(" BPM, SpO₂: ");
    Serial.print(spo2);
    Serial.println(" %");

    // ✅ Check for valid and updated GPS location
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

    // Send to Firebase
    sendToFirebase(heartRate, spo2, latitude, longitude);
  } else {
    Serial.println("Invalid data: Heart rate or SpO₂ not valid.");
  }

  delay(5000); // Avoid flooding the server
}

void sendToFirebase(int32_t heartRate, int32_t spo2, double latitude, double longitude) {
  StaticJsonDocument<400> jsonDoc;
  jsonDoc["heart_rate"] = heartRate;
  jsonDoc["spo2"] = spo2;
  jsonDoc["latitude"] = latitude;
  jsonDoc["longitude"] = longitude;
  jsonDoc["timestamp"] = millis();

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
/*#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <TinyGPS++.h>
#include "heartRate.h" // for testing
#include <ArduinoJson.h> // ✅ CORRECT
 // for testing


// WiFi Credentials
const char* ssid = "shuussh";
const char* password = "password";


// Firebase Details
const char* firebaseHost = "riskband-7551a-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* firebaseAuth = "fJ0y6TGCa730ewDi3ols8we5DWWGZjHMeeodcOQF";

// WiFi & HTTP Client
WiFiSSLClient wifiClient;  // ✅ Using SSL for HTTPS
HttpClient client = HttpClient(wifiClient, firebaseHost, 443);  // ✅ Use HTTPS (port 443)

// Sensor Objects
MAX30105 particleSensor;
TinyGPSPlus gps;


int32_t heartRate = 0;
int32_t spo2 = 0;
int8_t validHeartRate = 0;
int8_t validSpO2 = 0;

// GPS Data
double latitude = 0.0;
double longitude = 0.0;

uint32_t irBuffer[100]; // Buffer for IR readings
uint32_t redBuffer[100]; // Buffer for Red readings
#define BUFFER_SIZE 100

void setup() {
    Serial.begin(115200);
    delay(1000);  // Allow time for the Serial monitor to start

    // Start Wi-Fi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 sensor not found. Check wiring!");
        while (1);  // Stop execution if sensor is not found
    }
    particleSensor.setup();
    Serial.println("MAX30102 sensor initialized successfully.");

    // Initialize GPS (using Serial1)
    Serial1.begin(9600);  // Initialize Serial1 for GPS communication
    Serial.println("GPS module initialized.");
}

void loop() {
     // Read GPS data
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }

    // Read data from MAX30102
    for (byte i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.check()) delay(1);
        irBuffer[i] = particleSensor.getIR();
        redBuffer[i] = particleSensor.getRed();
    }

    // Calculate heart rate & SpO₂
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

    // Output data to Serial Monitor if valid
    if (validHeartRate == 1 && validSpO2 == 1) {
        Serial.print("Heart Rate: ");
        Serial.print(heartRate);
        Serial.print(" BPM, SpO₂: ");
        Serial.print(spo2);  // Now it's an int32_t, so it will print as an integer
        Serial.println(" %");

       
        // Check for GPS fix
        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            Serial.print("Latitude: ");
            Serial.print(latitude, 6);
            Serial.print(", Longitude: ");
            Serial.println(longitude, 6);
        } else {
            Serial.println("Waiting for GPS fix...");
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());  // Debugging number of satellites
        }

        // Call the function to send data to Firebase
        sendToFirebase(heartRate, spo2, latitude, longitude);
    } else {
        Serial.println("Invalid data: Heart rate or SpO₂ not valid.");
    }

    delay(5000); // Add a delay to avoid overloading the server
}

// Function to send data to Firebase
void sendToFirebase(int32_t heartRate, int32_t spo2, double latitude, double longitude) {
    // Create JSON object
    StaticJsonDocument<400> jsonDoc;
    jsonDoc["heart_rate"] = heartRate;
    jsonDoc["spo2"] = spo2;
    jsonDoc["latitude"] = latitude;
    jsonDoc["longitude"] = longitude;
    jsonDoc["timestamp"] = millis();  // Timestamp for when the data was captured

    String jsonStr;
    serializeJson(jsonDoc, jsonStr);

    // Send HTTP request to Firebase
    String path = "/health_data.json?auth=" + String(firebaseAuth);
    
    client.beginRequest();
    client.post(path);  // Use 'post' to append data to Firebase
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonStr.length());
    client.beginBody();
    client.print(jsonStr);  // Send the JSON payload
    client.endRequest();

    // Check response code from Firebase
    int statusCode = client.responseStatusCode();
    if (statusCode == 200) {
        Serial.println("Data sent to Firebase successfully!");
    } else {
        Serial.print("Error sending data. HTTP Status Code: ");
        Serial.println(statusCode);
    }
}*/

/* original code
void setup() {
    Serial.begin(115200);
    delay(1000);  // Allow time for the Serial monitor to start

    // Start Wi-Fi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 sensor not found. Check wiring!");
        while (1);  // Stop execution if sensor is not found
    }
    particleSensor.setup();
    Serial.println("MAX30102 sensor initialized successfully.");

    // Initialize GPS (using Serial1)
    Serial1.begin(9600);  // Initialize Serial1 for GPS communication
    Serial.println("GPS module initialized.");
}

void loop() {
    // Read data from MAX30102
    for (byte i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.check()) delay(1);
        irBuffer[i] = particleSensor.getIR();
        redBuffer[i] = particleSensor.getRed();
    }

    // Calculate heart rate & SpO₂
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

    // Output data to Serial Monitor if valid
    if (validHeartRate == 1 && validSpO2 == 1) {
        Serial.print("Heart Rate: ");
        Serial.print(heartRate);
        Serial.print(" BPM, SpO₂: ");
        Serial.print(spo2);  // Now it's an int32_t, so it will print as an integer
        Serial.println(" %");

        // Read GPS data
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }

        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            Serial.print("Latitude: ");
            Serial.print(latitude, 6);
            Serial.print(", Longitude: ");
            Serial.println(longitude, 6);
        } else {
    Serial.println("Waiting for GPS fix...");
    }
        sendToFirebase(heartRate, spo2, latitude, longitude);  // Send data to Firebase
    } else {
        Serial.println("Invalid data: Heart rate or SpO₂ not valid.");
    }

    delay(5000); // Add a delay to avoid overloading the server
}

// Function to send data to Firebase
void sendToFirebase(int32_t heartRate, int32_t spo2, double latitude, double longitude) {
    // Create JSON object
    StaticJsonDocument<400> jsonDoc;
    jsonDoc["heart_rate"] = heartRate;
    jsonDoc["spo2"] = spo2;
    jsonDoc["latitude"] = latitude;
    jsonDoc["longitude"] = longitude;
    jsonDoc["timestamp"] = millis();  // Timestamp for when the data was captured

    String jsonStr;
    serializeJson(jsonDoc, jsonStr);

    // Send HTTP request to Firebase
    String path = "/health_data.json?auth=" + String(firebaseAuth);
    
    client.beginRequest();
    client.post(path);  // Use 'post' to append data to Firebase
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonStr.length());
    client.beginBody();
    client.print(jsonStr);  // Send the JSON payload
    client.endRequest();

    Serial.println("Data sent to Firebase!");
}*/

/*
// Variables for HR & SpO₂
uint32_t irBuffer[BUFFER_SIZE]; // IR LED sensor data
uint32_t redBuffer[BUFFER_SIZE]; // Red LED sensor data
int32_t spo2; // Calculated SpO₂ value
int32_t heartRate; // Change to int32_t
int8_t validHeartRate; // Flag for HR validity
int8_t validSpO2; // Flag for SpO₂ validity
*/


/*
void setup() {
  Serial1.begin(9600);      // GPS Module via Serial1 (Nano 33 IoT: pins 0 & 1)
  Serial.begin(115200);     // Serial Monitor
  delay(2000);              // Wait for things to settle
  Serial.println("GPS Test Initialized...");
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());   // Read GPS data from Serial1
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.println("------------------------");
  }

  delay(1000); // Delay for clarity
}*/
/*
// actual code that runs
void setup() {
    Serial.begin(115200);
    delay(1000);  // Allow time for the Serial monitor to start

    Serial.println("Starting WiFi connection...");

    WiFi.begin(ssid, password);

    // Wait for Wi-Fi to connect
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    
    // Initialize MAX30102
    Serial.println("Initializing MAX30102 sensor...");
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 sensor not found. Check wiring!");
        while (1);  // Stop execution if sensor is not found
    }
    particleSensor.setup();
    Serial.println("MAX30102 sensor initialized successfully.");
}

void loop() {
    // Read data from MAX30102
    Serial.println("Reading sensor data...");

    for (byte i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.check()) delay(1);
        irBuffer[i] = particleSensor.getIR();
        redBuffer[i] = particleSensor.getRed();
    }

    // Calculate heart rate & SpO₂
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

    // Output data to Serial Monitor if valid
    if (validHeartRate == 1 && validSpO2 == 1) {
        Serial.print("Heart Rate: ");
        Serial.print(heartRate);
        Serial.print(" BPM, SpO₂: ");
        Serial.print(spo2);
        Serial.println(" %");

        sendToFirebase(heartRate, spo2);  // Call the function to send data to Firebase
    } else {
        Serial.println("Invalid data: Heart rate or SpO₂ not valid.");
    }

    delay(5000); // Add a delay to avoid overloading the server
}

// Function to send heart rate and SpO₂ data to Firebase
void sendToFirebase(int32_t heartRate, float spo2) {
    // Create JSON object
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["heart_rate"] = heartRate;
    jsonDoc["spo2"] = spo2;
    jsonDoc["timestamp"] = millis(); // for timestamp 

    String jsonStr;
    serializeJson(jsonDoc, jsonStr);

    // Send HTTP request to Firebase
    String path = "/health_data.json?auth=" + String(firebaseAuth);
    
    client.beginRequest();
    client.post(path);  // Use 'post' to append data to Firebase
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonStr.length());
    client.beginBody();
    client.print(jsonStr);  // Send the JSON payload
    client.endRequest();

    Serial.println("Data sent to Firebase!");
}*/
//last line for the actual code

/* for deleting older health data records //for testing
// Function to retrieve all health data from Firebase
void getHealthDataFromFirebase() {
    String path = "/health_data.json?auth=" + String(firebaseAuth);

    // Send GET request to fetch all data
    client.beginRequest();
    client.get(path);  // Use GET to retrieve all records
    client.endRequest();

    String response = client.responseBody();  // Store the response (all records)
    Serial.println("Received data: " + response);

    // Parse the received JSON data
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, response);

    // Process and delete older records
    deleteOldRecords(doc);
}

// Function to delete the 3 oldest records
void deleteOldRecords(DynamicJsonDocument &doc) {
    int recordCount = doc.size();
    if (recordCount <= 3) {
        Serial.println("There are 3 or fewer records, no deletion needed.");
        return;  // If there are 3 or fewer records, don't delete anything
    }

    // Sort records by timestamp or your chosen key
    // Assuming each record has a 'timestamp' field, you can sort them by it.
    // You'll need to store the IDs of the records to delete.

    String deleteIds[3];
    int count = 0;
    for (JsonPair record : doc.as<JsonObject>()) {
        // Get the key (ID) and value from the JSON object
        String recordKey = record.key().c_str();  // Get record ID
        if (count < 3) {
            deleteIds[count++] = recordKey;  // Store the key (ID) of the record
        }
    }

    // Delete the 3 oldest records
    for (int i = 0; i < 3; i++) {
        String deletePath = "/health_data/" + deleteIds[i] + ".json?auth=" + String(firebaseAuth);
        client.beginRequest();
        client.sendHeader("Content-Type", "application/json");  // You can send an empty body
        client.beginBody();  // Empty body for DELETE
        client.endRequest();  // Send the DELETE request
    }
}*/


/*
// Constants(for guide only)
#define EMERGENCY_HR_LOW 50
#define EMERGENCY_HR_HIGH 120
#define EMERGENCY_SPO2_LOW 90
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
    delay(2000);

    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected to Wi-Fi!");

    // Initialize MAX30102 Sensor
    if (!particleSensor.begin()) {
        Serial.println("MAX30102 sensor not found!");
        while (1);
    }
    Serial.println("MAX30102 sensor initialized.");

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
        long timestamp = millis();
        String path = "/sensorData.json?auth=" + String(firebaseAuth);  // POST request to add new record

        String jsonPayload = "{\"irValue\":" + String(irValue) + 
                             ",\"redValue\":" + String(redValue) + 
                             ",\"timestamp\":" + String(timestamp) + "}";

        Serial.println("Sending sensor data to Firebase...");

        client.beginRequest();
        client.post(path);  // ✅ Changed from PUT to POST
        client.sendHeader("Content-Type", "application/json");
        client.sendHeader("Content-Length", jsonPayload.length());
        client.beginBody();
        client.print(jsonPayload);
        client.endRequest();

        // Read Firebase response
        int statusCode = client.responseStatusCode();
        String response = client.responseBody();
        Serial.print("Response Code: ");
        Serial.println(statusCode);
        Serial.println("Firebase Response: " + response);
    } else {
        Serial.println("WiFi Disconnected. Cannot send data.");
    }
}

void sendEmergencyData(int heartRate, int spo2, String location, int batteryPercentage) {
    if (WiFi.status() == WL_CONNECTED) {
        String path = "/emergency.json?auth=" + String(firebaseAuth);  // POST request to add new emergency record

        String jsonPayload = "{\"heartRate\":" + String(heartRate) + 
                             ",\"spo2\":" + String(spo2) +
                             ",\"location\":\"" + location + 
                             "\",\"battery\":" + String(batteryPercentage) + "}";

        Serial.println("Sending emergency data to Firebase...");

        client.beginRequest();
        client.post(path);  // ✅ Changed from PUT to POST
        client.sendHeader("Content-Type", "application/json");
        client.sendHeader("Content-Length", jsonPayload.length());
        client.beginBody();
        client.print(jsonPayload);
        client.endRequest();

        // Read Firebase response
        int statusCode = client.responseStatusCode();
        String response = client.responseBody();
        Serial.print("Response Code: ");
        Serial.println(statusCode);
        Serial.println("Firebase Response: " + response);
    } else {
        Serial.println("WiFi Disconnected. Cannot send emergency data.");
    }
}*/
