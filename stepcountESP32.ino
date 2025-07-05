#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const float threshold = 1.8;  // Sensitivity threshold for step detection
const int bufferLength = 15;  // Number of accelerometer readings in the buffer
float buffer[bufferLength];
int bufferIndex = 0;
int stepCount = 0;
bool stepDetected = false;

const unsigned long debounceDelay = 300;  // Debounce delay for step detection
unsigned long lastStepTime = 0;

// Kalman filter variables for Z-axis (to reduce Zerk noise)
float kalmanZ = 0;
float Q = 0.001;  // Process noise covariance
float R = 0.1;    // Measurement noise covariance
float P = 1;      // Estimate covariance
float K = 0;      // Kalman gain

// WiFi credentials
const char* ssid = "STUDENT";
const char* password = "cosmos1213";
 

WebServer server(80);

// OLED display update
void updateOledDisplay() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Step Count: ");
  display.print(stepCount);
  display.display();
}

// Kalman filter function for Z-axis to reduce Zerk noise
float kalmanFilter(float measurement, float estimate) {
  // Prediction
  P = P + Q;

  // Kalman gain
  K = P / (P + R);

  // Correction
  estimate = estimate + K * (measurement - estimate);

  // Update the estimate covariance
  P = (1 - K) * P;

  return estimate;
}

// Handle root HTML page
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Step Counter</title><style>body{font-family: Arial, sans-serif;background-color: #f9f9f9;margin: 0;padding: 0;}#container{width: 300px;margin: 50px auto;padding: 20px;background-color: #ffffff;border-radius: 10px;box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);}h1{text-align: center;color: #333333;}.info{text-align: center;font-size: 24px;color: #555555;margin-bottom: 20px;}.btn{display: block;width: 100%;padding: 10px;text-align: center;color: #ffffff;background-color: #007bff;border: none;border-radius: 5px;font-size: 18px;cursor: pointer;transition: background-color 0.2s ease;}.btn:hover{background-color: #0056b3;}#footer{text-align: center;color: #888888;position: fixed;bottom: 20px;left: 0;right: 0;}</style></head><body><div id=\"container\"><h1>Step Counter</h1><p class=\"info\">Step count: <span id=\"step-count\">" + String(stepCount) + "</span></p><button class=\"btn\" onclick=\"resetCount()\">Reset Count</button></div><div id=\"footer\"><p>IP Address: <span id=\"ip-address\">Loading...</span></p></div><script>function updateStepCount(count){document.getElementById(\"step-count\").innerText=count;}function updateIpAddress(ip){document.getElementById(\"ip-address\").innerText=ip;}function resetCount(){var xhr=new XMLHttpRequest();xhr.open(\"GET\",\"/reset\",true);xhr.onreadystatechange=function(){if(xhr.readyState==4&&xhr.status==200){var response=JSON.parse(xhr.responseText);updateStepCount(response.stepCount);}};xhr.send();}function getIpAddress(){var xhr=new XMLHttpRequest();xhr.open(\"GET\",\"https://api.ipify.org?format=json\",true);xhr.onreadystatechange=function(){if(xhr.readyState==4&&xhr.status==200){var response=JSON.parse(xhr.responseText);updateIpAddress(response.ip);}};xhr.send();}window.onload=function(){getIpAddress();setInterval(function(){location.reload();}, 5000);};</script></body></html>";
  server.send(200, "text/html", html);
}

// Reset step counter handler
void handleReset() {
  stepCount = 0;
  String response = "{\"stepCount\":" + String(stepCount) + "}";
  server.send(200, "application/json", response);
}

// WiFi connection handling and serial monitor logging
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Clear previous connections
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(1000);
    Serial.print("Attempt ");
    Serial.print(retries + 1);
    Serial.print(" - WiFi Status: ");
    Serial.println(WiFi.status());
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());  // Print IP address to the serial monitor
  } else {
    Serial.println("WiFi Connection Failed.");
    while (true); // Halt if connection fails
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!accel.begin()) {
    Serial.println("ADXL345 not found. Check wiring!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED display not found. Check wiring!");
    while (1);
  }

  display.display();//
  delay(1000);
  display.clearDisplay();

  // Zero out buffer
  for (int i = 0; i < bufferLength; i++) buffer[i] = 0;

  // Connect to WiFi and print the IP address
  connectToWiFi();

  // Start the web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/reset", HTTP_GET, handleReset);
  server.begin();

  Serial.println("Web server started");
}

void loop() {
  server.handleClient();  // Handle incoming client requests

  sensors_event_t event;
  accel.getEvent(&event);

  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  // Apply Kalman filter to reduce Zerk (Z-axis noise)
  kalmanZ = kalmanFilter(az, kalmanZ);

  // Further reduce Z-axis influence by adjusting the weight on the Z-axis
  float adjustedZ = kalmanZ * 0.2;  // Further reduced influence for Z-axis

  // Compute the magnitude with reduced Z-axis sensitivity
  float magnitude = sqrt(ax * ax + ay * ay + adjustedZ * adjustedZ);

  // Update buffer with the latest magnitude reading
  buffer[bufferIndex] = magnitude;
  bufferIndex = (bufferIndex + 1) % bufferLength;

  // Calculate the average magnitude for smoothing
  float avg = 0;
  for (int i = 0; i < bufferLength; i++) {
    avg += buffer[i];
  }
  avg /= bufferLength;

  unsigned long now = millis();

  // Step detection logic with dynamic threshold based on average magnitude
  if (magnitude > (avg + threshold)) {
    if (!stepDetected && (now - lastStepTime) > debounceDelay) {
      stepCount++;
      lastStepTime = now;
      stepDetected = true;

      Serial.println("Step detected!");
      Serial.print("Total Steps: ");
      Serial.println(stepCount);

      updateOledDisplay();
    }
  } else {
    stepDetected = false;
  }
}
