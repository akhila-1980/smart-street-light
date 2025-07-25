#include <WiFi.h>
#include "DHT.h"
#include <NewPing.h>  // Include NewPing library for ultrasonic sensor
#include <PubSubClient.h>  // Include the MQTT client library

// ====== GPIO Pin Definitions ======
const int redLED = 26;
const int DHTPIN = 5;
#define DHTTYPE DHT11
const int ldrPin = 34;  // LDR sensor pin

// Ultrasonic sensor pins
const int trigPin = 13;
const int echoPin = 12;

// ====== Global Variables ======
DHT dht(DHTPIN, DHTTYPE);
const int maxDistance = 200;  // Maximum distance in cm (for ultrasonic sensor)
NewPing sonar(trigPin, echoPin, maxDistance);  // Create a NewPing object for the ultrasonic sensor

unsigned long lastSensorRead = 0;  // Timestamp for sensor reading (DHT11 and LDR)
const long sensorInterval = 1000;  // Sensor read interval (100 ms) - same as ultrasonic sensor

unsigned long lastUltrasonicRead = 0;  // Timestamp for ultrasonic sensor reading
const long ultrasonicInterval = 500;  // Ultrasonic sensor read interval (100 ms)

unsigned long lastLEDOnTime = 0;  // Timestamp for when the LED was last turned ON
const long ledOnDuration = 2000;  // Duration for which the LED stays ON (2 seconds)
bool isLEDOn = false;  // Flag to track LED state

// New variables for distance-based timing control
unsigned long distanceGreaterStartTime = 0;  // Timestamp when distance became > 10cm
const long distanceThresholdTime = 10000;  // 10 seconds threshold for distance > 10cm
bool distanceTimerRunning = false;  // Flag to track if the distance timer is running

// ====== Seasonal Control Timings ======
int lightOnTimeWinter = 18;  // 6 PM (for Winter)
int lightOffTimeWinter = 7;  // 7 AM (for Winter)

int lightOnTimeSummer = 20;  // 8 PM (for Summer)
int lightOffTimeSummer = 6;  // 6 AM (for Summer)

// ====== Variables for User Input ======
int currentHour = 0;
int currentMonth = 0;
bool seasonalDecisionMade = false;  // Flag to track if seasonal logic was already applied

// ====== MQTT Setup ======
const char* mqtt_server = "mqtt.platinumrobotics.com";
const int mqttPort = 1883;
const char* mqtt_username = "mqtt_user"; // Replace with your MQTT username
const char* mqtt_password = "8655167646"; 
const char* mqtt_temp_topic = "sicteam/temperature";  // Topic for temperature data
const char* mqtt_humidity_topic = "sicteam/humidity";  // Topic for humidity data
const char* mqtt_ldr_topic = "sicteam/ldr";  // Topic for LDR value
const char* mqtt_distance_topic = "sicteam/distance";  // Topic for ultrasonic distance
const char* mqtt_led_state_topic = "sicteam/led_state";  // Topic for LED state (ON/OFF)
const char* mqtt_alert_topic = "sicteam/alerts";  // Topic to send alerts

WiFiClient espClient;
PubSubClient client(espClient);

// ====== Wi-Fi Setup ======
const char* ssid = ".";          // Replace with your Wi-Fi SSID
const char* password = "1234567890";  // Replace with your Wi-Fi password

// ====== Object Stuck Detection Variables ======
unsigned long obstacleDetectionStartTime = 0;  // Start time when obstacle is first detected
const long obstacleThresholdTime = 5000;  // Time threshold for continuous detection (5 seconds)
bool isObstacleDetected = false;  // Flag to track obstacle detection status

// ====== LED Control Function ======
void controlLED(String msg) {
  if (msg == "on1") {
    digitalWrite(redLED, HIGH);
    publishLEDState("ON");  // Publish LED state as ON
  } else if (msg == "off1") {
    digitalWrite(redLED, LOW);
    publishLEDState("OFF");  // Publish LED state as OFF
  } 
}

// ====== Publish LED State to MQTT ======
void publishLEDState(const char* state) {
  client.publish(mqtt_led_state_topic, state);  // Publish LED state to the topic
  Serial.println("[INFO] LED State: " + String(state));
}

// ====== MQTT Connect Function ======
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ====== Publish Sensor Data (Non-Blocking) ======
void publishSensorData() {
  // Seasonal Time-Based Control based on User Hour and Month (only once)
  if (!seasonalDecisionMade) {
    bool isWinter = (currentMonth == 12 || currentMonth <= 2);  // Simple check for Winter (Dec, Jan, Feb)
    bool isSummer = (currentMonth >= 6 && currentMonth <= 8);   // Simple check for Summer (Jun, Jul, Aug)

    if (isWinter) {
      if (currentHour >= lightOnTimeWinter || currentHour < lightOffTimeWinter) {
        digitalWrite(redLED, HIGH);  // Turn ON the LED
        publishLEDState("ON");  // Publish LED state as ON
        isLEDOn = true;
        Serial.println("[INFO] Winter Mode: Lights ON");
      }
    } else if (isSummer) {
      if (currentHour >= lightOnTimeSummer || currentHour < lightOffTimeSummer) {
        digitalWrite(redLED, HIGH);  // Turn ON the LED
        publishLEDState("ON");  // Publish LED state as ON
        isLEDOn = true;
        Serial.println("[INFO] Summer Mode: Lights ON");
      }
    } else {
      digitalWrite(redLED, LOW);  // Turn OFF the LED
      publishLEDState("OFF");  // Publish LED state as OFF
      isLEDOn = false;
      Serial.println("[INFO] Default Mode: Lights OFF");
    }

    seasonalDecisionMade = true;  // Set flag to true after seasonal logic is applied
  }
  
  // ====== Sensor Data Read (DHT11 and LDR) ======
  if (millis() - lastSensorRead >= sensorInterval) {
    lastSensorRead = millis();  // Update timestamp
    
    // Read DHT11 sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int ldrValue = analogRead(ldrPin);  // Read LDR value
  
    if (!isnan(temperature) && !isnan(humidity)) {
      // Publish the sensor data to MQTT topic (Temperature)
      String tempPayload = String(temperature);
      String tempSerialPayload = String("Temperature: ") + temperature + " C";
      client.publish(mqtt_temp_topic, tempPayload.c_str());
      Serial.println("[INFO] " + tempSerialPayload);

      // Publish humidity data to MQTT topic (Humidity)
      String humidityPayload = String(humidity) ;
      String humiditySerialPayload = String("Humidity: ")+humidity + " %";
      client.publish(mqtt_humidity_topic, humidityPayload.c_str());
      Serial.println("[INFO] " + humiditySerialPayload);

      // Publish LDR value to MQTT topic (LDR)
      String ldrPayload = String(ldrValue);
      String ldrSerialPayload = "LDR Value: " + String(ldrValue);
      client.publish(mqtt_ldr_topic, ldrPayload.c_str());
      Serial.println("[INFO] " + ldrSerialPayload);

      // NEW CODE: If LDR value < 2000 (enough light), turn OFF LED regardless of other conditions
      if (ldrValue < 2000) {
        if (isLEDOn) {
          digitalWrite(redLED, LOW);
          publishLEDState("OFF");  // Publish LED state as OFF
          isLEDOn = false;
          Serial.println("[INFO] Sufficient light detected (LDR < 2000), LED turned OFF");
        }
        // Skip other LED control logic since we want to keep LED OFF in sufficient light
        return;
      }

      // Brightness control: LED ON based on humidity conditions (only if LDR >= 2000)
      bool isHumid = humidity > 80;  // If humidity is high

      if (isHumid) {
        digitalWrite(redLED, HIGH);
        publishLEDState("ON");  // Publish LED state as ON
        isLEDOn = true;
        lastLEDOnTime = millis();
        Serial.println("[INFO] LED turned ON based on humidity");
      }
    } else {
      Serial.println("[ERROR] Failed to read from DHT sensor!");
    }
  }

  // ====== Ultrasonic Sensor Logic ======
  if (millis() - lastUltrasonicRead >= ultrasonicInterval) {
    lastUltrasonicRead = millis();  // Update timestamp
    
    // Read ultrasonic sensor data
    long distance = sonar.ping_cm();
    if (distance == 0) {
      Serial.println("[WARNING] Ultrasonic sensor out of range");
      distance = maxDistance;  // Assume no object detected
    }
    
    // Publish the ultrasonic distance to MQTT topic (Distance)
    String distanceSerailPayload = "Distance: " + String(distance) + " cm";
    String distancePayload = String(distance);  // Only the distance value as a string
client.publish(mqtt_distance_topic, distancePayload.c_str());
Serial.println("[INFO] " + distanceSerailPayload);

    // Read current LDR value
    int ldrValue = analogRead(ldrPin);

    // NEW CODE: If LDR value < 2000 (enough light), turn OFF LED regardless of other conditions
    if (ldrValue < 2000) {
      if (isLEDOn) {
        digitalWrite(redLED, LOW);
        publishLEDState("OFF");  // Publish LED state as OFF
        isLEDOn = false;
        Serial.println("[INFO] Sufficient light detected (LDR < 2000), LED turned OFF");
      }
      // Skip other ultrasonic sensor logic since we want to keep LED OFF in sufficient light
      return;
    }

    // Object Stuck Detection Logic (Obstacle detection) - Only run this if LDR >= 2000
    if (distance < 10) {  // If distance is less than 10 cm, obstacle is detected
      if (!isObstacleDetected) {
        obstacleDetectionStartTime = millis();
        isObstacleDetected = true;
      } else {
        if (millis() - obstacleDetectionStartTime >= obstacleThresholdTime) {
          // Send MQTT alert to notify about the object stuck
          String alertMessage = "ALERT: Object stuck on streetlight, obstacle detected for " + String(obstacleThresholdTime / 1000) + " seconds.";
          client.publish(mqtt_alert_topic, alertMessage.c_str());
          Serial.println("[ALERT] Object stuck detected, alert sent to maintenance.");
          
          isObstacleDetected = false;
        }
      }
      
      // If LDR > 2000 (dark) and distance < 10 cm, turn ON LED
      digitalWrite(redLED, HIGH);
      publishLEDState("ON");  // Publish LED state as ON
      isLEDOn = true;
      Serial.println("[INFO] Dark condition and object detected, LED turned ON");
      
      // Reset the distance timer since object is now close
      distanceTimerRunning = false;
    } else {
      // Distance > 10 cm
      isObstacleDetected = false;
      
      // If LDR > 2000 (dark) and distance > 10 cm, start timing
      if (!distanceTimerRunning) {
        distanceGreaterStartTime = millis();
        distanceTimerRunning = true;
        Serial.println("[INFO] Distance > 10cm timer started");
      } else {
        // Check if 10 seconds have passed with distance > 10 cm
        if (millis() - distanceGreaterStartTime >= distanceThresholdTime) {
          if (isLEDOn) {
            digitalWrite(redLED, LOW);
            publishLEDState("OFF");  // Publish LED state as OFF
            isLEDOn = false;
            Serial.println("[INFO] No object detected for 10 seconds, LED turned OFF");
          }
        }
      }
    }
  }
}

// ====== Arduino Setup ======
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(redLED, OUTPUT);
  pinMode(ldrPin, INPUT);
  analogReadResolution(12);
  dht.begin();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);  // Use the credentials defined above
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to Wi-Fi. Exiting...");
    return;  // Stop the program if unable to connect
  }
  Serial.println("WiFi connected");

  // Connect to MQTT broker
  client.setServer(mqtt_server, mqttPort);
  reconnect();

  // Get user input for current month and hour
  Serial.println("Please enter the current month (1-12): ");
  while (!Serial.available()) {}  // Wait for user input
  currentMonth = Serial.parseInt();
  Serial.read();  // Consume newline
  Serial.println("Month received: " + String(currentMonth));

  Serial.println("Please enter the current hour (0-23): ");
  while (!Serial.available()) {}  // Wait for user input
  currentHour = Serial.parseInt();
  Serial.read();  // Consume newline
  Serial.println("Hour received: " + String(currentHour));

  if (currentMonth < 1 || currentMonth > 12 || currentHour < 0 || currentHour > 23) {
    Serial.println("[ERROR] Invalid input. Restart and enter correct values.");
  } else {
    publishSensorData();
  }
}

// ====== Arduino Loop ======
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  publishSensorData();
}