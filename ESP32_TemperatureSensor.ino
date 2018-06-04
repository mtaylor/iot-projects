#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SDA 19
#define SCL 18
#define BME280_ADDRESS = 0x76
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

// WiFi Details
static const char* ssid = "H&M-2.4G";
static const char* pass = "EleanorT16";

// MQTT Details
static const char* mqttServer = "test.mosquitto.org";
static const int mqttPort = 8883;
static const char* mqttUser =  NULL;
static const char* mqttPass =  NULL;
static const char* clientId = "myClient123";

// Device Config
static const String applicationId = "mtaylor";
static const String deviceId = "123";
static const String deviceTopic = "/" + applicationId + "/" + "devices/" + deviceId;

static const int retryPeriod = 1000;
static const int maxRetries = 5;
static const int sleepPeriod = 1 * 1000 * 1000; // 15mins - sleep period in uS

// Program variables
unsigned int lastMillis = 1000;
unsigned int sentMessages = 0;
Adafruit_BME280 bme;

WiFiClientSecure net;
MQTTClient client;

unsigned long shutdownTimer;
unsigned int messageCount = 1;  // Send 1 reading per wake time

void connectWiFi() {
  Serial.println("Connecting to WiFi ");
  Serial.print(ssid);

  WiFi.begin(ssid, pass);
  unsigned int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    Serial.print(".");
    delay(retryPeriod);
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected, ");
    Serial.print("IP address: ");
    Serial.print(WiFi.localIP());
    Serial.println("");
  } else {
    Serial.println("WiFi Connect Failed.  Max Retries Reached");
  }
}

void connectMQTT() {
  client.begin(mqttServer, mqttPort, net);
  client.onMessage(messageReceived);
  
  boolean connected = false;
  int retries = 0;
  
  Serial.println("Connecting to MQTT Broker");
  while (!connected && retries < maxRetries) {
    connected = client.connect(clientId, mqttUser, mqttPass);
    if (!connected) {
       delay(retryPeriod);
       Serial.print(".");
       retries ++;
    }
  }

  if (connected) {
    Serial.println("MQTT Client Connected.");
  } else {
    Serial.println("MQTT Connect Failed.  Max Retries Reached");
  }
}

void subscribeMQTT(const String &topic) {
  client.subscribe(topic.c_str());
  Serial.println(("Subcribed to " + topic).c_str());
}

void messageReceived(String &topic, String &payload) {
  Serial.println("");
  Serial.println("Received Message.");
  Serial.println("From: " + topic);
  Serial.println("Payload: " + payload);
  Serial.println("");
}

void sendMessage(const String &topic, const String &payload) {
  client.publish(topic, payload);
  Serial.println("");
  Serial.println("Sent Message.");
  Serial.println("To: " + topic);
  Serial.println("Payload: " + payload);
  Serial.println("");
}

void initSensor() {
  Wire.begin(19, 18); 
  
  bool status;
  status = bme.begin(0x76);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
  Serial.println("Sensor Activated");
}

void setup() {
  Serial.begin(115200);

  initSensor();
  connectWiFi();
  connectMQTT();
  String reading = sensorReading();
  subscribeMQTT(deviceTopic);

  shutdownTimer = millis() + 10000L;
//  esp_sleep_enable_timer_wakeup(sleepPeriod);
//  esp_deep_sleep_start();
}

String sensorReading() {
//  This JSON string does not play nice with the MQTT Client
//  String reading = "{\"id\":" + deviceId +
//    "," + wrapJSON("temperature", bme.readTemperature(), "*C") + 
//    "," + wrapJSON("pressure", bme.readPressure() / 100.0F, "hPa") +
//    "," + wrapJSON("altitude", bme.readAltitude(SEA_LEVEL_PRESSURE_HPA), "m") +
//    "," + wrapJSON("humidity", bme.readPressure() / 100.0F, "%") +
//    "}";

  String reading = "temperature=";
  reading += bme.readTemperature();
  Serial.println(("Sensor Reading" + reading).c_str());
  return reading;
}

String wrapJSON(String reading, float value, String unit) {
   return "\"" + reading + "\":{\"value\":" + value + ",\"unit\":\"" + unit + "\"}";
}

void loop() {
  if (millis() >= shutdownTimer) {
    esp_sleep_enable_timer_wakeup(sleepPeriod);
    esp_deep_sleep_start();
  }

  client.loop();
  delay(10);

  if (!client.connected()) {
    connectWiFi();
    connectMQTT();
    subscribeMQTT(deviceTopic);
  }

  while (sentMessages < messageCount) {
      //sendMessage(deviceTopic, sensorReading());
      client.publish(deviceTopic, sensorReading());
      sentMessages++;
  }
}
