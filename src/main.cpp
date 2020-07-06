#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoJson.h>
#include <Math.h>
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"
#include "config.h"

#define DEVICE_ID "MyNodeESP32"
#define MESSAGE_MAX_LEN 256 // size of message buffer
#define ONBOARD_LED_PIN 2

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being stored in plaintext
const char* ssid = CONFIG_WIFI_NAME;
const char* password = CONFIG_WIFI_PASSWORD;

// string containing HostName, Device Id & Device Key 
static const char* connectionString = CONFIG_CONNECTION_STRING;

// telemetry message
const char* messageData = "{\"messageId\":%d, \"x_distance\":%d, \"y_distance\":%d}";

int messageCount = 1; 
static long interval = 2000; //ms between telemetry messages
static bool hasWifi = false;
static bool messageSending = true;
static uint64_t send_interval_ms;
static bool ledValue = false;

/* DEMO: X AND Y VALUES */
static int xValue = 500;
static int yValue = 200;

/* //////////////// Utilities //////////////// */

static void InitWifi() {
  Serial.print(F("Connecting to ")); Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  // hang while not connected (bit of a hack)
  // TODO make connection time out, return with hasWifi false 
  do {
    delay(500);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED);

  hasWifi = true;
  Serial.println(F("WiFi connected"));
  Serial.print(F("IP Address: ")); Serial.println(WiFi.localIP());
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result) {
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
    Serial.println(F("Send Confirmation Callback finished."));
  }
}

static void MessageCallback(const char* payload, int size) {
  Serial.println(F("Message callback:"));
  Serial.println(payload);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char* payload, int size) {
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payload, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}

static int DeviceMethodCallback(const char* methodName, const unsigned char* payload, int size, unsigned char** response, int* response_size) {
  LogInfo("Try to invoke method %s", methodName);
  const char *responseMessage = "{\"status\":200}";
  int result = 200;

  StaticJsonDocument<MESSAGE_MAX_LEN> doc;
  deserializeJson(doc,payload,MESSAGE_MAX_LEN);
  Serial.println(F("Received payload:"));
  serializeJsonPretty(doc, Serial);
  Serial.println();

  if (strcmp(methodName, "start") == 0) {
    LogInfo("Start sending temperature and humidity data");
    messageSending = true;
  }
  else if (strcmp(methodName, "stop") == 0) {
    LogInfo("Stop sending temperature and humidity data");
    messageSending = false;
  } else if (strcmp(methodName, "led") == 0) {
    LogInfo("Toggling on-board LED");
    ledValue = !ledValue;
    digitalWrite(ONBOARD_LED_PIN, ledValue);
  } else if (strcmp(methodName, "interval") == 0) {
    LogInfo("Changing telemetry interval time");

    JsonVariant newInterval = doc["interval"];
    interval = newInterval.as<int>();
  } else if (strcmp(methodName, "move") == 0) {
    LogInfo("Moving motors");

    JsonVariant newXValue = doc["x"];
    JsonVariant newYValue = doc["y"];

    /* DEMO: PUT STEPPER MOTOR CODE HERE */
    xValue = newXValue.as<int>();
    yValue = newYValue.as<int>();

  } else {
    LogInfo("No method %s found", methodName);
    responseMessage = "{\"status\":404}";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

int max(int x, int y) { return (x > y) ? x : y;}
int min(int x, int y) { return (x < y) ? x : y;}


/* //////////////// Arduino Sketch //////////////// */

void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP32 Device"));
  Serial.println(F("Initializing..."));

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, ledValue);

  // initialize wifi module
  Serial.println(F("> WiFi"));
  hasWifi = false;
  InitWifi();
  if (!hasWifi) return;

  randomSeed(analogRead(0));

  Serial.println(F(" > IoT Hub"));
  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "GetStarted");
  Esp32MQTTClient_Init((const uint8_t*)connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  send_interval_ms = millis();
}

void loop() {
  if (hasWifi) {
    if (messageSending && (int)(millis() - send_interval_ms) >= interval) {
      char messagePayload[MESSAGE_MAX_LEN];

      /* DEMO: RANDOM VALUES, REPLACE WITH SENSOR CODE */
      int xDistance = min(max(xValue + random(0, 20) - 10, 0), 1000);
      int yDistance = min(max(yValue + random(0, 20) - 10, 0), 1000);

      // copy into message
      snprintf(messagePayload, MESSAGE_MAX_LEN, messageData, messageCount++, xDistance, yDistance);
      
      Serial.println(F("Sending message: "));
      Serial.println(messagePayload);

      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);

      Serial.println(F("------------------"));
      Serial.println();

      send_interval_ms = millis();
    } else {
      Esp32MQTTClient_Check();
    }
  }

  delay(100);
}