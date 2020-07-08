#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoJson.h>
#include <Math.h>
#include <FreeRTOS.h>
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"
#include "config.h"

#define MESSAGE_MAX_LEN 256 // size of message buffer

#define ONBOARD_LED_PIN 2

#define ULTRASOUND_X_TRIG_PIN 5 
#define ULTRASOUND_X_ECHO_PIN 18
#define ULTRASOUND_Y_TRIG_PIN 16
#define ULTRASOUND_Y_ECHO_PIN 17

#define MOTOR_X_STEP_PIN 23
#define MOTOR_X_DIR_PIN 22

#define MOTOR_Y_STEP_PIN 21
#define MOTOR_Y_DIR_PIN 19

#define CLK_PIN 4
#define DAT_PIN 0

#define STEP_SPEED 100

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being stored in plaintext
const char* ssid = CONFIG_WIFI_NAME;
const char* password = CONFIG_WIFI_PASSWORD;

// string containing HostName, Device Id & Device Key 
static const char* connectionString = CONFIG_CONNECTION_STRING;

// telemetry message
const char* messageData = "{\"messageId\":%d, \"x_distance\":%lf, \"y_distance\":%lf}";

int messageCount = 1; 
static long interval = 2000; //ms between telemetry messages
static bool messageSending = true;
static uint64_t send_interval_ms;
static bool ledValue = false;

// DEMO: STORED X AND Y VALUES, USED TO SIMULATE MOTOR LOCATION
static int yValue = 0;
static int xValue = 0;

// Hardware flags for passing info to comms
static bool newMotorInput = false;
static int newMotorXInput = 0;
static int newMotorYInput = 0;
SemaphoreHandle_t motorMutex;

// task handlers 
TaskHandle_t commsTask;
TaskHandle_t motorTask;

/* //////////////// Utilities //////////////// */

static bool InitWifi() {
  Serial.print(F("Connecting to ")); Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  // hang while not connected (bit of a hack)
  // TODO make connection time out, return with hasWifi false 
  do {
    delay(500);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED);

  Serial.println(F("WiFi connected"));
  Serial.print(F("IP Address: ")); Serial.println(WiFi.localIP());
  return true;
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
    LogInfo("Start sending coordinate data");
    messageSending = true;
  }
  else if (strcmp(methodName, "stop") == 0) {
    LogInfo("Stop sending coordinate data");
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

    // pass new values into shared variables, set flag that new data is available
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    newMotorInput = true;
    newMotorXInput = newXValue.as<int>();
    newMotorYInput = newYValue.as<int>();
    xSemaphoreGive(motorMutex);
  } else if (strcmp(methodName, "reset") == 0) {
    LogInfo("Resetting motors");
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    newMotorInput = true;
    newMotorXInput = 9999;
    newMotorYInput = 9999;
    xSemaphoreGive(motorMutex);
  } else {
    LogInfo("No method %s found", methodName);
    responseMessage = "{\"status\":404}";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

static double GetUltrasoundDistanceInInches(int trigPin, int echoPin) {
  // pinModes are necessary for some reason. Code does not work without them

  long duration;
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  return ((double) duration) / 74 / 2;  
}

static void ResetMotors() {
    digitalWrite(MOTOR_X_DIR_PIN, LOW);
    digitalWrite(MOTOR_Y_DIR_PIN, HIGH);
    while(xValue > 0 || yValue > 0) {
      if (xValue > 0) {
        digitalWrite(MOTOR_X_STEP_PIN, HIGH);
        xValue--;
      }
      if (yValue > 0) {
        digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
        yValue--;
      }
      delayMicroseconds(STEP_SPEED);
      digitalWrite(MOTOR_X_STEP_PIN, LOW);
      digitalWrite(MOTOR_Y_STEP_PIN, LOW);
      delayMicroseconds(STEP_SPEED);
    }
}

static void MoveMotors(int xxx, int yyy) {
  unsigned long ctrstepx = 0;
  unsigned long ctrstepy = 0;

  int flagsx = 0;                    // assigning sign to x value input. 0 means positive 1 means negative
  int flagsy = 0;                    // assigning sign to y value input. 0 means positive 1 means negative

  if(xxx > 0)                
    digitalWrite(MOTOR_X_DIR_PIN, HIGH);     // setting x motor direction according to input
  if(xxx < 0) {
    digitalWrite(MOTOR_X_DIR_PIN, LOW);      // setting x motor direction according to input
    xxx = abs(xxx);
    flagsx = 1;
  }
  if(yyy > 0)
    digitalWrite(MOTOR_Y_DIR_PIN, LOW);      // setting y motor direction according to input
  if(yyy < 0) {
    digitalWrite(MOTOR_Y_DIR_PIN, HIGH);     // setting y motor direction according to input
    yyy = abs(yyy);
    flagsy = 1;
  }

  // reset 
  if(xxx == 9999 && yyy == 9999) {    // reset input 9999,9999
    ResetMotors();
    return;
  }

  while(ctrstepx <= xxx || ctrstepy <= yyy) {
    if (ctrstepx <= xxx) {
      digitalWrite(MOTOR_X_STEP_PIN, HIGH);
      ctrstepx++;
    }
    if (ctrstepy <= yyy) {
      digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
      ctrstepy++;
    }
    delayMicroseconds(STEP_SPEED);
    digitalWrite(MOTOR_X_STEP_PIN, LOW);
    digitalWrite(MOTOR_Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_SPEED);
  }

  if(flagsx == 1)               // flags[ign]x is determined in the function input(), it is to keep track of the sign
    xValue = xValue - ctrstepx;     // of the input.
  if(flagsx == 0)
    xValue = xValue + ctrstepx;
    ctrstepx = 0;
    
  if(flagsy == 1)               // flags[ign]y is determined in the function input(), it is to keep track of the sign
    yValue = yValue - ctrstepy;     // of the input.
  if(flagsy == 0)
    yValue = yValue + ctrstepy;
  ctrstepy = 0;
}

int max(int x, int y) { return (x > y) ? x : y;}
int min(int x, int y) { return (x < y) ? x : y;}

/* //////////////// Tasks //////////////// */

static void CommsTask(void* pvParameters) {
  Serial.print("Starting messaging task on core ");
  Serial.println(xPortGetCoreID());

  // initialize wifi module
  Serial.println(F("> WiFi"));
  bool hasWifi = InitWifi();
  if (!hasWifi) return;

  Serial.println(F(" > IoT Hub"));
  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "GetStarted");
  Esp32MQTTClient_Init((const uint8_t*)connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  send_interval_ms = millis();

  while (true) {
    if (hasWifi) {
      if (messageSending && (int)(millis() - send_interval_ms) >= interval) {
        char messagePayload[MESSAGE_MAX_LEN];

        /* DEMO: RANDOM VALUES, REPLACE WITH SENSOR CODE */
        // increase priority for strict timing requirements with ultrasound sensor
        vTaskPrioritySet(commsTask, 2);
        double xDistance = GetUltrasoundDistanceInInches(ULTRASOUND_X_TRIG_PIN, ULTRASOUND_X_ECHO_PIN);
        double yDistance = GetUltrasoundDistanceInInches(ULTRASOUND_Y_TRIG_PIN, ULTRASOUND_Y_ECHO_PIN);
        vTaskPrioritySet(commsTask, 1);

        // copy into message
        snprintf(messagePayload, MESSAGE_MAX_LEN, messageData, messageCount++, xDistance, yDistance);
      
        Serial.println(F("Sending message: "));
        Serial.println(messagePayload);

        EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
        Esp32MQTTClient_SendEventInstance(message);

        Serial.println(F("------------------"));
        Serial.println();

        send_interval_ms = millis();
      } 
    }

    vTaskDelay(100);
    Esp32MQTTClient_Check();
    //TODO check if wifi is still connected
  }
}

static void MotorTask(void* pvParameters) {
  Serial.print("Starting motor task on core ");
  Serial.println(xPortGetCoreID());

  // task-specific copy of new x and y values
  int x;
  int y;

  while (true) {
    // TODO implement thread sleep/wake instead of using flags to communicate new data
    // check for new data from communications thread
    if (newMotorInput) {  
      xSemaphoreTake(motorMutex, portMAX_DELAY);
      newMotorInput = false;
      x = newMotorXInput;
      y = newMotorYInput;
      xSemaphoreGive(motorMutex);

      /* SERVO MOTOR CODE HERE */
      Serial.print(F("Moving (x,y): ("));
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.println(")");

      MoveMotors(x,y);

      Serial.println(F("Move finished"));
    }

    vTaskDelay(100);
  }
}

/* //////////////// Arduino Sketch //////////////// */

void setup() {
  pinMode(ULTRASOUND_X_ECHO_PIN, INPUT);
  pinMode(ULTRASOUND_X_TRIG_PIN, OUTPUT);
  pinMode(ULTRASOUND_Y_ECHO_PIN, INPUT);
  pinMode(ULTRASOUND_Y_TRIG_PIN, OUTPUT);
  pinMode(MOTOR_X_DIR_PIN, OUTPUT);
  pinMode(MOTOR_X_STEP_PIN, OUTPUT);
  pinMode(MOTOR_Y_DIR_PIN, OUTPUT);
  pinMode(MOTOR_Y_STEP_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(CLK_PIN, INPUT);
  pinMode(DAT_PIN, INPUT);
  
  digitalWrite(ULTRASOUND_X_TRIG_PIN, LOW);
  digitalWrite(ULTRASOUND_Y_TRIG_PIN, LOW);
  digitalWrite(MOTOR_X_DIR_PIN, LOW);
  digitalWrite(MOTOR_X_STEP_PIN, LOW);
  digitalWrite(MOTOR_Y_DIR_PIN, LOW);
  digitalWrite(MOTOR_Y_STEP_PIN, LOW);

  digitalWrite(ONBOARD_LED_PIN, ledValue);

  Serial.begin(115200);
  Serial.println(F("ESP32 Device"));
  Serial.println(F("Initializing..."));

  motorMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    CommsTask,
    "Comms Task",
    65536,
    NULL,
    1,
    &commsTask,
    0
  );

  xTaskCreatePinnedToCore(
    MotorTask,
    "Motor Task",
    16384,
    NULL,
    1,
    &motorTask,
    1
  );
}

void loop() {
}