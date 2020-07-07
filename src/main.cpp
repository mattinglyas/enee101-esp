#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoJson.h>
#include <Math.h>
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

/* //////////////// Constants //////////////// */

/* Communications */

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being stored in plaintext
const char* ssid = CONFIG_WIFI_NAME;
const char* password = CONFIG_WIFI_PASSWORD;

// string containing HostName, Device Id & Device Key 
static const char* connectionString = CONFIG_CONNECTION_STRING;

// telemetry message
const char* messageData = "{\"messageId\":%d, \"x_distance\":%lf, \"y_distance\":%lf}";

int messageCount = 1; 
static long interval = 2000; //ms between telemetry messages
static bool hasWifi = false;
static bool messageSending = true;
static uint64_t send_interval_ms;
static bool ledValue = false;

/* Hardware */

// initial values
static unsigned long xnum = 0;
static unsigned long ynum = 0;
static unsigned long t0;
static unsigned long t0m;
static unsigned long t1 = 0;
static unsigned long t1m = 0;
static bool xstat = LOW;
static bool ystat = LOW;
static bool reset = false;

// misc vars for step motor
static int ctrstep0 = 0;
static int ctrstep1 = 0;
static unsigned long ctrstepx = 0;
static unsigned long ctrstepy = 0;
static int flag0 = 0;      // flag for stepper motor taking turns per main loop
static int flagsx = 0;     // flag for keeping track of input signs for x
static int flagsy = 0;     // flag for keeping track of input signs for y
static int xValue = 0;     // used as storage to pass inputted values in
static int yValue = 0;

// misc vars for encoder operation
int enccntx = 0;
int last;
int cur;

/* //////////////// Utilities //////////////// */

/* Communications */ 

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

    /* DEMO: PUT STEPPER MOTOR CODE HERE */
    xValue = newXValue.as<int>();
    yValue = newYValue.as<int>();

    if (xValue == 9999 && yValue == 9999) reset = true;

  } else {
    LogInfo("No method %s found", methodName);
    responseMessage = "{\"status\":404}";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

/* Hardware */

void mcontrol() {
  if (xValue > 0) {
    // encod();
    /*  flag0 is used to switch between stepping x and y motors once per loop because they use the same
     *  timer (t0 - t1 > 100, i.e. ~100 microseconds) to step.
     *  the or case is used for both axes of opposing axes in case one was moved more than the other
     */
    if((t0 - t1 > 100 && flag0 == 0) || (t0 - t1 > 100 && yValue == 0)) {
      if(xstat == LOW)    // stepper motor function i.e. swap b/w HIGH & LOW on the xstep pin with a ~200us period
        xstat = HIGH;
      else
        xstat = LOW;
      digitalWrite(MOTOR_X_STEP_PIN, xstat);
      t1 = t0;            // t0 updates every main loop, this only updates once ~100 us *if condition is met above
      flag0 = 1;          // flag0 swaps back and forth between x and y motors
      ctrstepx++;         // ctrstepx keeps track of stepper motor steps on x axis
    }
    // one step equates to a incredibly small rotation so axis[] (inputted number) is upscaled by 50
    if(ctrstepx > xValue) {   // the motor stops stepping once this condition is met.
    //if (enccntx >= xValue) {     // alternative control scheme with encoder..
      if(flagsx == 1)               // flags[ign]x is determined in the function input(), it is to keep track of the sign
        xnum = xnum - ctrstepx;     // of the input.
      if(flagsx == 0)
        xnum = xnum + ctrstepx;
        
      ctrstepx = 0;                 // step counter resets after motor is done moving
      xValue = 0;                  // input axis # resets after motor is done moving
      // enccntx = 0;
      // Serial.println(xnum);
    }
  }
  if (yValue > 0) {
    // encody();
    /*  flag0 is used to switch between stepping x and y motors once per loop because they use the same
     *  timer (t0 - t1 > 100, i.e. ~100 microseconds) to step.
     *  the or case is used for both axes of opposing axes in case one was moved more than the other
     */
    if((t0 - t1 > 100 && flag0 == 1) || (t0 - t1 > 100 && xValue == 0)) {
      if (ystat == LOW)
        ystat = HIGH;
      else
        ystat = LOW;
      digitalWrite(MOTOR_Y_STEP_PIN, ystat);
      t1 = t0;            // t0 updates every main loop, this only updates once ~100 us *if condition is met above
      flag0 = 0;          // flag0 swaps back and forth between x and y motors
      ctrstepy++;         // ctrstepx keeps track of stepper motor steps on x axis
    }
                                    // one step equates to a incredibly small rotation so axis[] (inputted number) is upscaled by 50
    if(ctrstepy > yValue) {   // the motor stops stepping once this condition is met.
    //if (enccntx >= xValue) {     // alternative control scheme with encoder..
      if(flagsy == 1)               // flags[ign]y is determined in the function input(), it is to keep track of the sign
        ynum = ynum - ctrstepy;     // of the input.
      if(flagsy == 0)
        ynum = ynum + ctrstepy;
      ctrstepy = 0;                 // step counter resets after motor is done moving
      yValue = 0;                  // input axis # resets after motor is done moving
      // enccnty = 0;
      // Serial.println(ynum);
    }
  }
}

void resetpos() {
  if(reset == true) {
    if (xnum > 0) {
      if((t0 - t1 > 100 && flag0 == 0) || (t0 - t1 > 100 && ynum == 0)) {
        if(xstat == LOW)
          xstat = HIGH;
        else
          xstat = LOW;
        digitalWrite(MOTOR_X_STEP_PIN, xstat);
        t1 = t0;
        flag0 = 1;
        xnum--;
      }
      // Serial.println(xnum);
    }
    if (ynum > 0) {     //if a received
      if((t0 - t1 > 100 && flag0 == 1) || (t0 - t1 > 100 && xnum == 0)) {
        if(ystat == LOW)
          ystat = HIGH;
        else
          ystat = LOW;
        digitalWrite(MOTOR_Y_STEP_PIN, ystat);
        t1 = t0;
        flag0 = 0;
        ynum--;
      }
      // Serial.println(ynum);
    }
    if(ynum == 0 && xnum == 0) {
      reset = false;
    }
  }
}

long GetUltrasoundDelayUs(int trigPort, int echoPort) {
  // pinModes are necessary for some reason. Code does not work without them

  long duration;
  pinMode(trigPort, OUTPUT);
  digitalWrite(trigPort, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPort, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPort, LOW);
  pinMode(echoPort, INPUT);
  return pulseIn(echoPort, HIGH);
}

double DelayToInches(long duration) {
  return ((double) duration) / 74 / 2;  
}

int max(int x, int y) { return (x > y) ? x : y;}
int min(int x, int y) { return (x < y) ? x : y;}

/* //////////////// Arduino Sketch //////////////// */

void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP32 Device"));
  Serial.println(F("Initializing..."));

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

      double xDistance = DelayToInches(GetUltrasoundDelayUs(ULTRASOUND_X_TRIG_PIN, ULTRASOUND_X_ECHO_PIN));
      double yDistance = DelayToInches(GetUltrasoundDelayUs(ULTRASOUND_Y_TRIG_PIN, ULTRASOUND_Y_ECHO_PIN));

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

  mcontrol(); // motor control
  resetpos(); // reset position if reset flag is true
}