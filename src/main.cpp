#include <Arduino.h>
#include <Wifi.h>
#include <ArduinoJson.h>
#include <Math.h>
#include <FreeRTOS.h>
#include <LinkedList.h>
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"
#include "config.h"

#define MESSAGE_MAX_LEN 2048 // size of message buffers
#define COMMAND_BUFFER_LEN 256  // local command queue max length

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

enum MotorState
{
  MOTOR_IDLE = 0,
  MOTOR_MOVE = 1,
  MOTOR_RESET = 2
};

struct MotorCommand
{
  enum MotorState commandType;
  int x;
  int y;
};

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being stored in plaintext
const char *ssid = CONFIG_WIFI_NAME;
const char *password = CONFIG_WIFI_PASSWORD;

// string containing HostName, Device Id & Device Key
static const char *connectionString = CONFIG_CONNECTION_STRING;

// telemetry message
const char *messageData = "{\"messageId\":%d, \"x_distance\":%lf, \"y_distance\":%lf}";
const char *responseMessageData = "{\"status\": %d}";

int messageCount = 1;
static long interval = 2000; //ms between telemetry messages
static bool messageSending = true;
static uint64_t send_interval_ms;
static bool ledValue = false;

// stored x and y values
static long yValue = 0;
static long xValue = 0;

// shared resource for hardware and comms thread to communicate values
static LinkedList<MotorCommand> motorCommandQueue;
static enum MotorState currentMotorState;
SemaphoreHandle_t motorCommandMutex;

// task handlers
TaskHandle_t commsTask;
TaskHandle_t motorTask;

/* //////////////// Azure Utilities //////////////// */

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    Serial.println(F("Info: Send Confirmation Callback finished"));
  }
}

static void MessageCallback(const char *payload, int size)
{
  Serial.print(F("Info: Message callback:"));
  Serial.println(payload);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload, int size)
{
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

static int DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
  Serial.print("Info: Trying to invoke method ");
  Serial.println(methodName);

  char responseMessage[MESSAGE_MAX_LEN];

  int result = 200;

  StaticJsonDocument<MESSAGE_MAX_LEN> doc;
  deserializeJson(doc, payload, MESSAGE_MAX_LEN);
  Serial.print(F("Info: Received payload: "));
  serializeJson(doc, Serial);
  Serial.println();

  if (strcmp(methodName, "start") == 0)
  {
    messageSending = true;
    Serial.println(F("Info: Started sending telemetry messages"));
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    messageSending = false;
    Serial.println(F("Info: Stopped sending telemetry messages"));
  }
  else if (strcmp(methodName, "led") == 0)
  {
    ledValue = !ledValue;
    digitalWrite(ONBOARD_LED_PIN, ledValue);
    Serial.println(F("Info: Toggled on-board LED"));
  }
  else if (strcmp(methodName, "interval") == 0)
  {
    JsonVariant newInterval = doc["interval"];
    interval = newInterval.as<int>();
    Serial.print(F("Info: Changed telemetry interval to "));
    Serial.print(interval);
    Serial.println(F("ms"));
  }
  else if (strcmp(methodName, "move") == 0)
  {
    JsonVariant newXValue = doc["x"];
    JsonVariant newYValue = doc["y"];

    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    if (motorCommandQueue.size() < COMMAND_BUFFER_LEN)
    {
      // pass new values into shared space
      struct MotorCommand newCommand;
      newCommand.commandType = MOTOR_MOVE;
      newCommand.x = newXValue.as<int>();
      newCommand.y = newYValue.as<int>();
      motorCommandQueue.add(newCommand);

      Serial.print(F("Info: Queued move command with input (x,y) = "));
      Serial.print(newCommand.x);
      Serial.print(F(","));
      Serial.print(newCommand.y);
      Serial.print(F(" at position "));
      Serial.println(motorCommandQueue.size());
    }
    else
    {
      Serial.println(F("Warning: Queue full; move command discarded"));
      result = 400;
    }
    xSemaphoreGive(motorCommandMutex);
  }
  else if (strcmp(methodName, "moveArray") == 0)
  {
    int arraySizeX = doc["x"].size();
    int arraySizeY = doc["y"].size();

    if (arraySizeX == arraySizeY) 
    { 
      xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
      for (int i = 0; i < arraySizeX; i++) 
      {
        if (motorCommandQueue.size() < COMMAND_BUFFER_LEN)
        {
          // pass new values into shared space
          struct MotorCommand newCommand;
          newCommand.commandType = MOTOR_MOVE;
          newCommand.x = doc["x"][i];
          newCommand.y = doc["y"][i];
          motorCommandQueue.add(newCommand);

          Serial.print(F("Info: Queued array move command with input (x,y) = "));
          Serial.print(newCommand.x);
          Serial.print(F(","));
          Serial.print(newCommand.y);
          Serial.print(F(" at position "));
          Serial.println(motorCommandQueue.size());
        }
        else
        {
          Serial.println(F("Warning: Queue full; remainder of array move command discarded"));
          result = 400;
          break;
        }
      }
      xSemaphoreGive(motorCommandMutex);
    }
    else
    {
      Serial.println(F("Warning: Received improperly formatted moveArray command; discarding"));
      result = 400;
    }
  }
  else if (strcmp(methodName, "reset") == 0)
  {
    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    if (motorCommandQueue.size() < COMMAND_BUFFER_LEN)
    {
      // pass new values into shared space
      struct MotorCommand newCommand;
      newCommand.commandType = MOTOR_RESET;
      motorCommandQueue.add(newCommand);
      Serial.print(F("Info: Queued reset command at position "));
      Serial.println(motorCommandQueue.size());
    }
    else
    {
      Serial.println(F("Warning: Queue full; reset command discarded"));
      result = 400;
    }
    xSemaphoreGive(motorCommandMutex);
  }
  else
  {
    result = 404;
    Serial.println(F("Warning: Received invalid command"));
  }

  snprintf(responseMessage, MESSAGE_MAX_LEN, responseMessageData, result);

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

/* //////////////// Device Utilities //////////////// */

static bool InitWifi()
{
  Serial.print(F("Info: Connecting to "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // hang while not connected (bit of a hack)
  // TODO make connection time out, return with hasWifi false
  do
  {
    delay(500);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED);

  Serial.println();
  Serial.print(F("Info: WiFi connected. Local IP Address: "));
  Serial.println(WiFi.localIP());
  return true;
}

static double GetUltrasoundDistanceInInches(int trigPin, int echoPin)
{
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
  return ((double)duration) / 74 / 2;
}

static void ResetMotors()
{
  digitalWrite(MOTOR_X_DIR_PIN, LOW);
  digitalWrite(MOTOR_Y_DIR_PIN, HIGH);
  while (xValue > 0 || yValue > 0)
  {
    if (xValue > 0)
    {
      digitalWrite(MOTOR_X_STEP_PIN, HIGH);
      xValue--;
    }
    if (yValue > 0)
    {
      digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
      yValue--;
    }
    delayMicroseconds(STEP_SPEED);
    digitalWrite(MOTOR_X_STEP_PIN, LOW);
    digitalWrite(MOTOR_Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_SPEED);
  }
}

static void MoveMotors(int xxx, int yyy)
{
  unsigned long ctrstepx = 0;
  unsigned long ctrstepy = 0;

  int flagsx = 0; // assigning sign to x value input. 0 means positive 1 means negative
  int flagsy = 0; // assigning sign to y value input. 0 means positive 1 means negative

  if (xxx > 0)
    digitalWrite(MOTOR_X_DIR_PIN, HIGH); // setting x motor direction according to input
  if (xxx < 0)
  {
    digitalWrite(MOTOR_X_DIR_PIN, LOW); // setting x motor direction according to input
    xxx = abs(xxx);
    flagsx = 1;
  }
  if (yyy > 0)
    digitalWrite(MOTOR_Y_DIR_PIN, LOW); // setting y motor direction according to input
  if (yyy < 0)
  {
    digitalWrite(MOTOR_Y_DIR_PIN, HIGH); // setting y motor direction according to input
    yyy = abs(yyy);
    flagsy = 1;
  }

  while (ctrstepx <= xxx || ctrstepy <= yyy)
  {
    if (ctrstepx <= xxx)
    {
      digitalWrite(MOTOR_X_STEP_PIN, HIGH);
      ctrstepx++;
    }
    if (ctrstepy <= yyy)
    {
      digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
      ctrstepy++;
    }
    delayMicroseconds(STEP_SPEED);
    digitalWrite(MOTOR_X_STEP_PIN, LOW);
    digitalWrite(MOTOR_Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_SPEED);
  }

  if (flagsx == 1)              // flags[ign]x is determined in the function input(), it is to keep track of the sign
    xValue = xValue - ctrstepx; // of the input.
  if (flagsx == 0)
    xValue = xValue + ctrstepx;
  ctrstepx = 0;

  if (flagsy == 1)              // flags[ign]y is determined in the function input(), it is to keep track of the sign
    yValue = yValue - ctrstepy; // of the input.
  if (flagsy == 0)
    yValue = yValue + ctrstepy;
  ctrstepy = 0;
}

/* //////////////// Tasks //////////////// */

static void CommsTask(void *pvParameters)
{
  Serial.print(F("Info: Starting messaging task on core "));
  Serial.println(xPortGetCoreID());

  // initialize wifi module
  bool hasWifi = InitWifi();
  if (!hasWifi)
    return;

  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "ENEE101-ESP32");
  Esp32MQTTClient_Init((const uint8_t *)connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  send_interval_ms = millis();

  while (true)
  {
    if (hasWifi)
    {
      if (messageSending && (int)(millis() - send_interval_ms) >= interval)
      {
        send_interval_ms = millis();
        char messagePayload[MESSAGE_MAX_LEN];

        // suspend task scheduler to ensure specific timing of ultrasonic sensors is met
        vTaskSuspendAll();
        double xDistance = GetUltrasoundDistanceInInches(ULTRASOUND_X_TRIG_PIN, ULTRASOUND_X_ECHO_PIN);
        double yDistance = GetUltrasoundDistanceInInches(ULTRASOUND_Y_TRIG_PIN, ULTRASOUND_Y_ECHO_PIN);
        xTaskResumeAll();

        // copy into message
        snprintf(messagePayload, MESSAGE_MAX_LEN, messageData, messageCount++, xDistance, yDistance);

        Serial.print(F("Info: Sending message to server "));
        Serial.println(messagePayload);

        EVENT_INSTANCE *message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
        Esp32MQTTClient_SendEventInstance(message);
      }

      Esp32MQTTClient_Check();
    }

    vTaskDelay(100);
    //TODO check if wifi is still connected
  }
}

static void MotorTask(void *pvParameters)
{
  Serial.print(F("Info: Starting motor task on core "));
  Serial.println(xPortGetCoreID());

  // task-specific copy of new command
  MotorCommand command;

  // task-specific copy of queue size
  int commandQueueLength;

  while (true)
  {
    // check for new data from communications thread
    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    commandQueueLength = motorCommandQueue.size();
    xSemaphoreGive(motorCommandMutex);

    while (commandQueueLength > 0)
    {
      // new data is available, copy first command
      xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
      command = motorCommandQueue.shift();
      commandQueueLength = motorCommandQueue.size();
      xSemaphoreGive(motorCommandMutex);

      currentMotorState = command.commandType;

      switch (command.commandType)
      {
        case MOTOR_IDLE:
        {
          // should not get here
          break;
        }
        case MOTOR_MOVE:
        {
          Serial.print(F("Info: Moving motors by x,y = "));
          Serial.print(command.x);
          Serial.print(F(","));
          Serial.println(command.y);
          
          MoveMotors(command.x, command.y);

          Serial.println(F("Info: Move finished"));
          break;
        }
        case MOTOR_RESET:
        {
          Serial.println(F("Info: Resetting motors"));
          ResetMotors();
          Serial.println(F("Info: Reset finished"));
          break;
        }
        default:
        {
          // should not get here
          break;
        }
      }
    }

    currentMotorState = MOTOR_IDLE;
    vTaskDelay(100);
  }
}

/* //////////////// Arduino Sketch //////////////// */

void setup()
{
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

  motorCommandMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
      CommsTask,
      "Comms Task",
      65536,
      NULL,
      1,
      &commsTask,
      0);

  xTaskCreatePinnedToCore(
      MotorTask,
      "Motor Task",
      16384,
      NULL,
      1,
      &motorTask,
      1);
}

void loop()
{
}