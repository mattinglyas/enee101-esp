#include <Arduino.h>
#include <Wifi.h>
#include "ArrayQueue.h"
#include "ArduinoJson.h"
#include "FreeRTOS.h"
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"
#include "Config.h"

#define MESSAGE_MAX_LEN 16384 // size of message buffers
#define COMMAND_BUFFER_LEN 256  // local command queue max length

#define WIFI_TIMEOUT_MS 10000 // timeout for connecting to wifi network

#define ONBOARD_LED_PIN 2

#define ULTRASOUND_X_TRIG_PIN 5
#define ULTRASOUND_X_ECHO_PIN 18
#define ULTRASOUND_Y_TRIG_PIN 16
#define ULTRASOUND_Y_ECHO_PIN 17

#define MOTOR_X_STEP_PIN 23
#define MOTOR_X_DIR_PIN 22

#define MOTOR_Y_STEP_PIN 21
#define MOTOR_Y_DIR_PIN 19

#define MOTOR_XY_ENABLE_PIN 4

#define LIMIT_X_LOW_PIN 12 // purple
#define LIMIT_X_HIGH_PIN 13 // orange
#define LIMIT_Y_LOW_PIN 34 // yellow 
#define LIMIT_Y_HIGH_PIN 35 // white 

#define ENCODER_X_A_PIN 25  // CLK X
#define ENCODER_X_A_REGISTER 0x2000000   // register for GPIO25
#define ENCODER_X_B_PIN 26  // DATA X
#define ENCODER_X_B_REGISTER 0x4000000   // register for GPIO26
#define ENCODER_Y_A_PIN 27  // CLK Y
#define ENCODER_Y_A_REGISTER 0x8000000    // register for GPIO27
#define ENCODER_Y_B_PIN 14  // DATA Y
#define ENCODER_Y_B_REGISTER 0x4000       // register for GPIO14

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
  long x;
  long y;
};

struct UltrasonicSensor {
  uint8_t trig;
  uint8_t echo;
};

// ultrasonic sensors
static const UltrasonicSensor xSensor = {ULTRASOUND_X_TRIG_PIN, ULTRASOUND_X_ECHO_PIN};
static const UltrasonicSensor ySensor = {ULTRASOUND_Y_TRIG_PIN, ULTRASOUND_Y_ECHO_PIN};

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being stored in plaintext
const char *ssid = CONFIG_WIFI_NAME;
const char *password = CONFIG_WIFI_PASSWORD;

// string containing HostName, Device Id & Device Key
static const char *connectionString = CONFIG_CONNECTION_STRING;

// telemetry message
const char *messageData = "{\"messageId\":%d, \"x_distance\":%lf, \"y_distance\":%lf}";
const char *responseMessageData = "{\"status\": %d}";

static int messageCount = 1;
static long interval = 2000; //ms between telemetry messages
static bool messageSending = true;
static uint64_t send_interval_ms;
static bool ledValue = false;

// shared resource for hardware and comms thread to communicate values
static ArrayQueue<MotorCommand> motorCommandQueue = ArrayQueue<MotorCommand>(COMMAND_BUFFER_LEN);
static enum MotorState currentMotorState;
SemaphoreHandle_t motorCommandMutex;

// task handlers
TaskHandle_t commsTaskHandler;
TaskHandle_t motorTaskHandler;

// encoder values
volatile byte aXFlag = 0;
volatile byte bXFlag = 0;
volatile long encoderXPosCur = 0;
volatile unsigned long readXReg = 0;
volatile byte aYFlag = 0;
volatile byte bYFlag = 0;
volatile long encoderYPosCur = 0;
volatile unsigned long readYReg = 0;

/* //////////////// Azure Utilities //////////////// */

static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    Serial.println(F("Info: Send Confirmation Callback finished"));
  }
}

static void messageCallback(const char *payload, int size)
{
  Serial.print(F("Info: Message callback:"));
  Serial.println(payload);
}

static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload, int size)
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

static int deviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
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
    if (motorCommandQueue.count() < COMMAND_BUFFER_LEN)
    {
      // pass new values into shared space
      struct MotorCommand newCommand;
      newCommand.commandType = MOTOR_MOVE;
      newCommand.x = newXValue.as<long>();
      newCommand.y = newYValue.as<long>();
      motorCommandQueue.push(newCommand);

      Serial.print(F("Info: Queued move command with input (x,y) = "));
      Serial.print(newCommand.x);
      Serial.print(F(","));
      Serial.print(newCommand.y);
      Serial.print(F(" at position "));
      Serial.println(motorCommandQueue.count());
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
        if (motorCommandQueue.count() < COMMAND_BUFFER_LEN)
        {
          // pass new values into shared space
          struct MotorCommand newCommand;
          newCommand.commandType = MOTOR_MOVE;
          newCommand.x = doc["x"][i].as<long>();
          newCommand.y = doc["y"][i].as<long>();
          motorCommandQueue.push(newCommand);

          Serial.print(F("Info: Queued array move command with input (x,y) = "));
          Serial.print(newCommand.x);
          Serial.print(F(","));
          Serial.print(newCommand.y);
          Serial.print(F(" at position "));
          Serial.println(motorCommandQueue.count());
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
      Serial.print(F("Warning: Received improperly formatted moveArray command. X array is length "));
      Serial.print(arraySizeX);
      Serial.print(F(" and Y array is length "));
      Serial.println(arraySizeY);
      result = 400;
    }
  }
  else if (strcmp(methodName, "reset") == 0)
  {
    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    if (motorCommandQueue.count() < COMMAND_BUFFER_LEN)
    {
      // pass new values into shared space
      struct MotorCommand newCommand;
      newCommand.commandType = MOTOR_RESET;
      motorCommandQueue.push(newCommand);
      Serial.print(F("Info: Queued reset command at position "));
      Serial.println(motorCommandQueue.count());
    }
    else
    {
      Serial.println(F("Warning: Queue full; reset command discarded"));
      result = 400;
    }
    xSemaphoreGive(motorCommandMutex);
  }
  else if (strcmp(methodName, "clear") == 0)
  {
    Serial.println(F("Info: Clearing motor command queue"));
    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    motorCommandQueue.clear();
    xSemaphoreGive(motorCommandMutex);
  }
  else if (strcmp(methodName, "clearReset") == 0) 
  {
    Serial.println(F("Info: Clearing motor command queue and resetting motor position"));
    xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
    motorCommandQueue.clear();

    struct MotorCommand newCommand;
    newCommand.commandType = MOTOR_RESET;
    motorCommandQueue.push(newCommand);
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

static bool initWifi(int timeoutInMs)
{
  Serial.print(F("Info: Attempting to connect to "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int endTime = millis() + timeoutInMs;

  do
  {
    //delay(500);
    //Serial.print(".");
    delay(50);
    if (millis() > endTime) 
    {
      Serial.println(F("Error: Could not connect to WiFi"));
      return false;
    }
  } while (WiFi.status() != WL_CONNECTED);

  Serial.print(F("Info: WiFi connected. Local IP Address: "));
  Serial.println(WiFi.localIP());
  return true;
}

static double getUltrasonicDistanceInInches(const UltrasonicSensor* s)
{
  // pinModes are necessary for some reason. Code does not work without them

  long duration;
  pinMode((*s).trig, OUTPUT);
  digitalWrite((*s).trig, LOW);
  delayMicroseconds(2);
  digitalWrite((*s).trig, HIGH);
  delayMicroseconds(10);
  digitalWrite((*s).trig, LOW);
  pinMode((*s).echo, INPUT);
  duration = pulseIn((*s).echo, HIGH);
  return ((double) duration) / 74 / 2;
}

static void resetMotors()
{
  // turn on motors
  digitalWrite(MOTOR_XY_ENABLE_PIN, LOW);

  digitalWrite(MOTOR_X_DIR_PIN, LOW);
  digitalWrite(MOTOR_Y_DIR_PIN, LOW);

  bool xMove = true;
  bool yMove = true;

  // move the motors until the limit switches are hit
  do
  {
    xMove = xMove && (digitalRead(LIMIT_X_LOW_PIN) == LOW);
    yMove = yMove && (digitalRead(LIMIT_Y_LOW_PIN) == LOW);

    // move motors if needed
    if (xMove)
    {
      digitalWrite(MOTOR_X_STEP_PIN, HIGH);
    }

    if (yMove)
    {
      digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
    }

    delayMicroseconds(STEP_SPEED);
    digitalWrite(MOTOR_X_STEP_PIN, LOW);
    digitalWrite(MOTOR_Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_SPEED);
  } while (xMove || yMove);

  // reset encoder position
  encoderXPosCur = 0;
  encoderYPosCur = 0;

  // turn off motors
  digitalWrite(MOTOR_XY_ENABLE_PIN, HIGH);
}

static void moveMotors(long xxx, long yyy)
{
  // turn on motors
  digitalWrite(MOTOR_XY_ENABLE_PIN, LOW);

  // calculate target encoder state
  long targetEncoderXPos = encoderXPosCur + xxx;
  long targetEncoderYPos = encoderYPosCur + yyy;

  Serial.print(F("Info: Calculated target encoder X/Y values as: "));
  Serial.print(targetEncoderXPos);
  Serial.print(F(","));
  Serial.println(targetEncoderYPos);

  bool xDir = false; // assigning sign to x value input. false means positive (high); true means negative (low)
  bool yDir = false; // assigning sign to y value input. false means positive (high); true means negative (low)

  // setting x motor direction according to input
  if (xxx >= 0) 
    digitalWrite(MOTOR_X_DIR_PIN, HIGH); 
  else
  {
    digitalWrite(MOTOR_X_DIR_PIN, LOW); 
    xDir = true;
  }

  if (yyy >= 0)
    digitalWrite(MOTOR_Y_DIR_PIN, HIGH); 
  else
  {
    digitalWrite(MOTOR_Y_DIR_PIN, LOW); 
    yDir = true; 
  }

  // sleep task for a bit to make sure direction is set
  vTaskDelay(1);

  // flag for if motors need to be moved in this direction
  bool xMove = xDir ? (targetEncoderXPos < encoderXPosCur) : (targetEncoderXPos > encoderXPosCur);
  bool yMove = yDir ? (targetEncoderYPos < encoderYPosCur) : (targetEncoderYPos > encoderYPosCur);

  uint8_t xLimitSwitchPin = (xDir ? LIMIT_X_LOW_PIN : LIMIT_X_HIGH_PIN);
  uint8_t yLimitSwitchPin = (yDir ? LIMIT_Y_LOW_PIN : LIMIT_Y_HIGH_PIN);

  do
  {
    // check if steps are still left in the motor move and that the limit switch has not been tripped
    xMove = xMove && (digitalRead(xLimitSwitchPin) == LOW) && (xDir ? (targetEncoderXPos < encoderXPosCur) : (targetEncoderXPos > encoderXPosCur));
    yMove = yMove && (digitalRead(yLimitSwitchPin) == LOW) && (yDir ? (targetEncoderYPos < encoderYPosCur) : (targetEncoderYPos > encoderYPosCur));

    // move motors 
    if (xMove) 
    {
      digitalWrite(MOTOR_X_STEP_PIN, HIGH);
    }

    if (yMove)
    {
      digitalWrite(MOTOR_Y_STEP_PIN, HIGH);
    }

    delayMicroseconds(STEP_SPEED);
    digitalWrite(MOTOR_X_STEP_PIN, LOW);
    digitalWrite(MOTOR_Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_SPEED);
  } while (xMove || yMove);

  // turn off motors
  digitalWrite(MOTOR_XY_ENABLE_PIN, HIGH);
}

/* //////////////// Tasks //////////////// */

static void commsTask(void *pvParameters)
{
  Serial.print(F("Info: Starting messaging task on core "));
  Serial.println(xPortGetCoreID());

  // initialize wifi module
  bool hasWifi = false;
  do
  {
    hasWifi = initWifi(WIFI_TIMEOUT_MS);
  } while (!hasWifi);

  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "ENEE101-ESP32");
  Esp32MQTTClient_Init((const uint8_t *) connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(sendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(messageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(deviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(deviceMethodCallback);

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
        double xDistance = getUltrasonicDistanceInInches(&xSensor);
        double yDistance = getUltrasonicDistanceInInches(&ySensor);
        xTaskResumeAll();

        // copy into message
        snprintf(messagePayload, MESSAGE_MAX_LEN, messageData, messageCount++, xDistance, yDistance);

        Serial.print(F("Info: Sending message to server "));
        Serial.println(messagePayload);

        EVENT_INSTANCE *message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
        Esp32MQTTClient_SendEventInstance(message);
      }

    }
    else 
    {
      // try to connect if connection is lost
      hasWifi = initWifi(WIFI_TIMEOUT_MS);
    }

    // check for new messages 
    if (WiFi.status() == WL_CONNECTED) 
    {
      Esp32MQTTClient_Check();
    }
    else
    {
      hasWifi = false;
    }
    
    vTaskDelay(100);
  }

}

static void motorTask(void *pvParameters)
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
    commandQueueLength = motorCommandQueue.count();
    xSemaphoreGive(motorCommandMutex);

    while (commandQueueLength > 0)
    {
      // new data is available, copy first command
      xSemaphoreTake(motorCommandMutex, portMAX_DELAY);
      command = motorCommandQueue.pop();
      commandQueueLength = motorCommandQueue.count();
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
          
          moveMotors(command.x, command.y);

          Serial.println(F("Info: Move finished"));
          break;
        }
        case MOTOR_RESET:
        {
          Serial.println(F("Info: Resetting motors"));
          resetMotors();
          Serial.println(F("Info: Reset finished"));
          break;
        }
        default:
        {
          // should not get here
          break;
        }
      }

      vTaskDelay(100);
    }

    currentMotorState = MOTOR_IDLE;
    vTaskDelay(100);
  }
}

/* //////////////// Interrupts //////////////// */

void IRAM_ATTR encoderX_A_interrupt() 
{
  cli();
  readXReg = GPIO_REG_READ(GPIO_IN_REG) &(ENCODER_X_A_REGISTER + ENCODER_X_B_REGISTER);
  if (readXReg == (ENCODER_X_A_REGISTER + ENCODER_X_B_REGISTER) && aXFlag)
  {
    encoderXPosCur++;
    bXFlag = 0;
    aXFlag = 0;
  }
  else if (readXReg == ENCODER_X_A_REGISTER)
    bXFlag = 1;
  sei();
}

void IRAM_ATTR encoderX_B_interrupt() 
{
  cli();
  readXReg = GPIO_REG_READ(GPIO_IN_REG) & (ENCODER_X_A_REGISTER + ENCODER_X_B_REGISTER);
  if (readXReg == (ENCODER_X_A_REGISTER + ENCODER_X_B_REGISTER) && bXFlag)
  {
    encoderXPosCur--;
    bXFlag = 0;
    aXFlag = 0;
  }
  else if (readXReg == ENCODER_X_B_REGISTER)
    aXFlag = 1;
  sei();
}

void IRAM_ATTR encoderY_A_interrupt()
{
  cli();
  readYReg = GPIO_REG_READ(GPIO_IN_REG) &(ENCODER_Y_A_REGISTER + ENCODER_Y_B_REGISTER);
  if (readYReg == (ENCODER_Y_A_REGISTER + ENCODER_Y_B_REGISTER) && aYFlag)
  {
    encoderYPosCur++;
    bYFlag = 0;
    aYFlag = 0;
  }
  else if (readYReg == ENCODER_Y_A_REGISTER)
    bYFlag = 1;
  sei();
}

void IRAM_ATTR encoderY_B_interrupt()
{
  cli();
  readYReg = GPIO_REG_READ(GPIO_IN_REG) & (ENCODER_Y_A_REGISTER + ENCODER_Y_B_REGISTER);
  if (readYReg == (ENCODER_Y_A_REGISTER + ENCODER_Y_B_REGISTER) && bYFlag)
  {
    encoderYPosCur--;
    bYFlag = 0;
    aYFlag = 0;
  }
  else if (readYReg == ENCODER_Y_B_REGISTER)
    aYFlag = 1;
  sei();
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
  pinMode(LIMIT_X_LOW_PIN, INPUT);
  pinMode(LIMIT_X_HIGH_PIN, INPUT);
  pinMode(LIMIT_Y_LOW_PIN, INPUT);
  pinMode(LIMIT_Y_HIGH_PIN, INPUT);
  pinMode(MOTOR_XY_ENABLE_PIN, OUTPUT);
  pinMode(ENCODER_X_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_X_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_Y_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_Y_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_X_A_PIN), encoderX_A_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_X_B_PIN), encoderX_B_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_A_PIN), encoderY_A_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_B_PIN), encoderY_B_interrupt, RISING);

  digitalWrite(ULTRASOUND_X_TRIG_PIN, LOW);
  digitalWrite(ULTRASOUND_Y_TRIG_PIN, LOW);
  digitalWrite(MOTOR_X_DIR_PIN, LOW);
  digitalWrite(MOTOR_X_STEP_PIN, LOW);
  digitalWrite(MOTOR_Y_DIR_PIN, LOW);
  digitalWrite(MOTOR_Y_STEP_PIN, LOW);
  digitalWrite(MOTOR_XY_ENABLE_PIN, HIGH);

  digitalWrite(ONBOARD_LED_PIN, ledValue);

  Serial.begin(115200);
  Serial.println(F("ESP32 Device"));
  Serial.println(F("Initializing..."));

  motorCommandMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    commsTask,
    "Comms Task",
    65536,
    NULL,
    1,
    &commsTaskHandler,
    0);

  delay(200);

  xTaskCreatePinnedToCore(
    motorTask,
    "Motor Task",
    16384,
    NULL,
    1,
    &motorTaskHandler,
    1);
}

void loop()
{
  vTaskSuspend(NULL);
}