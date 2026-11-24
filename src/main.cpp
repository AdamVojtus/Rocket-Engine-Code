#include <Arduino.h>
#include "control.cpp"
#include <HX711.h>
TaskHandle_t CriticalTask;
HX711 scale;

constexpr int TC_CLK_PIN = 5;
constexpr int TC_CS_PIN = 4;
constexpr int TC_DO_PIN = 2;
const int minThreshold = 5;    // gnd
const int maxThreshold = 1018; // vcc
const float V_MIN = 0.33f;          // Voltage at minimum pressure (V)
const float V_MAX = 2.97f;          // Voltage at maximum pressure (V)
const float P_MIN_PA = 0.0f;        // Minimum pressure in Pascals
const float P_MAX_PA = 1000000.0f;  // Maximum pressure in Pascals (10 bar = 1,00
const int initializedCounterMax = 5;
const int countOfStampsForConfirmation = 2;
unsigned long lastAttempt = 0;
unsigned long timePast = 0;
unsigned long timePastCombustion = 0;
unsigned long timeSinceIngnition = 0;
char incomingByte;
bool abortActivated = false;
bool thrustIsActive = true;
bool pressureGrowth = false;
bool combustionPressureGrwoth = false;
int initiliazedCounter = 0;

#define TXD1 19
#define RXD1 21
#define DOUT 16
#define CLK  17
#define PIN_BATTERY 13
#define VALVE1 27
#define VALVE2 14
#define IGNITER 12

enum FlightState {
    STATE_IDLE,
    STATE_ARMED,
    STATE_IGNITION,
    STATE_BURN,
    STATE_ABORT
};

enum PressureCombustionSensorStatus {
    SENSOR_OK_COMBUSTION_BAROMETER,
    SENSOR_DISCONNECTED_COMBUSTION_BAROMETER,
    SENSOR_OVERFLOW_COMBUSTION_BAROMETER,
};

enum ForceSensorStatus {
    SENSOR_OK_NEWTONMETER,
    SENSOR_UNFUNCTIONAL_NEWTONMETER,
};

enum PressureSensorStatus {
    SENSOR_OK_BAROMETER,
    SENSOR_DISCONNECTED_BAROMETER,
    SENSOR_OVERFLOW_BAROMETER,
};

enum CommunicationStatus {
    COMMUNICATION_LOST,
    COMMUNICATION_ESTABLISHED,
    COMMUNICATION_PENDING,
};

enum BatteryStatus {
    BATTERY_OK,
    BATTERY_LOW,
};

struct telemetryData 
  {
    float pressureRaw;
    float pressureVoltage;
    float pressure;
    float thrust;
    float combustionPressureRaw;
    float combustionPressureVoltage;
    float combustionPressure;
    float batteryVoltage;
    uint32_t rawVoltage;
    float batteryPercentage;
  };

telemetryData currentData;

FlightState currentState = STATE_IDLE;
CommunicationStatus currentStatus = COMMUNICATION_PENDING;

telemetryData sendTelemetry()
{
  currentData.pressureRaw = analogRead(20);
  currentData.pressureVoltage = (currentData.pressureRaw / 4095.0f) * 1000.0f;
  currentData.pressure = ((currentData.pressureVoltage - V_MIN) / (V_MAX - V_MIN)) * (P_MAX_PA - P_MIN_PA) + P_MIN_PA;
  if (thrustIsActive != false) currentData.thrust = scale.get_units(4);
  currentData.combustionPressureRaw = analogRead(21);
  currentData.combustionPressureVoltage = (currentData.combustionPressureRaw / 4095.0f) * 1000.0f;
  currentData.combustionPressure = ((currentData.combustionPressureVoltage - V_MIN) / (V_MAX - V_MIN)) * (P_MAX_PA - P_MIN_PA) + P_MIN_PA;
  currentData.rawVoltage = analogReadMilliVolts(PIN_BATTERY);
  currentData.batteryVoltage = (currentData.rawVoltage / 1000.0) * 1.27;
  currentData.batteryPercentage  = (currentData.batteryVoltage - 3.0) / (4.2 - 3.0) * 100.0;
  currentData.batteryPercentage = constrain(currentData.batteryPercentage, 0, 100);

  return currentData;
}

PressureCombustionSensorStatus combustionPressureSensorInitialization()
{
  telemetryData currentData = sendTelemetry();

  if (currentData.combustionPressureRaw < 200 ) return SENSOR_DISCONNECTED_COMBUSTION_BAROMETER;
  if (currentData.combustionPressureRaw > 4090) return SENSOR_OVERFLOW_COMBUSTION_BAROMETER;
  else 
  {
    initiliazedCounter++;
    return SENSOR_OK_COMBUSTION_BAROMETER;
  }
}

PressureSensorStatus pressureSensorInitialization()
{
  telemetryData currentData = sendTelemetry();

  if (currentData.pressureRaw < 200) return SENSOR_DISCONNECTED_BAROMETER;
  if (currentData.pressureRaw > 4090) return SENSOR_OVERFLOW_BAROMETER;
  else 
  {
    initiliazedCounter++;
    return SENSOR_OK_BAROMETER;
  }
}

ForceSensorStatus forceSensorStatusInitialization()
{
  telemetryData currentData = sendTelemetry();

  if (scale.is_ready())
  {
    initiliazedCounter++;
    return SENSOR_OK_NEWTONMETER;
  }
  else 
  {
    return SENSOR_UNFUNCTIONAL_NEWTONMETER;
  }
}

CommunicationStatus communicationChannelInitialization()
{
  Serial1.println('O');

  if (Serial1.available()) 
  {
    incomingByte = Serial.read();
    if (incomingByte == 'O')
    {
      initiliazedCounter++;
      return COMMUNICATION_ESTABLISHED;
    }
    else
    {
      if (millis() - lastAttempt >= 500) 
      {
        lastAttempt = millis();
        return COMMUNICATION_LOST;
      }

      lastAttempt = 0;
    }
  }
  else
  {
    if (millis() - lastAttempt >= 500) 
    {
      lastAttempt = millis();
      return COMMUNICATION_LOST;
    }

    lastAttempt = 0;
  }

  return COMMUNICATION_PENDING;
}

BatteryStatus batteryStatusInitialization()
{
  telemetryData currentData = sendTelemetry();

  if (currentData.batteryPercentage >= 80)
  {
    initiliazedCounter++;
    return BATTERY_OK;
  }
  else return BATTERY_LOW;
}

FlightState checkInitialization()
{
  if (initiliazedCounter >= initializedCounterMax)
  {
    currentState = STATE_ARMED;
    return STATE_ARMED;
  }

  return STATE_IDLE;
}


void openValve()
{
  digitalWrite(VALVE1, HIGH);
  digitalWrite(VALVE2, HIGH);
}

void closeValve()
{
  digitalWrite(VALVE1, LOW);
  digitalWrite(VALVE2, LOW);
}

void ignite()
{
  if (currentState == STATE_ARMED)
  {
    openValve();
    digitalWrite(IGNITER, HIGH);
  }
}

void stop()
{
  closeValve();
  digitalWrite(IGNITER, LOW);

   if (millis() - lastAttempt >= 500) {
        lastAttempt = millis();
        closeValve();
    }

    lastAttempt = 0;
  abortActivated = true;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  analogSetAttenuation(ADC_11db);
  scale.begin(DOUT, CLK);
  scale.set_scale(2280.f);
  scale.tare();

  xTaskCreatePinnedToCore(
        loopCritical,
        "CommsTask",
        10000,
        NULL,
        1,
        &CriticalTask,
        0
  );
}

void loopCritical(void * pvParameters) {
  for (;;) {
    button_state = digitalRead(START_PIN);
    button_state1 = digitalRead(STOP_PIN);
    if (button_state == LOW)
    {
      ignite();
    }
    else if (button_state1 == LOW)
    {
      stop();
    }

    currentData = sendTelemetry();
    if (millis() >= 50 && currentData.thrust <= 0)
    {
      thrustIsActive = false;
    }

    if (currentData.combustionPressure >= 11)
    {
      if (!combustionPressureGrwoth)
      {
        timePastCombustion = millis();
        combustionPressureGrwoth = true;
      }

      if (millis() - timePastCombustion >= 30 && currentData.combustionPressure >= 12.2)
      { 
        stop();
        combustionPressureGrwoth = false;
      }
    }
    else combustionPressureGrwoth = false;

    if (currentData.pressure >= 2.4)
    {
      if (!pressureGrowth)
      {
        timePast = millis();
        pressureGrowth = true;
      }

      if (millis() - timePast >= 30 && currentData.pressure >= 2.4)
      {
        stop();
        pressureGrowth = false;
      }
    }

    incomingByte = Serial1.read();
    if (incomingByte != 'O' || incomingByte != -1)
    {
      if (millis() - lastAttempt >= 500) 
      {
        lastAttempt = millis();
        stop();
      }
    }

    timeSinceIngnition = millis();
    if (millis() - timeSinceIngnition == 30000)
    {
      stop();
    }

    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}

void loop() {
  currentData = sendTelemetry();
  button_state = digitalRead(START_PIN);
  if (button_state == LOW)
  {
    currentState = STATE_IGNITION;
    currentState = STATE_BURN;
  }

  if (abortActivated == true)
  {
    currentState = STATE_ABORT;
  }

  if (incomingByte != 'O' || incomingByte != -1)
  {
    if (millis() - lastAttempt >= 500) 
    {
      lastAttempt = millis();
      currentStatus = COMMUNICATION_LOST;
    }
  }
  else currentStatus = COMMUNICATION_ESTABLISHED;

  vTaskDelay(10 / portTICK_PERIOD_MS);
}

