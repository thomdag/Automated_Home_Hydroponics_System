#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_PH.h"
#include "RTClib.h"
#include <TimeLib.h>

// Define core IDs for each task
#define CORE_TASK1 0
#define CORE_TASK2 1

// Pin Definitions
#define PH_SENSOR_PIN A0
#define TDS_SENSOR_PIN A1
#define TEMP_SENSOR_PIN A2

#define NUTRIENT_RELAY_PIN 7
#define PH_UP_RELAY_PIN 6
#define PH_DOWN_RELAY_PIN 5
#define WATER_RELAY_PIN 4
#define LIGHT_PIN 3

// Constants for sensor calibration and configuration
const float PH_UPPER_LIMIT = 6.0;
const float PH_LOWER_LIMIT = 5.5;
const int TDS_UPPER_LIMIT = 840;
const int TDS_LOWER_LIMIT = 560;
const int TDS_VARIANCE = 150;
const float PH_VARIANCE = 1.0;
const int LIGHT_START = 9;
const int LIGHT_END = 15;
const int MIX_TIME = 60;


// Sensor objects
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
DFRobot_PH phSensor;

// RTC Object. 
RTC_DS3231 realTimeClock; // Esp32 has a RTC however lacks a battery.
DateTime lastModification;


// Task handles
TaskHandle_t readSensorTaskHandle;
TaskHandle_t printSensorTaskHandle;
TaskHandle_t controlLightsTaskHandle;
TaskHandle_t modifyWaterLevelsTaskHandle;
TaskHandle_t modifyPHLevelTaskHandle;

// Shared variables for sensor values
float temperature;
float temperatureDelayed;
int tdsValue;
int tdsValueDelayed;
float phValue;
float phValueDelayed;

SemaphoreHandle_t mutexSharedVars;
SemaphoreHandle_t mutexDelayedVars;

// Task functions
void readSensorTask(void* pvParameters);
void printSensorTask(void* pvParameters);
void controlLightsTask(void* pvParameters);
void modifyWaterLevelsTask(void* pvParameters);
void modifyPHLevelTask(void* pvParameters);

void setup() {
    // Set pin modes
    pinMode(NUTRIENT_RELAY_PIN, OUTPUT);
    pinMode(PH_UP_RELAY_PIN, OUTPUT);
    pinMode(PH_DOWN_RELAY_PIN, OUTPUT);
    pinMode(WATER_RELAY_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);

    pinMode(PH_SENSOR_PIN, INPUT);
    pinMode(TDS_SENSOR_PIN, INPUT);
    pinMode(TEMP_SENSOR_PIN, INPUT);

    // Initialize sensors and communication
    Wire.begin();
    tempSensor.begin();
    phSensor.begin();

    // Create tasks
    mutexSharedVars = xSemaphoreCreateMutex();
    mutexDelayedVars = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(readSensorTask, "ReadSensorTask", 10000, NULL, 1, &readSensorTaskHandle, CORE_TASK1);
    xTaskCreatePinnedToCore(printSensorTask, "PrintSensorTask", 10000, NULL, 1, &printSensorTaskHandle, CORE_TASK1);
    xTaskCreatePinnedToCore(controlLightsTask, "ControlLightsTask", 10000, NULL, 1, &controlLightsTaskHandle, CORE_TASK1);
    xTaskCreatePinnedToCore(modifyWaterLevelsTask, "ModifyWaterLevelsTask", 10000, NULL, 1, &modifyWaterLevelsTaskHandle, CORE_TASK1);
    xTaskCreatePinnedToCore(modifyPHLevelTask, "ModifyPHLevelTask", 10000, NULL, 1, &modifyPHLevelTaskHandle, CORE_TASK1);

    lastModification = realTimeClock.now();

    Serial.begin(9600);
    Serial.flush();

}

void loop() {
    //Add small screen here in future

}

void readSensorTask() {
    while (1) {
        // Read temperature sensor
        tempSensor.requestTemperaturesByIndex(0);
        float temp = tempSensor.getTempCByIndex(0);

        // Read TDS sensor
        int tdsRawValue = analogRead(TDS_SENSOR_PIN);
        int tds = map(tdsRawValue, 0, 1023, 0, 1000);  // Assuming a 0-1000 TDS range, adjust as needed

        // Read pH sensor
        float voltage = analogRead(PH_SENSOR_PIN) / 4096.0 * 3300.0;  // Assuming a 0-5V input range, adjust as needed
        float ph = phSensor.readPH(voltage, temp);

        // Lock the mutex only during updating shared variables
        xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
        temperature = temp;
        tdsValue = tds;
        phValue = ph;
        xSemaphoreGive(mutexSharedVars);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms (1 second)
    }
}

// Task function to print sensor values
void printSensorTask() {
    while (1) {
        DateTime currentTime = realTimeClock.now();
        unsigned long elapsedSeconds = currentTime.unixtime() - lastModification.unixtime();
        float temp;
        int tds;
        float ph;
        if (Serial.available() > 0 && elapsedSeconds > MIX_TIME) {
            xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
            temp = temperature;
            tds = tdsValue;
            ph = phValue;
            xSemaphoreGive(mutexSharedVars);

        }
        else if ((Serial.available() > 0 && elapsedSeconds < MIX_TIME)) {
            // Lock the mutex only during accessing shared variables
            xSemaphoreTake(mutexDelayedVars, portMAX_DELAY);
            temp = temperatureDelayed;
            tds = tdsValueDelayed;
            ph = phValueDelayed;
            xSemaphoreGive(mutexDelayedVars);
        }
        else {
            return;
        }
        // Print sensor values
        Serial.print("Temperature: ");
        Serial.println(temp);
        Serial.print("TDS Value: ");
        Serial.println(tds);
        Serial.print("pH Value: ");
        Serial.println(ph);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms (1 second)
    }
}

// Rest of the task functions remain the same as in the previous code...


void controlLightsTask(void* pvParameters) {
    while (1) {
        // Control lights based on current time
        DateTime now = realTimeClock.now();
        int currentHour = now.hour();

        if (currentHour >= 9 && currentHour <= 15) {
            digitalWrite(LIGHT_PIN, HIGH);
        }
        else {
            digitalWrite(LIGHT_PIN, LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms (1 second)
    }
}

void modifyWaterLevelsTask(void* pvParameters) {
    while (1) {
        // Modify water levels based on TDS value
        if (tdsValue - TDS_VARIANCE > TDS_UPPER_LIMIT) {
            setDelayedVariables();
            digitalWrite(WATER_RELAY_PIN, HIGH);
            digitalWrite(WATER_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms (60 seconds)
        }
        else if (tdsValue + TDS_VARIANCE < TDS_LOWER_LIMIT) {
            setDelayedVariables();
            digitalWrite(NUTRIENT_RELAY_PIN, HIGH);
            digitalWrite(NUTRIENT_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms (60 seconds)
        }
        else {
            digitalWrite(WATER_RELAY_PIN, LOW);
            digitalWrite(NUTRIENT_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(6000)); // Delay for 6000ms (6 seconds)
        }

    }
}

void modifyPHLevelTask(void* pvParameters) {
    while (1) {
        // Modify pH level based on pH value
        if (phValue + PH_VARIANCE > PH_UPPER_LIMIT) {
            setDelayedVariables();
            digitalWrite(PH_DOWN_RELAY_PIN, HIGH);
            digitalWrite(PH_DOWN_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms (60 seconds)
        }
        else if (phValue - PH_VARIANCE < PH_LOWER_LIMIT) {
            setDelayedVariables();
            digitalWrite(PH_UP_RELAY_PIN, HIGH);
            digitalWrite(PH_DOWN_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms (60 seconds)
        }
        else {
            digitalWrite(PH_DOWN_RELAY_PIN, LOW);
            digitalWrite(PH_UP_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(6000)); // Delay for 6000ms (6 seconds)
        }

    }
}

void setDelayedVariables() {
    xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
    xSemaphoreTake(mutexDelayedVars, portMAX_DELAY);
    temperatureDelayed = temperature;
    tdsValueDelayed = tdsValue;
    phValueDelayed = phValue;
    xSemaphoreGive(mutexDelayedVars);
    xSemaphoreGive(mutexSharedVars);
}

float readTDSTask() {
    const int numReadings = 10;
    const float conversionFactor = 3.3 / 4096;
    const float temperatureCoefficient = 0.02;

    float analogTotal = 0.0;

    // Read TDS sensor
    for (int i = 0; i < numReadings; i++) {
        analogTotal += analogRead(TdsSensorPin);
        delay(20); // Consider using an alternative delay for non-ESP32 platforms. 
    }

    float averageReading = analogTotal / numReadings;
    float averageVoltage = averageReading * conversionFactor;

    // Temperature compensation
    float compensation = 1.0 + temperatureCoefficient * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensation;

    // TDS calculation
    float tdsReading = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

    return tdsReading;
}

float readPHTask() {
    const int numReadings = 10;
    const float conversionFactor = 3.3 / 4096;
    const float temperatureCoefficient = 0.02;

    float analogTotal = 0.0;

    for (int i = 0; i < numReadings; i++) {
        analogTotal += analogRead(PH_SENSOR_PIN);
        delay(20); // Consider using an alternative delay for non-ESP32 platforms. 
    }
    // PH calculation
    float averageReading = analogTotal / numReadings;
    float pHReading = averageReading * conversionFactor;

    
    return  pHReading;



}

