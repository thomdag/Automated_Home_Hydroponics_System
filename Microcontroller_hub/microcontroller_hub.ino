#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_PH.h"
#include "RTClib.h"
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>

// Define core IDs for each task
#define CORE_TASK1 0
#define CORE_TASK2 1

// PIN DEFINITIONS
// Sensor pins
#define PH_SENSOR_PIN 27
#define TDS_SENSOR_PIN 26
#define TEMP_SENSOR_PIN 25

// Mosfet pins
#define PH_SENSOR_FET 19
#define TDS_SENSOR_FET 18
#define TEMP_SENSOR_FET 17

// Control Pins
#define NUTRIENT_RELAY_PIN 23
#define WATER_RELAY_PIN 33
#define PH_UP_RELAY_PIN 22
#define PH_DOWN_RELAY_PIN 32
#define LIGHT_PIN 21

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

// RTC object. 
RTC_DS3231 realTimeClock; // Esp32 has a RTC however lacks a battery.
DateTime lastModification;

// LCD 16x2 display
LiquidCrystal_I2C lcd(0x3F, 16, 2);

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
    digitalWrite(PH_SENSOR_PIN, LOW);
    digitalWrite(TDS_SENSOR_FET, LOW);
    digitalWrite(TEMP_SENSOR_PIN, LOW);

    pinMode(PH_SENSOR_FET, OUTPUT);
    pinMode(TDS_SENSOR_FET, OUTPUT);
    pinMode(TEMP_SENSOR_FET, OUTPUT);


    // Initialise sensors and communication
    Wire.begin();
    tempSensor.begin();
    phSensor.begin();

    // Lcd initialise
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Starting")

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

void readSensorTask(void* pvParameters) {
    while (1) {
        // Read temperature sensor
        digitalWrite(TEMP_SENSOR_FET, HIGH);
        delay(20);
        tempSensor.requestTemperaturesByIndex(0);
        float temp = tempSensor.getTempCByIndex(0);
        digitalWrite(TEMP_SENSOR_FET, LOW);
        delay(20);

        // Read pH sensor
        digitalWrite(PH_SENSOR_FET, HIGH);
        delay(20);
        float voltage = analogRead(PH_SENSOR_PIN) / 4096.0 * 3300.0;  // 
        float ph = phSensor.readPH(voltage, temp);
        digitalWrite(PH_SENSOR_FET, LOW);
        delay(20);

        // Read TDS sensor
        digitalWrite(TDS_SENSOR_FET, HIGH);
        delay(20);
        float tds = readTDSTask(temp);
        digitalWrite(TDS_SENSOR_FET, LOW);
        delay(20);

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
void printSensorTask(void* pvParameters) {
    while (1) {
        xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
        String printTemp = temperature;
        String printTDS = tdsValue;
        String printPH = phValue;
        xSemaphoreGive(mutexSharedVars);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TMP:");
        lcd.print(printTemp);
        lcd.print("TDS:");
        lcd.print(printTDS);
        lcd.setCursor(0, 1);
        lcd.print("PH:");
        lcd.print(printPH);

        DateTime currentTime = realTimeClock.now();
        unsigned long elapsedSeconds = currentTime.unixtime() - lastModification.unixtime();
        if (Serial.available() > 0 && elapsedSeconds > MIX_TIME) {
            sprintf(buffer, "///*%s*%s*%s*%i///", printTemp, PrintTDS, printPH, 0);
            Serial.println(buffer);
        }
        else if ((Serial.available() > 0 && elapsedSeconds < MIX_TIME)) {
            // Lock the mutex only during accessing shared variables
            xSemaphoreTake(mutexDelayedVars, portMAX_DELAY);
            String printTempDelayed = temperatureDelayed;
            String printTDSDelayed = tdsValueDelayed;
            String printPHDelayed = phValueDelayed;
            xSemaphoreGive(mutexDelayedVars);
            sprintf(buffer, "///*%s*%s*%s*%i///", printTempDelayed, PrintTDSDelayed, printPHDelayed, 1);
            Serial.println(buffer);
        }
        else {
            return;
        }

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
    //taking semaphores
    xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
    xSemaphoreTake(mutexDelayedVars, portMAX_DELAY);
    temperatureDelayed = temperature;
    tdsValueDelayed = tdsValue;
    phValueDelayed = phValue;
    xSemaphoreGive(mutexDelayedVars);
    xSemaphoreGive(mutexSharedVars);
}

float readTDSTask(float givenTemp) {
    const int numReadings = 10;
    const float conversionFactor = 3.3 / 4096;
    const float temperatureCoefficient = 0.02;

    float analogTotal = 0.0;

    // Read TDS sensor
    for (int i = 0; i < numReadings; i++) {
        analogTotal += analogRead(TDS_SENSOR_PIN);
        delay(20); // Consider using an alternative delay for non-ESP32 platforms. 
    }

    float averageReading = analogTotal / numReadings;
    float averageVoltage = averageReading * conversionFactor;

    // Temperature compensation
    float compensation = 1.0 + temperatureCoefficient * (givenTemp - 25.0);
    float compensationVoltage = averageVoltage / compensation;

    // TDS calculation
    float tdsReading = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

    return tdsReading;
}

