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
// Sensor pins connections
#define PH_SENSOR_PIN 27
#define TDS_SENSOR_PIN 26
#define TEMP_SENSOR_PIN 25

// Mosfet pins connections
#define PH_SENSOR_FET 19
#define TDS_SENSOR_FET 18
#define TEMP_SENSOR_FET 17

// Pins to control relays.
#define NUTRIENT_RELAY_PIN 23
#define WATER_RELAY_PIN 33
#define PH_UP_RELAY_PIN 22
#define PH_DOWN_RELAY_PIN 32
#define LIGHT_PIN 21

// Switch pins and LED
#define PUMP_PIN 10
#define LIGHT_MANUAL_ON 11
#define LIGHT_AUTO_ON 12
#define DEBOUNCE_DELAY 50
#define LED_PIN 13

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

char LIGHT_STATE = "OFF";
unsigned long lightManualLastDebounceTime = 0;
unsigned long lightAutoLastDebounceTime = 0;
unsigned long pumpLastDebounceTime = 0;
int lightManualLastState = HIGH;
int lightAutoLastState = HIGH;
int pumpLastState = HIGH;


// Shared variables for sensor values
float temperature;
float tdsValue;
float phValue;

// Sensor objects
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
DFRobot_PH phSensor;

// RTC object. 
RTC_DS3231 realTimeClock; // Esp32 has a RTC however lacks a battery.
DateTime lastModification;
DateTime lastButtonPress;

// LCD 16x2 display
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Task handles
TaskHandle_t readSensorTaskHandle = NULL;
TaskHandle_t printSensorTaskHandle = NULL;
TaskHandle_t controlLightsTaskHandle = NULL;
TaskHandle_t modifyWaterLevelsTaskHandle = NULL;
TaskHandle_t modifyPHLevelTaskHandle = NULL;
TaskHandle_t buttonPollTaskHandle = NULL;

// Mutex handles.
SemaphoreHandle_t mutexSharedVars;
SemaphoreHandle_t mutexLastMixTime;
SemaphoreHandle_t mutexRTCAccess;
SemaphoreHandle_t mutexLightControl;

// Task functions prototypes
void readSensorTask(void* pvParameters);
void printSensorTask(void* pvParameters);
void controlLightsTask(void* pvParameters);
void modifyMediumQualityTask(void* pvParameters);
void buttonPoll(void* pvParameters);
void writePinHighLow(int pin, int timeDelay);
float readTDSSensor(float givenTemp);


void setup() {
    // Lcd initialise
    if (!lcd.begin()) {
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Starting");

    lcd.clear();
    lcd.print("Starting pins");
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
    digitalWrite(TDS_SENSOR_PIN, LOW);
    digitalWrite(TEMP_SENSOR_PIN, LOW);

    pinMode(PH_SENSOR_FET, OUTPUT);
    pinMode(TDS_SENSOR_FET, OUTPUT);
    pinMode(TEMP_SENSOR_FET, OUTPUT);
    digitalWrite(PH_SENSOR_FET, LOW);
    digitalWrite(TDS_SENSOR_FET, LOW);
    digitalWrite(TEMP_SENSOR_FET, LOW);

    pinMode(LED_PIN, OUTPUT)
    pinMode(PUMP_PIN, INPUT_PULLUP);
    pinMode(LIGHT_MANUAL_ON, INPUT_PULLUP);
    pinMode(LIGHT_AUTO_ON, INPUT_PULLUP);

    lcd.clear();
    lcd.print("Starting sensors")
    // Initialise sensors and communication
    if (!Wire.begin()) {
        lcd.clear();
        lcd.println("Wire Error");
        while (1) delay(10);
    }
    if (!tempSensor.begin()) {
        lcd.clear();
        lcd.println("Temp Error");
        while (1) delay(10);
    }
    if (!phSensor.begin()) {
        lcd.clear();
        lcd.println("PH Error");
        while (1) delay(10);
    }
    if (!realTimeClock.begin()) {
        lcd.clear();
        lcd.println("RTC Error");
        while (1) delay(10);
    }
    lcd.clear();
    lcd.print("Starting tasks")
    // Create tasks
    mutexSharedVars = xSemaphoreCreateMutex();
    mutexLastMixTime = xSemaphoreCreateMutex();
    mutexRTCAccess = xSemaphoreCreateMutex();
    lightControlState = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(readSensorTask, "ReadSensorTask", 1024, NULL, 1, &readSensorTaskHandle, CORE_TASK2);
    xTaskCreatePinnedToCore(printSensorTask, "PrintSensorTask", 1024, NULL, 1, &printSensorTaskHandle, CORE_TASK2);
    xTaskCreatePinnedToCore(controlLightsTask, "ControlLightsTask", 1024, NULL, 1, &controlLightsTaskHandle, CORE_TASK1);
    xTaskCreatePinnedToCore(modifyMediumQualityTask, "modifyMediumQualityTask", 1024, NULL, 2, &modifyPHLevelTaskHandle, CORE_TASK1);
    xTaskCreate(buttonPoll,"ButtonPollTask", 1024,NULL,2,&buttonPollTaskHandle);

    lastModification = getTime();

    lcd.clear();
    lcd.print("Starting Serial")
    if (! Serial.begin(9600)){
        lcd.clear();
        lcd.print("Serial not connected")
    }
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(100));
}

void loop() {

}

void readSensorTask(void* pvParameters) {
    while (1) {
        // Read temperature sensor
        digitalWrite(TEMP_SENSOR_FET, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20));
        tempSensor.requestTemperaturesByIndex(0);
        float temp = tempSensor.getTempCByIndex(0);
        digitalWrite(TEMP_SENSOR_FET, LOW);

        // Read pH sensor
        digitalWrite(PH_SENSOR_FET, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20));
        float voltage = analogRead(PH_SENSOR_PIN) / 4096.0 * 3300.0;  // 
        float ph = phSensor.readPH(voltage, temp);
        digitalWrite(PH_SENSOR_FET, LOW);

        // Read TDS sensor
        digitalWrite(TDS_SENSOR_FET, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20));
        float tds = readTDSSensor(temp);
        digitalWrite(TDS_SENSOR_FET, LOW);

        // Lock the mutex only during updating shared variables
        xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
        temperature = temp;
        tdsValue = tds;
        phValue = ph;
        xSemaphoreGive(mutexSharedVars);
        xTaskNotifyGive(printSensorTaskHandle);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms (0.5 second)
    }
}

// Task function to print sensor values
void printSensorTask(void* pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
        float printTemp = temperature;
        float printTDS = tdsValue;
        float printPH = phValue;
        xSemaphoreGive(mutexSharedVars);
        lcd.clear(); // set lcd. Updates constantly to display real time
        lcd.setCursor(0, 0);
        lcd.print("TMP:");
        lcd.print(printTemp);
        lcd.print("TDS:");
        lcd.print(printTDS);
        lcd.setCursor(0, 1);
        lcd.print("PH:");
        lcd.print(printPH);
        // Send to external source. Use delayed variables if recently modified to avoid sending many false reports.
        char buffer[50];
        xSemaphoreTake(mutexLastMixTime, portMAX_DELAY);
        float lastModificationTemp = lastModification;
        xSemaphoreGive(mutexLastMixTime);
        DateTime currentTime = getTime();
        unsigned long elapsedSeconds = currentTime.unixtime() - lastModificationTemp.unixtime();
        if (Serial.available() > 0 && elapsedSeconds > MIX_TIME) {
            sprintf(buffer, "///*%.2f*%.2f*%.2f*%i///", printTemp, printTDS, printPH, 0);
            Serial.println(buffer);
        }
        else if ((Serial.available() > 0 && elapsedSeconds < MIX_TIME)) {
            // Lock the mutex only during accessing shared variables
            sprintf(buffer, "///*%.2f*%.2f*%.2f*%i///", printTemp, printTDS, printPH, 1);
            Serial.println(buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms (0.5 second)
    }
}
//Controls light function, 
void controlLightsTask(void* pvParameters) {
    while (1) {
        xSemaphoreTake(mutexLightControl, portMAX_DELAY);
        char tempLightControl = LIGHT_STATE;
        xSemaphoreGive(mutexLightControl);
        if (strcmp(tempLightControl, "AUTO") == 0) {
            // Control lights based on current time
            DateTime now = getTime();
            int currentHour = now.hour();

            if (currentHour >= LIGHT_START && currentHour <= LIGHT_END) {
                digitalWrite(LIGHT_PIN, HIGH);
            }
            else {
                digitalWrite(LIGHT_PIN, LOW);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10000ms (10 second)
    }
}
//Modify medium quality. 
void modifyMediumQualityTask(void* pvParameters) {
    while (1) {
        xSemaphoreTake(mutexSharedVars, portMAX_DELAY);
        float temp = temperature;
        float tds = tdsValue;
        float ph = phValue;
        xSemaphoreGive(mutexSharedVars);

        if (tds - TDS_VARIANCE > TDS_UPPER_LIMIT) {
            writePinHighLow(WATER_RELAY_PIN, 3000);
            vTaskDelay(pdMS_TO_TICKS(60000)); //Delay for 60000ms(60 seconds)
            continue;
        }
        else if (tdsValue + TDS_VARIANCE < TDS_LOWER_LIMIT) {
            writePinHighLow(NUTRIENT_RELAY_PIN, 3000);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms (60 seconds)
            continue;
        }
        else {
            digitalWrite(WATER_RELAY_PIN, LOW);
            digitalWrite(NUTRIENT_RELAY_PIN, LOW);
        }
        //Modifying water nutrients can modify ph. Allow time to mix
        if (ph + PH_VARIANCE > PH_UPPER_LIMIT) {
            writePinHighLow(PH_DOWN_RELAY_PIN, 3000);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms(60 seconds)
            continue;

        }
        else if (ph - PH_VARIANCE < PH_LOWER_LIMIT) {
            writePinHighLow(PH_UP_RELAY_PIN, 3000);
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay for 60000ms(60 seconds)
            continue;
        }
        else {
            digitalWrite(PH_DOWN_RELAY_PIN, LOW);
            digitalWrite(PH_UP_RELAY_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(6000)); // Delay for 6000ms (6 seconds)
        }
    }
}
//Move pin to high, wait delay then set pin low.
void writePinHighLow(int pin, int timeDelay) {
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(timeDelay));
    digitalWrite(pin, LOW);
    xSemaphoreTake(mutexLastMixTime, portMAX_DELAY);
    lastModification = getTime();
    xSemaphoreGive(mutexLastMixTime);
}

float readTDSSensor(float givenTemp) {
    const int numReadings = 10;
    const float conversionFactor = 3.3 / 4096;
    const float temperatureCoefficient = 0.02;

    float analogTotal = 0.0;

    // Read TDS sensor
    for (int i = 0; i < numReadings; i++) {
        analogTotal += analogRead(TDS_SENSOR_PIN);
        ets_delay_us(100); // Consider using an alternative delay for non-ESP32 platforms. 
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

DateTime getTime() {
    xSemaphoreTake(mutexRTCAccess, portMAX_DELAY);
    DateTime tempTime = realTimeClock.now();
    xSemaphoreGive(mutexRTCAccess);
    return tempTime;
}
void buttonPoll(void* pvParameters) {
    while (1) {
        unsigned long currentTime = millis();

        int lightManualReading = digitalRead(LIGHT_MANUAL_ON);
        int lightAutoReading = digitalRead(LIGHT_AUTO_ON);
        int pumpReading = digitalRead(PUMP_OVERRIDE_PIN);

        // Debounce lightManual button
        if (lightManualReading != lightManualLastState) {
            lightManualLastDebounceTime = currentTime;
        }
        if ((currentTime - lightManualLastDebounceTime) > DEBOUNCE_DELAY) {
            if (lightManualReading == LOW) {
                digitalWrite(LIGHT_PIN, HIGH);
                xSemaphoreTake(mutexRTCAccess, portMAX_DELAY);
                LIGHT_STATE = "MANUAL";
                xSemaphoreGive(mutexRTCAccess);
            }
        }
        lightManualLastState = lightManualReading;

        // Debounce lightAuto button
        if (lightAutoReading != lightAutoLastState) {
            lightAutoLastDebounceTime = currentTime;
        }
        if ((currentTime - lightAutoLastDebounceTime) > DEBOUNCE_DELAY) {
            if (lightAutoReading == LOW) {
                xSemaphoreTake(mutexRTCAccess, portMAX_DELAY);
                LIGHT_STATE = "AUTO";
                xSemaphoreGive(mutexRTCAccess);
            }
        }
        lightAutoLastState = lightAutoReading;

        // Debounce pump button
        if (pumpReading != pumpLastState) {
            pumpLastDebounceTime = currentTime;
        }
        if ((currentTime - pumpLastDebounceTime) > DEBOUNCE_DELAY) {
            if (pumpReading == LOW) {
                digitalWrite(PUMP_PIN, HIGH);
            }
            else {
                digitalWrite(PUMP_PIN, LOW);
            }
        }
        pumpLastState = pumpReading;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}