#include <ArduinoJson.h>
#include "MAX30105.h"
#include <WiFiMulti.h>
#include <WiFi.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include <cstdlib>


// Wifi Data //
char hostname[100];
char port[6];
char *portPtr;
uint16_t portFinal;

const uint16_t batchSizeOxi = 190; // 3 seconds of sample if each second -> 62.5 samples are gathered
const uint16_t batchSizeAcc = 100; // Maximum number of samples to store
static byte secPerReqAcc = 3;      // Number of seconds to wait before sending the accelerometer

const byte ledBrightness = 0x26;
const byte sampleAverage = 16; // Options: 1, 2, 4, 8, 16, 32
const byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
const int sampleRate = 1000;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
const int pulseWidth = 118;    // Options: 69, 118, 215, 411
const int adcRange = 16384;    // Options: 2048, 4096, 8192, 16384

#define INTERRUPT_PIN 15
#define WIFI_RESET_PIN 34
#define EEPROM_SIZE 200
#define EARTH_GRAVITY_MS2 9.80665 // m/s2
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// Useful Objects //
WiFiManager wm;
MAX30105 particleSensor;
WiFiClient client;
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial ss(2);

// JSON Data Buffers //
static JsonDocument oxiJson; // JSON document to store the Oxi data
static JsonArray irBuffer = oxiJson["IR"].to<JsonArray>();
static JsonArray redBuffer = oxiJson["RED"].to<JsonArray>();
static JsonArray timeStamp = oxiJson["TIME"].to<JsonArray>();
JsonDocument accJson; // JSON document to store the Acc data
JsonObject accelData = accJson["accelData"].add<JsonObject>();
JsonArray accelDataX = accelData["accelDataX"].to<JsonArray>();
JsonArray accelDataY = accelData["accelDataY"].to<JsonArray>();
JsonArray accelDataZ = accelData["accelDataZ"].to<JsonArray>();
JsonObject gyroData = accJson["gyroData"].add<JsonObject>();
JsonArray gyroDataX = gyroData["gyroDataX"].to<JsonArray>();
JsonArray gyroDataY = gyroData["gyroDataY"].to<JsonArray>();
JsonArray gyroDataZ = gyroData["gyroDataZ"].to<JsonArray>();
JsonDocument gpsJson; // JSON document to store the GPS data

size_t sizeOxi;
size_t sizeAcc;
size_t sizeGps;

// WiFi Manager Vars //
bool shouldSaveConfig = false;
bool shouldResetWifi = false;

// MPU control/status vars //
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars //
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            Accel sensor measurements
VectorInt16 gg;      // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            world-frame accel sensor measurements

// String Data Buffers //
static char jsonPayloadOxi[5000];
static char jsonPayloadAcc[10000];
static char jsonPayloadGps[300];

// Temp Data Buffers //
static uint16_t oxiIndex = 0;
static uint16_t accIndex = 0;
static uint16_t accCount = 0;
unsigned long prevStamp = 0;
unsigned long currStamp = 0;
float AAX = 0;
float AAY = 0;
float AAZ = 0;
float AGX = 0;
float AGY = 0;
float AGZ = 0;

// Task Handles //
TaskHandle_t oxiHandle = NULL;
TaskHandle_t accHandle = NULL;
TaskHandle_t gpsHandle = NULL;
TaskHandle_t sendValsHandle = NULL;

// Mutexes & Multi Tasking Variables //
static SemaphoreHandle_t mutexAcc;
static SemaphoreHandle_t mutexOxi;
static SemaphoreHandle_t mutexGps;
static bool oxiNotified = false;
static bool accNotified = false;
static bool gpsNotified = false;

// Multi-Tasking Functions //
void taskReadOxi(void *parameters);
void taskReadAcc(void *parameters);
void taskReadGps(void *parameters);
void taskSendVals(void *parameters);

// Helper Function Declarations //
void sendVals(char *jsonPayload, size_t size, const char *route);
void setupWiFiManager();
static bool str_to_uint16(const char *str, uint16_t *res);

// Callbacks //
void saveConfigCallback()
{
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

// Interrupt Service Routines //
void ICACHE_RAM_ATTR dmpDataReady()
{
    vTaskNotifyGiveFromISR(accHandle, 0);
}

void ICACHE_RAM_ATTR uartDataRdy()
{
    vTaskNotifyGiveFromISR(gpsHandle, 0);
}

void ICACHE_RAM_ATTR resetWifi()
{
    shouldResetWifi = true;
}

void setup()
{
    WiFi.mode(WIFI_STA);
    Serial.begin(921600);
    Serial.println("Starting...");
    Wire.begin();
    Wire.setClock(400000);

    setupWiFiManager();
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(-3640);
    mpu.setYAccelOffset(-3472);
    mpu.setZAccelOffset(890);
    mpu.setXGyroOffset(117);
    mpu.setYGyroOffset(-25);
    mpu.setZGyroOffset(-33);

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings

    Serial.println("Initializing GPS");
    ss.onReceive(uartDataRdy, false);
    ss.begin(9600);

    oxiIndex = 0;
    accIndex = 0;
    prevStamp = millis();

    mutexAcc = xSemaphoreCreateMutex();
    mutexOxi = xSemaphoreCreateMutex();
    mutexGps = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(taskReadOxi,
                            "Oximeter Reader Task",
                            2048,
                            NULL,
                            4,
                            &oxiHandle,
                            1);
    xTaskCreatePinnedToCore(taskReadAcc,
                            "Accelerometer Reader Task",
                            2048,
                            NULL,
                            2,
                            &accHandle,
                            1);
    xTaskCreatePinnedToCore(taskReadGps,
                            "GPS Reader Task",
                            2048,
                            NULL,
                            1,
                            &gpsHandle,
                            0);
    xTaskCreatePinnedToCore(taskSendVals,
                            "Oximeter Sender Task",
                            2048,
                            NULL,
                            2,
                            &sendValsHandle,
                            0);
    vTaskDelete(NULL);
}

void loop() { vTaskDelete(NULL); }

void taskReadAcc(void *parameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!dmpReady)
            continue;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
            mpu.resetFIFO();
            Serial.println("FIFO overflow!");
        }
        else if (mpuIntStatus & 0x02)
        {
            while (fifoCount < packetSize)
                fifoCount = mpu.getFIFOCount();
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            if (accCount % secPerReqAcc == 0)
            {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGyro(&gg, fifoBuffer);
                mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
                mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);

                AAX = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
                AAY = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
                AAZ = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
                AGX = ggWorld.x * mpu.get_gyro_resolution() * RAD_TO_DEG;
                AGY = ggWorld.y * mpu.get_gyro_resolution() * RAD_TO_DEG;
                AGZ = ggWorld.z * mpu.get_gyro_resolution() * RAD_TO_DEG;

                if (xSemaphoreTake(mutexAcc, 0) == pdTRUE)
                {
                    accelDataX.add(String(AAX, 3));
                    accelDataY.add(String(AAY, 3));
                    accelDataZ.add(String(AAZ, 3));
                    gyroDataX.add(String(AGX, 3));
                    gyroDataY.add(String(AGY, 3));
                    gyroDataZ.add(String(AGZ, 3));
                    xSemaphoreGive(mutexAcc);
                    accIndex++;
                    if (accIndex >= batchSizeAcc)
                    {
                        if (xSemaphoreTake(mutexAcc, 0) == pdTRUE)
                        {
                            accNotified = true;
                            xSemaphoreGive(mutexAcc);
                        }
                        xTaskNotifyGive(sendValsHandle);
                        accIndex = 0;
                    }
                }
            }
            accCount++;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void taskReadOxi(void *parameters)
{
    for (;;)
    {
        // Check For Reset
        if (shouldResetWifi)
        {
            Serial.println("wifi reset");
            wm.resetSettings();
            Serial.println("wifi reset1");
            shouldResetWifi = false;
            Serial.println("wifi reset2");
            ESP.restart();
        }
        particleSensor.check();
        while (particleSensor.available())
        {
            if (xSemaphoreTake(mutexOxi, 0) == pdTRUE)
            {
                irBuffer.add<uint32_t>(particleSensor.getFIFOIR());
                redBuffer.add<uint32_t>(particleSensor.getFIFORed());
                currStamp = millis();
                timeStamp.add((currStamp - prevStamp) / 1000.0);
                xSemaphoreGive(mutexOxi);
                prevStamp = currStamp;
                oxiIndex++;
                if (oxiIndex >= batchSizeOxi)
                {
                    if (xSemaphoreTake(mutexOxi, 0) == pdTRUE)
                    {
                        oxiNotified = true;
                        xSemaphoreGive(mutexOxi);
                    }
                    xTaskNotifyGive(sendValsHandle);
                    oxiIndex = 0;
                }
            }
            particleSensor.nextSample();
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void taskReadGps(void *parameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (ss.available())
            gps.encode(ss.read());
        gpsJson["Latitude"] = (String(gps.location.lat(), 7));
        gpsJson["Longitude"] = (String(gps.location.lng(), 7));
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d", gps.date.month(), gps.date.day(), gps.date.year());
        gpsJson["Date"] = (sz);

        sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        gpsJson["Time"] = (sz);
        gpsJson["Altitude"] = (String(gps.altitude.meters(), 7));

        if (xSemaphoreTake(mutexGps, 0) == pdTRUE)
        {
            gpsNotified = true;
            xSemaphoreGive(mutexGps);
        }
        xTaskNotifyGive(sendValsHandle);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (millis() > 5000 && gps.charsProcessed() < 10)
            Serial.println(F("No GPS data received: check wiring"));
    }
}

void taskSendVals(void *parameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (accNotified == true && xSemaphoreTake(mutexAcc, 0) == pdTRUE)
        {
            if (accelDataX.size() > 0)
            {
                sizeAcc = measureJson(accJson);
                serializeJson(accJson, jsonPayloadAcc);
                sendVals(jsonPayloadAcc, sizeAcc, "/api/hw/acc");
                accelDataX.clear();
                accelDataY.clear();
                accelDataZ.clear();
                gyroDataX.clear();
                gyroDataY.clear();
                gyroDataZ.clear();
            }
            accNotified = false;
            xSemaphoreGive(mutexAcc);
        }
        if (oxiNotified == true && xSemaphoreTake(mutexOxi, 0) == pdTRUE)
        {
            if (irBuffer.size() > 0)
            {
                sizeOxi = measureJson(oxiJson);
                serializeJson(oxiJson, jsonPayloadOxi);
                sendVals(jsonPayloadOxi, sizeOxi, "/api/hw/oxi");
                irBuffer.clear();
                redBuffer.clear();
                timeStamp.clear();
            }
            oxiNotified = false;
            xSemaphoreGive(mutexOxi);
        }
        if (gpsNotified == true && xSemaphoreTake(mutexGps, 0) == pdTRUE)
        {
            if (gpsJson.size() > 0)
            {
                sizeGps = measureJson(gpsJson);
                serializeJson(gpsJson, jsonPayloadGps);
                sendVals(jsonPayloadGps, sizeGps, "/api/hw/gps");
                gpsJson.clear();
            }
            gpsNotified = false;
            xSemaphoreGive(mutexGps);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void sendVals(char *jsonPayload, size_t size, const char *route = "")
{
    if (!client.connect(hostname, portFinal))
    {
        Serial.println("Connection failed");
        return;
    }
    client.println("POST /" + String(route) + " HTTP/1.1");
    client.println("Host: " + String(hostname));
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(size);
    client.println();
    client.println(jsonPayload);
    client.stop();
}

void setupWiFiManager()
{
    attachInterrupt(digitalPinToInterrupt(WIFI_RESET_PIN), resetWifi, FALLING);
    EEPROM.begin(512);

    bool res;
    wm.setSaveConfigCallback(saveConfigCallback);
    wm.setConnectTimeout(20);
    WiFiManagerParameter customHostname("HostName", "Server Hostname", hostname, 100);
    WiFiManagerParameter customPort("Port", "Connection Port", port, 6);
    wm.addParameter(&customHostname);
    wm.addParameter(&customPort);
    res = wm.autoConnect("AutoConnectAP", "FYPpassword"); 

    if (!res)
    {
        Serial.println("Failed to connect");
    }
    else
    {
        Serial.println("connected...yeey :)");
        strcpy(hostname, customHostname.getValue());
        strcpy(port, customPort.getValue());
        if (shouldSaveConfig)
        {
            EEPROM.writeString(0, hostname);
            EEPROM.writeString(sizeof(hostname) + 1, port);
            EEPROM.commit();
            delay(100);
            Serial.println(EEPROM.readString(0) + ", " + EEPROM.readString(sizeof(hostname) + 1));
        }
        strcpy(hostname, EEPROM.readString(0).c_str());
        strcpy(port, EEPROM.readString(sizeof(hostname)+1).c_str());
    }
    long value = strtol(port, &portPtr, 10); // Convert character array to long integer
    portFinal = static_cast<uint16_t>(value); // Convert to int if necessary

}