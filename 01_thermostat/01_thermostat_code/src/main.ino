

#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SEALEVELPRESSURE_HPA (1050)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77
#define SSD1306_ADDR_1 0x3C
#define SSD1306_ADDR_2 0x3D

bool bmp280_detected = false;
bool ssd1306_detected = false;

Adafruit_BMP280 bmp;
float temperature, humidity, pressure, altitude;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
char Stringtosend[7];
  String data_send;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      deviceConnected = true;
    };
  
    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
    }
};
  
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue =(String) pCharacteristic->getValue().c_str();
  
        if (rxValue.length() > 0) {
            Serial.println("*********");
            Serial.print("Received Value: ");
            for (int i = 0; i < rxValue.length(); i++) {
                Serial.print(rxValue[i]);
            }

            Serial.println();
            Serial.println("*********");
        }
    }
};
  
void oled_startup() {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, 16);
    display.println("ESP32");

    display.setTextSize(1);
    display.setCursor(34, 40);
    display.println("THERMOSTAT");

    display.display();

    delay(3000);

    display.clearDisplay();
    display.display();
}

void sendBLEData(const String& data) {
    size_t len = data.length();
    size_t maxChunkSize = 20;
    for (size_t i = 0; i < len; i += maxChunkSize) {
        String chunk = data.substring(i, min(i + maxChunkSize, len));
        pTxCharacteristic->setValue(chunk.c_str());
        pTxCharacteristic->notify();
        delay(50);
    }
}

const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

const char* mqtt_server = "SERVER_SSID";

WiFiClient espClient;
PubSubClient client(espClient);

float targetTemperature = 25.0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
  
    Serial.print("Message: ");
    String message;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    Serial.println(message);
 
    if (String(topic) == "thermostat/target") {
      targetTemperature = message.toFloat();
      Serial.print("Updated target temperature: ");
      Serial.println(targetTemperature);
    }
}

void setup_wifi() {
    delay(10);
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("Wi-Fi connected!");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
  
  void reconnect() {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP32_Thermostat")) {
        Serial.println("connected");
        client.subscribe("thermostat/target");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
}

void handleThermostat() {
    float currentTemperature = bmp.readTemperature();
    currentTemperature = currentTemperature + random(-10, 10) * 0.1;
  
    String tempPayload = String(currentTemperature);
    client.publish("thermostat/current", tempPayload.c_str());
    Serial.println("Published current temperature: " + tempPayload);

    if (currentTemperature < targetTemperature) {
      client.publish("thermostat/status", "Heating");
      Serial.println("HVAC Status: Heating");
    } else if (currentTemperature > targetTemperature) {
      client.publish("thermostat/status", "Cooling");
      Serial.println("HVAC Status: Cooling");
    } else {
      client.publish("thermostat/status", "Idle");
      Serial.println("HVAC Status: Idle");
    }
  
    delay(5000); 
  }
  
  
void setup() {
    Serial.begin(115200);
    Wire.begin(1,2);
    delay(100);

    bmp.begin(0x76);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    delay(3000);

    oled_startup();

    BLEDevice::init("ESP32 Thermostat");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");

    client.setServer(mqtt_server, 1883);
    client.setCallback(mqtt_callback);
}

void loop() {

    char Stringtosend[10];
    String data_send;

    bmp280_detected = false;
    ssd1306_detected = false;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            if (address == BMP280_ADDR_1 || address == BMP280_ADDR_2) {
                bmp280_detected = true;

                float temperature = bmp.readTemperature();
                float pressure = bmp.readPressure() / 100.0F;
                float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.setCursor(0, 0);
                display.print("Temp: ");
                display.print(temperature);
                display.println(" *C");
                display.print("Pressure: ");
                display.print(pressure);
                display.println(" hPa");
                display.print("Altitude: ");
                display.print(altitude);
                display.println(" m");
                display.display();

                Serial.println("----------------------------------");
                Serial.print("Temp: ");
                Serial.print(temperature);
                Serial.println(" *C");
                Serial.print("Pressure: ");
                Serial.print(pressure);
                Serial.println(" hPa");
                Serial.print("Altitude: ");
                Serial.print(altitude);
                Serial.println(" m");
                Serial.println("----------------------------------");

                dtostrf(temperature, 4, 2, Stringtosend); 
                data_send = "Temperature = ";
                data_send += Stringtosend;
                data_send += " C\r\n"; 
    
                dtostrf(pressure, 6, 2, Stringtosend); 
                data_send += "Pressure = ";
                data_send += Stringtosend;
                data_send += " hPa\r\n";
    
                dtostrf(altitude, 6, 2, Stringtosend);
                data_send += "Altitude = ";
                data_send += Stringtosend;
                data_send += " m\r\n\n";

                delay(1000);
            } 
            else if (address == SSD1306_ADDR_1 || address == SSD1306_ADDR_2) {
                ssd1306_detected = true;
            }
        }
    }

    if (!bmp280_detected) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);

        display.println("BMP280 not detected.");
        display.display();

        Serial.println("----------------------------------");
        Serial.println("BMP280 not detected.");
        Serial.println("----------------------------------");
    }

    if (!ssd1306_detected) {
        Serial.println("----------------------------------");
        Serial.println("SSD1306 not detected.");
        Serial.println("----------------------------------");
    }

    
    if (isnan(temperature) || isnan(pressure) || isnan(altitude)) {  
        Serial.println("Failed to read sensor data!"); 
        return;
    }

    if (deviceConnected) {
        sendBLEData(data_send);
    }

    Serial.println(data_send);
    
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }    

    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    handleThermostat();
}

