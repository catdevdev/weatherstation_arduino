// including lora e22 library
#define DESTINATION_ADDL 3
#define ENABLE_RSSI true
#include "Arduino.h"
#include "LoRa_E22.h"
// including bmp180 library
#include <Wire.h>
#include <BMP180.h>
// including dht22 library
#include "DHTesp.h"
// including wifi library
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
// including json library
#include <ArduinoJson.h>

const char* ssid = "JOJO";
const char* password = "4311067vladvk";

BMP180 myBMP(BMP180_ULTRAHIGHRES);
DHTesp dht22;
const int rainSensorGPIOPin = A0;

HTTPClient http;
WiFiClient wifiClient;

#define PIN_RX 14   //D5 on the board (Connect this to the EBYTE TX pin)
#define PIN_TX 12   //D6 on the board (connect this to the EBYTE RX pin)
#define PIN_M0 0    //D1 on the board (connect this to the EBYTE M0 pin)
#define PIN_M1 2    //D2 on the board (connect this to the EBYTE M1 pin)
#define PIN_AX 16   //D0 on the board (connect this to the EBYTE AUX pin)

LoRa_E22 e22ttl100(PIN_RX, PIN_TX, PIN_AX, PIN_M0, PIN_M1);

struct WEATHER_DATA {
  char username[30] = "weatherstation_home";
  char password[30] = "4311067vladvk";
  int pressureFromBMP180;
  double temperatureFromBMP180;
  double temperatureFromDTH22;
  double humidityFromDTH22;
  int analogSignalFromRainSensor;
  int rssi;
};

struct WEATHER_STATION_AUTH {
  String token;
  boolean isAuthorized = false;
};

WEATHER_DATA ownWeatherData;
WEATHER_DATA weatherDataFromSomeWeatherStation;

WEATHER_STATION_AUTH ownWeatherStationAuth;
WEATHER_STATION_AUTH someWeatherStationAuth;

void setup() {
  Serial.begin(9600);
  Serial.println("starting e22 sensor...");
  e22ttl100.begin();
  Serial.println("success");

  Serial.println("starting wifi...");
  WiFi.begin(ssid, password);

  Serial.println("success");

  Serial.println("starting bmp180...");
  while (myBMP.begin(4, 5) != true)
  {
    Serial.println(F("Bosch BMP180/BMP085 is not connected or fail to read calibration coefficients"));
    delay(500);
  }
  Serial.println("success");

  Serial.println("starting dht22...");
  dht22.setup(16, DHTesp::DHT22);
  pinMode(rainSensorGPIOPin, INPUT);
  Serial.println("success");
}



void sendDataToServer(WEATHER_DATA *weather_data, WEATHER_STATION_AUTH *weatherStationAuth) {
  Serial.println(weather_data->pressureFromBMP180);
  Serial.println(weatherStationAuth->isAuthorized);
  Serial.println(weather_data->pressureFromBMP180);
  if (!weatherStationAuth->isAuthorized) {
    http.begin(wifiClient, "http://192.168.1.152:3000/auth/login");
    http.addHeader("Content-Type", "application/json");
    char body[200];
    StaticJsonDocument<400> doc;

    doc["username"] = weather_data->username;
    doc["password"] = weather_data->password;

    serializeJson(doc, body);
    Serial.println(body);
    int httpCode = http.POST(body);

    if (httpCode > 0) {
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);
      String payload = http.getString();
      Serial.println(payload);
      StaticJsonDocument<1000> payloadJson;
      DeserializationError error = deserializeJson(payloadJson, payload);
      Serial.println(error.f_str());
      String token = payloadJson["token"];
      weatherStationAuth->token = token;
      weatherStationAuth->isAuthorized = true;
      Serial.println(weatherStationAuth->token);
      Serial.println(weatherStationAuth->isAuthorized);
    }
    else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
  }
  StaticJsonDocument<400> doc;
  http.begin(wifiClient, "http://192.168.1.152:3000/weather-record");
  http.addHeader("Content-Type", "application/json");
  String Authorization = "Bearer " + weatherStationAuth->token;
  http.addHeader("Authorization", Authorization);
  char body[400];
  doc["pressureFromBMP180"] = weather_data->pressureFromBMP180;
  doc["temperatureFromBMP180"] = weather_data->temperatureFromBMP180;
  doc["temperatureFromDTH22"] = weather_data->temperatureFromDTH22;
  doc["humidityFromDTH22"] = weather_data->humidityFromDTH22;
  doc["analogSignalFromRainSensor"] = weather_data->analogSignalFromRainSensor;
  doc["rssi"] = weather_data->rssi;
  serializeJson(doc, body);
  int httpCode = http.POST(body);
  Serial.printf("[HTTP] POST... code: %d\n", httpCode);
  String payload = http.getString();


  http.end();

}

unsigned long previousTime = 0;

void loop() {
  if (e22ttl100.available() > 1) {
#ifdef ENABLE_RSSI
    ResponseStructContainer rsc = e22ttl100.receiveMessageRSSI(sizeof(WEATHER_DATA));
#else
    ResponseStructContainer rsc = e22ttl100.receiveMessage(sizeof(WEATHER_DATA));
#endif
    Serial.println(rsc.status.code);
    if (rsc.status.code != 1) {
      Serial.println(rsc.status.getResponseDescription());
    } else {
      Serial.println(rsc.status.getResponseDescription());
      struct WEATHER_DATA weatherDataFromSomeWeatherStation = *(WEATHER_DATA*) rsc.data;
      Serial.print("username -> ");
      Serial.println(weatherDataFromSomeWeatherStation.username);
      Serial.print("password -> ");
      Serial.println(weatherDataFromSomeWeatherStation.password);
      Serial.print("pressureFromBMP180 -> ");
      Serial.println(weatherDataFromSomeWeatherStation.pressureFromBMP180);
      Serial.print("temperatureFromBMP180 -> ");
      Serial.println(weatherDataFromSomeWeatherStation.temperatureFromBMP180);
      Serial.print("temperatureFromDTH22 -> ");
      Serial.println(weatherDataFromSomeWeatherStation.temperatureFromDTH22);
      Serial.print("humidityFromDTH22 -> ");
      Serial.println(weatherDataFromSomeWeatherStation.humidityFromDTH22);
      Serial.print("analogSignalFromRainSensor -> ");
      Serial.println(weatherDataFromSomeWeatherStation.analogSignalFromRainSensor);

#ifdef ENABLE_RSSI
      weatherDataFromSomeWeatherStation.rssi = rsc.rssi;
      Serial.print("RSSI: "); Serial.println(weatherDataFromSomeWeatherStation.rssi, DEC);
#endif
      sendDataToServer(&weatherDataFromSomeWeatherStation, &someWeatherStationAuth);
    }
  }

  ownWeatherData.pressureFromBMP180 = myBMP.getPressure();
  ownWeatherData.temperatureFromBMP180 = myBMP.getTemperature();
  ownWeatherData.temperatureFromDTH22 = dht22.getTemperature();
  ownWeatherData.humidityFromDTH22 = dht22.getHumidity();
  ownWeatherData.analogSignalFromRainSensor = analogRead(rainSensorGPIOPin);

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 2000) {
    sendDataToServer(&ownWeatherData, &ownWeatherStationAuth);
    previousTime = currentTime;
  }




  //    Last = millis();
}
