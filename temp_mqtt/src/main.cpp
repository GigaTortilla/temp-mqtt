/*************************************

// MIT License

// Copyright (c) 2021 Martin Appel

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

 **************************************
 * 
 * MQTT Temperature Sensor
 * for use with IoD-24T
 * 
 * Version: 0.1.3
 * 
 * Creator: Martin Appel | 182048/AUT
 * 
 *************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <mqtt_client.h>

#define DEFAULT_BAUD 115200
#define MQTT_PORT 1883
#define MQTT_PORT_SSL 8883
#define QOS0 0
#define QOS1 1
#define QOS2 2
#define OUT_TOPIC "/everything/smart/home\0"
#define HUB_STATUS_TOPIC "/IoD/status/out\0"
#define HUB_RQ_STATUS_TOPIC "/IoD/status/rq\0"
#define STATUS_TOPIC "/ESP/status/out\0"
#define RQ_STATUS_TOPIC "/ESP/status/rq\0"
#define TEMP_TOPIC "/ESP/temp/out\0"

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
bool processData(const char* topic, int topic_length, const char* data, int data_length);
 
// Globale Variablen

bool b_sendFlag = false;

// MQTT client handle and config

esp_mqtt_client_handle_t espClient;
esp_mqtt_client_config_t espConf;


// Task handles

static TaskHandle_t taskMeasureTemp_h = NULL;

// Task records and transmits the current temperature every 5 mins
void taskMeasureTemp(void *parameter)
{
  // The whole process of measuring and sending the temperature data is done
  // in this singular task so the variables only have to be valid within the task.

  float voltage;
  float temperature;
  char c_temp[6];

  while (1)
  {
    // The default ADC width is 12 bit for the ESP32
    voltage = ((float)analogRead(36) / 4096.0f) * 5.0f;
    // The temperature sensor used in this application is the Analog Devices TMP36
    temperature = (voltage - 0.5f) * 50.0f;

    // Convert the float value to a string for sending it via MQTT
    dtostrf((double)temperature, 2, 2, c_temp);

    // Publish the recorded temperature to homeassistant every 5 mins
    if (b_sendFlag)
    {
      esp_mqtt_client_publish(espClient, TEMP_TOPIC, c_temp, 0, QOS0, 1);
    }
    vTaskDelay(300000UL / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // No access to SD cards or external storage ICs so just writing the 
  // Wifi and MQTT access directly into the code
  const char* ssid = "DieWildeBrunhilde";
  const char* wpa2 = "REDACTED";
  const char* mqttServer = "mqtt://192.168.1.3";
  const char* mqttUser = "mqtt-user";
  const char* mqttPassword = "REDACTED";
 
  Serial.begin(DEFAULT_BAUD);
  
  // Wifi-connection
  WiFi.begin(ssid, wpa2);  
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(WiFi.waitForConnectResult());
  }
  Serial.print("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());
 
  // prepare MQTT config
  espConf.uri = mqttServer;
  espConf.port = MQTT_PORT;
  espConf.username = mqttUser;
  espConf.password = mqttPassword;
  espConf.event_handle = mqtt_event_handler_cb;
 
  espClient = esp_mqtt_client_init(&espConf);
  esp_mqtt_client_start(espClient);
 
  // preparing the ADC0(GPIO36) channel of ADC1
  pinMode(36, ANALOG);

  // Create the task to measure and send temperature
  xTaskCreate(taskMeasureTemp, "Temperature Measurement", 2048, NULL, 1, &taskMeasureTemp_h);

  // Delete yourself
  vTaskDelete(NULL);
}
 
void loop()
{
  // Unreachable through deletion of the task
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            msg_id = esp_mqtt_client_subscribe(client, HUB_STATUS_TOPIC, QOS0);
            msg_id = esp_mqtt_client_subscribe(client, RQ_STATUS_TOPIC, QOS0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            break;
 
        case MQTT_EVENT_SUBSCRIBED:
            msg_id = esp_mqtt_client_publish(client, OUT_TOPIC, "Subscribed to IoD", 0, QOS0, 0);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            break;
        case MQTT_EVENT_PUBLISHED:
            break;
        case MQTT_EVENT_DATA:
            // processData(event->topic, event->topic_len, event->data, event->data_len);
            Serial.printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            Serial.printf("DATA=%.*s\r\n", event->data_len, event->data);
            if (processData(event->topic, event->topic_len, event->data, event->data_len))
            {
              msg_id = esp_mqtt_client_publish(client, STATUS_TOPIC, "Beginning to send...", 0, QOS0, 0);
              b_sendFlag = true;
            }
            break;
        case MQTT_EVENT_ERROR:
            break;
        default:
            break;
    }
    return ESP_OK;
}
 
bool processData(const char* topic, int topic_length, const char* data, int data_length)
{
  char c_topic[topic_length + 1];
  char c_data[data_length + 1];
 
  strncpy(c_topic, topic, topic_length);
  strncpy(c_data, data, data_length);
 
  return !(bool)strncmp(RQ_STATUS_TOPIC, c_topic, topic_length);
}