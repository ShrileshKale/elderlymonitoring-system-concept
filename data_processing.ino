# /***************************
#  *  data_processing.ino
#  * Version Number  : 1.0
#  * Configuration Identifier: 
#  * Modified by:  Shrilesh Kale       
#  * Modified Date:  22/12/2021       
#  * Description: Source code for data proceesing of sensor values 
#  * Note- This code was not tested so it might contain some errors.
#  **************************/



/*******************************************************************************
                              Includes
*******************************************************************************/
#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include <WiFiClientSecure.h>
#include <string.h>
#else
#include <WiFi.h>
#endif
/*******************************************************************************
                              defines
*******************************************************************************/
#define ESPALEXA_DEBUG
#define ESPALEXA_MAXDEVICES   5
#define APPLIANCE_LED          4          // GPIO 4 (On board: D2)
#define MSG_BUFFER_SIZE       (50)
/*******************************************************************************
                             Global Variables
*******************************************************************************/
WiFiClient espClient;
PubSubClient client(espClient);
WiFiClientSecure net_ssl;
const char* ssid = "ssid";
const char* password = "password";
const char* mqtt_server = "mqtt broker ip";
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
int i = 0;
const int uart_buff_size = 3;
char buf[uart_buff_size];

/*******************************************************************************
  Function    : setup
  Description : This function will Initialize the MQTT server, Connect ESP8266
                to Internet(via WiFi). Also, Initialize Alexa with ESP
  Input       : NA
  Output      : NA
  Return      : NA
*******************************************************************************/
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);              // Initialize the LED_BUILTIN pin as an output
  Serial.begin(115200);                      // Set Baud Rate to 115200
  Serial.println('\n');
  WiFi.begin(ssid, password);                // Connect ESP8266 to WiFi using ssid and password
  Serial.println("Connecting.....");         // Print "Connecting....." on Serial Monitor
  client.setServer(mqtt_server, 1883);       // Set MQTT Server
  client.setCallback(callback);              // Initialize callback func. of MQTT

  while (WiFi.status() != WL_CONNECTED )     // Check whether ESP8266 is connected to WiFi
  {
    delay(500);
    Serial.println("Not connected");
  }

  if (WiFi.status() == WL_CONNECTED)         // Once ESP8266 is connected to WiFi
  {
    Serial.println('\n');
    Serial.println("Connected to :- ");      // Print "Connected" to connected ssid on Serial Monitor
    delay(1000);
    Serial.println("IP address is:- ");      // Print "IP Address of ESP8266" on Serial Monitor
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());

    Serial.println("Let's add devices");

  }
  else
  {
    Serial.println("not connected");
  }
  Serial.flush();


  // Subscribe to topic"
  client.subscribe("ai_planner/actuation/receive");

}

/*******************************************************************************
  Function    : loop
  Description : This function will execute the MQTT server and listens for UART data
                indefinitely.
  Input       : NA
  Output      : NA
  Return      : NA
*******************************************************************************/
void loop()
{
  if (!client.connected())  // Reconnect the ESP8266 to MQTT server if not connected
  {
    reconnect();
  }
  receive_via_uart(); 
  client.loop();  // Execute MQTT Server to Publish/ Subscribe
}

/*******************************************************************************
  Function    : receive_via_uart
  Description : This function collects data from UART and publishes data via MQTT
                topic.
  Input       : NA
  Output      : NA
  Return      : Array (data buffer)
*******************************************************************************/

int * receive_via_uart()
{
  if (Serial.available() > 0)
  {
    int read_bytes = Serial.readBytesUntil('\n', buf, uart_buff_size);
    Serial.println("Received data from BLE sense");
    for (int i = 0; i < read_bytes; i++)
      Serial.print(buf[i]);

    // Publish to topic"
    client.publish("ble_sense/sensor/gesture", buf[0]); // assuming that sensor values will be of 1 byte each
    client.publish("ble_sense/sensor/microphone", buf[1]);
    client.publish("ble_sense/sensor/proximity", buf[2]);

    return buf;

  }
}




/*******************************************************************************
  Function    : callback
  Description : This function gets called when data arrives on mqtt topic and does actuation
                if required.
  Input       : MQTT: subscribed topic, payload from subscribed topic, length of
                payload
  Output      : LED ON/OFF
  Return      : NA
*******************************************************************************/
void callback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, "ai_planner/actuation/receive") == 0)
  {
    Serial.println();
    Serial.print("Message arrived from AI planner");
    Serial.println();
    Serial.print("Topic: ");
    Serial.print(topic);
    String received_data =  String((char*)payload );    // Get payload in received data
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload); // Get the JSON object

    if (error)                                                  // Test if parsing succeeds
    {
      Serial.println(F("deserializeJson() failed: "));
      Serial.print(error.c_str());                              // Print parsing error
    }


    for (int i = 0; i < length; i++)
    {
      String received_data = doc["actuation"];   // Store the string of JSON object item (actuation) sent via MQTT in received_data
    }

    if (received_data[23] == 'n' || received_data[23] == 'N' )  // Check whether string received_data is ON(/On) or OFF(/Off)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else  // ON received
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }


}

/*******************************************************************************
  Function    : reconnect
  Description : This function reconnects the ESP8266 to MQTT server
  Input       : NA
  Output      : NA
  Return      : NA
*******************************************************************************/
void reconnect()
{
  while (!client.connected())                                 // Loop until ESP8266 is connected to MQTT server
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";                       // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str()))                     // Attempt to connect to MQTT server
    {
      Serial.println("Connected to MQTT Server");
      client.publish("lighting_control/central/lighting/esp8266", "Smart Central Lighting"); // Once connected, publish an announcement.
      client.subscribe("lighting_control/central/appliance/Rpi");       // Resubscribe to "lighting_control/central/Rpi"
      client.subscribe("lighting_control/central/lighting/PC"); // Subscribe to "lighting_control/central/Rpi"
    }
    else
    {
      Serial.print("failed, rc=");                            // Failed to connect to MQTT server
      Serial.print(client.state());                           // Print "client state" on serial monitor
      Serial.println("Trying again in 5 seconds");
      delay(5000);                                            // Wait 5 seconds before retrying
    }
  }
}

/******************************************************************************
                      End of File
******************************************************************************/
