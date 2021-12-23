/***************************
#  *  smartplug.ino
#  * Version Number  : 1.0
#  * Configuration Identifier: 
#  * Modified by:  Shrilesh Kale      
#  * Modified Date:  22/12/2021       
#  * Description: Source code for smarty device (smartplug)
#  **************************/

/*******************************************************************************
                              Defines
*******************************************************************************/
#define buzzer            4
#define MaxThreshold      359
#define MaxTiltThreshold  359
#define sample_num_mdate  5000
#define triggerPin        7
#define echoPin           8
#define pinRST            9
#define pinSDA            10
#define doorlockPin       6
#include <ESP8266WiFi.h>
#define LED_BUILTIN 2

/*******************************************************************************
                            Global Variables
*******************************************************************************/
const char* ssid = "wifissid";
const char* password = "wifipassword";
int ENA = 4;
int IN1 = 0;
int IN2 = 14;
WiFiServer server(80);


/*******************************************************************************
  Function Name : setup
  Description   : This function will Initialize and set up all the sensors,
                  actuators and display configuration
  Input         : NA
  Output        : NA
  Return        : NA
*******************************************************************************/


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());
}

/*******************************************************************************
  Function Name : loop
  Description   : This function will get GET/POST reques from android app

  Input         : NA
  Output        : NA
  Return        : NA
*******************************************************************************/


void loop() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  Serial.println("new client");
  while (!client.available())
  {
    delay(1);
  }
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  if (req.indexOf("/fanon") != -1)
  {

    digitalWrite(LED_BUILTIN, LOW);
    motor_drive_anticlockwise();
  }
  else if (req.indexOf("/fanoff") != -1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    motor_drive_clockwise();
  }

  if (req.indexOf("/lighton") != -1)
  {
    digitalWrite(LED_BUILTIN, LOW);
    motor_drive_anticlockwise();

  }
  else if (req.indexOf("/lightoff") != -1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    motor_drive_clockwise();
  }


  String web = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";

  client.print(web);
}

/*******************************************************************************
  Function Name : motor_drive_clockwise
  Description   : This function will rotate motor clokwise,

  Input         : NA
  Output        : NA
  Return        : NA
*******************************************************************************/

void motor_drive_clockwise() {

  // turn on motor

  digitalWrite(ENA, HIGH); // set speed to 200 out of possible range 0~255
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(1000);

  digitalWrite(IN1, LOW); // to set motor to initial position
  digitalWrite(IN2, HIGH);
  delay(250);


  // now turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}


/*******************************************************************************
  Function Name : motor_drive_anticlockwise
  Description   : This function will rotate motor anticlockwise

  Input         : NA
  Output        : NA
  Return        : NA
*******************************************************************************/


void motor_drive_anticlockwise() {

  digitalWrite(ENA, HIGH); // set speed to 200 out of possible range 0~255
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(1000);


  digitalWrite(IN1, HIGH); // to set motor to initial position
  digitalWrite(IN2, LOW);
  delay(250);


  // turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
