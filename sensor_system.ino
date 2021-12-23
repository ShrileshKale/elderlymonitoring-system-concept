// This code is untested due to lack of hardware
#include <Arduino_HTS221.h>
#include <Arduino_APDS9960.h>
#include <PDM.h>



/*******************************************
  Declaration of global varibales
*******************************************/
float temp_sensed;
float humidity_sensed;
int gesture;
// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
// number of samples read
volatile int samplesRead;
unsigned long previousMillis = 0;
const long intervalLong = 1000;
const long intervalMed = 500;
const long intervalShort = 100;

/*******************************************
   Initializing and setting up sensors
 ********************************************/
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  PDM.onReceive(onPDMdata); // callback function for microphone


  HTS.begin();

  if (!APDS.begin()) {
    Serial.println("Error initializing gesture sensor!");
  }

  // initialize PDM (microphone) with one channel at 16khz rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

}

/******************************************
   Sensing data and passing the same
 *******************************************/
void loop() {

  gesture_temp_humid_detect();
  microphone_detect();
  proximity_detect()

}


/*******************************************************************************
* Function Name : microphone_detect
* Description   : This function will detect person's voice
*                 and will send value via UART
* Input         : NA
* Output        : voice value
* Return        : NA
*******************************************************************************/


void microphone_detect()
{
  if (samplesRead) {

    for (int i = 0; i < samplesRead; i++) {
      // check if the sound value is higher than 500
      if (sampleBuffer[i] >= 500) {
        Serial.println("suspecious");
      }
      if (sampleBuffer[i] >= 250 && sampleBuffer[i] < 500) {
        Serial.println("abnormal");
      }
      if (sampleBuffer[i] >= 0 && sampleBuffer[i] < 250) {
        Serial.println("normal");
      }
    }

    // clear the read count
    samplesRead = 0;
  }

}

// inspired from arduino tutorial
void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}



/*******************************************************************************
* Function Name : proximity_detect
* Description   : This function will detect person's presence inside a room 
*                 and will send value via UART
* Input         : NA
* Output        : sensor values
* Return        : NA
*******************************************************************************/



void proximity_detect()
{
  unsigned long currentMillis = millis();

  // check if a proximity reading is available
  if (APDS.proximityAvailable()) {
    // read the proximity
    // - 0   => close
    // - 255 => far
    // - -1  => error
    int proximity = APDS.readProximity();
    // far condition
    if (proximity > 150) {
      if (currentMillis - previousMillis >= intervalLong) {
        previousMillis = currentMillis;
        Serial.println("close"); // send data via UART
      }
    }

    // close condition
    if (proximity < 150) {
      if (currentMillis - previousMillis >= intervalLong) {
        previousMillis = currentMillis;
        Serial.println("far"); // send data via UART
      }
    }


  }

}

/*******************************************************************************
* Function Name : gesture_temp_humid_detect
* Description   : This function will detect person's gestures, temperature and humidty value
*                 and will send values via UART
* Input         : NA
* Output        : sensor values
* Return        : NA
*******************************************************************************/
void gesture_temp_humid_detect()
{

  //Gesture detection
  for (int i = 0; i < 1000; i++)
  {
    APDS.setGestureSensitivity(90);
    if (APDS.gestureAvailable()) {
      gesture = APDS.readGesture();
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Gesture:");
      Serial.println(gesture);
    }
    delay(10);
  }

  //Temperature monitoring
  temp_sensed = HTS.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temp_sensed); // send via UART

  humidity_sensed = HTS.readHumidity();
  Serial.print("Humidity: ");
  Serial.println(humidity_sensed); // send via UART
  digitalWrite(LED_BUILTIN, LOW);

}
