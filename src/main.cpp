/*******************************************************************
**File Name: main.c                                               **
**Original Project Name: Garden Light Software.                   **
**Author Name: Jeremiah A.                                        **
**Version No:  1.0                                                **
**Date Created: 13th March 2024.                                  **
**Last Edited: 22nd March 2024.                                   **
********************************************************************/


#include <gardenlighthw.h>
#include <millisDelay.h>
#include <WiFi.h>

// TODO: Configure your WiFi here
//#define WIFI_SSID "VM1628887"
//#define WIFI_PSK  "jeojc7gwbnPkvzcg"
#define WIFI_SSID "Freeloadersunited"
#define WIFI_PSK  "klak8385"

/** Check if we have multiple cores */
#if CONFIG_FREERTOS_UNICORE
#define TASK_CORE 0
#else
#define TASK_CORE 1
#endif

#if 0
// Include certificate data (see note above)
#include "cert.h"
#include "private_key.h"

// We will use wifi


// Includes for the server
#include <HTTPSServer.h>
#include <SSLCert.h>
#include <HTTPRequest.h>
#include <HTTPResponse.h>

// The HTTPS Server comes in a separate namespace. For easier use, include it here.
using namespace httpsserver;

#endif

#define DEFAULT_DELAY_TIME      10000 // 10 seconds
#define DUMMY_LIGHT_SENSE       1
#define SWITCH_DEBOUNCE_DELAY   50    // 50 mS
#define LIGHT_DEBOUNCE_DELAY    100   // 100 mS
#define PIR_SENSE_DEBOUNCE      25    //25 mS
#define LOOP_DEBUG_DELAY        3000  // 6 Seconds
#define TEST_MANUAL_DELAY       1     // is high when test manual state is active

#define SIMPLE_HTTP_SERVER      1   //simple http server active if 1

// put function declarations here:
  FloodLight gardenLight("gardenlight", GARDEN_LIGHT);
  Sensor pirSensor1("pir sensor 1", MOTION_SENSOR);
  Sensor mockSensor("Mock Sensor", MOCK_SENSOR);
  Sensor lightSensor("Light Sensor", DIOEX1);
  Switch keypadLock1("keypad lock", KEYPAD_LOCK);
  Switch manualTest("manual test switch", MANUAL_TEST_SW);
  Indicator wifiConnectLamp("Wifi connection Lamp", WIFICON_LAMP);
  Indicator manualModeLamp("Manual Mode Lamp",MANUAL_MODE_LAMP);
  Indicator motionModeLamp1("motion mode lamp", MOTION_MODE_LAMP);
  Indicator remoteSenseLamp("remote sensor lamp", REMOTE_SENSOR_LAMP);
  Indicator motionSenseLamp("Motion Sensor Lamp", MOTION_SENSOR_LAMP);

// Declare delay timer class variables
  millisDelay timeDelay;
  millisDelay manualDelay;
  millisDelay pirDebounce;
  millisDelay mockSenseDebounce;
  millisDelay manualtestDebounce;
  millisDelay keypadDebounce;
  millisDelay lightSenseDebounce;

// Declare state variables and flags
  uint8_t currManualSwState = 0;
  uint8_t manualTimeoutFlag = 0;
  #if TEST_MANUAL_DELAY
    uint32_t manualDelayTime = 3000;    // 3 seconds delay
  #else
    uint32_t manualDelayTime = 300000;  // 5 minutes delay
  #endif
  uint8_t manualActiveFlag = 0;

  uint8_t currManualTestState = 1;
  uint8_t prevManualTestState = 1;
  uint8_t manualTestReading = 0;
  uint8_t lastManualTestReading = 0;

  uint8_t currPirSenseState = 0;
  uint8_t pirReading = 0;
  uint8_t lastPireading = 0;
  uint8_t motionLightActiveFlag = 0;
  uint8_t timeDelayStartFlag = 0;
  uint16_t motionTimeDelay = 20000; // 20 seconds default time delay

  uint8_t currKeypadState = 0;
  uint8_t keypadReading = 0;
  uint8_t lastKeypadReading = 0;

  uint8_t currMockSenseState = 0;
  uint8_t mockSenseReading = 0;
  uint8_t lastMockSenseReading = 0;

  uint8_t lightSenseState = 1;
  uint8_t lightSenseReading = 0;
  uint8_t lastLightSenseReading = 0;

  uint16_t goToSleep __section(".noinit");

// Wifi and webserver variables
// Set web server port number to 80
  WiFiServer server(80);
  unsigned long previousReconMillis = 0;
  unsigned long currReconMillis = 0;
  unsigned long reconDelay = 10000;

// Variable to store the HTTP request
  String header;
// Current time
  unsigned long currentTime = millis();
// Previous time
  unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
  const long timeoutTime = 2000;

// State Logic functions definitons:
  /**************************************************************************/
/*!
    @brief  Describes the manual state logic. The floodlight is switched on
            when the manual webserver/remote switch or onboard manual test 
            switch is activated/pressed.
            The floodlight is switched off either when the manual timeout
            expires or the manual webserver/remote switch or onboard manual
            test switch is deactivated/unpressed.
*/
/**************************************************************************/
  void ManualStateLogic(void);
  /**************************************************************************/
/*!
    @brief  Describes the Motion state logic, the floodlight is switch on
            when motion is detected (PIR sensor is active) or mock sensor
            is pressed, a user defined time delay is activated after motion
            is no longer detected and when the delay timer expires, the
            floodlight is switched off.
*/
/**************************************************************************/
  void MotionStateLogic(void);
  /**************************************************************************/
/*!
    @brief  Describes the Keypad logic, when the keypad lock switch is high,
            the keypad is locked and the onboard switches states are deactiv
            ated, and when keypad is unlocked, the onboard switches states
            are read and stored. 
*/
/**************************************************************************/
  void KeypadLogic(void);
  /**************************************************************************/
/*!
    @brief This is the web server FreeRTOS task function.  
*/
/**************************************************************************/
void WebServerTask(void* params);  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);
  #if DEBUG
  delay(10000);
  #endif
  while(esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, 1) != ESP_OK);

  Serial.println("Welcome, Starting Garden Light Control!!!");
  manualModeLamp.Activate();
  wifiConnectLamp.Activate();
  motionModeLamp1.Activate();
  motionSenseLamp.Activate();
  remoteSenseLamp.Activate();
  #if DEBUG
  delay(10000);
  #else
  delay(1000);
  #endif
   manualModeLamp.Deactivate();
  wifiConnectLamp.Deactivate();
  motionModeLamp1.Deactivate();
  motionSenseLamp.Deactivate();
  remoteSenseLamp.Deactivate();

  currKeypadState = keypadLock1.GetState();
  currManualTestState =manualTest.GetState();
  currMockSenseState = mockSensor.GetSignal();
  currPirSenseState = pirSensor1.GetSignal();

  TaskHandle_t wifiTaskHandle = NULL;
 
 // If go to sleep is set, start deep sleep
  if(goToSleep == 0xdd11)
  {
    goToSleep = 0;
    Serial.println("Going to sleep");
    esp_deep_sleep_start();
  }
  lightSenseState = lightSensor.GetSignal();

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  #if !DEBUG
    WiFi.begin(WIFI_SSID, WIFI_PSK);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
    xTaskCreatePinnedToCore(WebServerTask, "http80", 3000, NULL, 1, &wifiTaskHandle, TASK_CORE);
   #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("in the loop");
  lightSenseReading = lightSensor.GetSignal();
  
  #if DEBUG
    if(lightSenseReading)
    {
      Serial.println("light sense reading is HIGH");
    }
    else{
      Serial.println("light sense reading is LOW");
    }
    delay(LOOP_DEBUG_DELAY);
  #endif
  // Debounce the Light Sensor input
  // light sensor output is 0(LOW) when no light is detected
  // and 1(HIGH) when light is detected
    if(lastLightSenseReading != lightSenseReading)
    {
      lightSenseDebounce.start(LIGHT_DEBOUNCE_DELAY);
      #if DEBUG
        Serial.println("debounce started");
        delay(LOOP_DEBUG_DELAY);
      #endif
    }
    if(lightSenseDebounce.justFinished())
    {
      if(lightSenseReading != lightSenseState)
      {
        lightSenseState = lightSenseReading;
        #if DEBUG
          if(lightSenseState)
          {
            Serial.println("light sense state is HIGH");
          }
          else{
            Serial.println("light sense state is LOW");
          }
          delay(LOOP_DEBUG_DELAY);
        #endif
      }
      lightSenseDebounce.finish();
    }
  #if DEBUG
    Serial.println("light sense??");
  #endif
  if(!(lightSenseState) && DUMMY_LIGHT_SENSE)
  {
    goToSleep = 0xdd11;
    Serial.println("Sleep mode activated.. restarting system");
    ESP.restart();
  }
  else
  {
   // Get Pir sensor digital input value
    pirReading = pirSensor1.GetSignal();
    #if DEBUG
    delay(LOOP_DEBUG_DELAY);
    #endif
  
    // Debounce the PIR Sensor input
    if(lastPireading != pirReading)
    {
      pirDebounce.start(PIR_SENSE_DEBOUNCE);
    }
    if(pirDebounce.justFinished())
    {
      if(pirReading != currPirSenseState)
      {
        currPirSenseState = pirReading;
        #if DEBUG
        Serial.println("pir deboucnce finished, curr Pir sense state is "+ currPirSenseState);
        delay(LOOP_DEBUG_DELAY);
        #endif
      }
      pirDebounce.finish();
    }

    // call Keypad Logic function
    KeypadLogic();
    // Call manual state logic
    ManualStateLogic();
    // Call motion state logic
    MotionStateLogic();

    lastManualTestReading = manualTestReading;
    lastMockSenseReading = mockSenseReading;
    lastPireading = pirReading;
    lastKeypadReading = keypadReading;
    
  }
  lastLightSenseReading = lightSenseReading;
}

/**************************************************************************/
/*!
    @brief  Describes the manual state logic. The floodlight is switched on
            when the manual webserver/remote switch or onboard manual test 
            switch is activated/pressed.
            The floodlight is switched off either when the manual timeout
            expires or the manual webserver/remote switch or onboard manual
            test switch is deactivated/unpressed.
*/
/**************************************************************************/
void ManualStateLogic(void)
{
  #if DEBUG
    if(currManualTestState)
    {
     Serial.println("current manual test state is high");
    }
    else
    {
      Serial.println("current manual test state is low");
    }
    if(currManualSwState)
    {
     Serial.println("current manual switch state is high");
    }
    else
    {
      Serial.println("current manual switch state is low");
    }
    delay(LOOP_DEBUG_DELAY);
  #endif
  // Turn on floodlight and start switch off delay when the
  // manual switch or manual test switch is high/ON (1)
  if (currManualSwState || !(currManualTestState))
  {
    // Carry on with the command if manual time out flag has not
    // been set
    #if DEBUG
    Serial.println("Entere Manual State");
    #endif
    
    if(!manualTimeoutFlag)
    {
      if(!manualActiveFlag)
      {
        motionModeLamp1.Deactivate();
        manualModeLamp.Activate();
        gardenLight.TurnOn();
        manualActiveFlag = 1;
        motionLightActiveFlag = 0;
        prevManualTestState = currManualTestState;
        #if DEBUG
          gardenLight.PrintStatus();
        #endif
        manualDelay.start(manualDelayTime);
      }
      
      if(manualDelay.justFinished())
      {
        manualModeLamp.Deactivate();
        gardenLight.TurnOff();
        manualTimeoutFlag = 1;
        manualActiveFlag = 0;
        manualDelay.finish();
        #if DEBUG
          gardenLight.PrintStatus();
        #endif
      }
    }
  }
  // Turn off light when either manual switch or manual test switch
  // is off/Low (0)
  else if (!(currManualSwState) || currManualTestState)
  {
    if(!motionLightActiveFlag)
    {
      if(prevManualTestState != currManualTestState)
      {
        if(currManualTestState)
        {
          manualModeLamp.Deactivate();
          gardenLight.TurnOff();
          manualActiveFlag = 0;
          manualTimeoutFlag = 0;
        }
      }
      else if(!currManualSwState)
      {
        manualModeLamp.Deactivate();
        gardenLight.TurnOff();
        manualActiveFlag = 0;
        manualTimeoutFlag = 0;
      }
    }
  }
}

/**************************************************************************/
/*!
    @brief  Describes the Motion state logic, the floodlight is switch on
            when motion is detected (PIR sensor is active) or mock sensor
            is pressed, a user defined time delay is activated after motion
            is no longer detected and when the delay timer expires, the
            floodlight is switched off.
*/
/**************************************************************************/
void MotionStateLogic(void)
{
  // Enter into motion detect mode when manual mode is inactive
  // that is manual switch is off or timed out
  if(!manualActiveFlag)
  {
    // if PIR sensor detects motion, turn on Light.
    if(currPirSenseState || !(currMockSenseState))
    {
      gardenLight.TurnOn();
      motionModeLamp1.Activate();
      motionLightActiveFlag = 1;
    }
    //when PIR sensor no longer detects motion and 
    // motion light active flag is set, then set
    // countdown timedown timer to switch off
    else if((!(currPirSenseState)|| currMockSenseState) && motionLightActiveFlag)
    {
      if(!timeDelayStartFlag)
      {
        timeDelay.start(motionTimeDelay);
        timeDelayStartFlag =1;
      }
      //when countdown timer is expired, turn off
      // the light
      if(timeDelay.justFinished())
      {
        motionModeLamp1.Deactivate();
        gardenLight.TurnOff();
        timeDelayStartFlag = 0;
        motionLightActiveFlag = 0;
        timeDelay.finish();
      }
    }
  }
}

/**************************************************************************/
/*!
    @brief  Describes the Keypad logic, when the keypad lock switch is high,
            the keypad is locked and the onboard switches states are deactiv
            ated, and when keypad is unlocked, the onboard switches states
            are read and stored. 
*/
/**************************************************************************/
void KeypadLogic(void)
{
  // Get Keypad lock switch digital input value
  keypadReading = keypadLock1.GetState();
  #if DEBUG
  if(keypadReading)
  {
  Serial.println("keypad reading is high");
  }
  else
  {
    Serial.println("keypad reading is low");
  }
  delay(LOOP_DEBUG_DELAY);
  #endif
  // Debounce the keypad lock switch input
  if (lastKeypadReading != keypadReading)
  {
    keypadDebounce.start(PIR_SENSE_DEBOUNCE);
  }
  
  if(keypadDebounce.justFinished())
  {
    if (keypadReading != currKeypadState)
    {
      currKeypadState = keypadReading;
      #if DEBUG
      Serial.println("keypad debounce finished, curr keypad lock state is "+ currKeypadState);
      delay(LOOP_DEBUG_DELAY);
      #endif
    }
      keypadDebounce.finish();
    
  }

  // Get manual test switch and mock sensor values
  // when the keypad is unlocked.
  #if DEBUG
  if(!currKeypadState)
  {
    Serial.println("current keypad lock state is unlock");
  }
  else{
    Serial.println("current keypad lock state is lock");
  }
    delay(LOOP_DEBUG_DELAY);
  #endif
  if(!currKeypadState)   
  {
    #if DEBUG
    Serial.println("keypad unlocked");
    #endif
    manualTestReading = manualTest.GetState();
    mockSenseReading= mockSensor.GetSignal();

    if(lastManualTestReading != manualTestReading)
    {
      manualtestDebounce.start(SWITCH_DEBOUNCE_DELAY);
    }
    if (manualtestDebounce.justFinished())
    {
      if(manualTestReading != currManualTestState)
      {
        currManualTestState = manualTestReading;
      }
      manualtestDebounce.finish();
    }
    if (lastMockSenseReading != mockSenseReading)
    {
      mockSenseDebounce.start(SWITCH_DEBOUNCE_DELAY);
    }
    if(mockSenseDebounce.justFinished())
    {
      if(mockSenseReading != currMockSenseState)
      {
        currMockSenseState = mockSenseReading;
      }
      mockSenseDebounce.finish();
    }
  
  }
  // Else if the keypad is locked, set the
  // current manual test state and mock sense state
  // to low/off (0)
  else
  {
    currManualTestState = 1;
    currMockSenseState = 1;
  }
}

#if SIMPLE_HTTP_SERVER
void WebServerTask(void* params)
{
  for(;;)
  {
    //Serial.println("in the core 1");
    if(WiFi.status() != WL_CONNECTED)
    {
      wifiConnectLamp.Deactivate();
      Serial.println("wifi is disconnected");
    }
    else{
      wifiConnectLamp.Activate();
    }
    if((WiFi.status() != WL_CONNECTED) && (currReconMillis - previousReconMillis >= reconDelay))
    {
      WiFi.disconnect();
      WiFi.reconnect();
      previousReconMillis = currReconMillis;
    }

    if(WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client = server.available();   // Listen for incoming clients

      if (client) {                             // If a new client connects,
        currentTime = millis();
        previousTime = currentTime;
        Serial.println("New Client.");          // print a message out in the serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
          currentTime = millis();
          if (client.available()) {             // if there's bytes to read from the client,
            char c = client.read();             // read a byte, then
            Serial.write(c);                    // print it out the serial monitor
            header += c;
            if (c == '\n') {                    // if the byte is a newline character
              // if the current line is blank, you got two newline characters in a row.
              // that's the end of the client HTTP request, so send a response:
              if (currentLine.length() == 0) {
                // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                // and a content-type so the client knows what's coming, then a blank line:
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println("Connection: close");
                client.println();
                
                // turns the GPIOs on and off
                if (header.indexOf("GET /26/on") >= 0) {
                  currManualSwState = 1;
                  
                } else if (header.indexOf("GET /26/off") >= 0) {
                  currManualSwState = 0;
                  
                }
                
                // Display the HTML web page
                client.println("<!DOCTYPE html><html>");
                client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                client.println("<link rel=\"icon\" href=\"data:,\">");
                // CSS to style the on/off buttons 
                // Feel free to change the background-color and font-size attributes to fit your preferences
                client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
                client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
                client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
                client.println(".button2 {background-color: #555555;}</style></head>");
                
                // Web Page Heading
                client.println("<body><h1>Garden Light Control</h1>");
                
                // Display current state, and ON/OFF buttons
                if(currManualSwState)
                {
                  client.println("<p>Switch - State ON </p>");
                } 
                else{
                  client.println("<p>Switch - State OFF</p>");
                }
                
                // If the output26State is off, it displays the ON button       
                if (currManualSwState==0) {
                  client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
                } else {
                  client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
                } 
                  
                client.println("</body></html>");
                
                // The HTTP response ends with another blank line
                client.println();
                // Break out of the while loop
                break;
              } else { // if you got a newline, then clear currentLine
                currentLine = "";
              }
            } else if (c != '\r') {  // if you got anything else but a carriage return character,
              currentLine += c;      // add it to the end of the currentLine
            }
          }
        }
        // Clear the header variable
        header = "";
        // Close the connection
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
      }
    }
  }
}
#else
void WebServerTask(void* params)
{
  for(;;)
  {

  }
}
#endif
