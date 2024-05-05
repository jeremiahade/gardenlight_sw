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
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <HTTPUpdate.h>
#include <SD.h>
#include <FS.h>
#include <SPIFFS.h>
#include <string.h>

// TODO: Configure your WiFi here
#define TEST_WIFI 0
#define VARIABLE_WIFI_CREDENTIALS 1

#if TEST_WIFI
  #define WIFI_SSID "Freeloadersunited"
  #define WIFI_PSK  "klak8385"

#else
  #define WIFI_SSID "VM1628887"
  #define WIFI_PSK  "jeojc7gwbnPkvzcg"
#endif

char ssid[] = "VM1628887";     // your network SSID (name)
char password[] = "jeojc7gwbnPkvzcg"; // your network key

#define WIFI_INIT_CONNECT_TIMEOUT 6   // 3 Seconds 500ms x6

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
#define TEST_MANUAL_DELAY       0     // is high when test manual state is active

#define SIMPLE_HTTP_SERVER      1     //simple http server active if 1
#define CUSTOM_WEB_SERVER       0     // custom webserver (1) or telegram (0)

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
    uint32_t manualDelayTime = 480000;  // 8 minutes delay
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
  uint8_t gotToSleepActive = 0;
  uint8_t goToSleepCount = 100;

// Wifi and webserver variables
// Set web server port number to 80
  WiFiServer server(80);
  unsigned long previousReconMillis = 0;
  unsigned long currReconMillis = 0;
  unsigned long reconDelay = 10000;
  uint8_t  initWifiConnectFlag = 0;

// Variable to store the HTTP request
  String header;
// Current time
  unsigned long currentTime = 0;
// Previous time
  unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
  const long timeoutTime = 2000;

// Telegram BOT API declarations
  #define BOTtoken            "6634609338:AAFCEWkTWJjriU3gBG8FR8dREXO3BOd4jBM"  // your Bot Token (Get from Botfather)
  #define GROUP_CHAT_ID       "-4238882024"
  #define CHAT_ID             "7095720678"
  #define TELEBOT_DEBUG       1
  #define LIGHT_ON_CALL       "LIGHT_ON"
  #define LIGHT_OFF_CALL      "LIGHT_OFF"
  #define TIMER_UPDATE_CALL   "TIMER_UPDATE"
  #define FW_UPDATE_CALL      "FIRMWARE_UPDATE"
  #define WIFI_UPDATE_CALL    "WIFI_UPDATE"
  #define CONTROLLER_UPDATE   "CONTROLLER_UPDATE"
  #define FWA_FILENAME_LENGTH     10
  WiFiClientSecure clientChat;
  WiFiClientSecure clientFW;
  UniversalTelegramBot botchat(BOTtoken, clientChat);
  UniversalTelegramBot botFW(BOTtoken, clientFW);
  int Bot_mtbs = 20; //mean time between scan messages in milli seconds
  long Bot_lasttime = 0;   //last time messages' scan has been done
  int numNewChatMessages = 0;
  int numNewFwMessages = 0;
  String messageText = "";
  String fwFilename = "defaulter";
  uint8_t fwUpdateFlag = 0;
  uint8_t fwUploadFlag = 0;
  uint8_t wifiSSidUpdateFlag = 0;
  uint8_t wifiPsswrdUpdateFlag = 0;
  uint8_t motionAlertState = 0;
  uint8_t manualAlertState = 0;

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

#if CUSTOM_WEB_SERVER
  /**************************************************************************/
/*!
    @brief This is the web server FreeRTOS task function.  
*/
/**************************************************************************/
void WebServerTask(void* params); 
#else 
/*!
    @brief This the telegram client FreeRTOS task function
*/
void TelegramBotClientTask(void* params);

void HandleNewMessages(void);
#endif
String gen_random(const int len);
void SendStateAlert(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);
  #if DEBUG
  delay(10000);
  #endif
  while(esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, 1) != ESP_OK);

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
  TaskHandle_t telegramTaskHandle = NULL;
 
 // If go to sleep is set, start deep sleep
  if(goToSleep == 0xdd11)
  {
    goToSleep = 0;
    Serial.println("Going to sleep");
    esp_deep_sleep_start();
  }
  Serial.println("Welcome, Starting Garden Light Control!!!");
  lightSenseState = lightSensor.GetSignal();
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }
  gotToSleepActive = 0;
  // Set random value to firmware file name
    fwFilename = gen_random(FWA_FILENAME_LENGTH);
  //Start wifi client for telegram bot
    clientFW.setInsecure();
    clientChat.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  // Connect to Wi-Fi network with SSID and password
  uint8_t wifiConnectCount = 0;
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  #if !DEBUG
    WiFi.mode(WIFI_STA);
    #if !VARIABLE_WIFI_CREDENTIALS
      WiFi.begin(WIFI_SSID, WIFI_PSK);
    #else
      WiFi.begin(ssid, password);
    #endif
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      wifiConnectCount++;
      Serial.print(".");
      if(wifiConnectCount > WIFI_INIT_CONNECT_TIMEOUT)
      {
        break;
      }
    }
    // Print local IP address and start web server
    if(WiFi.status() == WL_CONNECTED)
    {
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      initWifiConnectFlag = 1;
    }
    #if CUSTOM_WEB_SERVER
    server.begin();
      xTaskCreatePinnedToCore(WebServerTask, "http80", 3000, NULL, 1, &wifiTaskHandle, TASK_CORE);
    #else
      String grpKeyboardJson = "[[\"/gardenmenu\"]]";
      String jayKeyboardJson = "[[\"/gardenmenu\"], [\"/settingsmenu\"]]";
      botchat.sendMessageWithReplyKeyboard(CHAT_ID,"Hello Jay, I am now awake.\n select the reply below or \ntype /gardenmenu or \n/settingsmenu for desired options", "",jayKeyboardJson, true);
      botchat.sendMessageWithReplyKeyboard(GROUP_CHAT_ID,"Hello everyone, I am now awake.\n send /gardenmenu to view available options", "",grpKeyboardJson, true);
      xTaskCreatePinnedToCore(TelegramBotClientTask, "telegrambot", 8000, NULL, 1, &telegramTaskHandle, TASK_CORE);

    #endif
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
    gotToSleepActive = 1;
    delay(2000);
    #if !CUSTOM_WEB_SERVER
    botchat.sendMessage(CHAT_ID,"I am going to sleep now, BYE BYE");
    botchat.sendMessage(GROUP_CHAT_ID, "It is day break, I need to sleep,\n BYE FOR NOW");
    delay(2000);
    #endif
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

#if CUSTOM_WEB_SERVER
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
        if(!initWifiConnectFlag)
        {
          Serial.println("");
          Serial.println("WiFi connected.");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
          initWifiConnectFlag = 1;
        }
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
#else

  void HandleNewMessages(void)
  {
    
    for (int i = 0; i < numNewChatMessages; i++) 
    {
      if(botchat.messages[i].chat_id != CHAT_ID && botchat.messages[i].chat_id != GROUP_CHAT_ID )
      {
        botchat.sendMessage(botchat.messages[i].chat_id, "You are not allowed here bucker, GTFOH!!");
        continue;
      }
      if (botchat.messages[i].type == "callback_query")
      {
       if(botchat.messages[i].chat_id == GROUP_CHAT_ID)
       {
          if(botchat.messages[i].text == LIGHT_ON_CALL)
          {
            if(currManualSwState == 1)
            {
              botchat.sendMessage(botchat.messages[i].chat_id, "I am already turned on");
            }
            else
            {
              currManualSwState = 1;
              botchat.sendMessage(botchat.messages[i].chat_id, "hold on buddy, Turning on light");
              #if TELEBOT_DEBUG
                Serial.println("bot turn on garden light");
              #endif
            } 
          }
          else if (botchat.messages[i].text == LIGHT_OFF_CALL)
          {
            if(currManualSwState == 0)
            {
              botchat.sendMessage(botchat.messages[i].chat_id, "I am already turned off");
            }
            else
            {
               currManualSwState = 0;
               botchat.sendMessage(botchat.messages[i].chat_id, "hold on buddy, Turning off light");
              #if TELEBOT_DEBUG
                Serial.println("bot turn off garden light");
              #endif
            } 
          }
        }
        if(botchat.messages[i].chat_id == CHAT_ID)
        {
         if(botchat.messages[i].text == LIGHT_ON_CALL)
          {
            if(currManualSwState == 1)
            {
              botchat.sendMessage(botchat.messages[i].chat_id, "I am already turned on");
            }
            else
            {
               currManualSwState = 1;
               botchat.sendMessage(botchat.messages[i].chat_id, "hold on buddy, Turning on light");
              #if TELEBOT_DEBUG
                Serial.println("bot turn on garden light");
              #endif
            } 
          }
          else if (botchat.messages[i].text == LIGHT_OFF_CALL)
          {
            if(currManualSwState == 0)
            {
              botchat.sendMessage(botchat.messages[i].chat_id, "I am already turned off");
            }
            else
            {
              currManualSwState = 0;
              botchat.sendMessage(botchat.messages[i].chat_id, "hold on buddy, Turning off light");
              #if TELEBOT_DEBUG
                Serial.println("bot turn off garden light");
              #endif
            } 
          }
          else if(botchat.messages[i].text == TIMER_UPDATE_CALL)
          {
            // String menuKeyboardJson = "[[{\"text\": \"TURN ON\", \"callback_data\": \"LIGHT_ON\"}],
            String timerKeyboardJson = "[[{\"text\": \"Manual Timer\", \"callback_data\": \"MANUAL_TIMER\"}], [{\"text\": \"Motion Sense Timer\", \"callback_data\": \"MOTION_TIMER\"}]]";
            botchat.sendMessageWithInlineKeyboard(botchat.messages[i].chat_id, "Tik Toc says the timer menu", "", timerKeyboardJson);
          }
          else if (botchat.messages[i].text == WIFI_UPDATE_CALL)
          {
            // to be completed
          }
          else if(botchat.messages[i].text == FW_UPDATE_CALL)
          {
            String fwKeyboardJson = "[[{\"text\": \"Update Controller Firmware\", \"callback_data\": \"CONTROLLER_UPDATE\"}]]";
            botchat.sendMessageWithInlineKeyboard(botchat.messages[i].chat_id, "my programming not good enough for you?", "", fwKeyboardJson);
          }
          else if (botchat.messages[i].text == CONTROLLER_UPDATE)
          {
            fwUpdateFlag = 1;
            botchat.sendMessage(botchat.messages[i].chat_id, "Provide desired full firmware filename");
          }    
        }
      }
      else
      {
        if(fwUpdateFlag)
        {
          String buffFileName = botchat.messages[i].text;
          if (botchat.messages[i].chat_id == CHAT_ID)
          {
            if(buffFileName.endsWith(".bin"))
            {
              fwFilename = buffFileName;
              botchat.sendMessage(CHAT_ID, "Firmware name successfully registered.\n Please upload file");
              fwUpdateFlag = 0;
              fwUploadFlag = 1;
            }
            else
            {
              botchat.sendMessage(CHAT_ID, "Invalid File name");
              fwUpdateFlag = 0;
            }
          }
        }
        if(botchat.messages[i].text == "/start")
        {
          String welcome = "";
          if(botchat.messages[i].from_name == "")
          {
            welcome = "Hello guest\n";
          }
          else
          {
            welcome = "welcome "+botchat.messages[i].from_name+"\n";
          }
          welcome += "select the /gardenmenu option below or type /gardenmenu to view options/n";
          String WelcomekeyboardJson = "[[\"/gardenmenu\"]]";
          botchat.sendMessageWithReplyKeyboard(botchat.messages[i].chat_id,welcome,"",WelcomekeyboardJson, true);
        }
        else if(botchat.messages[i].text == "/gardenmenu")
        {
            //String keyboardJson = "[[{ \"text\" : \"Go to Google\", \"url\" : \"https://www.google.com\" }],[{ \"text\" : \"Send\", \"callback_data\" : \"This was sent by inline\" }]]";
          // bot.sendMessageWithInlineKeyboard(chat_id, "Choose from one of the following options", "", keyboardJson); 
          String menuKeyboardJson = "[[{\"text\": \"TURN ON\", \"callback_data\": \"LIGHT_ON\"}],[{\"text\": \"TURN OFF\", \"callback_data\": \"LIGHT_OFF\"}]]";  
          botchat.sendMessageWithInlineKeyboard(botchat.messages[i].chat_id, "Here is your menu, Bon Appetit!", "", menuKeyboardJson);      
        }
        else if(botchat.messages[i].text == "/settingsmenu")
        {
          if(botchat.messages[i].chat_id == CHAT_ID)
          {
            String settingsKeyboardJson = "[[{\"text\": \"Update Timer\", \"callback_data\": \"TIMER_UPDATE\"}], [{\"text\": \"Update Firmware\", \"callback_data\": \"FIRMWARE_UPDATE\"}], [{\"text\": \"Update WiFi Info\", \"callback_data\": \"WIFI_UPDATE\"}]]";
            botchat.sendMessageWithInlineKeyboard(botchat.messages[i].chat_id, "What would like changed Sir", "", settingsKeyboardJson);
          }
          else
          {
            botchat.sendMessage(botchat.messages[i].chat_id, "Invalid Command");
          }
        }
      }
      if (fwUploadFlag)
      {
        if (botFW.messages[i].type == "message") 
        {
          if (botFW.messages[i].hasDocument == true)
          {
            httpUpdate.rebootOnUpdate(false);
            t_httpUpdate_return ret = (t_httpUpdate_return)3;
            if (botFW.messages[i].file_caption == "update firmware")
            {
              botFW.sendMessage(botFW.messages[i].chat_id, "Firmware writing...", "");
              ret = httpUpdate.update(clientFW, botFW.messages[i].file_path);
              fwFilename = gen_random(FWA_FILENAME_LENGTH);
              fwUploadFlag = 0;
              fwUpdateFlag = 0;   
            }
            switch (ret)
            {
              case HTTP_UPDATE_FAILED:
                botFW.sendMessage(botFW.messages[i].chat_id, "HTTP_UPDATE_FAILED Error (" + String(httpUpdate.getLastError()) + "): " + httpUpdate.getLastErrorString(), "");
                break;

              case HTTP_UPDATE_NO_UPDATES:
                botFW.sendMessage(botFW.messages[i].chat_id, "HTTP_UPDATE_NO_UPDATES", "");
                break;

              case HTTP_UPDATE_OK:
                botFW.sendMessage(botFW.messages[i].chat_id, "UPDATE OK.\nRestarting...", "");
                //numNewFwMessages = botFW.getUpdates(botFW.last_message_received + 1);
                ESP.restart();
                break;
              default:
                break;
            }
          }
        }     
      }   
    }
  }

  void TelegramBotClientTask(void* params)
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
        if(!initWifiConnectFlag)
        {
          Serial.println("");
          Serial.println("WiFi connected.");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
          initWifiConnectFlag = 1;
        }
      }
      if((WiFi.status() != WL_CONNECTED) && (currReconMillis - previousReconMillis >= reconDelay))
      {
        WiFi.disconnect();
        WiFi.reconnect();
        previousReconMillis = currReconMillis;
      }

      if(WiFi.status() == WL_CONNECTED)
      {
        SendStateAlert();
        
        if (!gotToSleepActive)
        {
          
          if (millis() > Bot_lasttime + Bot_mtbs)  
          {
            numNewChatMessages = botchat.getUpdates(botchat.last_message_received + 1);
            numNewFwMessages = botFW.getUpdates(botFW.last_message_received + 1);
            #if DEBUG
                Serial.printf("bot1message: %d \n",newnum1);
                Serial.printf("bot2message: %d \n\n",newnum2);
            #endif
          
            if(numNewFwMessages > 0 && numNewChatMessages == 0)
            {
              numNewChatMessages = botchat.getUpdates(botchat.last_message_received + 1);
              #if DEBUG
                Serial.println("other 1 update\n");
                Serial.printf("bot1message: %d \n",newnum1);
                Serial.printf("bot2message: %d \n\n",newnum2);
              #endif
            }
            if(numNewChatMessages > 0 && numNewFwMessages == 0)
            {
              numNewFwMessages = botFW.getUpdates(botFW.last_message_received + 1);
              #if DEBUG
              Serial.println("other 2 update\n");
              #endif
            }
            
            while (numNewChatMessages && numNewFwMessages)
            {
              HandleNewMessages();
              numNewChatMessages = botchat.getUpdates(botchat.last_message_received + 1);
              numNewFwMessages = botFW.getUpdates(botFW.last_message_received + 1);
              #if DEBUG
                Serial.printf("bot1message: %d \n",newnum1);
                Serial.printf("bot2message: %d \n\n",newnum2);
              #endif

              if(numNewFwMessages > 0 && numNewChatMessages == 0)
              {
                numNewChatMessages = botchat.getUpdates(botchat.last_message_received + 1);
                #if DEBUG
                  Serial.println("other 1 update\n");
                  Serial.printf("bot1message: %d \n",newnum1);
                  Serial.printf("bot2message: %d \n\n",newnum2);
                #endif
              }
              if(numNewChatMessages > 0 && numNewFwMessages == 0)
              {
                numNewFwMessages = botFW.getUpdates(botFW.last_message_received + 1);
                #if DEBUG
                Serial.println("other 2 update\n");
                #endif
              }
            }
            Bot_lasttime = millis();   
          }
        }
      }
    }
  }

  String gen_random(const int len) 
  {
    static const char alphanumLUT[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    String tmp_s;
    tmp_s.reserve(len);
    
    for (int i = 0; i < len; ++i) {
        tmp_s += alphanumLUT[rand() % (sizeof(alphanumLUT) - 1)];
    }
    
    return tmp_s;
  }

  void SendStateAlert(void)
  {
    if (motionAlertState != motionLightActiveFlag)
    {
      motionAlertState = motionLightActiveFlag;
      if (motionLightActiveFlag)
      {
        botchat.sendMessage(CHAT_ID, "Motion detected.\n Garden Light is On");
        botchat.sendMessage(GROUP_CHAT_ID, "Motion detected.\n Garden Light is On");
      }
      else if((!motionLightActiveFlag) && (!manualActiveFlag))
      {
        botchat.sendMessage(CHAT_ID, "Garden Light is off");
        botchat.sendMessage(GROUP_CHAT_ID, "Garden Light is off");
      }
    }
    else if (manualAlertState != manualActiveFlag)
    {
      manualAlertState = manualActiveFlag;
      if (manualAlertState)
      {
        botchat.sendMessage(CHAT_ID, "I was told to turn on Light.\nLight is on");
        botchat.sendMessage(GROUP_CHAT_ID, "I was told to turn on Light.\nLight is on");
      }
      else if((!motionLightActiveFlag) && (!manualActiveFlag))
      {
        botchat.sendMessage(CHAT_ID, "Garden Light is off");
        botchat.sendMessage(GROUP_CHAT_ID, "Garden Light is off");
      }
    }  
  }
#endif
