/*******************************************************************
**File Name: main.c                                               **
**Library Name: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx.          **
**Original Project Name: Garden Light Software.                   **
**Author Name: Jeremiah A.                                        **
**Version No:  1.0                                                **
**Date Created: 13th March 2024.                                  **
**Last Edited: 22nd March 2024.                                   **
********************************************************************/


#include <gardenlighthw.h>
#include <millisDelay.h>


#define DEFAULT_DELAY_TIME      10000 // 10 seconds
#define DUMMY_LIGHT_SENSE       0
#define SWITCH_DEBOUNCE_DELAY   50    // 50 mS
#define PIR_SENSE_DEBOUNCE      25    //25 mS
#define LOOP_DEBUG_DELAY        3000 // 6 Seconds

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

// Declare state variables and flags
  uint8_t currManualSwState = 0;
  uint8_t manualSwReading = 0;
  uint8_t manualTimeoutFlag = 0;
  uint16_t manualDelayTime = 10000;
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

  uint8_t currKeypadState = 0;
  uint8_t keypadReading = 0;
  uint8_t lastKeypadReading = 0;

  uint8_t currMockSenseState = 0;
  uint8_t mockSenseReading = 0;
  uint8_t lastMockSenseReading = 0;

  uint16_t goToSleep __section(".noinit");

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

  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10000);
  while(esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, 0) != ESP_OK);
  Serial.println("Welcome, Starting Garden Light Control!!!");
  manualModeLamp.Activate();
  wifiConnectLamp.Activate();
  motionModeLamp1.Activate();
  motionSenseLamp.Activate();
  remoteSenseLamp.Activate();
  delay(10000);
   manualModeLamp.Deactivate();
  wifiConnectLamp.Deactivate();
  motionModeLamp1.Deactivate();
  motionSenseLamp.Deactivate();
  remoteSenseLamp.Deactivate();

  currKeypadState = keypadLock1.GetState();
  currManualTestState =manualTest.GetState();
  currMockSenseState = mockSensor.GetSignal();
  currPirSenseState = pirSensor1.GetSignal();
 
 // If go to sleep is set, start deep sleep
  if(goToSleep == 0xdd11)
  {
    goToSleep = 0;
    Serial.println("Going to sleep");
    esp_deep_sleep_start();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(lightSensor.GetSignal() && DUMMY_LIGHT_SENSE)
  {
    //goToSleep = 0xdd11;
    Serial.println("Sleep mode activated.. restarting system");
    //ESP.restart();
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
    else if(!currPirSenseState && motionLightActiveFlag)
    {
      if(!timeDelayStartFlag)
      {
        timeDelay.start(20000);
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
