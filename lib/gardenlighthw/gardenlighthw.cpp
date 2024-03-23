/*******************************************************************
**File Name: xxxxxxxxxxxx.h/c                                     **
**Library Name: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx.          **
**Original Project Name: Null.                                    **
**Author Name: Jeremiah A.                                        **
**Version No:  2.0                                                **
**Date Created: 13th March 2024.                                  **
**Last Edited: 29th August 2020.                                  **
********************************************************************/


#include "gardenlighthw.h"

gpio_num_t wifiConIndicator = GPIO_NUM_5;			//Output
gpio_num_t manualModeActive = GPIO_NUM_13;		    //Output
gpio_num_t motionModeLamp = GPIO_NUM_14;		    //Output
gpio_num_t remoteSensorLamp = GPIO_NUM_16;		    //Output
gpio_num_t motionSensorLamp = GPIO_NUM_17;			//Output
gpio_num_t manualSwitchBtnPin = GPIO_NUM_18;		//Input

gpio_num_t ldrControlBtnPin = GPIO_NUM_19;			//Input (deprecated, to be commented out)

gpio_num_t modeSwitch = GPIO_NUM_32;	            //Input (deprecated, to be commented out)
gpio_num_t motionRelayPin = GPIO_NUM_21;			//Output (deprecated, to be commented out)
gpio_num_t motionSensor = GPIO_NUM_22;			    //Input
gpio_num_t dioex2 = GPIO_NUM_23;				    //Input
gpio_num_t gardenlight = GPIO_NUM_25;			    //Output

gpio_num_t MockSensor = GPIO_NUM_27;				//Input
gpio_num_t keypadLock = GPIO_NUM_33;				//Input
gpio_num_t manualTestSw = GPIO_NUM_18;               //Input
gpio_num_t dioex1 = GPIO_NUM_26;		            //Output


/**************************************************************************/
/*!
    @brief  Sensor class constructor function for initialising sensor GPIO
            pin, and getting sensor name
    @param  Name, the sensor name
    @param  pinNumber, the gpio pin number assigned to the sensor
*/
/**************************************************************************/
Sensor::Sensor(String Name, int pinNumber){
    name = Name;
    gpioPin = pinNumber;   
    pinMode(gpioPin, INPUT_PULLUP);
    #if DEBUG
    Serial.println(name+ " sensor is initialised successfully");
    #endif
}
/**************************************************************************/
/*!
    @brief  Get sensor reading/state from the GPIO and store the reading
            in the status variable.
    @return integer sensor reading/state 
*/
/**************************************************************************/
int Sensor::GetSignal(void){
    status =digitalRead(gpioPin);
    #if DEBUG
    Serial.println(name + " sensor status is "+ status);
    #endif
    return status;
}

/**************************************************************************/
/*!
    @brief  Floodlight class constructor function for initialising sensor GPIO
            pin, and getting sensor name
    @param  Name, the floodlight name
    @param  pinNumber, the gpio pin number assigned to the floodlight
*/
/**************************************************************************/
FloodLight::FloodLight(String Name, int pinNumber){
    name = Name;
    gpioPin = pinNumber;
    pinMode(gpioPin, OUTPUT);
    #if DEBUG
    Serial.println(name+ " is initialised successfully");
    #endif
}

/**************************************************************************/
/*!
    @brief  Turn on the floodlight
*/
/**************************************************************************/
void FloodLight::TurnOn(void){
    digitalWrite(gpioPin, HIGH);
    status = 1;
    #if DEBUG
    Serial.println(name + " floodlight status is "+ status);
    #endif
}

/**************************************************************************/
/*!
    @brief  Turn on the floodlight
*/
/**************************************************************************/
void FloodLight::TurnOff(void){
    digitalWrite(gpioPin, LOW);
    status = 0;
    #if DEBUG
    Serial.println(name + " floodlight status is "+ status);
    #endif
}

/**************************************************************************/
/*!
    @brief  Returns the floodlight current status (High or Low)
    @return 8-bit unsigned integer floodlight status
*/
/**************************************************************************/
uint8_t FloodLight::GetStatus(void){
    return status;
}

/**************************************************************************/
/*!
    @brief  Prints the floodlight current status through the UART Serial
            interface. 
*/
/**************************************************************************/
void FloodLight::PrintStatus(void){
    if(status)
    {
        Serial.println(name + "is switched on");
    }
    else{
        Serial.println(name + "is switched off");
    }
}

/**************************************************************************/
/*!
    @brief  Prints the Sensor current state through the UART Serial
            interface. 
*/
/**************************************************************************/
void Sensor::PrintStatus(void){
    if(status)
    {
        Serial.println(name + "is Active");
    }
    else{
        Serial.println(name + "is not active");
    }
}

/**************************************************************************/
/*!
    @brief  Indicator lamp class constructor function for initialising 
            indicator GPIO pin, and getting indicator name
    @param  Name, the indicator lamp name
    @param  pinNumber, the gpio pin number assigned to the indicator
*/
/**************************************************************************/
Indicator::Indicator(String Name, uint8_t pinNumber){
    name = Name;
    gpioPin = pinNumber;
    pinMode(gpioPin, OUTPUT);
    #if DEBUG
    Serial.println(name+ " Indicator is initialised successfully");
    #endif
}

/**************************************************************************/
/*!
    @brief  Turn on the indicator lamp
*/
/**************************************************************************/
void Indicator::Activate(void){
    digitalWrite(gpioPin, HIGH);
    status = 1;
    #if DEBUG
    Serial.println(name + " indicator status is "+ status);
    #endif
}

/**************************************************************************/
/*!
    @brief  Turn off the indicator lamp
*/
/**************************************************************************/
void Indicator::Deactivate(void){
    digitalWrite(gpioPin, LOW);
    status = 0;
    #if DEBUG
    Serial.println(name + " indicator status is "+ status);
    #endif
}

/**************************************************************************/
/*!
    @brief  Returns the indicator lamp current status (High or Low)
    @return 8-bit unsigned integer indicator status
*/
/**************************************************************************/
uint8_t Indicator::GetStatus(void){
    return status;
}

/**************************************************************************/
/*!
    @brief  Switch class constructor function for initialising 
            switch GPIO pin, and getting indicator name
    @param  Name, the switch name
    @param  pinNumber, the gpio pin number assigned to the switch
*/
/**************************************************************************/
Switch::Switch(String Name, uint8_t pinNumber){
    name = Name;
    gpioPin = pinNumber;
    pinMode(gpioPin, INPUT);
    #if DEBUG
    Serial.println(name+ " switch is initialised successfully");
    #endif
}

/**************************************************************************/
/*!
    @brief  Get switch reading/state from the GPIO and store the reading
            in the status variable.
    @return integer switch reading/state 
*/
/**************************************************************************/
int Switch::GetState(void){
    status = digitalRead(gpioPin);
    #if DEBUG
        Serial.println("getting "+name+" state");
        if (status)
        {
            /* code */
            Serial.println(name+ " switch status is high");
        }
        else
        {
            /* code */
            Serial.println(name+ " switch status is low");
        }
        
    #endif
    return(status);
}


/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
esp_err_t Initialise_PIR(gpio_num_t pin)
{
	esp_err_t status;
	gpio_reset_pin(pin);
	status = gpio_set_direction(pin, GPIO_MODE_INPUT);
	status = gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
	status = gpio_pullup_en(pin);
	return status;
}

uint8_t Get_PIRState(gpio_num_t pir1, gpio_num_t pir2, gpio_num_t pir3)
{
	uint8_t pirState = 0;
	pirState |= gpio_get_level(pir1);
	pirState |= gpio_get_level(pir2);
	pirState |= gpio_get_level(pir3);

#if DEBUG
	printf("PIR state is %d\n\r", pirState);
#endif

	return pirState;
}

uint8_t Get_1PIRState(gpio_num_t pir)
{
	uint8_t pirState = 0;
	pirState = gpio_get_level(pir);

#if DEBUG
	printf("Test PIR state is %d\n\r", pirState);
#endif

	return pirState;
}

esp_err_t Initialise_RelaySwitch(gpio_num_t pin)
{
	esp_err_t status;
	gpio_reset_pin(pin);
	gpio_set_direction(pin, GPIO_MODE_OUTPUT);
	status = gpio_set_level(pin, RELAY_OFF);
#if 0
	if (status == ESP_OK)
	{
		printf("Relay is Initialised\n\r");
	}
	else
	{
		printf("error turning initialising relay\n\r");
	}

#endif

	return status;
}

esp_err_t SwitchOn_Relay(gpio_num_t relay)
{
	esp_err_t status;
	status = gpio_set_level(relay, RELAY_ON);
#if 0
	if (status == ESP_OK)
	{
		printf("Relay is On\n\r");
	}
	else
	{
		printf("error turning on relay\n\r");
	}

#endif
	return status;
}

esp_err_t SwitchOff_Relay(gpio_num_t relay)
{
	esp_err_t status;
	status = gpio_set_level(relay, RELAY_OFF);
#if 0
	if (status == ESP_OK)
	{
		printf("Relay is Off\n\r");
	}
	else
	{
		printf("error turning off relay\n\r");
	}

#endif
	return status;
}

esp_err_t Initialise_GPIO_Input(gpio_num_t pin)
{
	esp_err_t status;
	gpio_reset_pin(pin);
	status = gpio_set_direction(pin, GPIO_MODE_INPUT);
	status = gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
	status = gpio_pullup_en(pin);
	return status;
}

uint8_t GetGPIOInput(gpio_num_t gpio)
{
	uint8_t pirState = 0;
	pirState = gpio_get_level(gpio);
	return pirState;
}

esp_err_t Initialise_Bypass_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status= Initialise_RelaySwitch(gardenlight);
	return status;
}
esp_err_t SwitchOn_Bypass_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(gardenlight);
    return status;
}
esp_err_t SwitchOff_Bypass_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(gardenlight);
    return status;
}

esp_err_t Initialise_Feedback_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(motionSensor);
    return status;

}
uint8_t Get_Feedback_Relaystate (void)
{
    uint8_t state;
    state = GetGPIOInput(motionSensor);
    return state;
}

esp_err_t Initialise_LDR_Sense(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(dioex2);
    return status;
}

uint8_t Get_LDR_Sense(void)
{
    uint8_t state;
    state = GetGPIOInput(dioex2);
    return state;
}

esp_err_t Initialise_Manual_Switch_Btn(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(manualSwitchBtnPin);
    return status;
}

uint8_t Get_Manual_Switch_Btn(void)
{
    uint8_t state;
    state = GetGPIOInput(manualSwitchBtnPin);
    return state;
}

esp_err_t Initialise_System_State_Btn(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(modeSwitch);
    return status;
}

uint8_t Get_System_State_Btn(void)
{
    uint8_t state;
    state = GetGPIOInput(modeSwitch);
    return state;
}

esp_err_t Initialise_LDR_Sense_Switch_Btn(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(ldrControlBtnPin);
    return status;
}

uint8_t Get_LDR_Sense_Switch_Btn(void)
{
    uint8_t state;
    state = GetGPIOInput(ldrControlBtnPin);
    return state;
}

esp_err_t Initialise_Test_PIR_Btn(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(MockSensor);
    return status;
}

uint8_t Get_Test_PIR_Btn(void)
{
    uint8_t state;
    state = GetGPIOInput(MockSensor);
    return state;
}

esp_err_t Initialise_KeyLock_Btn(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_GPIO_Input(keypadLock);
    return status;
}

uint8_t Get_KeyLock_Btn(void)
{
    uint8_t state;
    state = GetGPIOInput(keypadLock);
    return state;
}

esp_err_t Initialise_Motion_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(motionRelayPin);
    return status;
}
esp_err_t SwitchOn_Motion_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(motionRelayPin);
    return status;
}
esp_err_t SwitchOff_Motion_Relay(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(motionRelayPin);
    return status;
}

esp_err_t Initialise_WIFI_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(wifiConIndicator);
    return status;
}
esp_err_t SwitchOn_WIFI_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(wifiConIndicator);
    return status;
}
esp_err_t SwitchOff_WIFI_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(wifiConIndicator);
    return status;
}

esp_err_t Initialise_Light_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(dioex1);
    return status;
}
esp_err_t SwitchOn_Light_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(dioex1);
    return status;
}
esp_err_t SwitchOff_Light_Stat_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(dioex1);
    return status;
}

esp_err_t Initialise_Manual_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(manualModeActive);
    return status;
}
esp_err_t SwitchOn_Manual_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(manualModeActive);
    return status;
}
esp_err_t SwitchOff_Manual_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(manualModeActive);
    return status;
}

esp_err_t Initialise_MD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(motionModeLamp);
    return status;
}
esp_err_t SwitchOn_MD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(motionModeLamp);
    return status;
}
esp_err_t SwitchOff_MD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(motionModeLamp);
    return status;
}

esp_err_t Initialise_RMD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(remoteSensorLamp);
    return status;
}
esp_err_t SwitchOn_RMD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(remoteSensorLamp);
    return status;
}
esp_err_t SwitchOff_RMD_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(remoteSensorLamp);
    return status;
}

esp_err_t Initialise_LDR_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = Initialise_RelaySwitch(motionSensorLamp);
    return status;
}
esp_err_t SwitchOn_LDR_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOn_Relay(motionSensorLamp);
    return status;
}
esp_err_t SwitchOff_LDR_State_LED(void)
{
    esp_err_t status = ESP_FAIL;
    status = SwitchOff_Relay(motionSensorLamp);
    return status;
}

