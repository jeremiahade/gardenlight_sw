/*******************************************************************
**File Name: gardenlighthw.cpp                                    **
**Library Name: GardenlightHW.                                    **
**Original Project Name: Garden Light Software.                   **
**Author Name: Jeremiah A.                                        **
**Version No:  1.0                                                **
**Date Created: 13th March 2024.                                  **
**Last Edited: 22nd March 2024.                                   **
********************************************************************/

#include "gardenlighthw.h"


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
