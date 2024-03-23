/*******************************************************************
**File Name: gardenlighthw.h                                     **
**Library Name: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx.          **
**Original Project Name: Null.                                    **
**Author Name: Jeremiah A.                                        **
**Version No:  2.0                                                **
**Date Created: 13th March 2024.                                  **
**Last Edited: 29th August 2020.                                  **
********************************************************************/

#ifndef GARDENLIGHTHW_H
#define GARDENLIGHHW_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

#define RELAY_ON		1
#define RELAY_OFF		0
#define DEBUG			1
#define APP_START       0
#define SYSTEM_INFO		1
#define TEST_PARAM		0
#define TEST_PIR		1
#define APP_PIR			0

#define WIFICON_LAMP            5       // Indicates WIFI connection status
#define MANUAL_MODE_LAMP        13      // Indicates manual mode state
#define MOTION_MODE_LAMP        14      // Indicates motion mode state
#define REMOTE_SENSOR_LAMP      16      // Indicates remote sensor motion detection state
#define MOTION_SENSOR_LAMP      17      // Indicates motion sensor motion detection state
#define MODE_SWITCH             32      // Manual switch between auto and manual (deprecated)
#define MOTION_SENSOR           22      // Motion sensor input signal
#define DIOEX2                  23      // Extra Digital I/O Port 2
#define GARDEN_LIGHT            25      // Floodlight output switch
#define DIOEX1                  26      // Extra Digital I/O Port 1
#define MOCK_SENSOR             27      // Test sensor button input
#define KEYPAD_LOCK             33      // Onboard Keypad lock switch
#define MANUAL_TEST_SW          18      // Manual test switch

// GPIO Pins and usage declarations


class Sensor{
    private:
        String name;
        uint8_t gpioPin;
        int status;
    public:
        Sensor(String Name, int pinNumber);
        int GetSignal(void);
        void PrintStatus(void);
};

class FloodLight{
    private:
        uint8_t gpioPin;
        String name;
        uint8_t status;
    public:
        FloodLight(String Name, int pinNumber);
        void TurnOn(void);
        void TurnOff(void);
        uint8_t GetStatus(void);
        void PrintStatus(void);
};

class Indicator{
    private:
        uint8_t gpioPin;
        String name;
        uint8_t status;
    public:
        Indicator(String Name, uint8_t pinNumber);
        void Activate(void);
        void Deactivate(void);
        uint8_t GetStatus(void);
};

class Switch{
    private:
        String name;
        uint8_t gpioPin;
        int status;
    public:
        Switch(String Name, uint8_t pinNumber);
        int GetState(void);
};


#ifdef __cplusplus
}
#endif

#endif 