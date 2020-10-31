/*
  ZE07CO_Sensor.cpp - ZE07-CO_Sensor library
  Developed by Fernando Fontes - 2020/02/14
*/


#ifndef __ZE07CO_H_
#define __ZE07CO_H_

//#include <SoftwareSerial.h>

#include <Arduino.h>
#include <stdint.h>

#define ZE07_CO_BAUD_RATE       9600
#define ZE07_CO_DATA_BITS       8
#define ZE07_CO_STOP_BITS       1
#define ZE07_MESSAGE_LENGTH     9

#define ZE07_START_BYTE 0xFF
#define ZE07_GAS_TYPE 0X04
#define ZE07_COMMAND 0X86

#define ZE07_TIMEOUT -1
#define ZE07_INV_CHECKSUM -2

class ZE07CO
{
    public:
    ZE07CO(Stream* Serial);	//read the uart signal by hardware uart,such as D0
    //ZE07CO_Sensor(SoftwareSerial* Serial);	//read the uart signal by software uart,such as D10
    ZE07CO(int pin,float ref);			//read the analog signal by analog input pin ,such as A2; ref:voltage on AREF pin
    
    /*
    * ZE07read()
    * Brief: Reads the CO value from the UART
    * Returns: ZE07_TIMEOUT on timeout, otherwise returns the ppm value multiplied by 10.
    */
    float read();

    /*
    * ZE07question()
    * Brief: Change the CO sensor to question mode
    */
    void question();

    /*
    * ZE07initiative()
    * Brief: Change the CO sensor to initiative mode, meaning that the sensor will send values at a periodicity of 1 second
    */
    void initiative();

    /*
    * ZE07dacReadPPM()
    * Brief: Reads the CO value from the ADC
    */
    float dacReadPPM();		//get the concentration(ppm) by analog signal

    private:
    Stream* mySerial;
    byte receivedCommandStack[ZE07_MESSAGE_LENGTH];
    byte checkSum(byte array[],byte length);
    byte _sensorPin;
    enum ZE07_MODE { 
        ZE07_MODE_INITIATIVE = 1, 
        ZE07_MODE_QUESTION = 2 
    };
    ZE07_MODE _mode = ZE07_MODE_QUESTION;
    float _ref;
    unsigned long _elapsed_time = 0;
};

#endif