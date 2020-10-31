/*
  ZE07CO_Sensor.cpp - ZE07-CO_Sensor library
  Developed by Fernando Fontes - 2020/02/14
*/

#include "ZE07CO.h"

#define ZE07Write(CHARVAL) mySerial->write(CHARVAL)
#define ZE07Read mySerial->read()
#define ZE07Available mySerial->available() 

//                            {start, gas, uni , deci, con H, con L, FUll range, Full range, check sum}
// default_uploud_message --> {0xFF, 0x04, 0x03, 0x01, 0x00, 0x25, 0x13, 0x88, 0x25};  //by default upload CO concentration every second
// co_ppm = ( (int) High Byte*256+ (int)Low Byte) * 1/(10^deci)

static unsigned char switch_to_question[]   = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
static unsigned char switch_to_initiative[] = {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};
static unsigned char request_reading[]      = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

ZE07CO::ZE07CO(Stream* Serial)
{
    mySerial = Serial;
}

/*
ZE07CO_Sensor::ZE07CO_Sensor(SoftwareSerial* Serial)	//read the uart signal by software uart,such as D10
{
    mySerial = Serial;
}
*/

ZE07CO::ZE07CO(int pin,float ref)
{ //read the analog signal by analog input pin ,such as A2; ref:voltage on AREF pin
    _sensorPin = pin;
    _ref = ref;											//for arduino uno ,the ref should be 5.0V(Typical)
}


void ZE07CO::initiative(){
    _mode = ZE07_MODE_INITIATIVE;
    uint8_t i;
    for (i = 0; i < ZE07_MESSAGE_LENGTH; i ++){
        ZE07Write(switch_to_initiative[i]);
    }
}


void ZE07CO::question(){
    _mode = ZE07_MODE_QUESTION;
    uint8_t i;
    for (i = 0; i < ZE07_MESSAGE_LENGTH; i ++){
        ZE07Write(switch_to_question[i]);
    }
}


float ZE07CO::read(){
    static uint8_t *response = receivedCommandStack;
    uint8_t bytes_read = 0, ch = 0;
    float coppm = ZE07_TIMEOUT; 

    // clear buffer
    memset(response,0,ZE07_MESSAGE_LENGTH);

    switch(_mode){
    case ZE07_MODE_INITIATIVE:
        _elapsed_time = millis();
        do{
            /* Try to receive from sensor until timeout*/
            while(ZE07Available){
                // ready recv byte
                ch = ZE07Read;
                // iterate
                if((ZE07_START_BYTE == ch) && (bytes_read == 0)){
                    response[bytes_read] = ch;
                    bytes_read++;
                }else if((ZE07_GAS_TYPE == ch) && (bytes_read == 1)){
                    response[bytes_read] = ch;
                    bytes_read++;
                }else if(bytes_read >= 2){
                    response[bytes_read] = ch;
                    bytes_read++;
                }
            }
        }while((bytes_read < ZE07_MESSAGE_LENGTH) && ((millis()-_elapsed_time) < 1200));

        if(bytes_read == ZE07_MESSAGE_LENGTH){
            if(response[ZE07_MESSAGE_LENGTH-1] == checkSum(response,ZE07_MESSAGE_LENGTH)){
                coppm = ((response[4]<<8) | response[5])*0.1;
            }else{
                coppm = ZE07_INV_CHECKSUM;
            }
        }
        break;
    case ZE07_MODE_QUESTION:
        // request new reading
        for (uint8_t i = 0; i < ZE07_MESSAGE_LENGTH; i ++){
            ZE07Write(request_reading[i]);
        }
        _elapsed_time = millis();
        do{
            /* Try to receive from sensor until timeout*/
            while(ZE07Available){
                // ready recv byte
                ch = ZE07Read;
                if((ZE07_START_BYTE == ch) && (bytes_read == 0)){
                    response[bytes_read] = ch;
                    bytes_read++;
                }else if((ZE07_COMMAND == ch) && (bytes_read == 1)){
                    response[bytes_read] = ch;
                    bytes_read++;
                }else if(bytes_read >= 2){
                    response[bytes_read] = ch;
                    bytes_read++;
                }
            }
        }while((bytes_read < ZE07_MESSAGE_LENGTH) && ((millis()-_elapsed_time) < 500));

        if(bytes_read == ZE07_MESSAGE_LENGTH){
            if(response[ZE07_MESSAGE_LENGTH-1] == checkSum(response,ZE07_MESSAGE_LENGTH)){
                coppm = ((response[2]<<8) | response[3])*0.1;
            }else{
                coppm = ZE07_INV_CHECKSUM;
            }
        }
        break;
    default:
        break;
    }

    return coppm;
}


byte ZE07CO::checkSum(byte array[],byte length){
    byte sum = 0;
    for(int i = 1; i < length-1; i ++){
        sum += array[i];
    }
    sum = (~sum) + 1;
    return sum;
}


float ZE07CO::dacReadPPM(){
    float analogVoltage = analogRead(_sensorPin) / 1024.0 * _ref;
    float ppm = (3.125 * analogVoltage - 1.25) * 100;	//linear relationship(0.4V for 0 ppm and 2V for 500ppm)
    if(ppm<0) ppm=0;
    else if(ppm>500) ppm = 500;
    return ppm;
}