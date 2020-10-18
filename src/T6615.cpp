/*
 * T6615.h
 *
 *  Created on: 10/02/2020
 *      Author: Fernando Fontes
 *
 * Brief:
 *  This library can be used to read the CO2 sensor T6615
 */

#include "T6615.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define T6615Write(CHARVAL) mySerial->write(CHARVAL)
#define T6615Read mySerial->read()
#define T6615Available mySerial->available() 

char SEND_START_OF_FRAME[] = {0xFF,0xFE,'\0'};
char RECV_START_OF_FRAME[] = {0xFF,0xFA,'\0'};

T6615::T6615(Stream* Serial)
{
    mySerial = Serial;
}

uint16_t T6615::read_co2(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[5];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_CO2_PPM);

    while((bytes_read < 5) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return ((response[3]<<8) | response[4]);
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::get_status(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[4];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x01); // command length
    T6615Write(T6615_CMD_STATUS);

    while((bytes_read < 4) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return response[3];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::read_elevation(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[5];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_ELEVATION);

    while((bytes_read < 5) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return ((response[3]<<8) | response[4]);
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::update_elevation(uint16_t elevation){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[3];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x04); // command length
    T6615Write(T6615_CMD_UPDATE);
    T6615Write(T6615_ELEVATION);
    T6615Write(elevation>>8);
    T6615Write(elevation&0x00FF);


    while((bytes_read < 3) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::set_single_point_calibration(uint16_t calval){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[3];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x04); // command length
    T6615Write(T6615_CMD_UPDATE);
    T6615Write(T6615_CMD_SET_SGPT_PPM);
    T6615Write(calval>>8);
    T6615Write(calval&0x00FF);


    while((bytes_read < 3) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::get_single_point_calibration(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[5];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_CMD_VFY_SGPT_PPM);

    while((bytes_read < 5) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return ((response[3]<<8) | response[4]);
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

void T6615::start_calibration(){
    uint8_t i;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x01); // command length
    T6615Write(T6615_CMD_SGPT_CALIBRATE);
}

void T6615::start_warmup(){
    uint8_t i;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x01); // command length
    T6615Write(T6615_CMD_WARM);
}

uint16_t T6615::idle_on(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[3];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_IDLE);
    T6615Write(T6615_CMD_ON);

    while((bytes_read < 3) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::idle_off(){
    uint8_t i,bytes_read = 0, ch;
    uint8_t response[3];
    uint16_t timeout = 10000;

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_IDLE);
    T6615Write(T6615_CMD_OFF);


    while((bytes_read < 3) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::read_serial_number(uint8_t * serial_number){
    uint8_t i,bytes_read = 0, ch;
    uint16_t timeout = 10000;
    uint8_t response[18];

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_SERIAL_NUMBER);

    while((bytes_read < 18) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        for(i = 0; i < response[2]; i++){
            serial_number[i] = response[3+i];
        }
        serial_number[i] = '\0';
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::read_compile_subvol(uint8_t * compile_subvol){
    uint8_t i,bytes_read = 0, ch;
    uint16_t timeout = 10000;
    uint8_t response[6];

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_COMPILE_SUBVOL);

    while((bytes_read < 6) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        for(i = 0; i < response[2]; i++){
            compile_subvol[i] = response[3+i];
        }
        compile_subvol[i] = '\0';
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

uint16_t T6615::read_compile_date(uint8_t * compile_date){
    uint8_t i,bytes_read = 0, ch;
    uint16_t timeout = 10000;
    uint8_t response[9];

    flushSerial();

    for (i = 0; i < strlen(SEND_START_OF_FRAME); i++){
        T6615Write(SEND_START_OF_FRAME[i]);
    }

    T6615Write(0x02); // command length
    T6615Write(T6615_CMD_READ);
    T6615Write(T6615_COMPILE_DATE);

    while((bytes_read < 9) && (timeout-- != 0)){
        /* Try to receive from sensor, otherwise wait 1us and try again until timeout*/
        if(T6615Available){
            ch = T6615Read;
            if((RECV_START_OF_FRAME[bytes_read] == ch) && (bytes_read < strlen(RECV_START_OF_FRAME))){
                response[bytes_read] = ch;
                bytes_read++;
            }else if(bytes_read >= strlen(RECV_START_OF_FRAME)){
                response[bytes_read] = ch;
                bytes_read++;
            }
        }else{
            Delay_us(1);
        }
    }

    if(timeout != T6615_TIMEOUT){
        for(i = 0; i < response[2]; i++){
            compile_date[i] = response[3+i];
        }
        compile_date[i] = '\0';
        return response[2];
    }else{
        // timeout
        return T6615_TIMEOUT;
    }
}

void T6615::Delay_us(uint32_t us){
    delayMicroseconds(us);
}

void T6615::flushSerial(){
    while(T6615Available){
        T6615Read;
    }
}