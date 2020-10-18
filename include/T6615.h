/*
 * T6615.h
 *
 *  Created on: 10/02/2020
 *      Author: Fernando Fontes
 *
 * Brief:
 *  This library can be used to read the CO2 sensor T6615
 */

#ifndef SRC_T6615_H_
#define SRC_T6615_H_

#include <Arduino.h>
#include <stdint.h>


// read options
#define T6615_CO2_PPM  0x03
#define T6615_SERIAL_NUMBER 0x01
#define T6615_COMPILE_SUBVOL 0x0D
#define T6615_COMPILE_DATE 0x0C
#define T6615_ELEVATION 0x0F

#define T6615_OK 0x00
#define T6615_ERROR 0x01
#define T6615_WARMUP_MODE 0x02
#define T6615_CALIBRATION 0x04
#define T6615_IDLE_MODE 0x08
#define T6615_SELF_TEST_MODE 0x80

#define T6615_TIMEOUT 0xFFFF

class T6615
{
    public:
    T6615(Stream* Serial);	//read the uart signal by hardware uart,such as D0

    /*
    *  Name: read_co2()
    *  Return: uint16_t CO2 value in ppm, or T6615_TIMEOUT
    * */
    uint16_t read_co2();

    /*
    *  Name: read_elevation()
    *  Return: uint16_t elevation in feet, or T6615_TIMEOUT
    * */
    uint16_t read_elevation();

    /*
    *  Name: get_status()
    *  Return: uint16_t status, or T6615_TIMEOUT
    * */
    uint16_t get_status();

    /*
    *  Name: update_elevation(uint16_t elevation)
    *  Input: uint16_t elevation in feet
    *  Returns: 0 to ack or T6615_TIMEOUT
    * */
    uint16_t update_elevation(uint16_t elevation);

    /*
    *  Name: set_single_point_calibration(uint16_t calval)
    *  Input: uint16_t calibration value in ppm
    *  Returns: 0 to ack or T6615_TIMEOUT
    * */
    uint16_t set_single_point_calibration(uint16_t calval);

    /*
    *  Name: get_single_point_calibration()
    *  Return: uint16_t calibration value in ppm or T6615_TIMEOUT
    * */
    uint16_t get_single_point_calibration();

    /*
    *  Name: start_calibration()
    *  Brief: Start calibration procedure, takes 2 - 4 seconds
    * */
    void start_calibration();

    /*
    *  Name: start_warmup()
    *  Brief: Resets the co2 ppm values and start heating and measuring again
    * */
    void start_warmup();

    /*
    *  Name: idle_on()
    *  Returns: 0 to ack or T6615_TIMEOUT
    * */
    uint16_t idle_on();

    /*
    *  Name: idle_off()
    *  Returns: 0 to ack or T6615_TIMEOUT
    * */
    uint16_t idle_off();

    /*
    *  Name: read_serial_number()
    *  Input: char array to store the output from the command (should be 20 bytes at least)
    *  Return: number of bytes read or T6615_TIMEOUT
    * */
    uint16_t read_serial_number(uint8_t * serial_number);

    /*
    *  Name: read_compile_subvol()
    *  Input: char array to store the output from the command (should be 5 bytes at least)
    *  Return: number of bytes read or T6615_TIMEOUT
    * */
    uint16_t read_compile_subvol(uint8_t * compile_subvol);

    /*
    *  Name: read_compile_date()
    *  Input: char array to store the output from the command (should be 10 bytes at least)
    *  Return: number of bytes read or T6615_TIMEOUT
    * */
    uint16_t read_compile_date(uint8_t * compile_date);

    private:
    Stream* mySerial;

    void flushSerial();

    void Delay_us(uint32_t us);

    enum T6615_CMD { 
        T6615_CMD_READ = 0x02,
        T6615_CMD_UPDATE = 0x03, // update command
        T6615_CMD_WARM = 0x84, // warmup command
        T6615_CMD_SGPT_CALIBRATE = 0x9B, // self-calibration
        T6615_CMD_SET_SGPT_PPM = 0x11, // setpoint for self-calibration
        T6615_CMD_VFY_SGPT_PPM = 0x11, // verify setpoint self-calibration
        T6615_CMD_STATUS = 0xB6,
        T6615_CMD_IDLE = 0xB9,
        T6615_CMD_ON = 0x01,
        T6615_CMD_OFF = 0x02,
        T6615_CMD_STREAM_DATA = 0xBD
    };
};

#endif /* SRC_T6615_H_ */
