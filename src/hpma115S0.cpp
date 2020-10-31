/**
 * @file HPMA115S0.cpp
 * @author Felix Galindo
 * @date June 2017
 * @brief Arduino Library for Honeywell's Particle Sensor (HPMA115S0-XXX)
 * @license MIT
 */

#include "Arduino.h"
#include "HPMA115S0.h"

extern "C" {
    #include <string.h>
    #include <stdlib.h>
    #include <stdio.h>
}

/**
 * @brief Constructor for HPMA115S0 class
 * @param  a Stream ({Software/Hardware}Serial) object.
 * @note The serial stream should be already initialized
 * @return  void
 */
HPMA115S0::HPMA115S0(Stream& serial):
    _serial(serial)
{
    _serial.setTimeout(100);
}

/**
 * @brief Function that initializes sensor
 * @return  a String containing sensor response
 */
void HPMA115S0::Init() {
    DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Initializing...\r\n");
    delay(100);
    StartParticleMeasurement();
    delay(100);
    DisableAutoSend();
}

/**
 * @brief Function that sends serial command to sensor
 * @param  a unsigned char * containing the command
 * @param size of buffer
 * @return  void
 */
void HPMA115S0::SendCmd(uint8_t * cmdBuf, uint16_t cmdSize) {
    //Clear RX
    while (_serial.available())
        _serial.read();

    //Send command
    DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Sending cmd...\r\n");
    DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"");
    for (uint16_t index = 0; index < cmdSize; index++) {
        DEBUG_PRINT((char*)" %x ",cmdBuf[index]);
        _serial.write(cmdBuf[index]);
    }
    DEBUG_PRINT((char*)"\r\n");
    return;
}

/**
 * @brief Function that reads command response from sensor
 * @param Buffer to store data in
 * @param Buffer size
 * @param Expected command type
 * @return  returns number of bytes read from sensor
 */
uint16_t HPMA115S0::ReadCmdResp(uint8_t * dataBuf, uint16_t dataBufSize, uint16_t cmdType) {
    static uint8_t respBuf[HPM_MAX_RESP_SIZE];
    static uint16_t respIdx = 0;
    static uint16_t calChecksum = 0;
    static unsigned long currTime = 0;

    //Read response
    respIdx = 0;
    calChecksum = 0;
    memset(respBuf, 0, sizeof(respBuf));
    _serial.setTimeout(100);
    DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Waiting for cmd resp...\r\n");
    if (_serial.readStringUntil(HPM_CMD_RESP_HEAD)) {
        DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Ok...\r\n");
        //delay(1); //wait for the rest of the bytes to arrive
        respBuf[HPM_HEAD_IDX] = HPM_CMD_RESP_HEAD;
        
        DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)" Len %u \r\n", respBuf[HPM_LEN_IDX]);
        currTime = millis();
        do{
            respBuf[HPM_LEN_IDX] = _serial.read(); //Read the command length
        }while((!respBuf[HPM_LEN_IDX] || (respBuf[HPM_LEN_IDX] == 255)) && ((millis()-currTime) <= 10));

        DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)" Len %u \r\n", respBuf[HPM_LEN_IDX]);

        //Ensure buffers are big enough
        if (respBuf[HPM_LEN_IDX] && ((respBuf[HPM_LEN_IDX] + 1) <= (uint16_t)(sizeof(respBuf) - 2))  && (uint16_t)(respBuf[HPM_LEN_IDX] - 1) <= dataBufSize ) {
            if ((uint16_t)_serial.readBytes(&respBuf[HPM_CMD_IDX], respBuf[HPM_LEN_IDX] + 1) == (respBuf[HPM_LEN_IDX] + 1)) { //read respBuf[HPM_LEN_IDX] num of bytes + calChecksum byte
                if (respBuf[HPM_CMD_IDX] == cmdType) { //check if CMD type matches

                    //Calculate and validate checksum
                    for (respIdx = 0; respIdx < (2 + respBuf[HPM_LEN_IDX]); respIdx++) {
                        calChecksum += respBuf[respIdx];
                    }
                    calChecksum = (65536 - calChecksum) % 256;
                    if (calChecksum == respBuf[2 + respBuf[HPM_LEN_IDX]]) {
                        DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Received valid data!!!\r\n");
                        memset(dataBuf, 0, dataBufSize);
                        memcpy(dataBuf, &respBuf[HPM_DATA_START_IDX], respBuf[HPM_LEN_IDX] - 1);
                        return (respBuf[HPM_LEN_IDX] - 1);
                    }
                }
            }
        }
    }
    return false;
}


/**
 * @brief Function that sends a read command to sensor
 * @return  returns true if valid measurements were read from sensor
 */
boolean HPMA115S0::ReadParticleMeasurement(uint16_t * pm2_5, uint16_t * pm10) {
    uint8_t cmdBuf[] = {0x68, 0x01, 0x04, 0x93};
    static uint8_t dataBuf[HPM_READ_PARTICLE_MEASUREMENT_LEN - 1];
    
    DEBUG_PRINT_VERBOSE(HPMA115S0_DEBUG_LEVEL,(char*)"Reading Particle Measurements...\r\n");

    //Send command
    SendCmd(cmdBuf, 4);

    //Read response
    if (ReadCmdResp(dataBuf, sizeof(dataBuf), READ_PARTICLE_MEASUREMENT) == (HPM_READ_PARTICLE_MEASUREMENT_LEN - 1)) {
        _pm2_5 = dataBuf[0] * 256 + dataBuf[1];
        _pm10 = dataBuf[2] * 256 + dataBuf[3];
        *pm2_5 = _pm2_5;
        *pm10 = _pm10;
        return true;
    }
    return false;
}

/**
 * @brief Function that starts sensor measurement
 * @return  void
 */
void HPMA115S0::StartParticleMeasurement() {
    uint8_t cmd[] = {0x68, 0x01, 0x01, 0x96};
    SendCmd(cmd, 4);
    _status = HPMA115S0_ON;
}

/**
 * @brief Function that stops sensor measurement
 * @return  void
 */
void HPMA115S0::StopParticleMeasurement() {
    uint8_t cmd[] = {0x68, 0x01, 0x02, 0x95};
    SendCmd(cmd, 4);
    _status = HPMA115S0_OFF;
}

/**
 * @brief Function that enables auto send
 * @return  void
 */
void HPMA115S0::EnableAutoSend() {
    uint8_t cmd[] = {0x68, 0x01, 0x40, 0x57};
    SendCmd(cmd, 4);
}

/**
 * @brief Function that stops auto send
 * @return  void
 */
void HPMA115S0::DisableAutoSend() {
    uint8_t cmd[] = {0x68, 0x01, 0x20, 0x77};
    SendCmd(cmd, 4);
}

/**
* @brief Function that returns the latest PM 2.5 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 2.5 reading (unsigned int)
*/
uint16_t HPMA115S0::GetPM2_5() {
    return _pm2_5;
}

/**
* @brief Function that returns the latest PM 10 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 10 reading (unsigned int)
*/
uint16_t HPMA115S0::GetPM10() {
    return _pm10;
}

uint8_t HPMA115S0::Status(){
    return _status;
}
