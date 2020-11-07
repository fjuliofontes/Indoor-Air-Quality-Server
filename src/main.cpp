#include <Arduino.h>
#include <wiring_private.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>
// Global Defines
#include "global_defs.h"
// MQTT
#include <ArduinoMqttClient.h>
// WiFi and WiFi OTA
#include <WiFi101.h>
#include <WiFi101OTA.h>
// Bosch Sensor BME680
#include "bsec.h"
#include "config/generic_33v_3s_4d/bsec_serialized_configurations_iaq.h"
// Honeywell Sensor HPMA115S0
#include "HPMA115S0.h"
// ZE07-CO 
#include "ZE07CO.h"
// T6615-CO2
#include "T6615.h"
// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
// WiFi secrets
#include "secrets.h"
// Sercom defs
#include "sercom_defs.h"
// Emulated EEPROM
#include <FlashAsEEPROM.h>
// Printf and others
#define DEBUG
#include "util.h"

// Reading periodicity in (ms)
#define HPMA115S0_PERIOD 5*60000    // every 5*60 seconds
#define ZE07_PERIOD 5*60000         // every 5*60 seconds
#define T6615_PERIOD 5*60000        // every 5*60 seconds
#define DS18B20_PERIOD 5000         // every 5 seconds
#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
//#define SEND_HEART_BEAT

/* Declared Functions */
void printWifiStatus();
void onMqttMessage(int messageSize);
void errLeds(void);
void checkIaqSensorStatus(void);
void loadState(void);
void updateState(void);
void heartBeat(void);

// Instantiate the extra Serial classes
Uart Serial2(&sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
Uart Serial3(&sercom0, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
// Interrupt Handlers
void SERCOM3_Handler(){ Serial2.IrqHandler(); }
void SERCOM0_Handler(){ Serial3.IrqHandler(); }

// Bme680 sensor
Bsec iaqSensor; // I2C
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

//Create an instance of the hpma115S0 library
HPMA115S0 hpma115S0(Serial1);
uint16_t pm2_5, pm10;
uint32_t _hpma115s0_period = 10000;

// ZE07-CO
ZE07CO ze07(&Serial2);
float co_ppm = 0;

// T6615
T6615 t6615(&Serial3);
uint16_t co2_ppm = 0, t6615Status;
uint32_t _t6615_period = 20000;

// DS18B20
OneWire  ds(7);                     // on pin 7 (a 4.7K resistor is necessary)
DallasTemperature ds18b20(&ds);     //
float tempC;
uint32_t _ds18b20_period = 1000;
enum ds18b20state {
    ds18b20_askTemperature = 0, 
    ds18b20_readTemperature 
}; 
uint8_t _ds18b20_status = ds18b20_askTemperature;

// General Use Variables
unsigned long bmeLastReading = 0, hpmaLastReading = 0, ze07LastReading = 0, \
              t6615LastReading = 0, ds18b20LastReading = 0, lastHeartBeat = 0;
uint8_t retries = 0;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// MQTT Variables
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
//
bool retained = false;
int qos = 1; // the the library supports subscribing at QoS 0, 1, or 2
bool dup = false;
int subscribeQos = 1; // the the library supports subscribing at QoS 0, 1, or 2
const char broker[]     = "192.168.200.1";
int        port         = 1883;
const char willTopic[]  = "MKR1000/will";
const char inTopic[]    = "MKR1000/updates";
const char * outTopic[] =  { "MKR1000/HPMA115S0/PM2.5",
                             "MKR1000/HPMA115S0/PM10",
                             "MKR1000/BME680/Temp",
                             "MKR1000/BME680/Humi",
                             "MKR1000/BME680/Pres",
                             "MKR1000/BME680/IAQ",
                             "MKR1000/BME680/sIAQ",
                             "MKR1000/BME680/eCO2",
                             "MKR1000/BME680/eVOCb",
                             "MKR1000/ZE07/CO",
                             "MKR1000/T6615/CO2",
                             "MKR1000/BME680/rawTemp",
                             "MKR1000/BME680/rawHumi",
                             "MKR1000/DS18B20/rawTemp"
                           };
String payload          = "";

void setup() {
    // Serial Port USB
    Serial.begin(115200);
    // Serial Port for Honeywell Sensor
    Serial1.begin(9600);
    // I2C 
    Wire.begin();
    // Set LED_BUILTIN as Output and turn it off
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    //Extra Serial Ports
    /// ZE07-CO Sensor - 9600bps
    pinPeripheral(0, PIO_SERCOM);   // Assign pins 0 & 1 SERCOM functionality
    pinPeripheral(1, PIO_SERCOM);
    Serial2.begin(9600);
    /// T6615-CO2 Sensor - 19200bps
    pinPeripheral(2, PIO_SERCOM);   // Assign pins 2 & 3 SERCOM functionality
    pinPeripheral(3, PIO_SERCOM);
    Serial3.begin(19200);

    // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        // failed, retry
        Serial.print(".");
        delay(5000);
    }
    Serial.println("You're connected to the network");
    printWifiStatus();

    // start the WiFi OTA library with internal (flash) based storage
    WiFiOTA.begin("Arduino", "jf123", InternalStorage);

    // You can provide a unique client ID, if not set the library uses Arduino-millis()
    // Each client must have a unique client ID
    mqttClient.setId("MKR1000");

    // You can provide a username and password for authentication
    // mqttClient.setUsernamePassword("username", "password");

    // set a will message, used by the broker when the connection dies unexpectedly
    // you must know the size of the message before hand, and it must be set before connecting
    String willPayload = "disconnected!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);

    while (!mqttClient.connect(broker, port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        delay(5000);
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    Serial.println();

    mqttClient.subscribe(inTopic, subscribeQos);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);

    Serial.print("Waiting for messages on topic: ");
    Serial.println(inTopic);
    Serial.println();

    // Sensors
    /// Honeywell Sensor
    hpma115S0.Init();

    /// BME680
    //// begin library and retreive the library version
    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    _printf((char*)"\nBSEC library version %u.%u.%u.%u \r\n",iaqSensor.version.major,iaqSensor.version.minor,iaqSensor.version.major_bugfix,iaqSensor.version.minor_bugfix);
    //// check if bme680 is connected
    checkIaqSensorStatus();
    /// Apply configs to bsec sensor
    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();
    /// load state from EEPROM
    loadState();
    //// specify the desired outputs
    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    //// update subscription
    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();

    /// ZE07-CO
    //// change the sensor to question mode
    ze07.question();

    /// T6615-CO2 
    //// start warmup
    t6615.start_warmup();

    /// DS18B20
    /// init sensor
    ds18b20.begin();
    /// set sensor to be async
    ds18b20.setWaitForConversion(false);

    // On success hold the led on
    digitalWrite(LED_BUILTIN,HIGH);
}

void loop() {
    // check if is still connected to WiFi
    if(WiFi.status() != WL_CONNECTED){
        _printf((char*)"Disconnected from WiFi \r\n");
        // free some old configurations by disconnecting safely
        WiFi.disconnect(); // stop wifi
        mqttClient.stop(); // stop mqtt 
        _printf((char*)"Attempting to re-connect to WPA SSID: %s ",ssid);
        while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
            _printf((char*)".");
            for(int i = 0; i < 25; i++) errLeds(); // wait 5 seconds and try again
        }
        _printf((char*)"Re-connected!\r\n");
        // re-configure OTA again
        WiFiOTA.begin("Arduino", "jf123", InternalStorage);
        // Re-enable the led state
        digitalWrite(LED_BUILTIN,HIGH);
    }

    // check for WiFi OTA updates
    WiFiOTA.poll();

    // check if mqttClient is still connected and re-connect if not
    if(!mqttClient.connected()){
        _printf((char*)"Disconnected from Broker!\r\n");
        _printf((char*)"Unsubscribing from topic %s retval %d \r\n",inTopic,mqttClient.unsubscribe(inTopic));
        while (!mqttClient.connect(broker, port) && (WiFi.status() == WL_CONNECTED)) {
            _printf((char*)"MQTT connection failed! Error code = %d \r\n",mqttClient.connectError());
            for(int i = 0; i < 25; i++) errLeds(); // wait 5 seconds and try again
        }
        // on success subscribe to inTopic
        if(mqttClient.connected()){
            _printf((char*)"Subscribing again to topic %s retval %d \r\n",inTopic,mqttClient.subscribe(inTopic, subscribeQos));
            // Re-enable the led state
            digitalWrite(LED_BUILTIN,HIGH);
        }
    }

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alive which avoids being disconnected by the broker
    mqttClient.poll();

    // read BME680 and publish results through MQTT (takes 80ms to send 9 messages)
    // this sensor needs to be readden every 3 seconds, otherwise the accuracy will be always zero
    // this means that after one read, we only have around 3 seconds of spare time to the other calls
    if (iaqSensor.run()) {
#ifdef SEND_HEART_BEAT
        bmeLastReading = millis();
#endif
        // Temperature
        payload = myFTOA(iaqSensor.temperature,3,10);
        mqttClient.beginMessage(outTopic[2], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // Humidity
        payload = myFTOA(iaqSensor.humidity,3,10);
        mqttClient.beginMessage(outTopic[3], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // raw Temperature
        payload = myFTOA(iaqSensor.rawTemperature,3,10);
        mqttClient.beginMessage(outTopic[11], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // raw Humidity
        payload = myFTOA(iaqSensor.rawHumidity,3,10);
        mqttClient.beginMessage(outTopic[12], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // Pressure
        payload = myFTOA(iaqSensor.pressure/100.0f,3,10);
        mqttClient.beginMessage(outTopic[4], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // IAQ
        payload = myFTOA(iaqSensor.iaq,3,10);
        mqttClient.beginMessage(outTopic[5], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();
        
        // Static IAQ
        payload = myFTOA(iaqSensor.staticIaq,3,10);
        mqttClient.beginMessage(outTopic[6], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // Static equivalent CO2
        payload = myFTOA(iaqSensor.co2Equivalent,3,10);
        mqttClient.beginMessage(outTopic[7], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        // Static equivalent breath VOCs
        payload = myFTOA(iaqSensor.breathVocEquivalent,3,10);
        mqttClient.beginMessage(outTopic[8], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();

        updateState();
    } else {
        checkIaqSensorStatus();
    }

    // if BME680 next call is in less then 200 ms, there is no time to play with other sensors, just 
    // return and wait until having more spare time. The 200ms was calculated based on the worst case
    // scenario of the time taken by the functions bellow.
    if((iaqSensor.nextCall - millis()) <= 200){
        return;
    }

    // read the CO sensor (takes 29ms to read and sent its value)
    // this sensor after the first utilization will have a fixed lifestamp given by the datasheet
    // no mater the read frequency the lifetime will not be affected
    if((millis() - ze07LastReading) > ZE07_PERIOD){
        ze07LastReading = millis();

        // read ze07
        retries = 5;
        do{ co_ppm = ze07.read(); } while(((retries--) > 0) && ((co_ppm < 0) || (co_ppm > 500)));
        
        payload = myFTOA(co_ppm,3,10);
        mqttClient.beginMessage(outTopic[9], payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();
    }

    // read the CO2 sensor (takes 4ms to wake up and 25ms to read and sent its value)
    // this sensor will degrate with the operational time, meaning that reading to often this sensor
    // will probably short it's the lifetime
    // to avoid that, we are only reading the sensor from 5 to 5 minutes, shuting it down on idle time
    // However, we are awaking the sensor only 60 seconds before the reading interval, maybe this isn't enough
    // We need to understand also if the sensor on awake should be set to warmup or if it is fine to just read it
    if((millis() - t6615LastReading) > _t6615_period){
        t6615LastReading = millis();
        t6615Status = t6615.get_status();
        if(t6615Status != T6615_TIMEOUT){
            /* Idle Mode */
            if(t6615Status == T6615_IDLE_MODE){
                _t6615_period = 60000; // back again in 60 seconds
                t6615.idle_off(); // go back to normal operation
                //t6615.start_warmup(); // go back to normal operation and do pre-heating
            }
            /* Normal Mode */
            else if(t6615Status == T6615_OK){
                _t6615_period = T6615_PERIOD-60000; // go back 60 seconds before period time to warmup
                // read the sensor
                retries = 5;
                do{ co2_ppm = t6615.read_co2();} while(((retries--) > 0) && (co2_ppm == T6615_TIMEOUT));
                // go to idle mode
                t6615.idle_on();
                // send via MQTT
                payload = myITOA(co2_ppm,10,0);
                mqttClient.beginMessage(outTopic[10], payload.length(), retained, qos, dup);
                mqttClient.print(payload);
                mqttClient.endMessage();
            }
            /* WARMUP_MODE */
            else if(t6615Status == T6615_WARMUP_MODE){
                _t6615_period = 30000; // try again 30 seconds later
            }
            /* default */
            else{
                _t6615_period = 5000; // try again 5 seconds later
            }
        }else{
            _t6615_period = 1000; // try again 1 seconds later
        }
    }

    // read the particle sensor (takes 0ms to wake up and 37ms to read and sent its values)
    // Just like the CO2 sensor, the lifetime of this sensor it is also affected by how often the sensor 
    // is redden. So the same logic is applied here. However, in this sensor, turning it on only 10 seconds
    // before the read interval is enough.
    if((millis() - hpmaLastReading) > _hpma115s0_period){
        hpmaLastReading = millis(); // restart timer

        if(hpma115S0.Status() == HPMA115S0_OFF){
            _hpma115s0_period = 10000; // return again to this function in 10 seconds 
            hpma115S0.StartParticleMeasurement(); // start particle measurement 
        }else{
            _hpma115s0_period = HPMA115S0_PERIOD-10000; // return in period less 10 seconds for heating 
            //The guideline stipulates that PM2.5 not exceed 10 μg/m³ annual mean, or 25 μg/m³ 24-hour mean; 
            //PM10 not exceed 20 μg/m³ annual mean, or 50 μg/m³ 24-hour mean.
            if (hpma115S0.ReadParticleMeasurement(&pm2_5, &pm10)) {
                //_printf((char*)"PM2.5: %u ug/m3 \r\n",pm2_5);
                //_printf((char*)"PM10: %u ug/m3 \r\n",pm10);

                //if(pm2_5 > 25) _printf((char*)"PM2.5 is exceeding the acceptable levels! \r\n");
                //if(pm10 > 50) _printf((char*)"PM10 is exceeding the acceptable levels! \r\n");
                
                // PM2.5
                payload = myITOA(pm2_5,10,0);
                mqttClient.beginMessage(outTopic[0], payload.length(), retained, qos, dup);
                mqttClient.print(payload);
                mqttClient.endMessage();
                
                // PM10
                payload = myITOA(pm10,10,0);
                mqttClient.beginMessage(outTopic[1], payload.length(), retained, qos, dup);
                mqttClient.print(payload);
                mqttClient.endMessage();
            }
       
            hpma115S0.StopParticleMeasurement(); // stop particle measurement
        }
    }
    
    // read the dallas temperature sensor (takes 2ms to trigger and 40ms to read and sent its value)
    // a new read should be triggered before the actually reading of the temperature value
    // this way, this function is being called 800ms before the actual reading interval
    // since the sensor needs 637ms to have the sample done
    if((millis() - ds18b20LastReading) > _ds18b20_period){
        ds18b20LastReading = millis(); // restart timer
        if(_ds18b20_status == ds18b20_askTemperature){
            _ds18b20_period = 800;                      // return again in 800 ms 
            _ds18b20_status = ds18b20_readTemperature;  // update the new state
            ds18b20.requestTemperatures();
        }else{
            _ds18b20_period = DS18B20_PERIOD-800;       // return in period minus 800 ms 
            _ds18b20_status = ds18b20_askTemperature;   // update the new state
            tempC = ds18b20.getTempCByIndex(0);         // read temperature
            
            // Send
            payload = myFTOA(tempC,3,10);
            mqttClient.beginMessage(outTopic[13], payload.length(), retained, qos, dup);
            mqttClient.print(payload);
            mqttClient.endMessage();
        }
    }

    // send heart beat to debug possible delays and possible overflows that may occur
    // takes 20ms to send heartbeat
#ifdef SEND_HEART_BEAT
    if((millis()-lastHeartBeat) > 1000){
        lastHeartBeat = millis();
        heartBeat();
    }
#endif
    
}

void onMqttMessage(int messageSize) {
    static char recv_topic[50];
    static char recv_msg[160];
    static char params[20];
    uint8_t i = 0;

    // check if we have space
    if((sizeof(recv_msg) < (uint16_t)messageSize) || (sizeof(recv_topic) < mqttClient.messageTopic().length())){
        return;
    }

    // clear buffers
    memset(recv_topic,'\0',sizeof(recv_topic));
    memset(recv_msg,'\0',sizeof(recv_msg));
    memset(params,'\0',sizeof(params));

    // get topic name
    mqttClient.messageTopic().toCharArray(recv_topic,sizeof(recv_topic));

    // use the Stream interface to print the contents
    while (mqttClient.available()) {
        recv_msg[i++] = (char)mqttClient.read();
    }

    // check if it is possible to be a setBme680Params command
    if(messageSize >= 15){
        memcpy(params,recv_msg,15);
        params[15] = '\0'; 
    }


    if (strcmp(recv_msg,(char*)"sendIp") == 0){
        IPAddress ip = WiFi.localIP();
        payload = "Connected to: "+ String(WiFi.SSID())+ " with IP: ";
        for (int k=0; k < 3; k++) payload += String(ip[k], DEC) + ".";
        payload += String(ip[3], DEC);
        mqttClient.beginMessage("MKR1000/status/IP", payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();
    }

    else if(strcmp(recv_msg,(char*)"sendBme680Accuracy") == 0){
        // Accuracy
        payload = "IAQ: " + String(iaqSensor.iaqAccuracy) + " eVOC: " + String(iaqSensor.breathVocAccuracy) + 
                " eCO2: " + String(iaqSensor.co2Accuracy) + " sIAQ: " + String(iaqSensor.staticIaqAccuracy);
        mqttClient.beginMessage("MKR1000/BME680/Accuracy", payload.length(), retained, qos, dup);
        mqttClient.print(payload);
        mqttClient.endMessage();
    }

    else if(strcmp(params,(char*)"setBme680Params") == 0){
        memcpy(bsecState,&recv_msg[15],BSEC_MAX_STATE_BLOB_SIZE);
        iaqSensor.setState(bsecState);
        checkIaqSensorStatus();
        _printf((char*)"Bme680 State Set Done!\r\n");
    }
    
    else{
        _printf((char*)"Received a message with topic (%s) with content (%s) \r\n",recv_topic,recv_msg);
    }
}

void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

// Helper function definitions
void checkIaqSensorStatus(void){
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            _printf((char*)"BSEC error code : %d \r\n",iaqSensor.status);
        for (;;)
            errLeds(); /* Halt in case of failure */
        } else {
            _printf((char*)"BSEC warning code : %d \r\n",iaqSensor.status);
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            _printf((char*)"BSEC error code : %d \r\n",iaqSensor.bme680Status);
        for (;;)
            errLeds(); /* Halt in case of failure */
        } else {
            _printf((char*)"BSEC warning code : %d \r\n",iaqSensor.bme680Status);
        }
    }
}

void errLeds(void){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void loadState(void){
    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
        // Existing state in EEPROM
        Serial.println("Reading state from EEPROM!");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsecState[i] = EEPROM.read(i + 1);
            //Serial.println(bsecState[i], HEX);
        }

        iaqSensor.setState(bsecState);
        checkIaqSensorStatus();
    } 
    else {
        uint8_t isEepromErased = 1;
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            if(EEPROM.read(i + 1) != 0){
                isEepromErased = 0;
                break;
            }
        }
        if(!isEepromErased){
            // Erase the EEPROM with zeroes
            Serial.println("Erasing EEPROM");
            for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++){
                EEPROM.write(i, 0);
            }
            EEPROM.commit();
        }
    }
}

void updateState(void){
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM!");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      //Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}

void heartBeat(void){
    payload = "timestamp: " + String(millis()) + " bme: " + String(bmeLastReading)
        + " ze07: " + String(ze07LastReading) + " t6615: " + String(t6615LastReading) 
        + " hpma: " + String(hpmaLastReading);
    mqttClient.beginMessage("MKR1000/HeartBeat", payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();
}