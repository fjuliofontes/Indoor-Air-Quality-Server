#include <Arduino.h>
#include <wiring_private.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>

// Global Defines
#include "global_defs.h"

// WiFi and WiFi OTA
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <RTCZero.h>

// Bosch Sensor BME680
#include "bsec.h"
#include "config/generic_33v_300s_4d/bsec_serialized_configurations_iaq.h"

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

// Typedefs
typedef struct {
    float bme_temp;
    float ds18b20_temp;
    float bme_humi;
    uint16_t co2;
    float bme_co2;
    float bme_evoc;
    float bme_iaq;
    float bme_press;
    uint16_t pm10;
    uint16_t pm2_5;
    float co;
    uint8_t accuracy;
    uint32_t timestamp;
} measurement_t;

// Printf and others
#define DEBUG
#include "util.h"

#define DEVICE_NAME_TEMPLATE "MKR1000-%02X%02X%02X%02X%02X%02X"
#define DEVICE_NAME_SIZE (sizeof("MKR1000-000000000000")+1)
#define PUBLISH_SIZE (sizeof("{\"timestamp\":4098895716000,\"t\":100.99,\"h\":100.00,\"p\":10000.00,\"iaq\":500.00,\"evoc\":1000.00,\"eco2\":10000.00,\"pm25\":10000,\"pm10\":10000,\"co2\":10000,\"co\":10000,\"dt\":100.00,\"a\":03}") + 1)
#define PUBLISH_TEMPLATE "{\"timestamp\":%ld,\"t\":%.2f,\"h\":%.2f,\"p\":%.2f,\"iaq\":%.2f,\"evoc\":%.2f,\"eco2\":%.2f,\"pm25\":%d,\"pm10\":%d,\"co2\":%d,\"co\":%.2f,\"dt\":%.2f,\"a\":%d}"
#define BOOTUP_TEMPLATE "{\"timestamp\":{\".sv\": \"timestamp\"},\"ip\":\"%d.%d.%d.%d\",\"ssid\":\"%s\",\"rssi\":%ld}"
#define BOOTUP_TEMPLATE_SIZE (sizeof("{\"timestamp\":{\".sv\": \"timestamp\"},\"ip\":\"255.255.255.255\",\"ssid\":\"ITHOME0123456789\",\"rssi\":-255}") + 1)
#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define MAX_PENDING_MEASURES 350

/* Declared Functions */
void printWifiStatus();
void errLeds(void);
void checkIaqSensorStatus(void);
void loadState(void);
void updateState(void);
void popElement(measurement_t * measurement, uint32_t* pending_measurements);
int64_t getTimeMs(void);
bool sendBootUpEvent();

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
uint16_t co2_ppm = 0, t6615Status, last_co2_ppm;
uint32_t _t6615_period = 20000;

// DS18B20
OneWire  ds(7);                     // on pin 7 (a 4.7K resistor is necessary)
DallasTemperature ds18b20(&ds);     //

// General Use Variables
measurement_t measurements[MAX_PENDING_MEASURES];
uint32_t pending_measurements = 0;
char payload[PUBLISH_SIZE] = {'\0'};
bool newReading = false;
int64_t lastTime = 0;
uint32_t millisOverflowCounter = 0;
char device_name[DEVICE_NAME_SIZE] = {'\0'};
bool sendBootUp = true;

uint8_t retries = 0;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// WiFi related 
WiFiClient client;
RTCZero rtc;
char server[] = FIREBASE_SERVER;    // name address for Google (using DNS)

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

    // Get device MAC ADDR 
    uint8_t mac[6];                 // the MAC address of your Wifi shield
    WiFi.macAddress(mac);
    snprintf(device_name,
        sizeof(device_name),
        DEVICE_NAME_TEMPLATE,
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

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

    // calibrate RTC
    rtc.begin();
    unsigned long epoch = 0;

    Serial.println("Attempting to connect to NTP Server");
    while (epoch == 0) {
        epoch = WiFi.getTime();
        Serial.print(".");
        delay(1000);
    }

    Serial.print("\nEpoch received: ");
    Serial.println(epoch);

    // set epoch
    rtc.setEpoch(epoch);

    // start the WiFi OTA library with internal (flash) based storage
    WiFiOTA.begin(device_name, "jf123", InternalStorage);

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
    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_ULP);
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
    /// set sensor to sync
    ds18b20.setWaitForConversion(true);

    // On success hold the led on
    digitalWrite(LED_BUILTIN,HIGH);
}

void loop() {

    // read BME680 and publish results through MQTT (takes 80ms to send 9 messages)
    // this sensor needs to be read every 3 seconds, otherwise the accuracy will be always zero
    // this means that after one read, we only have around 3 seconds of spare time to the other calls
    if (iaqSensor.run(getTimeMs())) {

        Serial.println("BME measure done!");
        Serial.flush();

        // EPOCH
        measurements[pending_measurements].timestamp = rtc.getEpoch();

        // Temperature
        measurements[pending_measurements].bme_temp = iaqSensor.temperature;

        // Humidity
        measurements[pending_measurements].bme_humi = iaqSensor.humidity;

        // Pressure
        measurements[pending_measurements].bme_press = iaqSensor.pressure/100.0f;

        // IAQ
        measurements[pending_measurements].bme_iaq = iaqSensor.iaq;

        // Static equivalent CO2
        measurements[pending_measurements].bme_co2 = iaqSensor.co2Equivalent;

        // Static equivalent breath VOCs
        measurements[pending_measurements].bme_evoc = iaqSensor.breathVocEquivalent;

        // get accuracy 
        measurements[pending_measurements].accuracy = iaqSensor.iaqAccuracy;

        // save state 
        updateState();

        // read the dallas temperature sensor (takes 2ms to trigger and 40ms to read and send its value)
        // a new read should be triggered before the actually reading of the temperature value
        // this way, this function is being called 800ms before the actual reading interval
        // since the sensor needs 637ms to have the sample done

        // request temperature and block till ready
        Serial.println("Going to read ds18b20!");
        Serial.flush();
        ds18b20.requestTemperatures();

        // save
        measurements[pending_measurements].ds18b20_temp = ds18b20.getTempCByIndex(0);  // read temperature
        Serial.println("ds18b20 read ok!");
        Serial.flush();

        // read the CO sensor (takes 29ms to read and send its value)
        // this sensor after the first utilization will have a fixed lifestamp given by the datasheet
        // no matter the read frequency the lifetime will be the same

        // read ze07
        Serial.println("Going to read ze07!");
        Serial.flush();
        retries = 5;
        do { 
            co_ppm = ze07.read(); 
        } while (
            ((retries--) > 0) && 
            ((co_ppm < 0) || (co_ppm > 500))
        );
        Serial.println("ze07 done!");
        Serial.flush();

        // co concentration in ppm
        measurements[pending_measurements].co = co_ppm;

        // read the CO2 sensor (takes 4ms to wake up and 25ms to read and send its value)
        // this sensor will degrate with the operational time, meaning that reading too often
        // will probably short it's the lifetime
        // to avoid that, we are only reading the sensor from 5 to 5 minutes, shutting it down on idle time
        // However, we are awaking the sensor only 60 seconds before the reading interval, maybe this isn't enough
        // We need to understand also if the sensor on awake should be set to warmup or if it is fine to just read it

        Serial.println("Going to read t6615!");
        Serial.flush();
        for (retries = 0; retries < 4; retries++) {

            t6615Status = t6615.get_status();

            if(t6615Status != T6615_TIMEOUT) {

                /* Idle Mode */
                if(t6615Status == T6615_IDLE_MODE) {
                    Serial.println("T6615_IDLE_MODE");
                    Serial.flush();
                    _t6615_period = 60000; // back again in 60 seconds
                    t6615.idle_off(); // go back to normal operation
                    //t6615.start_warmup(); // go back to normal operation and do pre-heating
                }

                /* Normal Mode */
                else if(t6615Status == T6615_OK){
                    Serial.println("T6615_OK");
                    Serial.flush();

                    // read the sensor
                    retries = 5;
                    do { 
                        co2_ppm = t6615.read_co2();
                    } while(((retries--) > 0) && (co2_ppm == T6615_TIMEOUT));
                    // go to idle mode
                    t6615.idle_on();
                    // save 
                    if (co2_ppm != T6615_TIMEOUT) measurements[pending_measurements].co2 = co2_ppm;
                    else measurements[pending_measurements].co2 = last_co2_ppm;
                    // backup reading
                    if (co2_ppm != T6615_TIMEOUT) last_co2_ppm = co2_ppm;
                    // done
                    break;
                }
                /* WARMUP_MODE */
                else if(t6615Status == T6615_WARMUP_MODE){
                    Serial.println("T6615_WARMUP_MODE");
                    Serial.flush();
                    _t6615_period = 30000; // try again 30 seconds later
                }
                /* default */
                else{
                    _t6615_period = 5000; // try again 5 seconds later
                }
            } else {
                Serial.println("INVALID");
                Serial.flush();
                _t6615_period = 1000; // try again 1 seconds later
            }

            // sleep necessary amount of time
            delay(_t6615_period);
        }

        Serial.println("t6615 done!");
        Serial.flush();

        // read the particle sensor (takes 0ms to wake up and 37ms to read and send its values)
        // Just like the CO2 sensor, the lifetime of this sensor it is also affected by how often the sensor 
        // is redden. So the same logic is applied here. However, in this sensor, turning it on only 10 seconds
        // before the read interval is enough.

        Serial.println("Going to read particle!");
        Serial.flush();
        for (retries = 0; retries < 3; retries++) {
            if(hpma115S0.Status() == HPMA115S0_OFF) {
                Serial.println("HPMA115S0_OFF");
                Serial.flush();
                _hpma115s0_period = 10000; // return again to this function in 10 seconds 
                hpma115S0.StartParticleMeasurement(); // start particle measurement 
            } else {
                //The guideline stipulates that PM2.5 not exceed 10 μg/m³ annual mean, or 25 μg/m³ 24-hour mean; 
                //PM10 not exceed 20 μg/m³ annual mean, or 50 μg/m³ 24-hour mean.
                if (hpma115S0.ReadParticleMeasurement(&pm2_5, &pm10)) {
                    Serial.println("ReadParticleMeasurement");
                    Serial.flush();
                    // stop particle measurements
                    hpma115S0.StopParticleMeasurement(); // stop particle measurement

                    // save
                    measurements[pending_measurements].pm10 = pm10;
                    measurements[pending_measurements].pm2_5 = pm2_5;

                    // done
                    break;
                } else {
                    _hpma115s0_period = 1000; // try again in 1 sec
                }
            }
            // sleep necessary amount of time
            delay(_hpma115s0_period);
        }

        Serial.println("particle done!");
        Serial.flush();

        // flag new reading
        pending_measurements ++;

        // pop old elements
        if (pending_measurements == MAX_PENDING_MEASURES) popElement(measurements,&pending_measurements);

    }

    else {
        checkIaqSensorStatus();
    }

    if (pending_measurements > 0) Serial.println(iaqSensor.nextCall - getTimeMs());

    // still have time ?
    if ( ( iaqSensor.nextCall - getTimeMs() ) < 30000) {
        return;
    }
    
    // disconnected? try to reconnect
    if (WiFi.status() != WL_CONNECTED){

        digitalWrite(LED_BUILTIN,LOW);

        _printf((char*)"Disconnected from WiFi \r\n");

        _printf((char*)"Attempting to re-connect to WPA SSID: %s ",ssid);

        if ( WiFi.begin(ssid, pass) == WL_CONNECTED ) {

            printf((char*)"Re-connected!\r\n");

            // Re-enable the led state
            digitalWrite(LED_BUILTIN,HIGH);

            // Re-send bootup event
            sendBootUp = true;
        }

    }

    // is to send bootup?
    if ((WiFi.status() == WL_CONNECTED) && sendBootUp) {
        sendBootUp = !sendBootUpEvent();
    }

    // ota related
    if( WiFi.status() == WL_CONNECTED ) {
        // check for WiFi OTA updates
        WiFiOTA.poll();
    }

    // publish new readings
    if( (WiFi.status() == WL_CONNECTED) && (pending_measurements > 0) ) {

        Serial.println("trying to connect to server");
        Serial.flush();
        
        if (client.connectSSL(server, 443)) {
            Serial.println("connected to server");
            Serial.flush();

            snprintf(payload,
                sizeof(payload),
                PUBLISH_TEMPLATE,
                measurements[0].timestamp,
                measurements[0].bme_temp,
                measurements[0].bme_humi,
                measurements[0].bme_press,
                measurements[0].bme_iaq,
                measurements[0].bme_evoc,
                measurements[0].bme_co2,
                measurements[0].pm2_5,
                measurements[0].pm10,
                measurements[0].co2,
                measurements[0].co,
                measurements[0].ds18b20_temp,
                measurements[0].accuracy
            );

            Serial.println(String(payload));
            Serial.flush();

            // Make a HTTP request:
            client.println("POST /" + String(device_name) + ".json HTTP/1.1");
            client.println("Host: " + String(server));
            client.println("Connection: close");
            client.println("Content-Type: application/json");
            client.print("Content-Length: ");
            client.println(String(payload).length());
            client.println();
            client.println(String(payload));
            client.println();

            unsigned long timeout = millis();
            while (client.connected() && millis() - timeout < 10000L) {
                // Print available data (HTTP response from server)
                while (client.available()) {
                    client.read();
                    // _printf((char*)"%c",c);
                    timeout = millis();
                }
                // _printf((char*)"\r\n");
            }

            // close connection
            client.stop();

            // on success pop element
            popElement(measurements,&pending_measurements);

        } else {
            Serial.println("failed to connect");
            Serial.flush();
        }
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
    int32_t rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

bool sendBootUpEvent() {
    Serial.println("Trying to send boot up event!");

    if (client.connectSSL(server, 443)) {

        char bootup[BOOTUP_TEMPLATE_SIZE] = {'\0'};

        // Get ip
        IPAddress ip = WiFi.localIP();

        snprintf(bootup,
            BOOTUP_TEMPLATE_SIZE,
            BOOTUP_TEMPLATE,
            ip[0],
            ip[1],
            ip[2],
            ip[3],
            WiFi.SSID(),
            WiFi.RSSI()
        );

        Serial.println("Going to send: " + String(bootup));

        // Make a HTTP request:
        client.println("POST /" + String(device_name) + "-BOOTUP.json HTTP/1.1");
        client.println("Host: " + String(server));
        client.println("Connection: close");
        client.println("Content-Type: application/json");
        client.print("Content-Length: ");
        client.println(String(bootup).length());
        client.println();
        client.println(String(bootup));
        client.println();

        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
            // Print available data (HTTP response from server)
            while (client.available()) {
                client.read();
                timeout = millis();
            }
        }

        // close connection
        client.stop();

        // return
        return true;

    } else {
        return false;
    }
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

void popElement(measurement_t * measurement, uint32_t* pending_measurements) {

    // only if we have elements to pop
    if (*pending_measurements <= 0) return;

    for (int i = 0; i < (int)((*pending_measurements) - 1); i++) {
        memcpy(&measurement[i],&measurement[i+1],sizeof(measurement_t));
    }
    *pending_measurements = *pending_measurements - 1;
}

int64_t getTimeMs(void)
{
    int64_t timeMs = millis();

    if (lastTime > timeMs) /* An overflow occurred */
    {
        millisOverflowCounter++;
    }

    lastTime = timeMs;

    return timeMs + ((int64_t)millisOverflowCounter << 32);
}