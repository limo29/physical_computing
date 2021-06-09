
 #define SIM800L_IP5306_VERSION_20200811  //Define the modem model


// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon

#include "utilities.h"

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

//Sensors 
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <string.h>
#include "SdsDustSensor.h"
#include <Wire.h>

Adafruit_BMP280 bmp;
SdsDustSensor sds(Serial2);


// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */


// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "internet"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any

// MQTT details
const char* broker = "test.mosquitto.org";
const char* dataTopic      = "esp32jonas/led";



TinyGsmClient client(modem);
PubSubClient  mqtt(client);
const int  port = 80;
const int mqttPort = 1883;

//Global control variables
bool sensorsDone = false;
bool notConnected = true;
//#define DataSaveModeOff true //Uncomment if you want to disable DataSaverMode with extra debug information

//Global data string
String data = "";

#define LED_PIN 13
int ledStatus = LOW;

uint32_t lastReconnectAttempt = 0;



boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  
  mqtt.publish(dataTopic, "GsmClientTest started"); //maybe only transmit sensor data once at init 

  return mqtt.connected();
}


void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

void turnOffNetlight()
{
    SerialMon.println("Turning off SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=0");
}

void turnOnNetlight()
{
    SerialMon.println("Turning on SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=1");
}

bool gsmConnect(){
  // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();

    // Turn off network status lights to reduce current consumption
    turnOffNetlight();

    // The status light cannot be turned off, only physically removed
    //turnOffStatuslight();

    // Or, use modem.init() if you don't need the complete restart
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem: ");
    SerialMon.println(modemInfo);

    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
        modem.simUnlock(simPIN);
    }

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork(240000L)) {
        SerialMon.println(" fail");
        delay(10000);
        return false;
    }
    SerialMon.println(" OK");

    // When the network connection is successful, turn on the indicator
    digitalWrite(LED_GPIO, LED_ON);

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    SerialMon.print(F("Connecting to APN: "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return false;
    }
    SerialMon.println(" OK");
return true;
}

String getSensoren(){
    sds.wakeup();
     delay(500);
     PmResult pm = sds.queryPm();

  char PM25Result[8];
  char PM10Result[8];

  if(pm.isOk())
  {
    dtostrf(pm.pm25, 8, 2, PM25Result);
    dtostrf(pm.pm10, 8, 2, PM10Result);

    Serial.println("PM2,5 - Wert:");
    Serial.println(PM25Result);
    Serial.println("PM10 - Wert:");
    Serial.println(PM10Result);
  } 
  
  sds.sleep();
/*
   * Berarbeitung des BMP280
   * Erhalt: Temperatur, Luftdruck und Höhe
   */
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float Altitude = bmp.readAltitude(1013.25);

  char temperatureResult[8];
  char pressureResult[16];
  char AltitudeResult[8];

  dtostrf(temperature, 8, 2, temperatureResult);
  dtostrf(pressure, 16, 2, pressureResult);
  dtostrf(Altitude, 8, 2, AltitudeResult);

  Serial.println("Temperatur - Wert:");
  Serial.println(temperatureResult);
  Serial.println("Luftdruck - Wert:");
  Serial.println(pressureResult);
  Serial.println("Höhe - Wert:");
  Serial.println(AltitudeResult);
  
  /*
   * Zusammenfügen der einzelnen Strings
   */

  char message[1024];
  strcat(message, "PM25 ");
  strcat(message, PM25Result);
  strcat(message, " PM10 ");
  strcat(message, PM10Result);
  strcat(message, " temp "); 
  strcat(message, temperatureResult);
  strcat(message, " pres ");
  strcat(message, pressureResult);
  strcat(message, " alti ");
  strcat(message, AltitudeResult); 

  return message;
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);

    delay(10);
  pinMode(LED_PIN, OUTPUT);
    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    while(1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
                  
  sds.begin();

  Serial.println(sds.queryFirmwareVersion().toString());
  Serial.println(sds.setQueryReportingMode().toString());
   

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
     // MQTT Broker setup
  mqtt.setServer(broker, 1883);

 sensorsDone = true; 
}





void loop()
{
if(notConnected){
  gsmConnect();
  notConnected = false;
}


volatile String sensoren = getSensoren();


 
if(sensorsDone){  //This Block only gets activated when all Sensor Data is ready and the Data String is ready to send, afterwards the ESP enters DeepSleep
//mqtt start
 if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
   // return;
  }
  mqtt.publish(dataTopic, sensoren);
  mqtt.loop();
  delay(500);
//mqtt finished
   
    modem.gprsDisconnect();
    notConnected = true;
    SerialMon.println(F("GPRS disconnected"));
    //After all off
    modem.poweroff();
    SerialMon.println(F("Poweroff"));

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); 
    esp_deep_sleep_start();   
}
    
}
