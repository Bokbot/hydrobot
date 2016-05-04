/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Example sketch showing how to create a node thay repeates messages
 * from nodes far from gateway back to gateway. 
 * It is important that nodes that has enabled repeater mode calls  
 * process() frequently. Repeaters should never sleep. 
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensor.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "DHT.h"
#include <Wire.h>
#include "floatToString.h" 

const int DHT_PIN = 2; // digital pin 2
DHT dht;

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define COMPARE_HUM 1 // Send humidity only if changed? 1 = Yes 0 = No
#define COMPARE_PH 1 // Send pH only if changed? 1 = Yes 0 = No
#define COMPARE_EC 1 // Send EC only if changed? 1 = Yes 0 = No

// Which DS18B20 is the water temp where the pH sensor resides?
#define WATERTEMPSENSOR 1

#define AIRTMP_ID 56
#define HUM_ID 57
#define PH_ID 58
#define EC_ID 59
#define ezophaddress 99               //default I2C ID number for EZO pH Circuit.
#define ezoecddress 98               //default I2C ID number for EZO EC Circuit.

char buffer[10];
//char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.   
//byte received_from_computer=0;   //we need to know how many characters have been received.    
//byte serial_event=0;             //a flag to signal when data has been received from the pc/mac/other. 
byte code=0;                     //used to hold the I2C response code. 
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit. 
byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the pH Circuit.   
byte i=0;                        //counter used for ph_data array. 
int time_=1800;                   //used to change the delay needed depending on the command sent to the EZO Class pH Circuit. 
float ph_float = 0;                  //float var used to hold the float value of the pH. 

#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected 
#define MAX_ATTACHED_DS18B20 16
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
float lastHumidity = 0;
float lastAirtemp = 0;
float lastPH = 0;
float lastEC = 0;
int numSensors=0;
boolean receivedConfig = false;
boolean metric = true; 
// Initialize temperature message
MyMessage tempmsg(0,V_TEMP);
//MyMessage airtempmsg(AIRTMP_ID,V_TEMP);
MyMessage airtempmsg(AIRTMP_ID,V_TEMP);
MyMessage humiditymsg(HUM_ID,V_HUM);
//MyMessage phmsg(PH_ID,V_PH);
MyMessage phmsg(PH_ID,V_TEMP);
//MyMessage ecmsg(EC_ID,V_EC);
MyMessage ecmsg(EC_ID,V_TEMP);

float readpH() {

    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(WATERTEMPSENSOR):sensors.getTempFByIndex(WATERTEMPSENSOR)) * 10.)) / 10.;
    float internal_ph_float;                  //float var used to hold the float value of the pH. 
    time_=1800;
    // set temp compensation
    Wire.beginTransmission(ezophaddress); //call the circuit by its ID number.
    Wire.write('T,');        //transmit the command that was sent through the serial port.
    Wire.write(floatToString(buffer, temperature , 2));        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
    delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction.
    Wire.requestFrom(ezophaddress,20,1); //call the circuit and request 20 bytes (this may be more than we need)
    delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction.
    // now begin query
    Wire.beginTransmission(ezophaddress); //call the circuit by its ID number.
    Wire.write('r');        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
    delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction.
    Wire.requestFrom(ezophaddress,20,1); //call the circuit and request 20 bytes (this may be more than we need)
    code=Wire.read();               //the first byte is the response code, we read this separately.
    switch (code){                  //switch case based on what the response code is.
      case 1:                       //decimal 1.
        //Serial.println("Success");  //means the command was successful.
      break;                        //exits the switch case.

     case 2:                        //decimal 2.
       //Serial.println("Failed");    //means the command has failed.
     break;                         //exits the switch case.

     case 254:                      //decimal 254.
      //Serial.println("Pending");   //means the command has not yet been finished calculating.
     break;                         //exits the switch case.

     case 255:                      //decimal 255.
      //Serial.println("No Data");   //means there is no further data to send.
     break;                         //exits the switch case.
    }
    while(Wire.available()){          //are there bytes to receive.
    //while(0){          //are there bytes to receive.
     in_char = Wire.read();           //receive a byte.
     ph_data[i]= in_char;             //load this byte into our array.
     i+=1;                            //incur the counter for the array element.
      if(in_char==0){                 //if we see that we have been sent a null command.
          i=0;                        //reset the counter i to 0.
          Wire.endTransmission();     //end the I2C data transmission.
          break;                      //exit the while loop.
      }
    }
    //Serial.println(ph_data);          //print the data.
    internal_ph_float=atof(ph_data);
    return internal_ph_float;
}

void setup() {
  // DHT temperature sensor setup
  dht.setup(DHT_PIN); // data pin
  delay(250);
  // DallasTemperature setup
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
   //while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  //}
  Wire.begin();      // join i2c bus as master
}

void presentation()  
{  
  //Send the sensor node sketch version information to the gateway
  //sendSketchInfo("Repeater Node", "1.0");
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Monitaur Multi HydroQ Sensor", "1.1");

  // DallasTemperature presentation
  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     present(i, S_TEMP);
  }
  // Humidity
  present(HUM_ID, S_HUM);
  // pH
  //present(PH_ID, S_WATER_QUALITY);
  present(PH_ID, S_TEMP);
  // EC
  //present(EC_ID, S_WATER_QUALITY);
  present(EC_ID, S_TEMP);
  // AIRTMP
  present(AIRTMP_ID, S_TEMP);
}

void loop() 
{
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sleep(conversionTime);

  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
    send(tempmsg.setSensor(i).set(temperature,1));
    /*Serial.print(i);*/
    /*Serial.print("-DS18B20-temp ");*/
    /*Serial.println(temperature);*/
 
    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
      if (temperature != -127.00 && temperature != 85.00) {
 
      // Send in the new temperature
   //   send(tempmsg.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
    }
  }
  }
  float airtemp = dht.getTemperature();
      send(airtempmsg.setSensor(AIRTMP_ID).set(airtemp,1));
    /*Serial.print("DHT1122-airtemp ");*/
    /*Serial.println(airtemp);*/
    // Only send data if airtemp has changed and no error
    if (lastAirtemp != airtemp ) {
 
      // Send in the new airtemp
      //send(airtempmsg.setSensor(AIRTMP_ID).set(airtemp,1));
      // Save new airtemp for next compare
      lastAirtemp=airtemp;
    }
  float humidity = dht.getHumidity();
  send(humiditymsg.setSensor(HUM_ID).set(humidity,1));
  //  Serial.print("DHT1122-airhum ");
  //  Serial.println(humidity);
    // Only send data if humidity has changed and no error
    if (lastHumidity != humidity) {
 
      // Send in the new humidity
      //send(humiditymsg.setSensor(HUM_ID).set(humidity,1));
      // Save new humidity for next compare
      lastHumidity=humidity;
  }
  float ph_float1 = static_cast<float>(static_cast<int>(readpH()));
      send(phmsg.setSensor(PH_ID).set(ph_float1,1));
  //  Serial.print("Atlas-ph ");
  //  Serial.println(ph_float1);
    if (lastPH != ph_float1) {
      // Send in the new ph_float1
      //send(phmsg.setSensor(PH_ID).set(ph_float1,1));
      // Save new ph_float1 for next compare
      lastPH=ph_float1;
  }
  sleep(SLEEP_TIME);
}
