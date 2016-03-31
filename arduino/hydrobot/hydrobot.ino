/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial monitor.

  The circuit:
   potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
   LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/
#include "DHT.h"
const int DHT_PIN = 2; // digital pin 2
DHT dht;

String dataString = "";
//PID section
#include <PID_v1.h>

#define PIN_INPUT 0
#define RELAY_PIN 6

#include <Wire.h>
const byte SlaveDeviceId = 108;
byte LastMasterCommand = 0;
int a, b, c, d, e;

void receiveDataPacket(int howMany){
  //  if (howMany != 11) return; // Error
  LastMasterCommand = Wire.read();
  a = Wire.read() << 8 | Wire.read();
  b = Wire.read() << 8 | Wire.read();
  c = Wire.read() << 8 | Wire.read();
  d = Wire.read() << 8 | Wire.read();
  e = Wire.read() << 8 | Wire.read();
}

void slavesRespond(){
 
  int returnValue = 0;
 
  switch(LastMasterCommand){
    case 0:   // No new command was received
       returnValue = 1; // i.e. error code #1
    break;
    
    case 1:   // Some function
      
    break;
 
    case 2:   // Our test function
      returnValue = sumFunction(a,b,c,d,e);  
    break;
 
  }
 
  byte buffer[2];              // split int value into two bytes buffer
  buffer[0] = returnValue >> 8;
  buffer[1] = returnValue & 255;
  Wire.write(buffer, 2);       // return response to last command
  LastMasterCommand = 0;       // null last Master's command
}
 
int sumFunction(int aa, int bb, int cc, int dd, int ee){  
  // of course for summing 5 integers You need long type of return,
  // but this is only illustration. Test values doesn't overflow
 
  int result = aa + bb + cc + dd + ee;
  return result;
}

#include <Time.h>
#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

#define moisture_input 0
#define divider_top 12
#define divider_bottom 13

const int VAL_PROBE1 = 0; // Analog pin 0
const int VAL_PROBE2 = 1; // Analog pin 1
const int VAL_PROBE3 = 2; // Analog pin 2
const int VAL_PROBE4 = 3; // Analog pin 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long WindowSize = 30000; // 30 seconds = 30,000 ms
unsigned long windowStartTime;
//PID section END

//Constants
const int nudge = 3;      // how much to nudge our value forward
const int yank = 11;      // how much to yank our value back
// They're used to give names
// to the pins used:
const int relayPin1 = 4;      // select the pin for the LED
const int relayPin2 = 5;      // select the pin for the LED
const int relayPin3 = 6;      // select the pin for the LED
const int relayPin4 = 7;      // select the pin for the LED
const int relayPin5 = 8;      // select the pin for the LED
const int relayPin6 = 9;      // select the pin for the LED
const int relayPin7 = 10;      // select the pin for the LED
const int relayPin8 = 11;      // select the pin for the LED
const int analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A1;  // Analog input pin that the potentiometer is attached to
const int analogInPin3 = A2;  // Analog input pin that the potentiometer is attached to
const int analogInPin4 = A3;  // Analog input pin that the potentiometer is attached to
const int analogInPin5 = A4;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 13; // Analog output pin that the LED is attached to

// These constants won't change
const unsigned long pumpOnTimeMax = 1800000; // maximum time pump should be on in ms: 1,800,000 ms = 30 minutes
const unsigned long pumpOnTimeMin = 540000; // minimum time pump should be on in ms: 540,000 ms = 9 minutes
const unsigned long pumpOffTimeMax = 32400000; // maximum time pump should be off in ms:  32,400,000 ms = 9 hours
const unsigned long pumpOffTimeMin = 5400000; // minimum time pump should be off in ms: 5,400,000 ms = 90 minutes
const unsigned long lcdInterval = 5000; // lcd refresh interval in ms: 5,000 ms = 5 seconds

bool ok;
bool ok2;
bool flop;
bool pumpOn;
bool PIDpumpOn;

int dryLimit = 255;        // this is the value of dryness we don't want to exceed
int wetLimit = 920;        // this is the vale of wetness we don't want to go above (below 320 is wetter)
int sensorValue1 = 0;        // value read from the pot
int sensorValue2 = 0;        // value read from the pot
int sensorValue3 = 0;        // value read from the pot
int sensorValue4 = 0;        // value read from the pot
int sensorValue5 = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int sensorLowValue = 1024;
int sensorHighValue = 0;
int sensorLastLowValue = 1024;
int sensorLastHighValue = 0;
int backgroundColor = 0;
int foregroundColor = 0;

unsigned int countZero = 0;
unsigned int pumpOnCount = 0;

unsigned int watchdog;
unsigned int watchdog2;
unsigned int fiveOn;
unsigned int fiveOff;

unsigned long now1;
unsigned long timeOn;
unsigned long timeOff;
unsigned long secondTime;
unsigned long throttleTime;
unsigned long serialthrottleTime;
unsigned long divideTime;
unsigned long multiplyTime;

unsigned long pumpOnTimes[5];
unsigned long pumpOffTimes[5];

float humidity    = 0;
float temperature = 0;

void printDigits(int digits){ // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// LCD init START

/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

//#include <Adafruit_GFX.h>    // Core graphics library
//#include <Adafruit_ST7735.h> // Hardware-specific library
//#include <SPI.h>
#include <Average.h>


// Reserve space for 10 entries in the average bucket.
// Change the type between < and > to change the entire way the library works.
Average<float> aveOn(10);
Average<float> aveOff(10);

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
/*#define TFT_CS     10*/
/*#define TFT_RST    9  // you can also connect this to the Arduino reset*/
                      /*// in which case, set this #define pin to 0!*/
/*#define TFT_DC     8*/

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
/*Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);*/

// Option 2: use any pins but a little slower!
/*#define TFT_SCLK 13   // set these to be whatever pins you like!*/
/*#define TFT_MOSI 11   // set these to be whatever pins you like!*/
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


float countdownOn() {
  //output in minutes
  return ((float)(pumpOnTimeMax - pumpOnTimes[fiveOn] ) / (60000));
}

float countdownOff() {
  //output in minutes
  return ((float)(pumpOffTimeMax - pumpOffTimes[fiveOff] ) / (60000));
}

unsigned long onCountMin() {
  return pumpOnTimeMin - timeOn;
}

void turnOnPump () {
  digitalWrite(relayPin3, HIGH);
  pumpOnCount++;
  pumpOn = 1;
  timeOff = millis();
  pumpOffTimes[fiveOff] = timeOff - timeOn;
  //push new off time to the avg
  //aveOff.push(((float)(pumpOffTimes[fiveOff] )) * (0.00001666666));
  now1 = micros();
  aveOff.push(((float)(pumpOffTimes[fiveOff] )) / (1000));
  divideTime = micros() - now1;
  //aveOff.push((float)(pumpOffTimes[fiveOff]));
  fiveOff++; if(fiveOff > 4){fiveOff = 0;}
}

void turnOffPump () {
  digitalWrite(relayPin3, LOW);
  pumpOn = 0;
  timeOn = millis();
  pumpOnTimes[fiveOn] = timeOn - timeOff;
  //push new on time to the avg
  //aveOn.push((float)(pumpOnTimes[fiveOn] ) / (60000));
  //aveOn.push(((float)(pumpOnTimes[fiveOn] )) * (0.00001666666));
  now1 = micros();
  aveOn.push(((float)(pumpOnTimes[fiveOn] )) * (0.001));
  multiplyTime = micros() - now1;
  fiveOn++; if(fiveOn > 4){fiveOn = 0;}
}

void myDelay () {
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
  // general delay here 
  delay(250);
}

const float pi = 3.1415926;

bool checkThrottle(unsigned long throttle, int dog){
  // if the pump is on
  if(pumpOn == 1){
  // calculate how long the pump is on
    timeOn = millis();
    pumpOnTimes[fiveOn] = timeOn - timeOff;
    // if the pump on time is less than the minimum pump on time
    if(pumpOnTimes[fiveOn] < pumpOnTimeMin){
    // return zero or 'not ok'
      return 0;
    }
  }
  // if the pump is off
  if(pumpOn == 0){
  // calculate how long the pump has been off
    timeOff = millis();
    pumpOffTimes[fiveOff] = timeOff - timeOn;
    // if the pump off time is less than the minimum pump off time
    if(pumpOffTimes[fiveOff] < pumpOffTimeMin){
    // return zero or 'not ok'
      return 0;
    }
  }
  if( millis() > throttle ) {
    // return one or 'ok'
    return 1;
  }
  else if( dog > 3000 ){
    // return one or 'ok'
    return 1;
  }
  else{
    return 0;
  }
}

bool serialcheckThrottle(unsigned long throttle, int dog){

  if( millis() > throttle ) {
    // return one or 'ok'
    return 1;
  }
  else if( dog > 40 ){
    // return one or 'ok'
    return 1;
  }
  else{
    return 0;
  }
}

void directOutput ( int inputValue ) {
  // map it to the range of the analog out:
  outputValue = map(inputValue, 0, 1023, 0, 255);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);
}

void printOutput () {
  // ardushipper 
     // make a string for assembling the data to log:
  /*dataString += String(int((10 * moisture1)));*/
  /*dataString += String(int((10 * moisture2)));*/
    /*dataString += ",";*/
  /*dataString += String(int((10 * moisture3)));*/
    /*dataString += ",";*/
  /*dataString += String(int((10 * moisture4)));*/
    /*dataString += ",";*/
  /*dataString += String(int((10 * humidity)));*/
    /*dataString += ",";*/
  /*dataString += String(int((10 * temperature)));*/
    /*dataString += ",";*/
  /*dataString += String(t);*/
  // Serial.print("DHT1122-DHTstatus ");
  dataString = "DHT1122-DHTstatus ";
  // Serial.println(dht.getStatusString());
  dataString += String(int((dht.getStatusString())));
  dataString += "\r\n";
  // Moisture
  //Serial.print("DHT1122-Moisture1 ");
  dataString += "DHT1122-Moisture1 ";
  //Serial.println(sensorValue1);
  dataString += String(int((sensorValue1)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Moisture2 ");
  dataString += "DHT1122-Moisture2 ";
  //Serial.println(sensorValue2);
  dataString += String(int((sensorValue2)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Moisture3 ");
  dataString += "DHT1122-Moisture3 ";
  //Serial.println(sensorValue3);
  dataString += String(int((sensorValue3)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Moisture4 ");
  dataString += "DHT1122-Moisture4 ";
  //Serial.println(sensorValue4);
  dataString += String(int((sensorValue4)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Moisture5 ");
  dataString += "DHT1122-Moisture5 ";
  //Serial.println(sensorValue5);
  dataString += String(int((sensorValue5)));
  //Serial.print("DHT1122-Humidity ");
  dataString += "DHT1122-Humidity ";
  //Serial.println(humidity, 1);
  dataString += String(int((humidity)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Celsius ");
  dataString += "DHT1122-Celsius ";
  //Serial.println(temperature);
  dataString += String(int((temperature)));
  dataString += "\r\n";
  //Serial.print("DHT1122-Fahrenheit ");
  dataString += "DHT1122-Fahrenheit ";
  //Serial.println(dht.toFahrenheit(temperature), 1);
  dataString += String(int((dht.toFahrenheit(temperature))));
  dataString += "\r\n";
  // print the results to the serial monitor:
  dataString += "DHT1122 ";
  dataString += " dog = ";
  dataString += String(int((watchdog)));
  dataString += " pumpOnTimes[fiveOn] = ";
  dataString += String((pumpOnTimes[fiveOn]));
  dataString += " pumpOffTimes[fiveOff] = ";
  dataString += String((pumpOffTimes[fiveOff]));
  dataString += " sensorHi = ";
  dataString += String(int((sensorHighValue)));
  dataString += " sensorLo = ";
  dataString += String(int((sensorLowValue)));
  dataString += "\r\n";
  // And show some interesting results.
  dataString += "DHT1122 ";
  dataString += " aveOn= ";
  dataString += "Mean:   "; 
  dataString += String(int((aveOn.mean())));
  dataString += "Mode:   "; 
  dataString += String(int((aveOn.mode())));
  dataString += "StdDev: "; 
  dataString += String(int((aveOn.stddev())));
  // And show some interesting results.
  dataString += "DHT1122 ";
  dataString += " aveOff= ";
  dataString += "Mean:   "; 
  dataString += String(int(aveOff.mean()));
  dataString += "Mode:   "; 
  dataString += String(int(aveOff.mode()));
  dataString += "StdDev: "; 
  dataString += String(int(aveOff.stddev()));
  dataString += "dryL= ";
  dataString += String(dryLimit);
  dataString += "wetL= ";
  dataString += String(wetLimit);
  dataString += "\r\n";
  dataString += "countOff= ";
  dataString += String((countdownOff()));
  dataString += "onCountMin= ";
  dataString += String(onCountMin());
  dataString += "pumpOnCount= ";
  dataString += String(pumpOnCount);
  dataString += "PIDoutput= ";
  dataString += String(Output);
  dataString += "\r\n";
  dataString += "3478-ENDTRANSMISSION";
  dataString += "\r\n";
  Serial.print(dataString);
  if(countZero == 0){
   /*backgroundColor = ST7735_BLACK;*/
   /*foregroundColor = ST7735_WHITE;*/
  }
  if(flop) {
    /*tft.invertDisplay(true);*/
    flop = 0;
  }
  else {
    /*tft.invertDisplay(false);*/
    flop = 1;
  }
  // large block of text
  /*tft.fillScreen(ST7735_BLACK);*/
  /*tft.setCursor(0, 0);*/
  /*tft.setTextSize(1);*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.setTextWrap(true);*/
  /*tft.print("sensor= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(sensorValue1);*/
  /*tft.setTextColor(ST7735_RED);*/
  /*tft.print("OnTime= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(pumpOnTimes[fiveOn]);*/
  /*tft.setTextColor(ST7735_MAGENTA);*/
  /*tft.print("OffTime= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(pumpOffTimes[fiveOff]);*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("dryL= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.print(dryLimit);*/
  /*tft.setTextColor(ST7735_MAGENTA);*/
  /*tft.print("wetL= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(wetLimit);*/
  /*tft.setTextColor(ST7735_YELLOW);*/
  /*tft.print("Hi= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(sensorHighValue);*/
  /*tft.setTextColor(ST7735_RED);*/
  /*tft.print("Lo= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(sensorLowValue);*/
  /*tft.setTextColor(ST7735_RED);*/
  /*tft.print("aveOn= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(aveOn.mean());*/
  /*tft.setTextColor(ST7735_RED);*/
  /*tft.print("aveOff= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(aveOff.mean());*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("countOn= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(countdownOn());*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("countOff= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(countdownOff());*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("onCountMin= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(onCountMin());*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("pumpOnCount= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(pumpOnCount);*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("PIDoutput= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(Output);*/
  /*tft.setTextColor(ST7735_GREEN);*/
  /*tft.print("MultiplyTime= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(multiplyTime);*/
  /*tft.print("DivideTime= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.println(divideTime);*/
  /*tft.setTextColor(ST7735_MAGENTA);*/
  /*tft.print("PIDpumpOn= ");*/
  /*tft.setTextColor(ST7735_WHITE);*/
  /*tft.print(PIDpumpOn);*/
} 


int SoilMoisture(){
  int reading;

  // drive a current through the divider in one direction
  digitalWrite(divider_top,LOW);
  digitalWrite(divider_bottom,HIGH);

  // wait a moment for capacitance effects to settle
  delay(200);

  // take a reading
  reading=analogRead(moisture_input);

  // reverse the current
  digitalWrite(divider_top,HIGH);
  digitalWrite(divider_bottom,LOW);

  // give as much time in 'reverse' as in 'forward'
  delay(200);

  // stop the current
  digitalWrite(divider_bottom,LOW);

  return reading;
}

// LCD init END

void setup() {
  // Start the I2C Bus as Slave  
  Wire.begin(SlaveDeviceId);      // join i2c bus with Slave ID
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveDataPacket); // register talk event
  // Attach a function to trigger when something is requested.
  Wire.onRequest(slavesRespond);  // register callback event
  // initialize serial communications at 9600 bps:
  throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
  serialthrottleTime = (millis() + 5);
  secondTime = (millis() + 1000); //1,000 ms = 1 second
  // Open serial communications and wait for port to open:
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  setTime(0);
  // temperature sensor setup
  dht.setup(DHT_PIN); // data pin
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();

  // Soil Moisture sensor
  // set driver pins to outputs
  pinMode(divider_top,OUTPUT);
  pinMode(divider_bottom,OUTPUT);

  // Relay Pins
  pinMode(relayPin1, OUTPUT);
  digitalWrite(relayPin1, LOW);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin2, LOW);
  pinMode(relayPin3, OUTPUT);
  digitalWrite(relayPin3, LOW);
  pinMode(relayPin4, OUTPUT);
  digitalWrite(relayPin4, LOW);
  pinMode(relayPin5, OUTPUT);
  digitalWrite(relayPin5, LOW);
  pinMode(relayPin6, OUTPUT);
  digitalWrite(relayPin6, LOW);
  pinMode(relayPin7, OUTPUT);
  digitalWrite(relayPin7, LOW);
  pinMode(relayPin8, OUTPUT);
  digitalWrite(relayPin8, LOW);
  timeOn = millis();
  delay(2);
  timeOff = millis();
  delay(2);
  pumpOn = 0;
  turnOnPump();
  printOutput();
  // LCD setup START
  //Serial.print("Hello! ST7735 TFT Test");

  // Use this initializer if you're using a 1.8" TFT
  /*tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab*/

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  //Serial.println("Initialized");

  uint16_t time = millis();
  /*tft.fillScreen(ST7735_BLACK);*/
  time = millis() - time;

  //Serial.println(time, DEC);
  delay(500);

  // large block of text
  /*tft.fillScreen(ST7735_BLACK);*/
  // testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE);
  // delay(1000);
  // tft.fillScreen(ST7735_BLACK);
  // turnOffPump();

  // PID start
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 512;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  // PID end

  /*turnOffPump();*/
  /*delay(1000);*/
  /*turnOnPump();*/
  /*delay(1000);*/
  /*turnOnPump();*/
  /*delay(1000);*/
  /*turnOffPump();*/
  /*delay(1000);*/
  /*turnOnPump();*/
  /*delay(1000);*/
  /*turnOffPump();*/
  /*delay(1000);*/
  /*turnOffPump();*/

}

void loop() {

  countZero++;
  //  PID start
  Input = analogRead(PIN_INPUT);
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) {
  //  digitalWrite(RELAY_PIN, HIGH);
    PIDpumpOn = 1;
  }
  else {
  //  digitalWrite(RELAY_PIN, LOW);
    PIDpumpOn = 0;
  }
  // PID end

  /*int minat = 0;*/
  /*int maxat = 0;*/
  watchdog++;
  watchdog2++;
  // read the analog in value:
  //sensorValue1 = analogRead(analogInPin1);
  sensorValue1 = SoilMoisture(); // assign the result of SoilMoisture() to the global variable 'moisture'
  // old method
  /*sensorValue2 = analogRead(analogInPin2);*/
  /*sensorValue3 = analogRead(analogInPin3);*/
  /*sensorValue4 = analogRead(analogInPin4);*/
  /*sensorValue5 = analogRead(analogInPin5);*/
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  //directOutput(sensorValue1);

  ok = checkThrottle( throttleTime, watchdog );
  ok2 = serialcheckThrottle( serialthrottleTime, watchdog2 );
  // Serial.print(ok);
  // Serial.println(" = ok");
  if(ok2 == 1) {
    printOutput();
    ok2 = 0;
    serialthrottleTime = (millis() + 5000); // 5,000 ms = 5 seconds
    watchdog2 = 0;
  }

  if(ok == 1) {
    ok = 0;
    watchdog = 0;
    throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
    // secondTime = (millis() + 1000); //1,000 ms = 1 second

    // eventual functualize this next block
    // checkSensors( sensorValue1 );
    //
    if(sensorValue1 < dryLimit) {
      turnOnPump();
      // dryLimit = dryLimit + nudge;
      dryLimit = dryLimit - nudge - ( 0.25 * (dryLimit - sensorValue1));
      //Serial.print("pumpon");
      //printOutput();
    }
    if(sensorValue1 > wetLimit) {
      turnOffPump();
      // wetLimit = wetLimit - nudge;
      wetLimit = wetLimit + nudge + ( 0.25 * (sensorValue1 - wetLimit));
      //Serial.print("pumpoff");
      //printOutput();
    }

    if(sensorValue1 > sensorHighValue) {
      sensorHighValue = sensorValue1;
    }
    if(sensorValue1 < sensorLowValue) {
      sensorLowValue = sensorValue1;
    }
    if(pumpOn == 1){
      timeOn = millis();
      pumpOnTimes[fiveOn] = timeOn - timeOff;
      if(pumpOnTimes[fiveOn] > pumpOnTimeMax){
        wetLimit = wetLimit - yank - ( 0.5 * (wetLimit - sensorValue1));
        turnOffPump();
        //Serial.print("pumpon by min");
        //printOutput();
      }
    }
    if(pumpOn == 0){
      timeOff = millis();
      pumpOffTimes[fiveOff] = timeOff - timeOn;
      if(pumpOffTimes[fiveOff] > pumpOffTimeMax){
        dryLimit = dryLimit + yank + ( 0.5 * (sensorValue1 - dryLimit));
        turnOnPump();
        //Serial.print("pumpon by max");
        //printOutput();
      }
    }
    // end giant block

    if( secondTime < millis() ) {
      secondTime = (millis() + lcdInterval);
      //printOutput();
    }
    else {
     // Serial.print("secondTime = ");
     // Serial.print(secondTime);
     // Serial.print(" time = ");
     // Serial.print(millis());
     // Serial.println("skip");
    }
  }

    myDelay();
} //end loop
