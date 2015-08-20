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

// These constants won't change.  They're used to give names
// to the pins used:
const int nudge = 3;      // how much to nudge our value forward
const int yank = 11;      // how much to nudge our value forward
const int ledPin = 7;      // select the pin for the LED
const int relayPin = 6;      // select the pin for the LED
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
const unsigned long pumpOnTimeMax = 1800000; // maximum time pump should be on in ms 1,800,000 ms = 30 minutes
const unsigned long pumpOffTimeMax = 43200000; // maximum time pump should be off in ms 43,200,000 ms = 12 hours
const unsigned long pumpOnTimeMin = 600000; // minimum time pump should be on in ms 600,000 ms = 10 minutes
const unsigned long pumpOffTimeMin = 600000; // minimum time pump should be off in ms 600,000 ms = 10 minutes

bool ok;
bool flop;
bool pumpOn;


int dryLimit = 599;        // this is the value of dryness we don't want to exceed
int wetLimit = 320;        // this is the vale of wetness we don't want to go above (below 320 is wetter)
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int sensorLowValue = 1023;
int sensorHighValue = 0;
int sensorLastLowValue = 0;
int sensorLastHighValue = 0;

unsigned int watchdog;
unsigned int fiveOn;
unsigned int fiveOff;

unsigned long now;
unsigned long timeOn;
unsigned long timeOff;
unsigned long secondTime;
unsigned long throttleTime;

unsigned long pumpOnTimes[5];
unsigned long pumpOffTimes[5];

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

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>


// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


float p = 3.1415926;

// LCD init END

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  timeOn = millis();
  delay(2);
  timeOff = millis();
  delay(2);
  pumpOn = 0;
  throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
  secondTime = (millis() + 1000); //1,000 ms = 1 second
  turnOnPump();
  // LCD setup START
  Serial.print("Hello! ST7735 TFT Test");

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  uint16_t time = millis();
  tft.fillScreen(ST7735_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  tft.fillScreen(ST7735_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE);
  delay(1000);

}

void loop() {
  watchdog++;
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  //directOutput(sensorValue);

  ok = checkThrottle( throttleTime, watchdog );
  // Serial.print(ok);
  // Serial.println(" = ok");

  if(ok == 1) {

    ok = 0;
    watchdog = 0;
    throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
    // secondTime = (millis() + 1000); //1,000 ms = 1 second


    // eventual functualize this next block
    // checkSensors( sensorValue );
    //
    if(sensorValue > dryLimit) {
      turnOnPump();
      dryLimit = dryLimit + nudge;
      Serial.print("pumpon");
      printOutput();
    }
    if(sensorValue < wetLimit) {
      turnOffPump();
      wetLimit = wetLimit - nudge;
      Serial.print("pumpoff");
      printOutput();
    }
  }
  else {
    /*Serial.print("throttleTime = ");*/
    /*Serial.print(throttleTime);*/
    /*Serial.print(" time = ");*/
    /*Serial.print(millis());*/
    /*Serial.println("skip");*/
  }
  if(sensorValue > sensorHighValue) {
    sensorHighValue = sensorValue;
  }
  if(sensorValue < sensorLowValue) {
    sensorLowValue = sensorValue;
  }
  if(pumpOn == 1){
    timeOn = millis();
    pumpOnTimes[fiveOn] = timeOn - timeOff;
    if(pumpOnTimes[fiveOn] > pumpOnTimeMax){
      wetLimit = wetLimit + yank;
      turnOffPump();
      Serial.print("pumpon by min");
      printOutput();
    }
  }
  if(pumpOn == 0){
    timeOff = millis();
    pumpOffTimes[fiveOff] = timeOff - timeOn;
    if(pumpOffTimes[fiveOff] > pumpOffTimeMax){
      dryLimit = dryLimit - yank;
      turnOnPump();
      Serial.print("pumpon by max");
      printOutput();
    }
  }
  // end giant block

  if( secondTime < millis() ) {
    secondTime = (millis() + 5000);
    printOutput();
  }
  else {
   // Serial.print("secondTime = ");
   // Serial.print(secondTime);
   // Serial.print(" time = ");
   // Serial.print(millis());
   // Serial.println("skip");
  }

  myDelay();

} //end loop

bool checkThrottle(unsigned long throttle, int dog){
  if(pumpOn == 1){
    pumpOnTimes[fiveOn] = timeOn - timeOff;
    if(pumpOnTimes[fiveOn] < pumpOnTimeMin){
      return 0;
    }
  }
  if(pumpOn == 0){
    pumpOffTimes[fiveOff] = timeOff - timeOn;
    if(pumpOffTimes[fiveOff] < pumpOffTimeMin){
      return 0;
    }
  }
  if( millis() > throttle ) {
    return 1;
  }
  else if( dog > 3000 ){
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
  // print the results to the serial monitor:
  Serial.print(" sensor = ");
  Serial.print(sensorValue);
  Serial.print(" dog = ");
  Serial.print(watchdog);
  Serial.print(" pumpOnTimes[fiveOn] = ");
  Serial.print(pumpOnTimes[fiveOn]);
  Serial.print(" pumpOffTimes[fiveOff] = ");
  Serial.print(pumpOffTimes[fiveOff]);
  Serial.print(" sensorHi = ");
  Serial.print(sensorHighValue);
  Serial.print(" sensorLo = ");
  Serial.println(sensorLowValue);
  if(flop) {
    //tft.invertDisplay(true);
    flop = 0;
  }
  else {
    //tft.invertDisplay(false);
    flop = 1;
  }
  // large block of text
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextWrap(true);
  tft.print(" sensor= ");
  tft.print(sensorValue);
  tft.setTextColor(ST7735_RED);
  tft.print(" dog= ");
  tft.print(watchdog);
  tft.setTextColor(ST7735_BLUE);
  tft.print(" pumpOnTimes[fiveOn]= ");
  tft.print(pumpOnTimes[fiveOn]);
  tft.setTextColor(ST7735_MAGENTA);
  tft.print(" pumpOffTimes[fiveOff]= ");
  tft.print(pumpOffTimes[fiveOff]);
  tft.setTextColor(ST7735_YELLOW);
  tft.print(" sensorHi= ");
  tft.print(sensorHighValue);
  tft.setTextColor(ST7735_GREEN);
  tft.print(" dryL= ");
  tft.print(dryLimit);
  tft.setTextColor(ST7735_MAGENTA);
  tft.print(" wetL= ");
  tft.print(wetLimit);
  tft.setTextColor(ST7735_RED);
  tft.print(" sensorLo= ");
  tft.println(sensorLowValue);
}

void turnOffPump () {
  digitalWrite(relayPin, LOW);
  pumpOn = 0;
  timeOff = millis();
  pumpOffTimes[fiveOff] = timeOff - timeOn;
  fiveOff++; if(fiveOff > 4){fiveOff = 0;}
}

void turnOnPump () {
  digitalWrite(relayPin, HIGH);
  pumpOn = 1;
  timeOn = millis();
  pumpOnTimes[fiveOn] = timeOn - timeOff;
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

void testlines(uint16_t color) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=tft.width()-1; x > 6; x-=6) {
    tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < tft.width(); x+=radius*2) {
    for (int16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  tft.fillScreen(ST7735_BLACK);
  int color = 0xF800;
  int t;
  int w = tft.width()/2;
  int x = tft.height()-1;
  int y = 0;
  int z = tft.width();
  for(t = 0 ; t <= 15; t+=1) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
}

void testroundrects() {
  tft.fillScreen(ST7735_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST7735_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST7735_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST7735_WHITE);
  tft.print(" seconds.");
}

void mediabuttons() {
  // play
  tft.fillScreen(ST7735_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ST7735_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, ST7735_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_GREEN);
}
