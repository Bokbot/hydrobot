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
const int ledPin = 13;      // select the pin for the LED
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

bool ok;

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int sensorLowValue = 1023;
int sensorHighValue = 0;
int sensorLastLowValue = 0;
int sensorLastHighValue = 0;

unsigned int count;
unsigned int fiveOn;
unsigned int fiveOff;

unsigned long now;
unsigned long timeOn;
unsigned long timeOff;
unsigned long secondTime;
unsigned long throttleTime;

unsigned long pumpOnTimes[5];
unsigned long pumpOffTimes[5];

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  timeOn = millis();
  timeOff = millis();
}

void loop() {
  count++;
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);
  if((throttleTime < millis()) || (count > 100000)){
    ok = 1;
    count = 0;
    throttleTime = (millis() + 60000);
    secondTime = (millis() + 1000);
  }
  if(ok = 1) {
    ok = 0;
    if(sensorValue > 599) {
        digitalWrite(13, HIGH);
        timeOn = millis();
        pumpOffTimes[fiveOn] = timeOn - timeOff;
        fiveOn++; if(fiveOn > 4){fiveOn = 0;}
    }
    if(sensorValue < 475) {
        digitalWrite(13, LOW);
        timeOff = millis();
        pumpOffTimes[fiveOff] = timeOff - timeOn;
        fiveOff++; if(fiveOff > 4){fiveOff = 0;}
    }
  }
  if(sensorValue > sensorHighValue) {
    sensorHighValue = sensorValue;
  }
  if(sensorValue < sensorLowValue) {
    sensorLowValue = sensorValue;
  }

  if(secondTime < millis()){
    secondTime = (millis() + 1000);
    // print the results to the serial monitor:
    Serial.print("sensor = ");
    Serial.print(sensorValue);
    Serial.print("sensorHi = ");
    Serial.print(sensorHighValue);
    Serial.print("sensorLo = ");
    Serial.println(sensorLowValue);
  }

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}

