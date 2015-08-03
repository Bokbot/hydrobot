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
const int pumpOnTimeMax = 1800000; // maximum time pump should be on in ms 1,800,000 ms = 30 minutes
const int pumpOffTimeMax = 43200000; // maximum time pump should be off in ms 43,200,000 ms = 12 hours

bool ok;
bool pumpOn;

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

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  timeOn = millis();
  delay(2);
  timeOff = millis();
  pumpOn = 0;
  throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
  secondTime = (millis() + 1000); //1,000 ms = 1 second
}

void loop() {
  watchdog++;
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  directOutput(sensorValue);

  ok = checkThrottle( throttleTime, watchdog );
  // Serial.print(ok);
  // Serial.println(" = ok");

  if(ok == 1) {
    Serial.print("throttleTime = ");
    Serial.print(throttleTime);
    Serial.print(" time = ");
    Serial.print(millis());
    Serial.println("skip");

    ok = 0;
    watchdog = 0;
    throttleTime = (millis() + 30000); // 30,000 ms = 30 seconds
    // secondTime = (millis() + 1000); //1,000 ms = 1 second


    // eventual functualize this next block
    // checkSensors( sensorValue );
    //
    if(sensorValue > 599) {
        digitalWrite(13, HIGH);
        pumpOn = 1;
        timeOn = millis();
        pumpOnTimes[fiveOn] = timeOn - timeOff;
        fiveOn++; if(fiveOn > 4){fiveOn = 0;}
    }
    if(sensorValue < 475) {
        digitalWrite(13, LOW);
        pumpOn = 0;
        timeOff = millis();
        pumpOffTimes[fiveOff] = timeOff - timeOn;
        fiveOff++; if(fiveOff > 4){fiveOff = 0;}
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
  if(pumpOn = 1){
    unsigned int pumpOnTimeCurrent;
    pumpOnTimeCurrent = timeOn - timeOff;
    if(pumpOnTimeCurrent > pumpOnTimeMax){
        digitalWrite(13, LOW);
        pumpOn = 0;
        timeOff = millis();
        pumpOffTimes[fiveOff] = timeOff - timeOn;
        fiveOff++; if(fiveOff > 4){fiveOff = 0;}
    }
  }
  if(pumpOn = 0){
    unsigned int pumpOffTimeCurrent;
    pumpOffTimeCurrent = timeOff - timeOn;
    if(pumpOffTimeCurrent > pumpOffTimeMax){
      digitalWrite(13, HIGH);
      pumpOn = 1;
      timeOn = millis();
      pumpOnTimes[fiveOn] = timeOn - timeOff;
      fiveOn++; if(fiveOn > 4){fiveOn = 0;}
    }
  }
  // end giant block

  if( secondTime < millis() ) {
    secondTime = (millis() + 1000);
    // print the results to the serial monitor:
    Serial.print("sensor = ");
    Serial.print(sensorValue);
    Serial.print(" sensorHi = ");
    Serial.print(sensorHighValue);
    Serial.print(" sensorLo = ");
    Serial.println(sensorLowValue);
  }
  else {
   // Serial.print("secondTime = ");
   // Serial.print(secondTime);
   // Serial.print(" time = ");
   // Serial.print(millis());
   // Serial.println("skip");
  }

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
  // general delay here 
  delay(250);

} //end loop

bool checkThrottle(unsigned long throttle, int dog){
  if(throttle < millis()){
    return 1;
    Serial.println("dont skip");
  }
  else if(dog > 3000){
    return 1;
    Serial.println("dog dont skip");
  }
  else{
    return 0;
    Serial.println("skip");
  }
}

void directOutput ( int inputValue ) {
  // map it to the range of the analog out:
  outputValue = map(inputValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);
}
