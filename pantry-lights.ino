// include i2c (2-wire) library
#include <Wire.h>

// the address setting of the eeprom chip
// bit order is A2, A1, A0
#define EEPROM_ADDRESS_SETTING 0b000
// make the full address byte for i2c to communicate with the eeprom chip
#define EEPROM_ADDRESS (0b1010 << 3) + EEPROM_ADDRESS_SETTING

// constants
// rotary dial clock pin
const int rotaryClockPin = 8;
// rotary dial direction pin
const int rotaryDirectionPin = 7;
// rotary dial switch pin - int0
const int rotarySwitchPin = 2;
// hall effect sensor pin - int1
const int hallSensorPin = 3;
// red color pin
const int redLedPin = 9;
// green color pin
const int greenLedPin = 10;
// blue color pin
const int blueLedPin = 11;
// maximum rotary value
const int rotaryMax = 255;
// delay/debounce in ms for reading the rotary position data
const int rotaryDelay = 10;
// delay/debounce in ms for reading the rotary switch button presses
const int rotarySwitchDebounce = 100;

// variables
// rotary dial position
int rotaryValue = 0;
// rotary dial clock state
bool rotaryClockState = LOW;
// rotary dial last clock state
bool rotaryClockLastState = HIGH;
// rotary dial switch state
bool rotarySwitchState = HIGH;
// rotary dial switch last state
bool rotarySwitchLastState = HIGH;
// last time the switch was pressed
long lastRotarySwitchMicros = 0L;
// last rotary dial direction - high for forward (clockwise), low for backward (counter-clockwise)
// bool rotaryDirection = HIGH;
// the current max brightness for the LEDs
int maxBright = 180;

// volatile vars, shared between interrupt routines and main routine
// what mode we are currently in. 0 = normal, 1 = adjust brightness, 2 = adjust color
volatile int mode = 0;

// interrupt function to change modes when rotary switch is pressed
void changeMode () {
  // get time now
  long now = micros();
  // calculate 125ms delay in microseconds for debounce
  long delay = 1000L * 125L;
  if (now > lastRotarySwitchMicros + delay) {
    // valid button press
    // leaving adjust brightness mode?
    if (mode == 1) {
      // save brightness setting in EEPROM
      writeI2CByte(0, maxBright);
    }
    // change mode from 0 -> 1 -> 2 -> 0 -> 1 etc...
    if (mode == 2) {
      mode = 0;
    } else {
      mode++;
    }
    Serial.print("changing modes to ");
    Serial.println(mode);
    // set last press time to now
    lastRotarySwitchMicros = now;
  }
}

// interrupt function to detect when the door opens or closes
void detectDoor () {
  Serial.println("detect door change!");
  // debounce?
  delay(100);
}

void setup() {
  // init serial output
  Serial.begin(9600);
  // hello
  Serial.println("Pantry Lights System Activated");
  // start i2c
  Wire.begin();
  Serial.println("I2C started");
  // get max brightness value from eeprom
  maxBright = readI2CByte(0);
  Serial.print("maxBright from EEPROM is ");
  Serial.println(maxBright);
  // set rotary dial pin modes
  pinMode(rotaryClockPin, INPUT);
  pinMode(rotaryDirectionPin, INPUT);
  // rotary dial switch needs pullup
  pinMode(rotarySwitchPin, INPUT_PULLUP);
  // Declaring red LED pin as output
  pinMode(redLedPin, OUTPUT);
  // Declaring green LED pin as output
  pinMode(greenLedPin, OUTPUT);
  // Declaring blue LED pin as output
  pinMode(blueLedPin, OUTPUT);
  // interrupt when rotary switch button is pressed
  attachInterrupt(digitalPinToInterrupt(rotarySwitchPin), changeMode, FALLING);
  // interrupt when hall effect sensor detects change
  // attachInterrupt(digitalPinToInterrupt(hallSensorPin), detectDoor, RISING);
  // start with all LEDs off
  allOff();
  // open the door to the pantry
  // openDoor();
}

void loop() {
  // check the mode
  if (mode == 0) {
    // normal mode
    // openDoor();
    // delay(3000);
    // closeDoor();
    // delay(3000);
    analogWrite(redLedPin, maxBright*2/2);
    analogWrite(greenLedPin, maxBright*2/5);
    analogWrite(blueLedPin, 0);
  } else if (mode == 1) {
    // adjust brightness
    // set rotary value to the value of current max brightness
    rotaryValue = maxBright;
    // check rotary position
    readRotary();
    // set maxBright to rotary value
    maxBright = rotaryValue;
    // Serial.print("Rotary position ");
    // Serial.println(rotaryValue);
    // set LEDs to max brightness
    analogWrite(redLedPin, maxBright*2/2);
    analogWrite(greenLedPin, maxBright*2/5);
    analogWrite(blueLedPin, 0);
  } else if (mode == 2) {
    // adjust color
  } else {
    // oops! should not be here!
    // reset mode to 0!
    mode = 0;
  }
  // continually read the rotary dial data
  // readRotary();
  // log rotary position value
  // Serial.print("Rotary position ");
  // Serial.println(rotaryValue);
  // delay before accepting more input
  // delay(rotaryDelay);
  // fadeLed(redLedPin);
  // fadeLed(greenLedPin);
  // fadeLed(blueLedPin);
  
  // just red
  // actually this makes blue?
  // analogWrite(redLedPin, 100);
  // white wire

  // this is actually red
  // analogWrite(greenLedPin, 100);
  // this is the green wire

  // so blueLedPin is green
  // this is the red wire
}

void openDoor () {
  for(int i=0; i<maxBright*2; i++){
    analogWrite(redLedPin, i/2);
    analogWrite(greenLedPin, i/5);
    analogWrite(blueLedPin, 0);
    delay(5);
  }
}

void closeDoor () {
  for(int i=maxBright*2; i>0; i--){
    analogWrite(redLedPin, i/2);
    analogWrite(greenLedPin, i/5);
    analogWrite(blueLedPin, 0);
    delay(5);
  }

  // make sure they are all off
  allOff();
}

void allOff () {
  analogWrite(redLedPin, 0);
  analogWrite(greenLedPin, 0);
  analogWrite(blueLedPin, 0);
}

void fadeLed (int pin) {
  //Fading the LED
  for(int i=0; i<255; i++){
    analogWrite(pin, i);
    delay(5);
  }
  for(int i=255; i>0; i--){
    analogWrite(pin, i);
    delay(5);
  }
}

void readRotary( ) {
  // gestion position
  rotaryClockState = digitalRead(rotaryClockPin);
  // did rotary dial move?
  if ((rotaryClockLastState == LOW) && (rotaryClockState == HIGH)) {
    //rotary moving
    // forward or backward?
    if (digitalRead(rotaryDirectionPin) == HIGH) {
      // backward/left/counter-clockwise
      // Serial.println("rotary dial left");
      rotaryValue = rotaryValue - 1;
      if ( rotaryValue < 0 ) {
        rotaryValue = 0;
      }
    } else {
      // forward/right/clockwise
      // Serial.println("rotary dial right");
      rotaryValue++;
      if ( rotaryValue > rotaryMax ) {
        rotaryValue = rotaryMax;
      }
    }
    Serial.print("Rotary position ");
    Serial.println(rotaryValue);
  }
  rotaryClockLastState = rotaryClockState;
  // //gestion bouton
  // rotarySwitchState = digitalRead(rotarySwitchPin);
  // if (rotarySwitchState == LOW && rotarySwitchLastState == HIGH) {
  //   Serial.println("Rotary pressed");
  //   delay(rotarySwitchDebounce);//debounce
  // }
  // rotarySwitchLastState = rotarySwitchState;
}

// this function writes a byte to the i2c eeprom
void writeI2CByte (byte dataAddress, byte data) {
  Serial.print("writing ");
  Serial.print(data);
  Serial.print(" to EEPROM address ");
  Serial.println(dataAddress);
  // wait for serial message  to flush??
  delay(100);
  // send I2C message for EEPROM to save data
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write(dataAddress);
  Wire.write(data);
  Wire.endTransmission();
  // wait for data to be sent?
  delay(100);
  // log
  Serial.print("wrote ");
  Serial.print(data);
  Serial.print(" to EEPROM address ");
  Serial.println(dataAddress);
  // wait for transmission to be received
  delay(5);
}

// this function reads a byte from the i2c eeprom
byte readI2CByte(byte dataAddress) {
  // init return var
  byte data = NULL;
  // start i2c transmission
  Wire.beginTransmission(EEPROM_ADDRESS);
  // send i2c address
  Wire.write(dataAddress);
  Wire.endTransmission();
  // request 1 byte from eeprom
  Wire.requestFrom(EEPROM_ADDRESS, 1);
  // wait for transmission to be received
  delay(1);
  // did we get data?
  if (Wire.available()) {
    data = Wire.read();
  }
  // Serial.print("read ");
  // Serial.print(data);
  // Serial.print(" from EEPROM address ");
  // Serial.println(dataAddress);
  return data;
}