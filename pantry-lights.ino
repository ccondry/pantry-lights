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
// delay/debounce in ms for reading the rotary position data
const int rotaryDelay = 10;
// delay/debounce in ms for reading the rotary switch button presses
const int rotarySwitchDebounce = 100;
// how much to adjust the value when moving rotary dial one notch
const int rotaryUnits = 3;

// variables
// rotary dial position
int rotaryValue = 0;
// maximum rotary value
int rotaryMax = 255;
// minimum rotary value
int rotaryMin = 0;
// rotary dial clock state
bool rotaryClockState = LOW;
// rotary dial last clock state
bool rotaryClockLastState = HIGH;
// rotary dial switch state
bool rotarySwitchState = HIGH;
// rotary dial switch last state
bool rotarySwitchLastState = HIGH;
// last time the switch was pressed
unsigned long lastRotarySwitchMicros = 0L;
// last rotary dial direction - high for forward (clockwise), low for backward (counter-clockwise)
// bool rotaryDirection = HIGH;
// the current max brightness for the LEDs
int maxBright = 180;
// color set index
int colorIndex = 4;

int currentBrightness = 100;
int redValue = 100;
int greenValue = 40;
int blueValue = 0;

float redMultiplier = 1.0;
float greenMultiplier = 2.0/5;
float blueMultiplier = 0;

// timer for when user has started setting the settings, but did not continue
unsigned long idleTimer = 0L;

// volatile vars, shared between interrupt routines and main routine
// what mode we are currently in. 0 = normal, 1 = adjust brightness, 2 = adjust color
volatile int mode = 0;
volatile int lastMode = 0;

// interrupt function to change modes when rotary switch is pressed
void changeMode () {
  // get time now in milliseconds
  unsigned long now = millis();
  // calculate 125ms delay in microseconds for debounce
  long debounce = 125L;
  if (now > lastRotarySwitchMicros + debounce) {
    // valid button press
    // store current mode as last mode
    lastMode = mode;
    // change mode from 0 -> 1 -> 2 -> 0 -> 1 etc...
    const int maxModes = 4;
    if (mode == maxModes) {
      mode = 0;
    } else {
      mode++;
    }
    Serial.print("changing mode to ");
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
  // test
  // writeI2CByte(0, 44);
  // get max brightness value from eeprom
  maxBright = readI2CByte(0);
  currentBrightness = readI2CByte(0);
  redValue = readI2CByte(1);
  greenValue = readI2CByte(2);
  blueValue = readI2CByte(3);

  // Serial.print("maxBright from EEPROM is ");
  // Serial.println(maxBright);
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
  delay(1000);
  // open the door to the pantry
  openDoor();
}

// blink all lights for feedback that we are done editing
void exitSettings () {
  // set mode to normal running
  mode = 0;
  // reset lastMode
  lastMode = 0;
  // turn off all lights
  setLights(currentBrightness, 0, 0, 0);
  delay(150);
  // turn on all lights to normal
  setLights();
  delay(300);
  // turn off all lights
  setLights(currentBrightness, 0, 0, 0);
  delay(150);
  // turn on all lights to normal
  setLights();
  delay(300);
  // turn off all lights
  setLights(currentBrightness, 0, 0, 0);
  delay(150);
  // turn on all lights to normal
  setLights();
  delay(300);
  // turn off all lights
  setLights(currentBrightness, 0, 0, 0);
  delay(150);
  // turn on all lights to normal
  setLights();
}

void loop() {
  // if not in normal mode, check idle timer
  if (mode != 0) {
    // get time now
    unsigned long now = millis();
    // has the user been idle in the settings menu longer than 10 seconds?
    if (now > 10 * 1000 && now - 10 * 1000 > idleTimer) {
      // user is idle - exit settings mode
      exitSettings();
    }
  }
  // was the mode changed?
  if (lastMode != mode) {
    // mode was changed
    if (lastMode == 0) {
      // was the last mode normal running?
      // calculate something dimmer than current brightness
      float brightness = currentBrightness * 0.5;
      // dim all for feedback that we editing brightness
      setLights(brightness, redValue, greenValue, blueValue);
      delay(300);
      setLights();
      delay(150);
      setLights(brightness, redValue, greenValue, blueValue);
      delay(300);
      setLights();
    } else if (lastMode == 1) {
      // was the last mode adjust brightness?
      // update max brightness setting in EEPROM
      writeI2CByte(0, currentBrightness);
      // blink red for feedback that we are now editing red
      setLights(currentBrightness, 255, 0, 0);
      delay(300);
      setLights();
      delay(150);
      setLights(currentBrightness, 255, 0, 0);
      delay(300);
      setLights();
    } else if (lastMode == 2) {
      // update red value
      writeI2CByte(1, redValue);
      // blink green for feedback that we are now editing green
      setLights(currentBrightness, 0, 255, 0);
      delay(300);
      setLights();
      delay(150);
      setLights(currentBrightness, 0, 255, 0);
      delay(300);
      setLights();
    } else if (lastMode == 3) {
      // update green value
      writeI2CByte(2, greenValue);
      // blink blue for feedback that we are now editing blue
      setLights(currentBrightness, 0, 0, 255);
      delay(300);
      setLights();
      delay(150);
      setLights(currentBrightness, 0, 0, 255);
      delay(300);
      setLights();
    } else if (lastMode == 4) {
      // update blue value
      writeI2CByte(3, blueValue);
      exitSettings();
    }
    // update lastMode to the current one
    lastMode = mode;
  }
  // check the mode
  if (mode == 0) {
    // normal mode - lights show using values in memory
    setLights();
  } else if (mode == 1) {
    // adjust brightness mode
    // set rotary value to the value of current max brightness
    rotaryValue = currentBrightness;
    // set max rotary size
    rotaryMax = 255;
    rotaryMin = 3;
    // check rotary position
    readRotary();
    // did we reach max value just now?
    if (currentBrightness < 255 && rotaryValue == 255) {
      // flash only red to feedback that red is at max value
      setLights(currentBrightness, 0, 0, 0);
      delay(250);
    }
    // set maxBright to rotary value
    currentBrightness = rotaryValue;
    setLights();
  } else if (mode == 2) {
    // adjust red mode
    // delay(100);
    // set rotary value to the value of current color index
    rotaryValue = redValue;
    // set max rotary value
    // rotaryMax = colorMultipliers;
    rotaryMax = 255;
    rotaryMin = 0;
    // check rotary position
    readRotary();
    // did we reach max value just now?
    if (redValue < 255 && rotaryValue == 255) {
      // flash only red to feedback that red is at max value
      setLights(currentBrightness, 255, 0, 0);
      delay(250);
    }
    // set red color value from rotary value
    redValue = rotaryValue;
    // was red set to the max value?
    // update lights
    setLights();
  } else if (mode == 3) {
    // adjust green mode
    // set rotary value to the value of current color index
    rotaryValue = greenValue;
    // set max rotary value
    // rotaryMax = colorMultipliers;
    rotaryMax = 255;
    rotaryMin = 0;
    // check rotary position
    readRotary();
    // did we reach max value just now?
    if (greenValue < 255 && rotaryValue == 255) {
      // flash only green to feedback that green is at max value
      setLights(currentBrightness, 0, 255, 0);
      delay(250);
    }
    // set green color value to rotary value
    greenValue = rotaryValue;
    // update lights
    setLights();
  } else if (mode == 4) {
    // adjust blue mode
    // set rotary value to the value of current color index
    rotaryValue = blueValue;
    // set max rotary value
    // rotaryMax = colorMultipliers;
    rotaryMax = 255;
    rotaryMin = 0;
    // check rotary position
    readRotary();
    // did we reach max value just now?
    if (blueValue < 255 && rotaryValue == 255) {
      // flash only blue to feedback that blue is at max value
      setLights(currentBrightness, 0, 0, 255);
      delay(250);
    }
    // set blue color value to rotary value
    blueValue = rotaryValue;
    // update lights
    setLights();
  } else {
    // oops! should not be here!
    // reset mode to 0!
    mode = 0;
  }
}

// set light brightness, red, green, and blue values
void setLights (int h, int r, int g, int b) {
  // set LED values
  float brightness = (h / 255.0);
  analogWrite(redLedPin, brightness * r);
  analogWrite(greenLedPin, brightness * g);
  analogWrite(blueLedPin, brightness * b);
}

// set light brightness only
void setLights (int b) {
  setLights(b, redValue, greenValue, blueValue);
}

// set lights to values in memory
void setLights () {
  setLights(currentBrightness);
}

// gradually bring up lights to full brightness, over 1 second or so
void openDoor () {
  // int brightnessStep = 1000.0 / currentBrightness;
  for (int i = 0; i < currentBrightness; i++) {
    setLights(i);
    delay(1000 / currentBrightness);
  }
}

// gradually dim the lights and then turn them off
void closeDoor () {
  for (int i = currentBrightness; i > 0; i--) {
    setLights(i);
    delay(1000 / currentBrightness);
  }
  // turn the lights all the way off
  allOff();
}

// turn off all lights
void allOff () {
  setLights(0);
}

void readRotary () {
  // gestion position
  rotaryClockState = digitalRead(rotaryClockPin);
  // did rotary dial move?
  if ((rotaryClockLastState == LOW) && (rotaryClockState == HIGH)) {
    // rotary has input value
    // reset the idle timer
    idleTimer = millis();
    // forward or backward?
    if (digitalRead(rotaryDirectionPin) == HIGH) {
      // backward/left/counter-clockwise
      // Serial.println("rotary dial left");
      rotaryValue -= rotaryUnits;
      if ( rotaryValue < rotaryMin ) {
        rotaryValue = rotaryMin;
      }
    } else {
      // forward/right/clockwise
      // Serial.println("rotary dial right");
      rotaryValue += rotaryUnits;
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