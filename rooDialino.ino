/*  Allows a rooDial to set Volume of an Amp or DAC by sending IR signals. 
 *  Expects pulses on Pin 17/27 of the Raspi running rooExtend, requiring rooxtend 2.3.x or later.
 *  
 *  Todo:
 *  - increase the IR power by using 2 or 3 IR diodes in series. One diode requires 1.1 to 1.5 volt so we can supply 3 @ 5V, 10-50 Ohm (test)
 *  - Test if we can live with 3.3V pulses (or need to go down to 2 LEDs. Or need a Transistor or Schmitt Trigger.
 * 
 *  Notes:
 *  - The default software generated PWM has problems on AVR running with 8 MHz. The PWM frequency is around 30 instead of 38 kHz and RC6 is not reliable. 
 *    You can switch to timer PWM generation by #define SEND_PWM_BY_TIMER
 *  
 *  Links:
 *  https://github.com/Arduino-IRremote/Arduino-IRremote
 *  https://arduino-irremote.github.io/Arduino-IRremote/group__Decoder.html#ga6168e3ad4e47c657c9f3de0e5d7590b3
 *  https://cdn.sparkfun.com/assets/c/6/2/2/1/ProMini8MHzv2.pdf
 *  https://gammon.com.au/interrupts
 *  https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
 *  
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2021 Friedemann Wachsmuth
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */
#include <Arduino.h>
#include "PinDefinitionsAndMore.h" 

#define IR_RECEIVE_PIN      7 //  Overwriting IR Pins to free up 2/3 for Interrupts
#define IR_SEND_PIN         8

#include <IRremote.h>

#define DELAY_AFTER_SEND 5  // shorter than 5 ms might make dirty signal

#define PIN2         2
#define PIN3         3
#define BUTTON_PIN   4

// LED Modes
#define OFF                40
#define ON                 41
#define FASTBLINK          42
#define BLINK              43
#define ONCE               44 
#define TWICE              45
#define THRICE             46

// Button States
#define BUTTON_IDLE        10
#define BUTTON_DOWN        11
#define BUTTON_DEBOUNCE1   12
#define BUTTON_WAIT        13
#define BUTTON_UP          14
#define BUTTON_DEBOUNCE2   15
#define BUTTON_IGNOREDOWN  16

// Settings Modes
#define SETTING_1          21
#define SETTING_2          22
#define SETTING_3          23
#define SETTINGS_EXIT      24

// States
#define LEARN_BASENOISE    30
#define LEARN_IR           31
#define TIMER_SHORT        32
#define TIMER_MID          33
#define TIMER_LONG         34


volatile int volSteps;  // keeps track of how many pulses came in from the rooDial

uint16_t sAddress; 
uint8_t sCommand; 
uint8_t sRepeats; 

// ******* LED things **************************** 

const byte ledPins[] = { 10, 11, 12};       // an array of pin numbers too which LEDs are attached
const byte ledPinCount = 3;       
byte ledMode[] = { BLINK, FASTBLINK, THRICE};
unsigned long fastblinkPrevMillis[] = { 0, 0, 0, 0 };        // will store last time LED was updated
unsigned long blinkPrevMillis[] = { 0, 0, 0, 0 };        // will store last time LED was updated
unsigned long currentMillis = 0;
int ledState[] = { LOW, LOW, LOW, LOW };             // ledState used to set the LED

const unsigned int ledSlowBlinkInterval = 200;
const unsigned int ledFastBlinkInterval = 80;
const unsigned int sampleBaseNoisePeriod = 3000;
const unsigned int learnBaseNoiseInitialDelay = 3000;
const unsigned int permanentNoiseMinLength = 5000;
const unsigned int longPressLength = 1000;
const unsigned int buttonDebounceInterval = 50;

byte ledBurstPatternCell = 0;
byte prevLedBurstPatternCell[] = { 0, 0, 0, 0 };


// ******* Button things **************************** 

unsigned long previousButtonMillis = 0;
unsigned long buttonDownMillis = 0;
unsigned long buttonUpMillis = 0;

// Button handling variables
int buttonPressLength;

boolean noCodeYetReceived = true;

// byte settingsButtonState = 0;
byte myState;
byte prevState;
byte buttonState;
byte prevButtonState;
byte learnState;



void volDownISR() {
  volSteps--;
}
void volUpISR() {
  volSteps++;
}

void setup() {
  pinMode(PIN2, INPUT_PULLUP);
  pinMode(PIN3, INPUT_PULLUP);
  pinMode(PIN3, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int thisLed = 0; thisLed < ledPinCount; thisLed++) pinMode(ledPins[thisLed], OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(PIN2), volDownISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN3), volUpISR, CHANGE);
  
  Serial.begin(115200);
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK); // Specify send pin and enable feedback LED at default feedback LED pin

  Serial.print(F("Ready to send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

#if defined(USE_SOFT_SEND_PWM) && !defined(ESP32) // for esp32 we use PWM generation by hw_timer_t for each pin
    /*
     * Print internal signal generation info
     */
    IrSender.enableIROut(38);

    Serial.print(F("Send signal mark duration is "));
    Serial.print(IrSender.periodOnTimeMicros);
    Serial.print(F(" us, pulse correction is "));
    Serial.print((uint16_t) PULSE_CORRECTION_NANOS);
    Serial.print(F(" ns, total period is "));
    Serial.print(IrSender.periodTimeMicros);
    Serial.println(F(" us"));
#endif
}



void loop() {
  currentMillis = millis();
  updateLeds();
  checkButton();
  
  // check button
  if (digitalRead(BUTTON_PIN)) { // Button is not pressed
    
  } else {  // Button is pressed

  }
  
  if (volSteps > 0) {
    sAddress = 0x16;
    sCommand = 0x10;
    sRepeats = 0;
    IrSender.sendRC5(sAddress & 0x1F, sCommand & 0x3F, sRepeats, true); // 5 address, 6 command bits
//    Serial.print(volSteps);
    volSteps--;
//    Serial.println(" Up");
    delay(DELAY_AFTER_SEND); 
  }
  if (volSteps < 0) {
    sAddress = 0x16;
    sCommand = 0x11;
    sRepeats = 0;
    IrSender.sendRC5(sAddress & 0x1F, sCommand & 0x3F, sRepeats, true); // 5 address, 6 command bits
//    Serial.print(volSteps);
    volSteps++;
//    Serial.println(" Down");
    delay(DELAY_AFTER_SEND); 
  }
}

// ****************************************************************************************************************


void checkButton() {
//  if (buttonState != prevButtonState) {
//    Serial.print("Button: ");
//    Serial.println(buttonState);
//    prevButtonState = buttonState;
//  }
  switch(buttonState) {
    case BUTTON_IDLE:
      if (digitalRead(BUTTON_PIN) == LOW) {
        buttonState = BUTTON_DOWN;
      }
      break;
    case BUTTON_DOWN:
      buttonDownMillis = currentMillis;
      buttonState = BUTTON_DEBOUNCE1;
      break;
    case BUTTON_DEBOUNCE1:
      if (currentMillis - buttonDownMillis >= buttonDebounceInterval) buttonState = BUTTON_WAIT;
      break;
    case BUTTON_WAIT:
      if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonState = BUTTON_UP;
      } else if (currentMillis - longPressLength >= buttonDownMillis) {
        buttonLongPress();
        buttonState = BUTTON_IGNOREDOWN;
      }
      break;
    case BUTTON_IGNOREDOWN:
      if (digitalRead(BUTTON_PIN) == HIGH) buttonState = BUTTON_DEBOUNCE2;
      break;
    case BUTTON_UP:
      buttonUpMillis = currentMillis;
      buttonPressLength = buttonUpMillis - buttonDownMillis;
      buttonState = BUTTON_DEBOUNCE2;
      if (buttonPressLength < longPressLength) {
        buttonShortPress();
      } else {
        buttonLongPress();
      }
      break;
    case BUTTON_DEBOUNCE2:
      if (currentMillis - buttonUpMillis >= buttonDebounceInterval) buttonState = BUTTON_IDLE;
      break;
  }
}

void buttonShortPress() {
  switch(myState) {
    case SETTING_1:
      setLedModes(ON, OFF, BLINK);
      myState = SETTING_2;
    break;
    case SETTING_2:
      setLedModes(ON, OFF, OFF);
      myState = SETTING_3;
    break;
    case SETTING_3:
      setLedModes(OFF, OFF, OFF);
//      myState = PERMANENT_SILENCE;
    break;
    default:
    break;
  }
}

void buttonLongPress() {
  switch(myState) {
    case SETTING_1:
      setLedModes(ON, FASTBLINK, OFF);
//      myState = LEARN_BASENOISE;
    break;
    case SETTING_2:
      setLedModes(ON, OFF, ONCE);
//      myState = TIMER_SHORT;
    break;
    case SETTING_3:
      setLedModes(ON, OFF, OFF);
      noCodeYetReceived = true;
      myState = LEARN_IR;
    break;
    default:
    break;
  }
}
void updateLeds() {
  for (byte thisLed = 0; thisLed < ledPinCount; thisLed++) {
    switch(ledMode[thisLed]) {
      case OFF:
        digitalWrite(ledPins[thisLed], LOW); 
      break;
      case ON:
        digitalWrite(ledPins[thisLed], HIGH); 
      break;
      case FASTBLINK:
        if (currentMillis - fastblinkPrevMillis[thisLed] >= ledFastBlinkInterval) {
          digitalWrite(ledPins[thisLed], ledState[thisLed] = !ledState[thisLed]); 
          fastblinkPrevMillis[thisLed] = currentMillis;
        }
      break;
      case BLINK:
        if (currentMillis - blinkPrevMillis[thisLed] >= ledSlowBlinkInterval) {
          digitalWrite(ledPins[thisLed], ledState[thisLed] = !ledState[thisLed]); 
          blinkPrevMillis[thisLed] = currentMillis;
        }
      break;
      case ONCE:
        ledBurstPatternCell = (currentMillis / 50 % 20);
        if (ledBurstPatternCell != prevLedBurstPatternCell[thisLed]) {
          prevLedBurstPatternCell[thisLed] = ledBurstPatternCell;
          switch (ledBurstPatternCell) {
            case 0: digitalWrite(ledPins[thisLed], HIGH); break;
            default: digitalWrite(ledPins[thisLed], LOW); break;
          }
        }
      break;
      case TWICE:
        ledBurstPatternCell = (currentMillis / 50 % 20);
        if (ledBurstPatternCell != prevLedBurstPatternCell[thisLed]) {
          prevLedBurstPatternCell[thisLed] = ledBurstPatternCell;
          switch (ledBurstPatternCell) {
            case 0: case 4: digitalWrite(ledPins[thisLed], HIGH); break;
            default: digitalWrite(ledPins[thisLed], LOW); break;
          }
        }
      break;
      case THRICE:
        ledBurstPatternCell = (currentMillis / 50 % 20);
        if (ledBurstPatternCell != prevLedBurstPatternCell[thisLed]) {
          prevLedBurstPatternCell[thisLed] = ledBurstPatternCell;
          switch (ledBurstPatternCell) {
            case 0: case 4: case 8: digitalWrite(ledPins[thisLed], HIGH); break;
            default: digitalWrite(ledPins[thisLed], LOW); break;
          }
        }
      break;
    }
  }
}

void setLedModes(byte newSettingsLedMode, byte newVolDownLedMode, byte newVolUpLedMode) {
  ledMode[0] = newSettingsLedMode;      
  ledMode[1] = newVolDownLedMode;
  ledMode[2] = newVolUpLedMode;
}
