/*  Allows a rooDial to set Volume of an Amp or DAC by sending IR signals.
    Expects pulses on Pin 17/27 of the Raspi running rooExtend, requiring rooxtend 2.3.x or later.

    Todo:
    - increase the IR power by using 2 or 3 IR diodes in series. One diode requires 1.1 to 1.5 volt so we can supply 3 @ 5V, 10-50 Ohm (test)
    - Test if we can live with 3.3V pulses (or need to go down to 2 LEDs. Or need a Transistor or Schmitt Trigger
    - Convert to a table based FSM (à la "Implementierung Einer Finite State Machine V1.1.pdf")

    Notes:
    - The default software generated PWM has problems on AVR running with 8 MHz. The PWM frequency is around 30 instead of 38 kHz and RC6 is not reliable.
      You can switch to timer PWM generation by #define SEND_PWM_BY_TIMER

    Links:
    https://github.com/Arduino-IRremote/Arduino-IRremote
    https://arduino-irremote.github.io/Arduino-IRremote/group__Decoder.html#ga6168e3ad4e47c657c9f3de0e5d7590b3
    https://cdn.sparkfun.com/assets/c/6/2/2/1/ProMini8MHzv2.pdf
    https://gammon.com.au/interrupts
    https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
    https://www.mikrocontroller.net/articles/Statemachine
    http://stefanfrings.de/multithreading_arduino/index.html

    FSM as UML:
    https://lucid.app/publicSegments/view/7cfc8020-8e97-4eba-ac17-6506d2f960f2/image.png

 ************************************************************************************
   MIT License

   Copyright (c) 2021 Friedemann Wachsmuth

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is furnished
   to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
   CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
   OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 ************************************************************************************
*/
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"

#define IR_RECEIVE_PIN      7 //  Overwriting IR Pins to free up 2/3 for Interrupts
#define IR_SEND_PIN         8

#include <IRremote.h>
#include <EEPROM.h>

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

// Button States. This is (was) for software debounce. An RC pair does the job much easier
// Short presses go from state 10 to 15
// Long presses go from 10 to 13 and then eventually to 16 (until letting go, which leds to 15 + 10)
#define BUTTON_IDLE        10
#define BUTTON_DOWN        11
#define BUTTON_DEBOUNCE1   12
#define BUTTON_WAIT        13
#define BUTTON_UP          14
#define BUTTON_DEBOUNCE2   15
#define BUTTON_IGNOREDOWN  16

// Settings Modes
// Intended to step sequentially through various settings. This leaves the normal FSM.
#define SETTING_1          21
#define SETTING_2          22
#define SETTING_3          23
#define SETTINGS_EXIT      24

// States (Siluino)
#define LEARN_BASENOISE    30
#define LEARN_IR           31
#define TIMER_SHORT        32
#define TIMER_MID          33
#define TIMER_LONG         34

// States (rooDialino)
#define RELAY_SIGNAL_ON       50
#define RELAY_SIGNAL_OFF      51
#define LEARN_IR_RELAY_TOGGLE 52
#define LEARN_IR_VOL_UP       53
#define LEARN_IR_VOL_DOWN     54

// Array indices for our IR code structs
#define IR_RELAY_TOGGLE 0
#define IR_VOL_UP       1
#define IR_VOL_DOWN     2
#define NUMBER_OF_CODES_STORED  3 // array size

volatile int volSteps;  // keeps track of how many pulses came in from the rooDial

// ******* IR things ****************************

// Storage for recorded IR code
struct storedIRDataStruct {
  IRData receivedIRData;
  // extensions for sendRaw
  uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
  uint8_t rawCodeLength; // The length of the code
};
struct storedIRDataStruct IRCodeLearned[NUMBER_OF_CODES_STORED];

// TODO: Do I need these?
void storeIRCode(IRData *aIRReceivedData);
void sendIRCode(storedIRDataStruct *aIRDataToSend);

// TOTO: Delete these
uint16_t sAddress;
uint8_t sCommand;
uint8_t sRepeats;

// ******* LED things ****************************

const byte ledPins[] = { 10, 11, 12 };       // an array of pin numbers too which LEDs are attached
const byte ledPinCount = 3;
byte ledMode[] = { ON, OFF, OFF };
unsigned long fastblinkPrevMillis[] = { 0, 0, 0, 0 };   // will store last time LED was updated
unsigned long blinkPrevMillis[] = { 0, 0, 0, 0 };       // will store last time LED was updated
unsigned long currentMillis = 0;
int ledState[] = { LOW, LOW, LOW, LOW };                // ledState Array used to easily set the LED

const unsigned int ledSlowBlinkInterval = 200;
const unsigned int ledFastBlinkInterval = 80;

byte ledBurstPatternCell = 0;
byte prevLedBurstPatternCell[] = { 0, 0, 0, 0 };


// ******* Button things ****************************

// Button & Debounce Timing
const unsigned long longPressLength = 3000;
const unsigned int buttonDebounceInterval = 50;
// Button Timers
unsigned long previousButtonMillis = 0;
unsigned long buttonDownMillis = 0;
unsigned long buttonUpMillis = 0;
int buttonPressLength;

boolean noCodeYetReceived = true;
unsigned long lastIRreceivedMillis = millis();

// current states
// byte settingsButtonState = 0;
byte myState = RELAY_SIGNAL_ON;
byte prevState;
byte buttonState = BUTTON_IDLE;
byte prevButtonState;
byte learnState;


// ISRs
void volDownISR() {
  volSteps--;
}
void volUpISR() {
  volSteps++;
}

void setup() {
  //  buttonState = BUTTON_IDLE;

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
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver, enable feedback LED

  Serial.print(F("Ready to send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

#if defined(USE_SOFT_SEND_PWM) && !defined(ESP32) // for esp32 we use PWM generation by hw_timer_t for each pin
  /*
     Print internal signal generation info
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

  if (readLearnedIRCodesFromEEPROM() == false) {
    transitionTo_LEARN_IR_RELAY_TOGGLE(); // Seems like the EEPROM is empty. Lets learn some codes.
  }

}


void loop() {
  // Debug Code below
  if (myState != prevState) {
    Serial.print("Mode: ");
    Serial.println(myState);
    prevState = myState;
  }

  currentMillis = millis(); // needed for async (non-blocking) blinking amd button debouncing
  updateLeds();
  checkButton();            // This debounces and calls buttonLongPress() and buttonShortPress(). The latter dispatch from state to state.

  switch (myState) {        // check if relaying was turned on or off via IR
    case RELAY_SIGNAL_ON:   // but only check when not in a LEARN mode.
      if (checkIRToggle())
        transitionTo_RELAY_SIGNAL_OFF();
    case RELAY_SIGNAL_OFF:
      if (checkIRToggle())
        transitionTo_RELAY_SIGNAL_ON();
      break;
    default:
      break;
  }

  switch (myState) {
    case RELAY_SIGNAL_ON:
      // TODO: turn off recever in these ifs... IrReceiver.stop();
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
      // TODO: turn rrecever back on after these ifs... IrReceiver.resume();
      break;
    case RELAY_SIGNAL_OFF:
      // flash the LEDs to show we are seing pulses (aka rooDial is in reach but relaying is off)
      // Might combine both cases since we want that same visual feedback while relaying too.
      // Alternatively, let relay=off just disable the IR Pin... :)
      break;
    case LEARN_IR_RELAY_TOGGLE:
      if (learnIRCode(IR_RELAY_TOGGLE)) {
        transitionTo_LEARN_IR_VOL_UP();
      }
      break;
    case LEARN_IR_VOL_UP:
      if (learnIRCode(IR_VOL_UP)) {
        transitionTo_LEARN_IR_VOL_DOWN();
      }
      break;
    case LEARN_IR_VOL_DOWN:
      if (learnIRCode(IR_VOL_DOWN)) {
        saveLearnedIRCodesToEEPROM();
        transitionTo_RELAY_SIGNAL_ON();
      }
      break;
    default:
      break;
  }

}

// ****************************************************************************************************************

void transitionTo_RELAY_SIGNAL_ON() {
  // read codes from EEEPROM
  // Enable IR LED
  setLedModes(ON, OFF, OFF);
  myState = RELAY_SIGNAL_ON;
}

void transitionTo_RELAY_SIGNAL_OFF() {
  // Disable IR LED
  setLedModes(OFF, OFF, OFF);
  myState = RELAY_SIGNAL_OFF;
}

void transitionTo_LEARN_IR_RELAY_TOGGLE() {
  setLedModes(FASTBLINK, OFF, OFF);
  myState = LEARN_IR_RELAY_TOGGLE;
}

void transitionTo_LEARN_IR_VOL_UP() {
  setLedModes(OFF, FASTBLINK, OFF);
  myState = LEARN_IR_VOL_UP;
}

void transitionTo_LEARN_IR_VOL_DOWN() {
  setLedModes(OFF, OFF, FASTBLINK);
  myState = LEARN_IR_VOL_DOWN;
}

void checkButton() { // call in loop(). This calls buttonLongPress() and buttonShortPress().

  // Debug Output below. Comment out if not debugging
//    if (buttonState != prevButtonState) {
//      Serial.print("Button: ");
//      Serial.println(buttonState);
//      prevButtonState = buttonState;
//    }

  switch (buttonState) {
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
      } else if (currentMillis >= longPressLength + buttonDownMillis) { // This will not work for 3 seconds every 50 days and 70 minutes, but who cares
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

bool checkIRToggle() {
  // see if a relay state toggle was requested
  bool received = false;

  if (false /* IrReceiver.decode() == 0xAFFE */) { // should be the correct value, not any value!
    //if (IrReceiver.available()) {
    if (millis() - lastIRreceivedMillis > 250) {  // If it's been at least 1/4 second since the last IR received, toggle the relay state
      received = true;
    }
    lastIRreceivedMillis = millis();
    IrReceiver.resume();
  }
  return received;
}

bool saveLearnedIRCodesToEEPROM() {
  // save just learned codes to EEPROM
  int eeAddress = 0;
  const unsigned long eeMagic = 0xb007ab1e; // leave a magic mark that we have been here here before.
  EEPROM.put(eeAddress, eeMagic);
  eeAddress = eeAddress + sizeof(eeMagic);
  for (byte thisCode = 0; thisCode < NUMBER_OF_CODES_STORED; thisCode++) {
    EEPROM.put(eeAddress, IRCodeLearned[thisCode]);
    eeAddress = eeAddress + sizeof(IRCodeLearned[thisCode]);
  }
}

bool readLearnedIRCodesFromEEPROM() {
 int eeAddress = 0;
  unsigned long eeMagic; 
  EEPROM.get(eeAddress, eeMagic); // attempt to fetch the magic indicating that we have been here before
  if (eeMagic == 0xb007ab1e) {
    eeAddress = eeAddress + sizeof(eeMagic);
    for (byte thisCode = 0; thisCode < NUMBER_OF_CODES_STORED; thisCode++) {
      EEPROM.get(eeAddress, IRCodeLearned[thisCode]);
//      storedIRDataStruct thisStruct;
//      EEPROM.get(eeAddress, thisStruct);
//      // struct storedIRDataStruct IRCodeLearned[NUMBER_OF_CODES_STORED];
//      Serial.println(thisStruct.receivedIRData.protocol);
//      Serial.println(thisStruct.receivedIRData.address);
//      Serial.println(thisStruct.receivedIRData.command);
//      Serial.println();
      eeAddress = eeAddress + sizeof(IRCodeLearned[thisCode]);
    }
    return true;
  } else {
    Serial.println("Virgin EEPROM!");
    return false;
  }
}

bool learnIRCode(byte IRStructArrayIndex) {
  bool received = false;
    if (IrReceiver.decode()) { 
      if (IrReceiver.decodedIRData.protocol != UNKNOWN) { // Too much Noise from e.g. LED bulbs around to allow Raw signal recording
        if (millis() - lastIRreceivedMillis > 250) {  // If it's been at least 1/4 second since the last IR received
        received = true;
  
        Serial.print("Storing code at index ");
        Serial.println(IRStructArrayIndex);
        IRCodeLearned[IRStructArrayIndex].receivedIRData = *IrReceiver.read();
        IRCodeLearned[IRStructArrayIndex].receivedIRData.flags = 0; // clear any flags -esp. repeat- for later sending
      
        IrReceiver.printIRResultShort(&Serial);
  
        Serial.println();
      }
    }
    lastIRreceivedMillis = millis();
    IrReceiver.resume(); /* 
      This still queues signal appearing in subsequent calls. Try approach from Siluino:L474.
      Or, like in the ReceiveAndSend Example, check IrReceiver.available() and then do a IrReceiver.read() instead of the .decode.
    */                         
  }
  return received;
}


void sendIRCode(storedIRDataStruct *aIRDataToSend) {
  if (aIRDataToSend->receivedIRData.protocol == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    IrSender.sendRaw(aIRDataToSend->rawCode, aIRDataToSend->rawCodeLength, 38);

    Serial.print(F("Sent raw "));
    Serial.print(aIRDataToSend->rawCodeLength);
    Serial.println(F(" marks or spaces"));
  } else {

    // Use the write function, which does the switch for different protocols. It's missing in the docs though. :/
    IrSender.write(&aIRDataToSend->receivedIRData, NO_REPEATS);

    Serial.print(F("Sent: "));
    printIRResultShort(&Serial, &aIRDataToSend->receivedIRData);
  }
}


void buttonShortPress() {
  switch (myState) {
    case RELAY_SIGNAL_ON:
      transitionTo_RELAY_SIGNAL_OFF();
      break;
    case RELAY_SIGNAL_OFF:
      transitionTo_RELAY_SIGNAL_ON();
      break;

    // a short button pres skips the expected config step (to keep the prev. stored setting)
    case LEARN_IR_RELAY_TOGGLE:
      transitionTo_LEARN_IR_VOL_UP();
      break;
    case LEARN_IR_VOL_UP:
      transitionTo_LEARN_IR_VOL_DOWN();
      break;
    case LEARN_IR_VOL_DOWN:
      transitionTo_RELAY_SIGNAL_ON();
      break;

    default:
      break;
  }
}

void buttonLongPress() {
  switch (myState) {
    case RELAY_SIGNAL_ON:
    case RELAY_SIGNAL_OFF:
      transitionTo_LEARN_IR_RELAY_TOGGLE();
      break;

    case LEARN_IR_RELAY_TOGGLE:
    case LEARN_IR_VOL_UP:
    case LEARN_IR_VOL_DOWN:
      // a long button press in Settings mode should just do nuthin (so far)
      break;

    default:
      break;
  }
}

void updateLeds() { // call in loop() to update the connected LEDs as set in ledMode[]
  for (byte thisLed = 0; thisLed < ledPinCount; thisLed++) {
    switch (ledMode[thisLed]) {
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
      //
      // The following three burst modes might become useful if we'll need to allow setting a multiplier. Otherwise
      // they are Siluino legacy.
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
      default:
        break;
    }
  }
}

void setLedModes(byte newSettingsLedMode, byte newVolDownLedMode, byte newVolUpLedMode) { // writes individual set LED modes to a the LED mode array
  ledMode[0] = newSettingsLedMode;
  ledMode[1] = newVolDownLedMode;
  ledMode[2] = newVolUpLedMode;
}
