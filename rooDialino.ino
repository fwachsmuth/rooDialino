/*  Allows a rooDial to set Volume of an Amp or DAC by sending IR signals.
    Expects pulses on Pin 17/27 of the Raspi running rooExtend, requiring rooxtend 2.3.x or later.

    Todo:

    Todo (PCB):
       - Wire up I2C (Pullups, TPs) for debugging w/o SPI

    Todo (Code):
      - Use Github issues instead! https://github.com/fwachsmuth/rooDialino/issues  
    
    Useful Links:
    https://github.com/Arduino-IRremote/Arduino-IRremote
    https://arduino-irremote.github.io/Arduino-IRremote/group__Decoder.html#ga6168e3ad4e47c657c9f3de0e5d7590b3
    https://cdn.sparkfun.com/assets/c/6/2/2/1/ProMini8MHzv2.pdf
    https://gammon.com.au/interrupts
    https://www.mikrocontroller.net/articles/Statemachine
    http://stefanfrings.de/multithreading_arduino/index.html
    https://support.jlcpcb.com/article/84-how-to-generate-the-bom-and-centroid-file-from-kicad
    https://yaqwsx.github.io/jlcparts/#/
    https://ravikiranb.com/projects/kicad-rpiz-uhat-template/
    https://pcbchecklist.com/
    http://nicecircuits.com/dual-port-serial-terminal/
    https://github.com/daniel5151/surface-dial-linux
    https://www.reddit.com/r/SurfaceLinux/comments/eqk22k/surface_dial_on_linux/
    https://arduino.stackexchange.com/a/9858

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
#include <IRremote.h> // Using library version 3.3.0
#include <EEPROM.h>

#define DEBUG true          // Set to false to avoid Serial.printlns
#define DELAY_AFTER_SEND  5  // shorter than 5 ms might make dirty signal

// Pin Naming

#define IR_RECEIVE_PIN    7 //  Overwriting IR Pins to free up 2/3 for Interrupts
#define IR_SEND_PIN       8

#define VOL_DOWN_PIN      3
#define VOL_UP_PIN        2
#define BUTTON_PIN        4

#define LED_RSTATE       10  // Reflects state of the IR Relay. Comment this out if SPI Debugging is enabled
#define LED_VOL_DOWN      6  
#define LED_VOL_UP        5
#define LED_ROFF         15  // aka A1. Reflects learning explicit OFF codes
#define LED_RON          16  // aka A2. Reflects learning explicit ON codes
#define LED_NONE         A3 
/* There is actually no LED connected to A3, in fact, A3 is NC. We just need to use a Pin !=0 to disable
the IR Feedback LED during programming (interfers with blinking) and a Pin !=[10|11|13] during Debugging 
via SPI. */


// State Enums
enum LedMode  /* for all the states an LED can have */
{
  off = 0,  /* force start at 0 to allow mapping debug strings in a shadow array */
  on,
  fastBlink,
  blink,
  once,
  twice,
  thrice,
  quadruple, /* Preapring for more blink states */
  quintuple, /* Preapring for more blink states */
};

// Button States. This is for software debounce. 
enum ButtonState {
  idle = 0,
  down,
  debounceDown,
  held,
  up,
  debounceUp,
  longPress,
};
const char* buttonStateStr[] = {"Idle", "Down", "Debounce Down", "Held", "Up", "Debounce Up", "Longpress"};


// States (rooDialino). Todo: Convert to enum.
#define RELAY_SIGNAL_ON       50
#define RELAY_SIGNAL_OFF      51
#define LEARN_IR_RELAY_TOGGLE 52
#define LEARN_IR_VOL_UP       53
#define LEARN_IR_VOL_DOWN     54
#define LEARN_IR_RELAY_ON     55
#define LEARN_IR_RELAY_OF     56

// Array indices for our IR code structs. Todo: Convert to enum.
#define IR_RELAY_TOGGLE 0
#define IR_VOL_UP       1
#define IR_VOL_DOWN     2
#define NUMBER_OF_CODES_STORED  3 // array size

volatile int16_t volSteps;  // keeps track of how many pulses came in from the rooDial

// ******* IR things ****************************

struct IRData IRCodeLearned[NUMBER_OF_CODES_STORED];

// TODO: Do I need these?
void storeIRCode(IRData *aIRReceivedData);
void sendIRCode(IRData *aIRDataToSend);

// TODO: Delete these
uint16_t sAddress;
uint8_t sCommand;
uint8_t sRepeats;

// ******* LED things ****************************

const byte ledPins[] = { LED_RSTATE, LED_VOL_DOWN, LED_VOL_UP, LED_RON, LED_ROFF, LED_NONE} ;   // an array of pin numbers to which LEDs are attached
const byte ledPinCount = 5;   // LED_NONE is not connected
LedMode ledMode[] = { on, off, off, off, off };           // array of enum'd LED states. Turn them off.

unsigned long fastblinkPrevMillis[] = { 0, 0, 0, 0, 0 } ; // will store last time LED was updated
unsigned long blinkPrevMillis[] = { 0, 0, 0, 0, 0 };      // will store last time LED was updated
unsigned long currentMillis = 0;
bool ledBlinkState[] = { LOW, LOW, LOW, LOW, LOW };             // ledState Array used to toggle them for blinking

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
ButtonState buttonState = idle;
byte prevState;
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

  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int thisLed = 0; thisLed < ledPinCount; thisLed++) pinMode(ledPins[thisLed], OUTPUT);

  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), volDownISR, RISING);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), volUpISR, RISING);

  Serial.begin(115200);
  // Just to know which program is running on my Arduino
#ifdef DEBUG
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
#endif
  IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK); // Specify send pin and enable feedback LED at default feedback LED pin
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, LED_VOL_UP); // Start the receiver, enable custom feedback LED
  FeedbackLEDControl.FeedbackLEDPin = LED_NONE;  // overwrite feedback LED as needed

#ifdef DEBUG
      Serial.print(F("Ready to send IR signals at pin "));
  Serial.println(IR_SEND_PIN);
#endif

#if defined(USE_SOFT_SEND_PWM) && !defined(ESP32) // for esp32 we use PWM generation by hw_timer_t for each pin
  /*
     Print internal signal generation info
  */
  IrSender.enableIROut(38);

#ifdef DEBUG
  Serial.print(F("Send signal mark duration is "));
  Serial.print(IrSender.periodOnTimeMicros);
  Serial.print(F(" us, pulse correction is "));
  Serial.print((uint16_t) PULSE_CORRECTION_NANOS);
  Serial.print(F(" ns, total period is "));
  Serial.print(IrSender.periodTimeMicros);
  Serial.println(F(" us"));
#endif
#endif

  if (!readLearnedIRCodesFromEEPROM()) {
    transitionTo_LEARN_IR_RELAY_TOGGLE(); // Seems like the EEPROM is empty. Lets learn some codes.
  }

}


void loop() {
  // Debug Code below
  if (myState != prevState) {
#ifdef DEBUG
    Serial.print("Mode: ");
    Serial.println(myState);
#endif
    prevState = myState;
  }

  currentMillis = millis(); // needed for async (non-blocking) blinking amd button debouncing
  updateLeds();
  checkButton();            // This debounces and calls buttonLongPress() and buttonShortPress(). The latter dispatch from state to state.
  checkForSerialCommand();

  switch (myState) {        // check if relaying was turned on or off via IR
    case RELAY_SIGNAL_ON:   // but only check when not in a LEARN mode.
      if (checkIRToggle())
        transitionTo_RELAY_SIGNAL_OFF();
      break;
    case RELAY_SIGNAL_OFF:
      if (checkIRToggle())
        transitionTo_RELAY_SIGNAL_ON();
      break;
    default:
      break;
  }

  switch (myState) {
    case RELAY_SIGNAL_ON:
      if (volSteps > 0) {   
        FeedbackLEDControl.FeedbackLEDPin = LED_VOL_UP;
        sendIRCode(IR_VOL_UP);
        volSteps--;
        if (volSteps == 0) // Switch back to default Feedback LED
        {
          FeedbackLEDControl.FeedbackLEDPin = LED_RON;
        }
#ifdef DEBUG
        Serial.print(volSteps);
        Serial.println(" Up");
#endif
        delay(DELAY_AFTER_SEND);
      }
      if (volSteps < 0) {
        FeedbackLEDControl.FeedbackLEDPin = LED_VOL_DOWN;
        sendIRCode(IR_VOL_DOWN);
        volSteps++;
        if (volSteps == 0) // Switch back to default Feedback LED
        {
          FeedbackLEDControl.FeedbackLEDPin = LED_RON;
        }
#ifdef DEBUG
        Serial.print(volSteps);
        Serial.println(" Down");
#endif
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
  // Todo: Read codes from EEEPROM
  // enable ISRs
  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), volDownISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), volUpISR, CHANGE);
  setLedModes(on, off, off);
  myState = RELAY_SIGNAL_ON;
}

void transitionTo_RELAY_SIGNAL_OFF() {
  // disable ISRs
  detachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN));
  detachInterrupt(digitalPinToInterrupt(VOL_UP_PIN));
  setLedModes(off, off, off);
  myState = RELAY_SIGNAL_OFF;
}

void transitionTo_LEARN_IR_RELAY_TOGGLE() {
  setLedModes(fastBlink, off, off);
  myState = LEARN_IR_RELAY_TOGGLE;
}

void transitionTo_LEARN_IR_VOL_UP() {
  setLedModes(off, fastBlink, off);
  myState = LEARN_IR_VOL_UP;
}

void transitionTo_LEARN_IR_VOL_DOWN() {
  setLedModes(off, off, fastBlink);
  myState = LEARN_IR_VOL_DOWN;
}

bool checkForSerialCommand() {
  if (Serial.available()) {
    char command = Serial.read();
    switch(command) {
      case 'n':
#ifdef DEBUG
        Serial.println(F("On!"));
#endif
      break;
      case 'f':
#ifdef DEBUG
        Serial.println(F("Off!"));
#endif
      break;
      case 's':
#ifdef DEBUG
        Serial.println(F("Status"));
#endif
      break;
      case 'v':
#ifdef DEBUG
        Serial.println(F("Version!"));
#endif
      break;
      default:
#ifdef DEBUG
        Serial.println(F("Kenn ich nicht."));
#endif
      break;
    }
    return true;
  }
  return false;
}


void checkButton() { // call in loop(). This calls buttonLongPress() and buttonShortPress().

#ifdef DEBUG
// Debug Output below. Comment out if not debugging
    if (buttonState != prevButtonState) {
      Serial.print("Button: ");
      Serial.println(buttonStateStr[buttonState]);
      prevButtonState = buttonState;
    }
#endif

  switch (buttonState) {
    case idle:
      if (digitalRead(BUTTON_PIN) == LOW) {
        buttonState = down;
      }
      break;
    case down:
      buttonDownMillis = currentMillis;
      buttonState = debounceDown;
      break;
    case debounceDown:
      if (currentMillis - buttonDownMillis >= buttonDebounceInterval) buttonState = held;
      break;
    case held:
      if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonState = up;
      } else if (currentMillis >= longPressLength + buttonDownMillis) { // This will not work for 3 seconds every 50 days and 70 minutes, but who cares
        buttonLongPress();
        buttonState = longPress;
      }
      break;
    case up:
      buttonUpMillis = currentMillis;
      buttonPressLength = buttonUpMillis - buttonDownMillis;
      buttonState = debounceUp;
      if (buttonPressLength < longPressLength) {
        buttonShortPress();
      } else {
        buttonLongPress();
      }
      break;
    case debounceUp:
      if (currentMillis - buttonUpMillis >= buttonDebounceInterval) buttonState = idle;
      break;
    case longPress:
      if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonUpMillis = currentMillis;
        buttonState = debounceUp;
      }
      break;
  }
}

bool checkIRToggle() {
  // see if a relay state toggle was requested
  bool received = false;
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == IRCodeLearned[0].protocol &&
        IrReceiver.decodedIRData.address  == IRCodeLearned[0].address  &&
        IrReceiver.decodedIRData.command  == IRCodeLearned[0].command) {
    
      if (millis() - lastIRreceivedMillis > 250) {  // If it's been at least 1/4 second since the last IR received, toggle the relay state
        received = true;
      }
      lastIRreceivedMillis = millis();
    
    }
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
#ifdef DEBUG
    Serial.println(eeAddress);
#endif
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
      IRData thisStruct;
      EEPROM.get(eeAddress, thisStruct);
#ifdef DEBUG
      Serial.println(thisStruct.protocol);
      Serial.println(thisStruct.address);
      Serial.println(thisStruct.command);
      Serial.println();
#endif
      eeAddress = eeAddress + sizeof(IRCodeLearned[thisCode]);
    }
    return true;
  } else {
#ifdef DEBUG
    Serial.println("Virgin EEPROM!");
#endif
    return false;
  }
}

bool learnIRCode(byte IRStructArrayIndex) {
  bool received = false;
    if (IrReceiver.decode()) { 
      if (IrReceiver.decodedIRData.protocol != UNKNOWN) { // Too much Noise from e.g. LED bulbs around to allow Raw signal recording
        if (millis() - lastIRreceivedMillis > 250) {  // If it's been at least 1/4 second since the last IR received
        received = true;
  
#ifdef DEBUG
        Serial.print("Storing code at index ");
        Serial.println(IRStructArrayIndex);
#endif
        IRCodeLearned[IRStructArrayIndex] = *IrReceiver.read();
        IRCodeLearned[IRStructArrayIndex].flags = 0; // clear any flags -esp. repeat- for later sending
      
#ifdef DEBUG
        IrReceiver.printIRResultShort(&Serial);
        Serial.println();
#endif
      }
    }
    lastIRreceivedMillis = millis();
    IrReceiver.resume();       
  }
  return received;
}

void sendIRCode(byte IRcommand) {
  /*
   *  Check SendDemo for more verbose documentation on the write function
   */
  // Use the write function, which does the switch for different protocols. It's missing in the docs though. :/

  IRData IRSendData;
  // prepare data
  IRSendData.protocol = IRCodeLearned[IRcommand].protocol;
  IRSendData.address = IRCodeLearned[IRcommand].address;
  IRSendData.command = IRCodeLearned[IRcommand].command;
  IRSendData.flags = IRDATA_FLAGS_EMPTY;

  IrSender.write(&IRSendData, NO_REPEATS);
#ifdef DEBUG
  Serial.print(F("Sent: "));
  printIRResultShort(&Serial, &IRSendData);
#endif

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
      case off:
        digitalWrite(ledPins[thisLed], LOW);
        break;
      case on:
        digitalWrite(ledPins[thisLed], HIGH);
        break;
      case fastBlink:
        if (currentMillis - fastblinkPrevMillis[thisLed] >= ledFastBlinkInterval) {
          digitalWrite(ledPins[thisLed], ledBlinkState[thisLed] = !ledBlinkState[thisLed]);
          fastblinkPrevMillis[thisLed] = currentMillis;
        }
        break;
      case blink:
        if (currentMillis - blinkPrevMillis[thisLed] >= ledSlowBlinkInterval) {
          digitalWrite(ledPins[thisLed], ledBlinkState[thisLed] = !ledBlinkState[thisLed]);
          blinkPrevMillis[thisLed] = currentMillis;
        }
        break;
      //
      // The following three burst modes might become useful if we'll need to allow setting a multiplier. Otherwise
      // they are Siluino legacy.
      case once:
        ledBurstPatternCell = (currentMillis / 50 % 20);
        if (ledBurstPatternCell != prevLedBurstPatternCell[thisLed]) {
          prevLedBurstPatternCell[thisLed] = ledBurstPatternCell;
          switch (ledBurstPatternCell) {
            case 0: digitalWrite(ledPins[thisLed], HIGH); break;
            default: digitalWrite(ledPins[thisLed], LOW); break;
          }
        }
        break;
      case twice:
        ledBurstPatternCell = (currentMillis / 50 % 20);
        if (ledBurstPatternCell != prevLedBurstPatternCell[thisLed]) {
          prevLedBurstPatternCell[thisLed] = ledBurstPatternCell;
          switch (ledBurstPatternCell) {
            case 0: case 4: digitalWrite(ledPins[thisLed], HIGH); break;
            default: digitalWrite(ledPins[thisLed], LOW); break;
          }
        }
        break;
      case thrice:
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

// writes individual set LED modes to a the LED mode array
void setLedModes(LedMode newSettingsLedMode, LedMode newVolDownLedMode, LedMode newVolUpLedMode) {
  ledMode[0] = newSettingsLedMode;
  ledMode[1] = newVolDownLedMode;
  ledMode[2] = newVolUpLedMode;
}
