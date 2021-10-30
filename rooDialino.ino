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

/* ---------------- Macros and Constants --------------------------------------------------------------- */

#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.h> // Using library version 3.3.0
#include <EEPROM.h>   // to store learned codes

#define Debugln(a) (Serial.println(a))
#define Debug(a) (Serial.print(a))
// #define Debugln(a)   // Declare the above Macros empty to turn off serial debug output
// #define Debug(a)

const char versionNo[] = "1.0.0";

#define DELAY_AFTER_SEND 5 // shorter than 5 ms might make dirty signal

/* ---------------- Pin Naming ------------------------------------------------------------------------- */

#define IR_RECEIVE_PIN 7 //  Overwriting IR Pins to free up 2/3 for Interrupts
#define IR_SEND_PIN 8

#define VOL_DOWN_PIN 3
#define VOL_UP_PIN 2
#define BUTTON_PIN 4

#define LED_RSTATE 10 // Reflects state of the IR Relay. Comment this out if SPI Debugging is enabled
#define LED_VOL_DOWN 6
#define LED_VOL_UP 5
#define LED_ROFF 15 // aka A1. Reflects learning explicit OFF codes
#define LED_RON 16  // aka A2. Reflects learning explicit ON codes
#define LED_NONE A3
/* There is actually no LED connected to A3, in fact, A3 is NC. We just need to use a Pin !=0 to disable
the IR Feedback LED during programming (interfers with blinking) and a Pin !=[10|11|13] during Debugging 
via SPI. */

/* ---------------- Many many States ------------------------------------------------------------------- */

/// all the states an LED can have
enum LedMode : byte
{
  off = 0, // force start at 0 to allow mapping debug strings in a shadow array
  on,
  fastBlink,
  slowBlink,
  once = 0x11, // this offset allows chosing subsequent blink modes via incrementing an index
  twice,
  thrice,
  quadruple,
  quintuple,
};
const char *ledModeStr[] = {"Off", "On", "Fast Blink", "Slow Blink"};

/// Button States. This is for software debounce.
enum ButtonState
{
  idle = 0,
  down,
  debounceDown,
  held,
  up,
  debounceUp,
  longPress,
};
const char *buttonStateStr[] = {"Idle", "Down", "Debounce Down", "Held", "Up", "Debounce Up", "Longpress"};

/// States (rooDialino)
enum State
{
  relaySignalOn = 0,
  relaySignalOff,
  learnIRRelayToggle,
  learnIRVolUp,
  learnIRVolDown,
  learnIRRelayOn,
  learnIRRelayOff,
};
const char *stateStr[] = {
    "Relay IR-Signal: On", "Relay IR-Signal: Off", "Learn-IR Code: Toggle Relay State", "Learn IR-Code: Vol Up",
    "Learn IR-Code: Vol Down", "Learn IR-Code: Relay expl. On", "Learn IR-Code: Relay expl. Off"};

// Array indices for our IR code structs. Todo: Convert to enum.
#define IR_RELAY_TOGGLE 0
#define IR_VOL_UP 1
#define IR_VOL_DOWN 2
#define NUMBER_OF_CODES_STORED 3 // array size

volatile int16_t volSteps; // keeps track of how many pulses came in from the rooDial

/* ---------------- IR Things ------------------------------------------------------------------------- */

struct IRData IRCodeLearned[NUMBER_OF_CODES_STORED];

// TODO: Do I need these?
void storeIRCode(IRData *aIRReceivedData);
void sendIRCode(IRData *aIRDataToSend);

/* ---------------- LED Things ------------------------------------------------------------------------ */

const byte ledPins[] = {LED_RSTATE, LED_VOL_DOWN, LED_VOL_UP, LED_ROFF, LED_RON, LED_NONE}; // an array of pin numbers to which LEDs are attached
const byte ledPinCount = 5;                                                                 // LED_NONE is not connected
LedMode ledMode[] = {on, off, off, off, off};                                               // array of enum'd LED states
LedMode prevLedMode[] = {on, off, off, off, off};

unsigned long currentMillis = 0;

/* ---------------- Button Things --------------------------------------------------------------------- */

// Button & Debounce Timing
const unsigned long longPressLength = 1000;
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
State myState = relaySignalOn;
ButtonState buttonState = idle;
byte prevState;
byte prevButtonState;
byte learnState;

/* ---------------- ISRs ----------------------------------------------------------------------------- */

void volDownISR()
{
  volSteps--;
}
void volUpISR()
{
  volSteps++;
}

/* ---------------- Setup begins --------------------------------------------------------------------- */

void setup()
{
  pinMode(VOL_DOWN_PIN, INPUT_PULLUP);
  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int thisLed = 0; thisLed < ledPinCount; thisLed++)
    pinMode(ledPins[thisLed], OUTPUT);

  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), volDownISR, RISING);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), volUpISR, RISING);

  Serial.begin(115200);
  // Just to know which program is running on my Arduino
  Debugln(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK);                  // Specify send pin and enable feedback LED at default feedback LED pin
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, LED_VOL_UP); // Start the receiver, enable custom feedback LED
  FeedbackLEDControl.FeedbackLEDPin = LED_NONE;                      // overwrite feedback LED as needed

  Debug(F("Ready to send IR signals at pin "));
  Debugln(IR_SEND_PIN);

#if defined(USE_SOFT_SEND_PWM) && !defined(ESP32) // for esp32 we use PWM generation by hw_timer_t for each pin
  /*
     Print internal signal generation info
  */
  IrSender.enableIROut(38);

  Debug(F("Send signal mark duration is "));
  Debug(IrSender.periodOnTimeMicros);
  Debug(F(" us, pulse correction is "));
  Debug((uint16_t)PULSE_CORRECTION_NANOS);
  Debug(F(" ns, total period is "));
  Debug(IrSender.periodTimeMicros);
  Debugln(F(" us"));
#endif

  if (!readLearnedIRCodesFromEEPROM())
  {
    transitionTo_learnIRRelayToggle(); // Seems like the EEPROM is empty. Lets learn some codes.
  }
}

/* ---------------- Loop begins --------------------------------------------------------------------- */

void loop()
{
  // Debug Code below
  if (myState != prevState)
  {
    Debug("Mode: ");
    Debugln(stateStr[myState]);
    prevState = myState;
  }

  currentMillis = millis(); // needed for async (non-blocking) blinking amd button debouncing
  updateLeds();
  checkButton(); // This debounces and calls buttonLongPress() and buttonShortPress(). The latter dispatch from state to state.
  checkForSerialCommand();

  switch (myState)
  {                   // check if relaying was turned on or off via IR
  case relaySignalOn: // but only check when not in a LEARN mode.
    if (checkIRToggle())
    {
      transitionTo_relaySignalOff();
    }
    break;
  case relaySignalOff:
    if (checkIRToggle())
    {
      transitionTo_relaySignalOn();
    }
    break;
  default:
    break;
  }

  switch (myState)
  {
  case relaySignalOn:
    if (volSteps > 0)
    {
      FeedbackLEDControl.FeedbackLEDPin = LED_VOL_UP;
      sendIRCode(IR_VOL_UP);
      volSteps--;
      if (volSteps == 0) // Switch back to default Feedback LED
      {
        FeedbackLEDControl.FeedbackLEDPin = LED_RON;
      }
      Debug(volSteps);
      Debugln(" Up");
      delay(DELAY_AFTER_SEND);
    }
    if (volSteps < 0)
    {
      FeedbackLEDControl.FeedbackLEDPin = LED_VOL_DOWN;
      sendIRCode(IR_VOL_DOWN);
      volSteps++;
      if (volSteps == 0) // Switch back to default Feedback LED
      {
        FeedbackLEDControl.FeedbackLEDPin = LED_RON;
      }
      Debug(volSteps);
      Debugln(" Down");
      delay(DELAY_AFTER_SEND);
    }
    // TODO: turn rrecever back on after these ifs... IrReceiver.resume();
    break;
  case relaySignalOff:
    // flash the LEDs to show we are seing pulses (aka rooDial is in reach but relaying is off)
    // Might combine both cases since we want that same visual feedback while relaying too.
    // Alternatively, let relay=off just disable the IR Pin... :)
    break;
  case learnIRRelayToggle:
    if (learnIRCode(IR_RELAY_TOGGLE))
    {
      Serial.println(F("toggle_learned"));
      transitionTo_learnIRVolUp();
    }
    break;
  case learnIRVolUp:
    if (learnIRCode(IR_VOL_UP))
    {
      Serial.println(F("voldown_learned"));
      transitionTo_learnIRVolDown();
    }
    break;
  case learnIRVolDown:
    if (learnIRCode(IR_VOL_DOWN))
    {
      saveLearnedIRCodesToEEPROM();
      Serial.println(F("volup_learned"));
      Serial.println(F("learning_complete"));
      transitionTo_relaySignalOn();
    }
    break;
  default:
    break;
  }
}

/* ---------------- End Loop -------------------------------------------------------------------------- */

void transitionTo_relaySignalOn()
{
  // Todo: Read codes from EEEPROM
  // enable ISRs
  attachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN), volDownISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VOL_UP_PIN), volUpISR, CHANGE);
  setLedModes(on, off, off, off, off);
  myState = relaySignalOn;
}

void transitionTo_relaySignalOff()
{
  // disable ISRs
  volSteps = 0;
  detachInterrupt(digitalPinToInterrupt(VOL_DOWN_PIN));
  detachInterrupt(digitalPinToInterrupt(VOL_UP_PIN));
  setLedModes(off, off, off, off, off);
  myState = relaySignalOff;
}

void transitionTo_learnIRRelayToggle()
{
  setLedModes(quintuple, off, off, off, off);
  myState = learnIRRelayToggle;
}

void transitionTo_learnIRVolUp()
{
  setLedModes(off, fastBlink, off, off, off);
  myState = learnIRVolUp;
}

void transitionTo_learnIRVolDown()
{
  setLedModes(off, off, thrice, off, off);
  myState = learnIRVolDown;
}

void transitionTo_learnIRRelayOn()
{
  setLedModes(off, off, off, fastBlink, off);
  myState = learnIRRelayOn;
}

void transitionTo_learnIRRelayOff()
{
  setLedModes(off, off, off, off, fastBlink);
  myState = learnIRRelayOff;
}

bool checkForSerialCommand()
{
  if (Serial.available())
  {
    char command = Serial.read();
    switch (command)
    {
    case 'n': // Turn Relay On
      transitionTo_relaySignalOn();
      Serial.println(F("okay"));
      break;
    case 'f': // Turn Relay Off
      transitionTo_relaySignalOff();
      Serial.println(F("okay"));
      break;
    case 's': // Status
      Serial.println(F("Status"));
      break;
    case 'v': // Version
      Serial.println(versionNo);
      break;
    case 'l': // Start Learning Essential Codes
      transitionTo_learnIRRelayToggle();
      break;
    case 'L': // Start Learning Codes for Explicit Relay Control
      break;
    case 'x': // Exit Code Learning now
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}

void checkButton()
{ // call in loop(). This calls buttonLongPress() and buttonShortPress().
  // if (buttonState != prevButtonState)
  // {
  //   Debug("Button: ");
  //   Debugln(buttonStateStr[buttonState]);
  //   prevButtonState = buttonState;
  // }

  switch (buttonState)
  {
  case idle:
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      buttonState = down;
    }
    break;
  case down:
    buttonDownMillis = currentMillis;
    buttonState = debounceDown;
    break;
  case debounceDown:
    if (currentMillis - buttonDownMillis >= buttonDebounceInterval)
      buttonState = held;
    break;
  case held:
    if (digitalRead(BUTTON_PIN) == HIGH)
    {
      buttonState = up;
    }
    else if (currentMillis >= longPressLength + buttonDownMillis)
    { // This will not work for 3 seconds every 50 days and 70 minutes, but who cares
      buttonLongPress();
      buttonState = longPress;
    }
    break;
  case up:
    buttonUpMillis = currentMillis;
    buttonPressLength = buttonUpMillis - buttonDownMillis;
    buttonState = debounceUp;
    if (buttonPressLength < longPressLength)
    {
      buttonShortPress();
    }
    else
    {
      buttonLongPress();
    }
    break;
  case debounceUp:
    if (currentMillis - buttonUpMillis >= buttonDebounceInterval)
      buttonState = idle;
    break;
  case longPress:
    if (digitalRead(BUTTON_PIN) == HIGH)
    {
      buttonUpMillis = currentMillis;
      buttonState = debounceUp;
    }
    break;
  }
}

bool checkIRToggle()
{
  // see if a relay state toggle was requested
  bool received = false;
  if (IrReceiver.decode())
  {
    if (IrReceiver.decodedIRData.protocol == IRCodeLearned[0].protocol &&
        IrReceiver.decodedIRData.address == IRCodeLearned[0].address &&
        IrReceiver.decodedIRData.command == IRCodeLearned[0].command)
    {

      if (millis() - lastIRreceivedMillis > 250)
      { // If it's been at least 1/4 second since the last IR received, toggle the relay state
        received = true;
      }
      lastIRreceivedMillis = millis();
    }
    IrReceiver.resume();
  }
  return received;
}

bool saveLearnedIRCodesToEEPROM()
{
  // save just learned codes to EEPROM
  int eeAddress = 0;
  const unsigned long eeMagic = 0xb007ab1e; // leave a magic mark that we have been here here before.
  EEPROM.put(eeAddress, eeMagic);
  eeAddress = eeAddress + sizeof(eeMagic);
  for (byte thisCode = 0; thisCode < NUMBER_OF_CODES_STORED; thisCode++)
  {
    EEPROM.put(eeAddress, IRCodeLearned[thisCode]);
    eeAddress = eeAddress + sizeof(IRCodeLearned[thisCode]);
    Debugln(eeAddress);
  }
}

bool readLearnedIRCodesFromEEPROM()
{
  int eeAddress = 0;
  unsigned long eeMagic;
  EEPROM.get(eeAddress, eeMagic); // attempt to fetch the magic indicating that we have been here before
  if (eeMagic == 0xb007ab1e)
  {
    eeAddress = eeAddress + sizeof(eeMagic);
    for (byte thisCode = 0; thisCode < NUMBER_OF_CODES_STORED; thisCode++)
    {
      EEPROM.get(eeAddress, IRCodeLearned[thisCode]);
      IRData thisStruct;
      EEPROM.get(eeAddress, thisStruct);
      Debugln(thisStruct.protocol);
      Debugln(thisStruct.address);
      Debugln(thisStruct.command);
      Debugln();
      eeAddress = eeAddress + sizeof(IRCodeLearned[thisCode]);
    }
    return true;
  }
  else
  {
    Debugln("Virgin EEPROM!");
    return false;
  }
}

bool learnIRCode(byte IRStructArrayIndex)
{
  bool received = false;
  if (IrReceiver.decode())
  {
    if (IrReceiver.decodedIRData.protocol != UNKNOWN)
    { // Too much Noise from e.g. LED bulbs around to allow Raw signal recording
      if (millis() - lastIRreceivedMillis > 250)
      { // If it's been at least 1/4 second since the last IR received
        received = true;

        Debug("Storing code at index ");
        Debugln(IRStructArrayIndex);
        IRCodeLearned[IRStructArrayIndex] = *IrReceiver.read();
        IRCodeLearned[IRStructArrayIndex].flags = 0; // clear any flags -esp. repeat- for later sending
#ifdef Debug
        IrReceiver.printIRResultShort(&Serial);
#endif
        Debugln();
      }
    }
    lastIRreceivedMillis = millis();
    IrReceiver.resume();
  }
  return received;
}

void sendIRCode(byte IRcommand)
{
  // Check SendDemo for more verbose documentation on the write function
  // Use the write function, which does the switch for different protocols. It's missing in the docs though. :/

  IRData IRSendData;
  // prepare data
  IRSendData.protocol = IRCodeLearned[IRcommand].protocol;
  IRSendData.address = IRCodeLearned[IRcommand].address;
  IRSendData.command = IRCodeLearned[IRcommand].command;
  IRSendData.flags = IRDATA_FLAGS_EMPTY;

  IrSender.write(&IRSendData, NO_REPEATS);
  Debug(F("Sent: "));
  printIRResultShort(&Serial, &IRSendData);
}

void buttonShortPress()
{
  switch (myState)
  {
  case relaySignalOn:
    transitionTo_relaySignalOff();
    break;
  case relaySignalOff:
    transitionTo_relaySignalOn();
    break;

  // a short button pres skips the expected config step (to keep the prev. stored setting)
  case learnIRRelayToggle:
    transitionTo_learnIRVolUp();
    break;
  case learnIRVolUp:
    transitionTo_learnIRVolDown();
    break;
  case learnIRVolDown:
    transitionTo_relaySignalOn();
    break;

  default:
    break;
  }
}

void buttonLongPress()
{
  switch (myState)
  {
  case relaySignalOn:
  case relaySignalOff:
    transitionTo_learnIRRelayToggle();
    break;

  case learnIRRelayToggle:
  case learnIRVolUp:
  case learnIRVolDown:
    // a long button press in Settings mode should just do nuthin (so far)
    break;

  default:
    break;
  }
}

// call in loop() to update the connected LEDs as set in ledMode[]
void updateLeds()
{
  for (byte currentLed = 0; currentLed < ledPinCount; currentLed++)
  {
    auto mode = ledMode[currentLed];
    byte pin = ledPins[currentLed];

    if (mode == off)
    {
      digitalWrite(pin, LOW);
    }
    else if (mode == on)
    {
      digitalWrite(pin, HIGH);
    }
    else if (mode == fastBlink)
    {
      bool lit = (currentMillis & 0b10000000 /* or 0x80 */) == 0; // 0x80 = 128ms via Bitmask
      digitalWrite(pin, lit ? HIGH : LOW);
    }
    else if (mode == slowBlink)
    {
      bool lit = (currentMillis & 0x200) == 0; // 0x200 = every 512ms via Bitmask
      digitalWrite(pin, lit ? HIGH : LOW);
    }
    else
    {
      static int offset = 0;
      if (mode != prevLedMode[currentLed])
      {
        offset = (currentMillis & 0x7FF) + 0x100;
        digitalWrite(pin, LOW);
      }
      else
      {
        int offsetMillis = 0;
        offsetMillis = currentMillis - offset;

        if ((offsetMillis & 0xC0) == 0)
        {
          byte blinkCount = mode & 0x07; // ?? once is 0x11
          bool lit = ((offsetMillis & 0x700) >> 8) < blinkCount;
          digitalWrite(pin, lit ? HIGH : LOW);
        }
        else
        {
          digitalWrite(pin, LOW);
        }
      }
    }

    prevLedMode[currentLed] = mode;
  }
}

void printLedMode(LedMode mode)
{
  Debug("LED Mode: ");
  if (mode > 0x10)
  {
    Debug("Blink x");
    Debugln(mode & 0x07);
  }
  else
  {
    Debugln(ledModeStr[mode]);
  }
}

/// writes individually set LED modes to a the LED mode array
void setLedModes(LedMode newSettingsMode, LedMode newVolDownMode, LedMode newVolUpMode, LedMode newRoffMode, LedMode newRonMode)
{
  ledMode[0] = newSettingsMode;
  ledMode[1] = newVolDownMode;
  ledMode[2] = newVolUpMode;
  ledMode[3] = newRoffMode;
  ledMode[4] = newRonMode;
}
