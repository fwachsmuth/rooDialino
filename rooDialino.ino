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
/*  The header defines macros for input and output pin etc.
 *  Default on a 328P is to Receive on Pin 2 and to send on Pin 3 — any should work though.
 *  See https://github.com/Arduino-IRremote/Arduino-IRremote for Pin implications.
 *  
 *  I'm overwirting these below to free up 2/3 for Interrupts.
 *  
 */
#define IR_RECEIVE_PIN      7 
#define IR_SEND_PIN         8


//#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 240 bytes program space if IrSender.write is used
//#define SEND_PWM_BY_TIMER
//#define USE_NO_SEND_PWM

#include <IRremote.h>

#define DELAY_AFTER_SEND 5  // shorter than 5 ms might make dirty signal

#define PIN2  2
#define PIN3  3


volatile int volSteps;

void volDownISR() {
  volSteps--;
}
void volUpISR() {
  volSteps++;
}

void setup() {
  pinMode(PIN2, INPUT_PULLUP);
  pinMode(PIN3, INPUT_PULLUP);
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

uint16_t sAddress; 
uint8_t sCommand; 
uint8_t sRepeats; 

void loop() {
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
/*    if (digitalRead(PIN1) == LOW) {
      Serial.println("Vol +");
      sAddress = 0x16;
      sCommand = 0x10;
      sRepeats = 0;
    } else if (digitalRead(PIN2) == LOW) {
      Serial.println("Vol -");
      sAddress = 0x16;
      sCommand = 0x11;
      sRepeats = 0;
    } else {
      sAddress = 0x0;
      sCommand = 0x0;   
      sRepeats = 0;
    }
    
    if (sAddress != 0 && sCommand != 0) {

      Serial.println();
      Serial.print(F("address=0x"));
      Serial.print(sAddress, HEX);
      Serial.print(F(" command=0x"));
      Serial.print(sCommand, HEX);
      Serial.print(F(" repeats="));
      Serial.println(sRepeats);
      Serial.println();
      Serial.println();
      Serial.flush();

      Serial.print(F("Send RC5: "));
      Serial.flush();
      IrSender.sendRC5(sAddress & 0x1F, sCommand & 0x3F, sRepeats, true); // 5 address, 6 command bits
//      sAddress = 0;
//      sCommand = 0;
      delay(DELAY_AFTER_SEND);
    }
    */
}
