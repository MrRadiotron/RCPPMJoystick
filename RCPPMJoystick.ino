/***
 * An Arduino Leonardo/Pro Micro version of a RC PPM to USB Joystick Converter - using Arduino Joystick Library.
 * 
 * Requires no modifications to the arduino core files.
 * 
 * Remix of single joystick example at: https://github.com/MHeironimus/ArduinoJoystickLibrary
 * and: https://github.com/voroshkov/Leonardo-USB-RC-Adapter/blob/master/Leonardo-USB-RC-Adapter.ino
 * based off: https://github.com/timonorawski/RCPPMJoystick 
 * 
 * Requires installation of single joystick library from: https://github.com/MHeironimus/ArduinoJoystickLibrary
 * 
 * Wiring:
 *  - transmitter trainer port center pole: Arduino D4
 *  - transmitter trainer port barrel: Arduino GND
 * 
 * by Timon Orawski
 * 
 * 2016-04-11
 * 
 * Copyright (c) 2016, Timon Orawski
 * 
 * modified by https://github.com/MrRadiotron 2020-10-16
 *
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <Joystick.h>
#include <limits.h>
#include <avr/interrupt.h>

// Use for Futaba transmitters (they have shifted center value and narrower range by default)
// #define FUTABA

// if you have a stick that isn't centred at 1500ppm, set your center below
#define CUSTOM_STICK_CENTER 1500

// if any of your controls are inverted, comment/uncomment the lines below
bool INVERT_THROTTLE = true;
bool INVERT_PITCH = true;
bool INVERT_ROLL = true;
bool INVERT_YAW = true;

// Use to enable output of PPM values to serial
//#define SERIALOUT

#define RC_CHANNELS_COUNT 7

#ifdef FUTABA
#define STICK_HALFWAY 370
#define STICK_CENTER 1500
#define THRESHOLD 200
#else
#ifdef CUSTOM_STICK_CENTER
#define STICK_CENTER CUSTOM_STICK_CENTER
#else
#define STICK_CENTER 1500
#endif
#define STICK_HALFWAY 500
#define THRESHOLD 100 // threshold is used to detect PPM values (added to range at both ends)
#endif

#define USB_STICK_VALUE_MIN 0
#define USB_STICK_VALUE_MAX 1023

#define USB_STICK_ROTATION_VALUE_MIN 0
#define USB_STICK_ROTATION_VALUE_MAX 1023

#define STICK_END_DEAD_ZONE 15

#define MIN_PULSE_WIDTH (STICK_CENTER - STICK_HALFWAY - STICK_END_DEAD_ZONE)
#define MAX_PULSE_WIDTH (STICK_CENTER + STICK_HALFWAY + STICK_END_DEAD_ZONE)

#define NEWFRAME_PULSE_WIDTH 3000

// timer capture ICP1 pin corresponds to Leonardo digital pin 4
#define PPM_CAPTURE_PIN 4
#define LED_PIN 13

// for timer prescaler set to 1/8 of 16MHz, counter values should be
//  divided by 2 to get the number of microseconds
#define TIMER_COUNT_DIVIDER 2

typedef struct ch_values_s {
  int min_val;
  int max_val;
  int val;
  int invert;
  int usb_val;
} ch_values_t;

ch_values_t ch_values[RC_CHANNELS_COUNT];

// this array contains the lengths of read PPM pulses in microseconds
volatile uint16_t rcValue_v[RC_CHANNELS_COUNT] = {STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER};
uint16_t rcValue[RC_CHANNELS_COUNT] = {STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER, STICK_CENTER};

// enum defines the order of channels
enum {
  YAW=0,
  PITCH=1,
  THROTTLE=2,
  ROLL=3,
  GEAR=4,
  AUX1=5,
  AUX2=6
};

// Create Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 1, 0,
  true, true, true, true, true, true,
  false, false, false, false, false);

void setup() {
  setupPins();
  initTimer();
  // Initialize Joystick Library
  Joystick.setXAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.setYAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.setZAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.setRxAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.setRyAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.setRzAxisRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
//  Joystick.setThrottleRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
//  Joystick.setRudderRange(USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX);
  Joystick.begin(false);
  
#ifdef SERIALOUT
  Serial.begin(115200);
#endif

// set inverted axis
  ch_values[ROLL].invert = INVERT_ROLL;
  ch_values[THROTTLE].invert = INVERT_THROTTLE;
  ch_values[PITCH].invert = INVERT_PITCH;
  ch_values[YAW].invert = INVERT_YAW;
// init min max values
  for (int i = 0; i < RC_CHANNELS_COUNT; ++i) {
    ch_values[i].val = STICK_CENTER;
    ch_values[i].min_val = INT_MAX;
    ch_values[i].max_val = INT_MIN;
    ch_values[i].usb_val = USB_STICK_VALUE_MAX / 2;
  }
}

void map_values(void) {
  int val, max_val, min_val, invert, usb_val;
  
  for (int i = 0; i < RC_CHANNELS_COUNT; ++i) {
    if (ch_values[i].val > ch_values[i].max_val) {
      ch_values[i].max_val = ch_values[i].val;
    }
    if (ch_values[i].val < ch_values[i].min_val) {
      ch_values[i].min_val = ch_values[i].val;
    }

    val = ch_values[i].val;
    max_val = ch_values[i].max_val - STICK_END_DEAD_ZONE;
    min_val = ch_values[i].min_val + STICK_END_DEAD_ZONE;
    invert = ch_values[i].invert;
    usb_val = ch_values[i].usb_val;
  
    usb_val = map(val,
                 min_val, max_val,
                 USB_STICK_VALUE_MIN, USB_STICK_VALUE_MAX
                );
                
    if (usb_val > USB_STICK_VALUE_MAX) {
      usb_val = USB_STICK_VALUE_MAX;
    }
    if (usb_val < USB_STICK_VALUE_MIN) {
      usb_val = USB_STICK_VALUE_MIN;
    }

    if (ch_values[i].invert) {
      usb_val = USB_STICK_VALUE_MAX - usb_val;
    }

    ch_values[i].usb_val = usb_val;
  }
}

void loop() {
  for (int i = 0; i < RC_CHANNELS_COUNT; ++i) {
    rcValue[i] = rcValue_v[i];
    ch_values[i].val = rcValue[i];
  }
  map_values();
  setControllerDataJoystick();
#ifdef SERIALOUT  

  Serial.print(rcValue[YAW]); 
  Serial.print("\t");
  Serial.print(rcValue[THROTTLE]); 
  Serial.print("\t");
  Serial.print(rcValue[ROLL]); 
  Serial.print("\t");
  Serial.print(rcValue[PITCH]); 
  Serial.print("\t");
  Serial.print(rcValue[AUX1]); 
  Serial.print("\t");
  Serial.print(rcValue[AUX2]); 
  Serial.print("\t");
  Serial.print(rcValue[GEAR]); 
  Serial.println("\t");

  Serial.print(ch_values[YAW].usb_val); 
  Serial.print("\t");
  Serial.print(ch_values[THROTTLE].usb_val); 
  Serial.print("\t");
  Serial.print(ch_values[ROLL].usb_val); 
  Serial.print("\t");
  Serial.print(ch_values[PITCH].usb_val); 
  Serial.print("\t");
  Serial.print(ch_values[AUX1].usb_val); 
  Serial.print("\t");
  Serial.print(ch_values[AUX2].usb_val);
  Serial.print("\t");
  Serial.print(ch_values[GEAR].usb_val); 
  Serial.println("\t");
#endif
  Joystick.sendState();
  delay(5);
}

void setControllerDataJoystick() {
  Joystick.setXAxis(ch_values[ROLL].usb_val);
  Joystick.setYAxis(ch_values[PITCH].usb_val);
  Joystick.setRxAxis(ch_values[YAW].usb_val);
  Joystick.setRyAxis(ch_values[THROTTLE].usb_val);

  Joystick.setZAxis(ch_values[AUX1].usb_val);
  Joystick.setRzAxis(ch_values[AUX2].usb_val);
    
  Joystick.setButton(0, rcValue[GEAR] > STICK_CENTER);
  
}

void setupPins(void) {
  // Set up the Input Capture pin
  pinMode(PPM_CAPTURE_PIN, INPUT);
  //digitalWrite(PPM_CAPTURE_PIN, 1); // enable the pullup
  pinMode(LED_PIN, OUTPUT);
}

void initTimer(void) {
  // Input Capture setup
  // ICNC1: =0 Disable Input Capture Noise Canceler to prevent delay in reading
  // ICES1: =1 for trigger on rising edge
  // CS11: =1 set prescaler to 1/8 system clock (F_CPU)
  TCCR1A = 0;
  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
  TCCR1C = 0;

  // Interrupt setup
  // ICIE1: Input capture
  TIFR1 = (1 << ICF1); // clear pending
  TIMSK1 = (1 << ICIE1); // and enable
}

uint16_t adjust(uint16_t diff, uint8_t chan) {
  // Here you can trim your rc values (e.g. if TX has no trims).

  // switch (chan) {
  //   case THROTTLE: return diff + 50;
  //   case YAW:      return diff + 60;
  //   case PITCH:    return diff + 60;
  //   case ROLL:     return diff + 90;
  //   case AUX1:     return diff + 10;
  // }

  //convert to microseconds (because of timer prescaler usage)
  return diff / TIMER_COUNT_DIVIDER;
}
ISR(TIMER1_CAPT_vect) {
  union twoBytes {
    uint16_t word;
    uint8_t  byte[2];
  } timeValue;

  uint16_t now, diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;

  timeValue.byte[0] = ICR1L;    // grab captured timer value (low byte)
  timeValue.byte[1] = ICR1H;    // grab captured timer value (high byte)

  now = timeValue.word;
  diff = now - last;
  last = now;

  //all numbers are microseconds multiplied by TIMER_COUNT_DIVIDER (as prescaler is set to 1/8 of 16 MHz)
  if (diff > (NEWFRAME_PULSE_WIDTH * TIMER_COUNT_DIVIDER)) {
    chan = 0;  // new data frame detected, start again
  }
  else {
    if (diff > (MIN_PULSE_WIDTH * TIMER_COUNT_DIVIDER - THRESHOLD)
        && diff < (MAX_PULSE_WIDTH * TIMER_COUNT_DIVIDER + THRESHOLD)
        && chan < RC_CHANNELS_COUNT)
    {
      rcValue_v[chan] = adjust(diff, chan); //store detected value
    }
    chan++; //no value detected within expected range, move to next channel
  }
}
