/*
  Melody

  Plays a melody

  circuit: - 8 ohm speaker or piezo on digital pin 3

  created 21 Jan 2010
  modified 30 Aug 2011
  by Tom Igoe

  Modified for 'Esduino use Aug 2020 by Rev Phil

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Tone
*/
#include <ESP8266WiFi.h>
#include <Esduino.h>
#include "pitches.h"

Esduino pp;  // an object for the Esduino library

#define GUEST_I2C_ADDRESS 9
#define INTERRUPT_PIN 2

#define TONE_PIN 3

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
uint8_t noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  Serial.begin(115200); // default Serial configuration
  Serial.setDebugOutput(true);  // for printf
  // no WiFi
  WiFi.disconnect();  // disconnect WiFi
  WiFi.mode(WIFI_OFF);    // turn WiFi OFF
  WiFi.forceSleepBegin(); // put WiFi to sleep
  delay(1000);
  printf("\r\n\r\nEsduino ESP8266 Host...\r\n");
  // start Esduino, (I2C address, Interrupt pin)
  if (pp.begin(GUEST_I2C_ADDRESS, INTERRUPT_PIN) < 0) pp.perror("Esduino begin:");

  // OK, we've started up correctly, let's print out the details of the Guest MCU
  pp.printDetails();

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    pp.tone(TONE_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    pp.noTone(TONE_PIN);
  }
}

void loop() {
  // no need to repeat the melody.
}
