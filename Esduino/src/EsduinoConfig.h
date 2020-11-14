#ifndef ESDUINOCONFIG_H
#define ESDUINOCONFIG_H

// internal commands common to Host and guestInfo
// DEBUG/DIGITAL/ANALOG
enum digital {
  pinMode_ = 0x01,
  digitalWrite_,
  digitalRead_,
  digitalWriteRead_,
  analogReference_,
  analogRead_,
  analogWrite_,
  tone1_,
  tone2_,
  noTone_
};

// PORTS
enum {
  regWrite_ = 0x11,
  regRead_,
  regBitWrite_,
  regBitRead_,
  regBitSet_,
  regBitClr_
};
// SERVO
enum {
  svoAttach1_ = 0x21,
  svoAttach2_,
  svoWrite_,
  svoWriteMicroseconds_,
  svoRead_,
  svoAttached_,
  svoDetach_,
};

// EEPROM
enum {
  epmRead_ = 0x31,
  epmWrite_,
  epmUpdate_,
  epmPut_,
  epmGet_
};
// MISC
enum {
  guestReset_ = 0x41,
  remoteFCPU_,
  getMcuType_,
  getLedPin_,
  getDigPins_,
  getAnaPins_,
  getEepromSize_,
  getServoPins_
};
// CONFIG - ADC Reference macros
enum {
  getDEFAULT_ = 0x51,
  getINTERNAL_,
  getEXTERNAL_,
  getINTERNAL1V1_,
  getINTERNAL2V56_,
  getINTERNAL2V56_EXTCAP_
};
// DIGITAL, ANALOG and PORT Maps
enum {
  getDigitalMapWord0_ = 0x61,
  getDigitalMapWord1_,
  getDigitalMapWord2_,
  getDigitalMapWord3_,
  getAnalogMapWord_,
  getPortMapWord_
};

// PinsPlus error definitions
enum {
  ESUCCESS,             // 0
  EPININUSE,            // 1 pin in use
  EPINNOTAVAILABLE,     // 2 pin not available
  ENOPINSAVAILABLE,     // 3 no pins available
  EGUESTTIMEOUT,        // 4 guestInfo did not trigger an interrupt
  ECHANNELNOTAVAILABLE, // 5 analog channel not available
  EANALOGMODEINVALID,   // 6 analog Mode invalid
  EINTPININVALID,       // 7 the interrupt pin is invalid
  EGUESTAVRTYPEINVALID, // 8 guestInfo AVR type invalid
  EPORTNOTAVAILABLE,    // 9 Port not avalable
  EBEYONDEEPROMLIMIT    // the address is beyond the EEPROM address limit
};

// only include the error strings etc. if this is the Host e.g. a Raspberry Pi
#if defined (__arm__) || defined (ESP8266)
// error strings
const char *guestErrors[] {
  "OK",                    // 0
  "Pin in use",           // 1
  "Pin not available",        // 2
  "No pins available",        // 3
  "guestInfo Timeout",              // 4
  "Analog Channel not available", // 5
  "Analog MODE invalid",      // 6
  "Invalid Interrupt Pin",        // 7
  "Invalid guestInfo AVR type",       // 8
  "Port not avalable",        // 9
  "Address is beyond the EEPROM limit"
};

#endif

// the common data structure for data between the Host and Guest
typedef struct comdata_t{
  uint8_t command = 0;  // command is always 1 Byte
  uint8_t pin = 0;      // pin is always 1 Byte
  uint16_t param1 = 0;  // the size of param1 can be altered
  uint16_t param2 = 0;  // the size of param2 can be altered
  uint16_t result = 0;  // the size of the result can be altered
  uint8_t guestStatus = 1;  // guestStatus is always 1 Byte
  uint8_t checksum = 0;	// TODO
};

#endif
