#if defined (ESP8266)

#include <Esduino.h>
#include <EsduinoConfig.h>
#include <Wire.h>

comdata_t comData;	// the common data structure for commands

volatile bool intFlag = false;	// a flag to indicate that an interrupt has taken place

// the actual interrupt ISR
void intISR() {
  intFlag = true;
}

// singleton for interrupt handling
Esduino *PPIs = NULL;
void ICACHE_RAM_ATTR ppIntChange();
void ppIntChange() {  // static function for the interrupt handler
  if (PPIs) intISR();
}

// BEGIN //

// set the interrupt pin and start the Wire library
int Esduino::begin(int I2Caddress_, int intPin_) {
  intPin = intPin_;
  I2Caddress = I2Caddress_;
  int result;
  PPIs = this; // singleton pointer
  attachInterrupt(intPin, ppIntChange, RISING);
  pinMode(intPin, INPUT_PULLUP);
  Wire.begin(); // Wire on ESP8266 returns no errors on startup
  // reset the Guest MCU
  reset();
  // get the MCU details
  if (((result = getMcuType()) >> 4) != 0xA) return forceGuestError(EGUESTAVRTYPEINVALID);
  // we're good so get the Guest info
  info.mcuType = result;   // internal MCU type
  if ((result = getRemoteFCPU()) < 0) return result;
  info.FCPU = result;        // Guest MCU F_CPU
  if ((result = getDigPins()) < 0) return result;
  info.maxPins = result;        // maximum number of data IO pins
  if ((result = getAnaPins()) < 0) return result;
  info.analogChannels = result;  // maximum number of analog channels
  if ((result = getEepromSize()) < 0) return result;
  info.epmSize = result;   // the EEPROM size
  if ((result = getLedPin()) < 0) return result;
  info.ledPin = result;
  if ((result = getServoPins()) < 0) return result;
  info.servoPins = result;
  // get the Analog Reference values
  if ((result = getDEFAULT()) < 0) return result;
  A_DEFAULT = result;
  if ((result = getINTERNAL()) < 0) return result;
  A_INTERNAL = result;
  if ((result = getEXTERNAL()) < 0) return result;
  A_EXTERNAL = result;
  if ((result = getINTERNAL1V1()) < 0) return result;
  A_INTERNAL1V1 = result;
  if ((result = getINTERNAL2V56()) < 0) return result;
  A_INTERNAL2V56 = result;
  if ((result = getINTERNAL2V56_EXTCAP()) < 0) return result;
  A_INTERNAL2V56_EXTCAP = result;
  // get the pin/channel/port maps
  if ((result = getDigitalMapWord3()) < 0) return result;
  info.digitalMap = result;
  info.digitalMap <<= 16;
  if ((result = getDigitalMapWord2()) < 0) return result;
  info.digitalMap |= result;
  info.digitalMap <<= 16;
  if ((result = getDigitalMapWord1()) < 0) return result;
  info.digitalMap |= result;
  info.digitalMap <<= 16;
  if ((result = getDigitalMapWord0()) < 0) return result;
  info.digitalMap |= result;
  result = getAnalogMapWord();
  info.analogMap = result;
  result = getPortMapWord();
  info.portMap = result;
  // return true as all seems well
  return true;
}// END OF begin()

// DIGITAL READ/WRITE //
// pinMode
int Esduino::pinMode(uint8_t pin, uint8_t mode) {
  if (!checkPin(pin)) return -1;
  comData.command = pinMode_, comData.pin = pin, comData.param1 = mode;
  return sendCommand(2);
}// END OF pinMode()

// digitalRead
int Esduino::digitalRead(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  comData.command = digitalRead_, comData.pin = pin;
  if (sendCommand(1) < 0) return -1;
  return fetchResult();
}// END OF digitalRead()

// digitalWrite
int Esduino::digitalWrite(uint8_t pin, uint8_t state) {
  if (!checkPin(pin)) return -1;
  comData.command = digitalWrite_, comData.pin = pin, comData.param1 = state;
  if (sendCommand(2) < 0) return -1;
  return true;
}// END OF digitalWrite()

// digitalWriteRead, write to the digital pin and then reads its value back
int Esduino::digitalWriteRead(uint8_t pin, uint8_t state) {
  if (!checkPin(pin)) return -1;
  comData.command = digitalWriteRead_, comData.pin = pin, comData.param1 = state;
  if (sendCommand(2) < 0) return -1;
  return fetchResult();
}// END OF digiotalWriteRead()

// ANALOG READ/WRITE //
// analogReference
int Esduino::analogReference(int mode) {
  // check that the MODE is valid for this MCU, -1 says it's NOT
  // so force an internal error state
  if (mode < 0) {
    comData.guestStatus = false;
    comData.result = EANALOGMODEINVALID;
    return -1;
  }
  comData.command = analogReference_, comData.pin = mode;
  return sendCommand(1);
}// END OF analogReference()

// analogRead
int Esduino::analogRead(uint8_t channel) {
  if (!checkAnalogChannel(channel)) return -1;
  comData.command = analogRead_, comData.pin = channel;
  if (sendCommand(1) < 0) return -1;
  return fetchResult();
}// END OF analogRead()

// analogWrite
int Esduino::analogWrite(uint8_t pin, uint8_t val) {
  if (!checkPin(pin)) return -1;
  comData.command = analogWrite_, comData.pin = pin, comData.param1 = val;
  return sendCommand(2);
}// END OF analogWrite()

// TONE //
// tone <freq>
int Esduino::tone(uint8_t pin, uint16_t freq) {
  if (!checkPin(pin)) return -1;
  comData.command = tone1_, comData.pin = pin, comData.param1 = freq;
  return sendCommand(2);
}// END OF tone(uint8_t pin, uint16_t freq)

// tone <freq>, <duration>
int Esduino::tone(uint8_t pin, uint16_t freq, uint16_t dur) {
  if (!checkPin(pin)) return -1;
  comData.command = tone2_, comData.pin = pin, comData.param1 = freq, comData.param2 = dur;
  return sendCommand(3);
}// END OF tone(uint8_t pin, uint16_t freq, uint16_t dur)

// noTone
int Esduino::noTone(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  comData.command = noTone_, comData.pin = pin;
  return sendCommand(1);
}// END OF noTone()

// DIRECT PORT ACCESS //

int Esduino::regWrite(uint8_t reg, uint8_t mask) {
  if (!checkPort(reg & 0x0F)) return -1;
  comData.command = regWrite_, comData.pin = reg, comData.param1 = mask;
  return sendCommand(2);
}// END OF regWrite()


int Esduino::regRead(uint8_t reg) {
  if (!checkPort(reg & 0x0F)) return -1;
  comData.command = regRead_, comData.pin = reg;
  if (sendCommand(1) < 0) return -1;
  return fetchResult();
}// END OF regRead()


int Esduino::regBitWrite(uint8_t reg, uint8_t bit, uint8_t state) {
  if (!checkPort(reg & 0x0F)) return -1;
  comData.command = regBitWrite_, comData.pin = reg, comData.param1 = bit, comData.param2 = state;
  return sendCommand(3);
}// END OF regBitWrite()


int Esduino::regBitRead(uint8_t reg, uint8_t bit) {
  int result = regRead(reg);
  if (result < 0) return result;
  if ((result & (1 << bit)) > 0) return true;
  else return false;
}// END OF regBitread()


// SERVOS //
// unlike the Arduino, servos are assigned to a pin only, the servo library instance is
// taken care of by the Guest. You can assign up to 12 servos on the ATmega328, 24 on the ATmega1284
// and up to 48 on the ATmega2560

// svoAttach(pin)
int Esduino::svoAttach(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  int result;
  comData.command = svoAttach1_, comData.pin = pin;
  if ((result = sendCommand(1)) < 0) return result;
  if ((result = fetchResult()) < 0) return result;
  if (!comData.guestStatus) return -1;
  return true;
}// END OF svoAttach(uint8_t pin)

// svoAttach(pin,min,max)
int Esduino::svoAttach(uint8_t pin, uint32_t min, uint32_t max) {
  if (!checkPin(pin)) return -1;
  int result;
  comData.command = svoAttach2_, comData.pin = pin, comData.param1 = min, comData.param2 = max;
  if ((result = sendCommand(3)) < 0) return result;
  if ((result = fetchResult()) < 0) return result;
  if (!comData.guestStatus) return -1;
  return true;
}// END OF svpAttach(uint8_t pin, uint32_t min, uint32_t max)

// svoWrite(pin,angle)
int Esduino::svoWrite(uint8_t pin, uint8_t angle) {
  if (!checkPin(pin)) return -1;
  comData.command = svoWrite_, comData.pin = pin, comData.param1 = angle;
  return sendCommand(2);
}// END OF svoWrite()

// svoWriteMicroseconds(pin,microseconds)
int Esduino::svoWriteMicroseconds(uint8_t pin, uint32_t microseconds) {
  if (!checkPin(pin)) return -1;
  comData.command = svoWriteMicroseconds_, comData.pin = pin, comData.param1 = microseconds;
  return sendCommand(2);
}// END OF svoWriteMicroseconds()

// svoRead(pin) returns the servo angle
int Esduino::svoRead(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  int result;
  comData.command = svoRead_, comData.pin = pin;
  if ((result = sendCommand(1)) < 0) return result;
  if ((result = fetchResult()) < 0) return result;
  if (!comData.guestStatus) return -1;
  return comData.result;
}// END OF svoRead()

// svoAttached(pin) returns true if a servo is assigned, false if not
int Esduino::svoAttached(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  int result;
  comData.command = svoAttached_, comData.pin = pin;
  if ((result = sendCommand(1)) < 0) return result;
  if ((result = fetchResult()) < 0) return result;
  if (!comData.guestStatus) return -1;
  return true;
}// END OF svoAttached()

// svoDetach(pin) return true if successful, false if not
int Esduino::svoDetach(uint8_t pin) {
  if (!checkPin(pin)) return -1;
  int result;
  comData.command = svoDetach_, comData.pin = pin;
  if ((result = sendCommand(1)) < 0) return result;
  if ((result = fetchResult()) < 0) return result;
  if (!comData.guestStatus) return -1;
  return true;
}// END OF svoDetach()

// EEPROM

int Esduino::epmRead(uint16_t address) {
  // check address is valid
  if (address >= info.epmSize) {
    return forceGuestError(EBEYONDEEPROMLIMIT);
  }
  comData.command = epmRead_, comData.param1 = address;
  if (sendCommand(2) < 0) return -1;
  return fetchResult();
}// END OF epmRead()

int Esduino::epmWrite(uint16_t address, uint8_t data) {
  // check address is valid
  if (address >= info.epmSize) {
    return forceGuestError(EBEYONDEEPROMLIMIT);
  }
  comData.command = epmWrite_, comData.pin = data, comData.param1 = address;
  if (sendCommand(2) < 0) return -1;
  return true;
}// END OF epmWrite()

int Esduino::epmUpdate(uint16_t address, uint8_t data) {
  // check address is valid
  if (address >= info.epmSize) {
    return forceGuestError(EBEYONDEEPROMLIMIT);
  }
  comData.command = epmUpdate_, comData.pin = data, comData.param1 = address;
  if (sendCommand(2) < 0) return -1;
  return true;
}// END OF epmUpdate()

int Esduino::epmPut(uint16_t address, void *data, uint8_t len) {
  // check address + len
  if ((address + len) >= info.epmSize) {
    return forceGuestError(EBEYONDEEPROMLIMIT);
  }

  uint8_t* eepBytes = reinterpret_cast<uint8_t*>(data);
  while (len--) {
    comData.command = epmUpdate_, comData.pin = *eepBytes++, comData.param1 = address++;
    if (sendCommand(2) < 0) return -1;
  }
  return true;
}// END OF epmPut()

int Esduino::epmGet(uint16_t address, void *data, uint8_t len) {
  // check address + len
  if ((address + len) >= info.epmSize) {
    return forceGuestError(EBEYONDEEPROMLIMIT);
  }

  uint8_t* eepBytes = reinterpret_cast<uint8_t*>(data);
  while (len--) {
    comData.command = epmRead_, comData.param1 = address++;
    if (sendCommand(2) < 0) return -1;
    if (fetchResult() < 0) return -1;
    *eepBytes++ = comData.result;
  }
  return true;
}// END OF epmGet()

// MISC
// print the details of the info MCU setup
void Esduino::printDetails(void) {
  printf("%16s\t%X\r\n", "MCU Type:", info.mcuType);
  printf("%16s\t%u MHz\r\n", "MCU F_CPU:", info.FCPU);
  printf("%16s\t%u\r\n", "Digital Pins:", info.maxPins);
  printf("%16s\t%u\r\n", "Analog Channels:", info.analogChannels);
  printf("%16s\t%u\r\n", "Servo Pins:", info.servoPins);
  printf("%16s\t%u\r\n", "EEPROM Size:", info.epmSize);
  printf("%s\r\n", "Digital Pin Map:");
  printReverseBinary(info.digitalMap, sizeof(info.digitalMap));
  printf("%s\r\n", "Analog Channel Map:");
  printReverseBinary(info.analogMap, sizeof(info.analogMap));
  printf("%s\r\n", "PORT Map:");
  printReverseBinary(info.portMap, sizeof(info.portMap));
}// END OF printDetails()

// an alternative to the perror function which also includes the
// Esduino info error codes
void Esduino::perror(const char * msg) {
  // is there an internal Esduino error ?
  if (!comData.guestStatus) {
    printf("%s %s\n", msg, guestErrors[comData.result]);
    return;
  }
  // if not an internal error, it must be a normal error ?
  ::perror(msg);
}// END OF perror()

// reset, causes a soft reset to address 0x0000 on the remote ATmega328
// bypassing the bootloader so, no 2 second reboot delay
void Esduino::reset() {
  comData.command = guestReset_;
  sendCommand(0);
  delay(100);  // wait for peripheral to reset
}// END OF reset()

void Esduino::printReverseBinary(uint64_t x, uint8_t len) {
  len *= 8; // convert len from Bytes to bits
  while (len) printf("%c", x & (1LL << --len) ? '1' : '0');
  printf("\r\n");
}// END OF printReverseBinary()

// PRIVATE METHODS //

// requests the AVR type from the info
int Esduino::getMcuType(void) {
  comData.command = getMcuType_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

// return the info CPU frequency in MHz
int Esduino::getRemoteFCPU(void) {
  comData.command = remoteFCPU_;
  sendCommand(0);
  return fetchResult();
}

int Esduino::getLedPin(void) {
  comData.command = getLedPin_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getDigPins(void) {
  comData.command = getDigPins_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getAnaPins(void) {
  comData.command = getAnaPins_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getEepromSize(void) {
  comData.command = getEepromSize_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getServoPins(void) {
  comData.command = getServoPins_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

// TX/RX //

// sends the required values from the comData structure to the I2C info
int Esduino::sendCommand(uint8_t numParams, uint16_t remProcDelay) {
  int result = 0;
  intFlag = false;
  comData.guestStatus = true;
#if PP_DEBUG
  uint32_t startMillis = millis();
#endif
  // send data structure as required
  if (numParams > 3) numParams = 3; // limit number of parameters

  Wire.beginTransmission(I2Caddress);
  // send comData.command
  Wire.write(comData.command);  // assumes that command is always 1 Byte
  // send comData.pin
  if (numParams > 0) Wire.write(comData.pin); // assumes that pin is always 1 Byte
  // send comData.param1
  if (numParams > 1) sendParameter(&comData.param1, sizeof(comData.param1));
  // send comData.param2
  if (numParams > 2) sendParameter(&comData.param2, sizeof(comData.param2));

  // send the I2C buffer
  if ((result = Wire.endTransmission()) > 0) {
    // on Wire error
    clearComData();
    comData.result = result;
    return -1;
  }
  // wait for the info to finish
  uint32_t timeout = millis();
  uint32_t period;
  // wait up to 2000 milliseconds if this is a reset command, or 500 milliseconds normally
  if (comData.command == guestReset_) period = 2000;
  else period = 500;
  // timeout loop
  while ((millis() - timeout) < period) {
    if (intFlag) break;
    yield();
  }
  // if no interrupt is triggered
  if (!intFlag) {
    clearComData();
    comData.guestStatus = false;
    comData.result = EGUESTTIMEOUT;
    return -1;
  }

#if PP_DEBUG  // print out the comData details in HEX
  uint32_t endMillis = millis();
  printf("Command: 0x%02X Pin: %u P1: %04X P2: %04X\tintFlag = %u\tin %u mS\n", comData.command, comData.pin, comData.param1, comData.param2, intFlag, (endMillis - startMillis));
#endif
  // all is good
  return true;
}// END OF sendCommand()

// breaks down the variable and puts it Byte at a time in the Wire buffer
void Esduino::sendParameter(const void* param, uint8_t numByates) {
  const uint8_t* my_bytes = reinterpret_cast<const uint8_t*>(param);
  for (uint8_t x = 0; x < numByates; x++) Wire.write(*my_bytes++);
}// END OF sendParameter()

// fetches the returning data from the info. 3 Bytes are returned, 1 = guestStatus
// Bytes 2 & 3 are the data
int Esduino::fetchResult() {
  int result;
  result = Wire.requestFrom(I2Caddress, sizeof(comData.result) + 1);
  if (result != (sizeof(comData.result) + 1)) return -1;
  // OK, we got the Bytes so read the data
  comData.guestStatus = Wire.read();  // error Status byte, true = good, false = error
  // now the incoming data into comData.result
  uint8_t* my_bytes = reinterpret_cast<uint8_t*>(&comData.result);
  for (uint8_t x = 0; x < sizeof(comData.result); x++) *my_bytes++ = Wire.read();
  // is the data valid or is it an error?
  if (!comData.guestStatus) return -1; // it's an internal error
  // data is good
  return comData.result;
}// END OF fetchResult()

// set an error condition as if the guest responded
int Esduino::forceGuestError(uint8_t error) {
  comData.guestStatus = false;
  comData.result = error;
  return -1;
}// END OF forceGuestError()

// clear the mdata structure
void Esduino::clearComData(void) {
  // clear the data structure
  comData.command = 0;
  comData.pin = 0;
  comData.param1 = 0;
  comData.param2 = 0;
  comData.result = 0;
  comData.guestStatus = 1;
  comData.checksum = 0;
}// END OF clearComData()

// checks for excluded pins and forces a info error condition if it fails
// CAN THESE BE PULLED INTO ONE FUNCTION????? //
bool Esduino::checkPin(uint8_t pin) {
  // clear the info error status
  comData.guestStatus = false;
  comData.result = EPINNOTAVAILABLE;
  if (pin >= (sizeof(info.digitalMap) * 8)) return false;
  uint8_t *myBytes = reinterpret_cast<uint8_t*>(&info.digitalMap);
  myBytes += ((pin / 8) ^ 7);  // get the Byte in the variable (inverted 0-X = X-0)
  // now get the bit in the Byte (inverted bit 0 = bit 7, 7 = bit 0)
  if (!(*myBytes & (1 << ((pin % 8) ^ 0x07)))) return false;
  comData.guestStatus = true;
  return true;
}// END OF checkPin()

// checks for excluded analog channels and forces a info error condition if it fails
bool Esduino::checkAnalogChannel(uint8_t chan) {
  // clear the info error status
  comData.guestStatus = false;
  comData.result = ECHANNELNOTAVAILABLE;
  if (chan >= (sizeof(info.analogMap) * 8)) return false;
  uint8_t *myBytes = reinterpret_cast<uint8_t*>(&info.analogMap);
  myBytes += ((chan / 8) ^ 1);  // get the Byte in the variable (inverted 0-X = X-0)
  // now get the bit in the Byte (inverted bit 0 = bit 7, 7 = bit 0)
  if (!(*myBytes & (1 << ((chan % 8) ^ 0x07)))) return false;
  comData.guestStatus = true;
  return true;
}// END OF checkAnalogChannel()

bool Esduino::checkPort(uint8_t port) {
  // clear the info error status
  comData.guestStatus = false;
  comData.result = EPORTNOTAVAILABLE;
  if (port >= (sizeof(info.portMap) * 8)) return false;
  uint8_t *myBytes = reinterpret_cast<uint8_t*>(&info.portMap);
  myBytes += ((port / 8) ^ 1);  // get the Byte in the variable (inverted 0-X = X-0)
  // now get the bit in the Byte (inverted bit 0 = bit 7, 7 = bit 0)
  if (!(*myBytes & (1 << ((port % 8) ^ 0x07)))) return false;
  comData.guestStatus = true;
  return true;
}// END OF checkPort()

// GUEST PARAMETERS //
// VREF
int Esduino::getDEFAULT(void) {
  comData.command = getDEFAULT_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getDEFAULT()

int Esduino::getINTERNAL(void) {
  comData.command = getINTERNAL_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getINTERNAL()

int Esduino::getEXTERNAL(void) {
  comData.command = getEXTERNAL_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getEXTERNAL()

int Esduino::getINTERNAL1V1(void) {
  comData.command = getINTERNAL1V1_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getINTERNAL1V1()

int Esduino::getINTERNAL2V56(void) {
  comData.command = getINTERNAL2V56_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getINTERNAL2V56()

int Esduino::getINTERNAL2V56_EXTCAP(void) {
  comData.command = getINTERNAL2V56_EXTCAP_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}// END OF getINTERNAL2V56_EXTCAP()

// PULL THE GETDIGITALMAPWORD FUNCTIONS BELOW INTO ONE FUNCTION - TODO //

// pin/analog/port maps
int Esduino::getDigitalMapWord0(void) {
  comData.command = getDigitalMapWord0_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getDigitalMapWord1(void) {
  comData.command = getDigitalMapWord1_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getDigitalMapWord2(void) {
  comData.command = getDigitalMapWord2_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getDigitalMapWord3(void) {
  comData.command = getDigitalMapWord3_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

// PULL THE GETDIGITALMAPWORD FUNCTIONS ABOVE INTO ONE FUNCTION - TODO //

int Esduino::getAnalogMapWord(void) {
  comData.command = getAnalogMapWord_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

int Esduino::getPortMapWord(void) {
  comData.command = getPortMapWord_;
  if (sendCommand(0) < 0) return -1;
  return fetchResult();
}

#endif
