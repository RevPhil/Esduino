//*****************************//
// THE MAIN PinsPlus FUNCTIONS //
//*****************************//

// process any incoming I2C command data
void processRxData(uint8_t _rxBytes) {
  if (_rxBytes) {
#if DEBUG
    volatile uint8_t* rx_bytes = reinterpret_cast<volatile uint8_t*>(&comData);
    printf("%u Bytes received... ", _rxBytes);
    for (uint8_t x = 0; x < rxBytes; x++) {
      printf("%02X ", *rx_bytes++);
    }
    printf("\t");
#endif
    // preset the index, result and error variables
    int8_t index = -1;
    comData.guestStatus = true;
    comData.result = 0;
    uint8_t tmp;
    // reg and regType are used later by the cirect port access functions
    uint8_t reg = (comData.pin & 0x0F) + 1; // register mapping starts at 1
    uint8_t regType = comData.pin >> 4;  // DDRx, PORTx or PINx

    switch (comData.command) {

      //*******************************************//
      // USER DEFINED COMMANDS SHOULE BE PUT HERE! //
      // ******************************************//

      // getAvrType
      case getMcuType_:
        printCommand("getMcuType_");
        comData.result = PP_MCU_TYPE;
        break;

      case getLedPin_:
        printCommand("getLedPin_");
        comData.result = LED_BUILTIN;
        break;

      // reset
      case guestReset_:
        printCommand("reset_");
        //DDRB = 0x00;
        //DDRC = 0x00;
        //DDRD = 0x00;
        resetFunc();
        break;

      // return this unit's F_CPU in MHz
      case remoteFCPU_:
        printCommand("remoteFCPU_");
        comData.result = F_CPU / 1000000LL;
        break;

      // return the number of digital pins
      case getDigPins_:
        printCommand("getDigPins_");
        comData.result = PP_DIG_PINS;
        break;

      // return the number of analog pins
      case getAnaPins_:
        printCommand("getAnaPins_");
        comData.result = PP_ANA_PINS;
        break;

      // return the EEPROM size in Bytes
      case getEepromSize_:
        printCommand("getEepromSize_");
        comData.result = PP_EEPROM_SIZE;
        break;

      // return the number of Servo pins
      case getServoPins_:
        printCommand("getServoPins_");
        comData.result = PP_NUM_SERVOS;
        break;

      // CONFIG - ADC voltage source
      case getDEFAULT_:
        printCommand("getDEFAULT_");
        comData.result = DEFAULT;
        break;

      case getEXTERNAL_:
        printCommand("getEXTERNAL_");
#if !defined EXTERNAL
        comData.result = DEFAULT;
#else
        comData.result = EXTERNAL;
#endif
        break;

      case getINTERNAL_:
        printCommand("getINTERNAL_");
#if !defined INTERNAL
        comData.result = DEFAULT;
#else
        comData.result = INTERNAL;
#endif
        break;

      case getINTERNAL1V1_:
        printCommand("getINTERNAL1V1_");
#if !defined INTERNAL1V1
        comData.result = DEFAULT;
#else
        comData.result = INTERNAL1V1;
#endif
        break;

      case getINTERNAL2V56_:
        printCommand("getINTERNAL2V56_");
#if !defined INTERNAL2V56
        comData.result = DEFAULT;
#else
        comData.result = INTERNAL2V56;
#endif
        break;

      case getINTERNAL2V56_EXTCAP_:
        printCommand("getINTERNAL2V56_EXTCAP_");
#if !defined getINTERNAL2V56_EXTCAP
        comData.result = DEFAULT;
#else
        comData.result = getINTERNAL2V56_EXTCAP;
#endif
        break;

      // PIN MAPPING
      case getDigitalMapWord0_:
        printCommand("getDigitalMapWord0_");
        comData.result = uint16_t(PP_DIGITALMAP);
        break;

      case getDigitalMapWord1_:
        printCommand("getDigitalMapWord1_");
        comData.result = uint16_t(PP_DIGITALMAP >> 16);
        break;

      case getDigitalMapWord2_:
        printCommand("getDigitalMapWord2_");
        comData.result = uint16_t(PP_DIGITALMAP >> 32);
        break;

      case getDigitalMapWord3_:
        printCommand("getDigitalMapWord3_");
        comData.result = uint16_t(PP_DIGITALMAP >> 48);
        break;

      case getAnalogMapWord_:
        printCommand("getAnalogMapWord_");
        comData.result = PP_ANALOGMAP;
        break;

      case getPortMapWord_:
        printCommand("getportMapWord_");
        comData.result = PP_PORTMAP;
        break;

      // Digital Functions
      case pinMode_:
        printCommand("pinMode_");
        pinMode(comData.pin, comData.param1);
        break;

      case digitalWrite_:
        printCommand("digitalWrite_");
        digitalWrite(comData.pin, comData.param1);
        break;

      case digitalWriteRead_:
        printCommand("digitalWriteRead_");
        digitalWrite(comData.pin, comData.param1);
        comData.result = digitalRead(comData.pin);
        break;

      case digitalRead_:
        printCommand("digitalRead_");
        comData.result = digitalRead(comData.pin);
        break;

      // Analog Functions
      case analogReference_:
        printCommand("analogReference_");
        analogReference(comData.pin);
        break;

      case analogRead_:
        printCommand("analogRead_");
        comData.result = analogRead(comData.pin);
        break;
      case analogWrite_:
        printCommand("analogWrite_");
        analogWrite(comData.pin, comData.param1);
        break;

      // TONE FUNCTIONS //
      case tone1_:
        printCommand("tone1_");
        tone(comData.pin, comData.param1);
        break;

      case tone2_:
        printCommand("tone2_");
        tone(comData.pin, comData.param1, comData.param2);
        break;

      case noTone_:
        printCommand("noTone_");
        noTone(comData.pin);
        break;

        // PORT FUNCTIONS // Only available on Classic AVR devices e.g. ATmega328, ATmega1284, ATmega2560 etc.
#if PP_IS_CLASSIC_AVR
      // We use the Arduino core macros to map the port registers
      case regWrite_:
        printCommand("regWrite_");
        switch (regType) {
          case DDRx:
            (*(volatile uint8_t *)(portModeRegister(reg))) = comData.param1; // DDRx
            break;
          case PORTx:
            (*(volatile uint8_t *)(portOutputRegister(reg))) = comData.param1;  // PORTx
            break;
        }
        break;

      case regRead_:
        printCommand("regRead_");
        switch (regType) {
          case DDRx:
            comData.result = (*(volatile uint8_t *)(portModeRegister(reg)));
            break;
          case PORTx:
            comData.result = (*(volatile uint8_t *)(portOutputRegister(reg)));
            break;
          case PINx:
            comData.result = (*(volatile uint8_t *)(portInputRegister(reg))); // PINx
            break;
        }
        break;

      case regBitWrite_:
        printCommand("regBitWrite_");
        switch (regType) {
          case DDRx:
            if (comData.param2) (*(volatile uint8_t *)(portModeRegister(reg))) |= (1 << comData.param1);
            else (*(volatile uint8_t *)(portModeRegister(reg))) &= ~(1 << comData.param1);
            break;
          case PORTx:
            if (comData.param2) (*(volatile uint8_t *)(portOutputRegister(reg))) |= (1 << comData.param1);
            else (*(volatile uint8_t *)(portOutputRegister(reg))) &= ~(1 << comData.param1);
            break;
        }
        break;
#endif

        // SERVO FUNCTIONS //

#if PP_SERVO_ENABLE

      // Servos, some of these functions return an error condition
      // to the Host from the PinsPlus error definitions so that the rpi
      // can use perror to give a more meaningful error message

      case svoAttach1_:
        printCommand("servoAttach1_");
        // see if the pin is already in use and exit if it is
        index = servoTableIndex(comData.pin);
        if (index >= 0) forceGuestError(EPININUSE);
        break;

        // look for a free entry in the servoTable
        index = servoTableIndex(-1);
        if (index >= 0) {
          // place the pin number in the empty servoTable entry
          servoTable[index] = comData.pin;
          // attach the servo to the corresponding servo instance
          servo[index].attach(comData.pin);
        } else {
          forceGuestError(EPINNOTAVAILABLE);
        }
        break;

      case svoAttach2_:
        printCommand("servoAttach2_");
        // see if the pin is already in use and exit if it is
        index = servoTableIndex(comData.pin);
        if (index >= 0) forceGuestError(EPININUSE);
        break;

        // look for a free entry in the servoTable
        index = servoTableIndex(-1);
        if (index >= 0) {
          // place the pin number in the empty servoTable entry
          servoTable[index] = comData.pin;
          // attach the servo to the corresponding servo instance
          servo[index].attach(comData.pin, comData.param1, comData.param2);
        } else {
          forceGuestError(EPINNOTAVAILABLE);
        }
        break;

      case svoWrite_:
        printCommand("servoWrite_");
        // look for the pin number entry in the servoTable
        index = servoTableIndex(comData.pin);
        if (index >= 0) servo[index].write(comData.param1);
        break;

      case svoWriteMicroseconds_:
        printCommand("servoWriteMicroseconds_");
        // look for the pin number entry in the servoTable
        index = servoTableIndex(comData.pin);
        if (index >= 0) servo[index].writeMicroseconds(comData.param1);
        break;

      case svoRead_:
        printCommand("servoRead_");
        // look for the pin number entry in the servoTable
        index = servoTableIndex(comData.pin);
        if (index >= 0) comData.result = servo[index].read();
        else {
          forceGuestError(EPINNOTAVAILABLE);
        }
        break;

      case svoAttached_:
        printCommand("servoAttached_");
        // look for the pin number entry in the servoTable
        index = servoTableIndex(comData.pin);
        if (index >= 0) comData.result = servo[index].attached();
        else {
          forceGuestError(EPINNOTAVAILABLE);
        }
        break;

      case svoDetach_:
        printCommand("servoDetach_");
        // look for the pin number entry in the servoTable
        index = servoTableIndex(comData.pin);
        if (index >= 0) {
          servo[index].detach();
          servoTable[index] = -1;
          comData.result = true;
        } else {
          forceGuestError(EPINNOTAVAILABLE);
        }
        break;

#endif

      // EEPROM

      case epmRead_:
        printCommand("eepromRead_");
        comData.result = EEPROM.read(comData.param1);
        break;

      case epmWrite_:
        printCommand("eepromWrite_");
        EEPROM.write(comData.param1, comData.pin);
        break;

      case epmUpdate_:
        printCommand("eepromUpdate_");
        EEPROM.update(comData.param1, comData.pin);
        break;

      default:
        break;
    }//END OF switch(comData.command)

    rxBytes = 0;
    comData.command = 0;
    comData.pin = 0;
    comData.param1 = 0;
    comData.param2 = 0;

#if DEBUG
    printf("GuestStatus: %u  Result: 0x%04X\n", comData.guestStatus, comData.result);
#endif

    // trigger the Host interrupt
    triggerInterrupt();
  }//END OF if(rxBytes)
}//END OF processRxData(uint8_t rxBytes)

// the Receive ISR
void receiveISR(uint8_t howMany) {
  // we fill the comData variables anyway from the Wire buffer regadless
  // of how many Bytes are actually sent. This should fill unused comData
  // variables with ZERO although it doesn't matter
  rxBytes = howMany;
  // command and pin will always be Bytes so we can just read their values
  comData.command = Wire.read();
  comData.pin = Wire.read();
  // param1 and param2 can be variable sizes so we read them differently
  // param1
  volatile uint8_t *_rxBytes = reinterpret_cast<volatile uint8_t*>(&comData.param1);
  uint8_t sze = sizeof(comData.param1);
  while (sze--) *_rxBytes++ = Wire.read();
  // param2
  *_rxBytes = reinterpret_cast<volatile uint8_t*>(&comData.param2);
  sze = sizeof(comData.param2);
  while (sze--) *_rxBytes++ = Wire.read();
}

// the Request ISR
void requestISR() {
  // return comData.guestStatus and comData.result as Bytes
  Wire.write(comData.guestStatus);
  // send the result as Bytes
  volatile uint8_t *_txBytes = reinterpret_cast<volatile uint8_t*>(&comData.result);
  for (uint8_t x = 0; x < sizeof(comData.result); x++) Wire.write(*_txBytes++);
  // clear the variables
  comData.guestStatus = true;
  comData.result = 0;
}

// Forces a Guest error which is sent to the Host if requested
int forceGuestError(uint8_t error) {
  comData.guestStatus = false;
  comData.result = error;
  return -1;
}

// triggers an interrupt on the Host device LOW to HIGH transition
void triggerInterrupt() {
  // trigger an interrupt
  //comData.guestReady = true;
  digitalWrite(PP_INT_TRIGGER_PIN, LOW);
  delay(TRIGGER_INT_DELAY);
  digitalWrite(PP_INT_TRIGGER_PIN, HIGH);
}

#if PP_SERVO_ENABLE
// the servo table search function. Searches the servoTable for the given pin
int8_t servoTableIndex(int8_t pin) {
  uint8_t index = 0;
  while (index < PP_NUM_SERVOS) {
    if (servoTable[index] == pin) return index;
    ++index;
  }
  return -1;
}
#endif
