#ifndef ESDUINO_H
#define ESDUINO_H

#if defined (ESP8266)
#include <Arduino.h>

#define PP_DEBUG  false	// true for DEBUG output, false for NONE

// define the DDRs for the Host to info control
enum {DDRA = 0x00, DDRB, DDRC, DDRD, DDRE, DDRF, DDRG, DDRH, DDRI, DDRJ, DDRK, DDRL, DDRM, DDRN, DDRO, DDRP};
// define the PORTS for the Host to info control
enum {PORTA = 0x10, PORTB, PORTC, PORTD, PORTE, PORTF, PORTG, PORTH, PORTI, PORTJ, PORTK, PORTL, PORTM, PORTN, PORTO, PORTP};
// define the PIN registers for the Host to info control
enum {PINA = 0x20, PINB, PINC, PIND, PINE, PINF, PING, PINH, PINI, PINJ, PINK, PINL, PINM, PINN, PINO, PINP};

// Arduino definitions for Raspberry Pi
// HIGH, LOW, INPUT & OUTPUT are already defined by wiringPi
// but INPUT_PULLUP isn't
#if not defined INPUT_PULLUP
#define INPUT_PULLUP  2
#endif


// the Esduino class
class Esduino {
  public:

    int begin(int I2Caddress_, int intPin_);

    // DIGITAL //
    int pinMode(uint8_t pin, uint8_t mode);           // Arduino pinMode
    int digitalRead(uint8_t pin);                     // Arduino digitalRead
    int digitalWrite(uint8_t pin, uint8_t state);     // Arduino digitalWrite
    int digitalWriteRead(uint8_t pin, uint8_t state); // write to the pin and then reads it back

    // ANALOG //
    int analogReference(int mode);                    // Arduino analogReference
    int analogRead(uint8_t channel);                  // Arduino analogRead
    int analogWrite(uint8_t pin, uint8_t val);        // Arduino analogWrite (PWM on valid pins)

    // TONE //
    int tone(uint8_t pin, uint16_t freq);               // Arduino Tone
    int tone(uint8_t pin, uint16_t freq, uint16_t dur); // Arduino Tone, frequency and duration
    int noTone(uint8_t pin);                            // Arduino noTone

    // PORTS //
    int regWrite(uint8_t reg, uint8_t mask);
    int regRead(uint8_t reg);
    int regBitWrite(uint8_t reg, uint8_t bit, uint8_t state);
    int regBitRead(uint8_t reg, uint8_t bit);

    // SERVO //
    int svoAttach(uint8_t pin);
    int svoAttach(uint8_t pin, uint32_t min, uint32_t max);
    int svoWrite(uint8_t pin, uint8_t angle);
    int svoWriteMicroseconds(uint8_t pin, uint32_t microseconds);
    int svoRead(uint8_t pin);
    int svoAttached(uint8_t pin);
    int svoDetach(uint8_t pin);

    // EEPROM
    int epmRead(uint16_t address);
    int epmWrite(uint16_t address, uint8_t data);
    int epmUpdate(uint16_t address, uint8_t data);
    int epmPut(uint16_t address, void *data, uint8_t len);
    int epmGet(uint16_t address, void *data, uint8_t len);

    // MISC
    void printDetails(void);
    void perror(const char * msg = "");
    void reset(void);         // resets the info MCU to boot at address 0
	void printReverseBinary(uint64_t x, uint8_t len);
	
	// Guest pin and analog configuration
	struct {
	  uint8_t mcuType = 0;
	  uint8_t FCPU = 0;
	  uint8_t maxPins = 0;
	  uint8_t analogChannels = 0;
	  uint8_t servoPins = 0;
	  uint16_t epmSize = 0;
	  uint8_t ledPin = 0;
	  uint64_t digitalMap = 0LL;
	  uint16_t analogMap;
	  uint16_t portMap;
	} info;
	
	// analog AREF variables for the Guest
	uint8_t A_DEFAULT = 0;
	uint8_t A_INTERNAL = 0;
	uint8_t A_EXTERNAL = 0;
	uint8_t A_INTERNAL1V1 = 0;
	uint8_t A_INTERNAL2V56 = 0;
	uint8_t A_INTERNAL2V56_EXTCAP = 0;
	
	
  private:
    // INFO //
    int getMcuType(void);     // returns 0xA1, 0xA2 etc.
	int getRemoteFCPU(void); // returns the Guest CPU frequency in MHz
    int getLedPin(void);
    int getDigPins(void);
    int getAnaPins(void);
    int getEepromSize(void);
    int getServoPins(void);
    
	  
	// command transmission/reception
    int sendCommand(uint8_t numParams, uint16_t remProcDelay = 0);
	void sendParameter(const void* param, uint8_t numByates);
    int fetchResult(void);
    int forceGuestError(uint8_t error);
	void clearComData(void);
	int intPin;       // the pin used to detect interrupts
    int I2Caddress;
	
    // pin and analog channel protection checks
    bool checkPin(uint8_t pin);
    bool checkAnalogChannel(uint8_t channel);
    bool checkPort(uint8_t port);
    
    // CONFIG - ADC, PIN and PORT definitiona/masks/maps
    int getDEFAULT(void);
    int getINTERNAL(void);
    int getEXTERNAL(void);
    int getINTERNAL1V1(void);
    int getINTERNAL2V56(void);
    int getINTERNAL2V56_EXTCAP(void);

    int getDigitalMapWord0(void);
    int getDigitalMapWord1(void);
    int getDigitalMapWord2(void);
    int getDigitalMapWord3(void);
    int getAnalogMapWord(void);
    int getPortMapWord(void);

};

#else
// this allows the AVR Guest sketch to use the config file without complications
#include <EsduinoConfig.h>
#endif
#endif