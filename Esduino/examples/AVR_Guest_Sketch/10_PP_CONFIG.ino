//*****************************//
// PinsPlus CONFIGURATION DATA //
//*****************************//

// create a structure for the common data between the Host and Guest
volatile comdata_t comData;

/*
   The bitmaps for the pins, analog inputs and port registers are configured in the following definitions
   A '1' indicates that the pin, input or port is available to this AVR type and is used to check
   the validity of the values provided in the functions. The Guest device reports its MCU type e.g. ATmega328
   The example here is for an Arduino UNO. Use this template to create/alter the binary map for each type.

                                      INCLUDED DIGITAL I/O PINS
                    |          111111111122222222223333333333444444444455555555556666|
                    |0123456789012345678901234567890123456789012345678901234567890123|
                     ----------------------------------------------------------------
   PP_DIGITALMAP = 0b0001111111111111110000000000000000000000000000000000000000000000LL

                  INCLUDED ANALOG CHANNELS
                     |          111111|
                     |0123456789012345|
                      ----------------
     PP_ANALOGMAP = 0b1111001100000000L

                     INCLUDED I/O PORTS
                     |ABCDEFGHIJKLMNOP|
                      ----------------
       PP_PORTMAP = 0b0111000000000000L
*/

// identify this processor type and set values
// UNO, NANO etc.
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
// INTERNAL 1.1V or EXTERNAL AREF
#define PP_IS_CLASSIC_AVR true  // uses DDR, PORT & PIN registers
#define PP_MCU_TYPE 0xA1       // the MCU type for internal use
#define PP_DIG_PINS 20          // total number of digital pins available
#define PP_ANA_PINS 8           // total number opf analog pins available
#define PP_EEPROM_SIZE  1024    // the EEPROM size in Bytes
#define PP_INT_TRIGGER_PIN 2   // the local pin used to trigger interrupt on the Host
#define PP_NUM_SERVOS 12        // fixed by the Servo library
// exclude Pins 0,1,2,18 and 19
#define PP_DIGITALMAP 0b0001111111111111110000000000000000000000000000000000000000000000LL
// exclude Analog channels 4 and 5
#define PP_ANALOGMAP  0b1111001100000000L
// enable Ports B,C and D
#define PP_PORTMAP    0b0111000000000000L
#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS

// LEONARDO
#elif defined (__AVR_ATmega32U4__)
// INTERNAL 2.56V or EXTERNAL AREF pin
#define PP_IS_CLASSIC_AVR true
#define PP_MCU_TYPE 0xA2
#define PP_DIG_PINS 20
#define PP_ANA_PINS 8
#define PP_EEPROM_SIZE  2560
#define PP_INT_TRIGGER_PIN 4
#define PP_NUM_SERVOS 12
// exclude Pins 0,1,2,3 and 4
#define PP_DIGITALMAP 0b0000011111111111111100000000000000000000000000000000000000000000LL
// exclude no Analog channels
#define PP_ANALOGMAP  0b1111111100000000L
// enable Ports B,C and D
#define PP_PORTMAP    0b0111000000000000L
#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS

/* TODO
// NANO EVERY, ONLY IN ATMEGA328 EMULATION MODE
// VREF DEFAULT, INTERNAL, VDD, EXTERNAL, INTERNAL0V55, INTERNAL1V1
// VREF INTERNAL2V5, INTERNAL4V3, INTERNAL1V5
#elif defined (__AVR_ATmega4809__)
#define PP_IS_CLASSIC_AVR false // does not use DDR, PORT & PIN registers
#define PP_MCU_TYPE 0xA3
#define PP_DIG_PINS 20
#define PP_ANA_PINS 8
#define PP_EEPROM_SIZE  256
#define PP_INT_TRIGGER_PIN 2
#define PP_NUM_SERVOS 12
// exclude Pins 0,1,2,18 and 19
#define PP_DIGITALMAP 0b0001111111111111110000000000000000000000000000000000000000000000LL
// exclude Analog channels 4 and 5
#define PP_ANALOGMAP  0b1111001100000000L
// the ATmega4809 does not use the DDR,PORT and PIN registers
#define PP_PORTMAP    0L
#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS
*/

// MEGA2560
#elif defined (__AVR_ATmega2560__)
// INTERNAL 1.1V and INTERNAL 2.56V or EXTERNAL AREF
#define PP_IS_CLASSIC_AVR true
#define PP_MCU_TYPE 0xA4
#define PP_DIG_PINS 54
#define PP_ANA_PINS 16
#define PP_EEPROM_SIZE  4096
#define PP_INT_TRIGGER_PIN 2   // the local pin used to trigger interrupt on the Host
#define PP_NUM_SERVOS 48    // fixed by the Servo library
// exclude Pins 0,1,2,20 and 21
#define PP_DIGITALMAP 0b0001111111111111111100111111111111111111111111111111110000000000LL
// exclude no Analog channels
#define PP_ANALOGMAP  0b1111111111111111L
// enable Ports A,B,C,D,E,F,G,H and K - Port I is not used
#define PP_PORTMAP    0b1111111101100000L
#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS

// UNDEFINED DEVICE
#else
#define PP_IS_CLASSIC_AVR false
#define PP_MCU_TYPE 0x00
#define PP_DIG_PINS 0
#define PP_ANA_PINS 0
#define PP_EEPROM_SIZE  0
#define PP_MCU_SUBTYPE 0
#define PP_INT_TRIGGER_PIN 2
#define PP_NUM_SERVOS 0
#define PP_DIGITALMAP 0LL
#define PP_ANALOGMAP 0L
#define PP_PORTMAP 0L
#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS
#endif

// internal use
#define DDRx  0
#define PORTx 1
#define PINx  2

#if PP_SERVO_ENABLE
#include <Servo.h>
Servo servo[PP_NUM_SERVOS];  // Servo instances
int8_t servoTable[PP_NUM_SERVOS] = { -1}; // a table for tracking Servo/pin attachments
#endif

// This resets the AVR and bypasses the
// bootloader by restarting at address 0x0000
void(*resetFunc) (void) = 0;

// print out the current command in DEBUG mode
void printCommand(const char * msg) {
#if DEBUG
  printf("\"%s\" ", msg);
#endif
}

volatile uint8_t rxBytes = 0; // the number of Bytes received from the Host device
