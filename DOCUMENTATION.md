## Esduino Pin Expander for ESP8266...

'Esduino' is a system to add the capabilities of an Arduino board such as the UNO, NANO, LEONARDO or MEGA2560 to an ESP8266 device. Esduino differs from some other approaches to this problem as it allows the use of the Arduino/Wiring commands locally in the user's code. The commands are sent to the Arduino board, attached via the I2C bus, which executes the command and then responds with the result if there is one.

### Commands supported by Esduino:

**Digital I/O**

* pinMode   
* digitalRead   
* digitalWrite   
* digitalWriteRead (Returns the read state of the written pin)   

**Analog I/O**

* analogReference   
* analogRead   
* analogWrite (PWM)

**Tone**

* Tone   
* noTone   

**Direct Port Access**

* regWrite   
* regRead   
* regBitWrite   
* regBitRead   

**Servos**

* svoAttach   
* svoWrite   
* svoWriteMicroseconds   
* svoRead   
* svoAttached   
* svoDetach   

**EEPROM**

* epmRead   
* epmWrite
* epmUpdate   
* epmPut   
* epmGet

### How it works:

The system comprises a Host ESP8266 device and a Guest Arduino board or stand-alone AVR device. Commands are sent to the Guest by the Host and responses, if required, are requested by the Host. The I2C interface is used for communication with the addition of a signalling line which tells the Host that a command has been performed.

**The pull-up resistors set by the Arduino Wire library are turned off at the Guest end of the connection so, external pull-up resistors connected to the ESP8266 3.3V supply must be provided on the SCL and SDA lines on the ESP8266.**

A set of 'internal' commands is shared by the Host and Guest. The 'Esduino' class translates the Wiring type commands and sends them to the Guest which, in turn, translates the received commands back into the corresponding Wiring commands. The shared commands and other configuration data is contained in the 'EsduinoConfig.h' file.

A common data structure is used on both the Host and Guest as follows:

typedef struct comdata\_t \{   
&nbsp;&nbsp;uint8\_t command = 0;  // command is always 1 Byte   
&nbsp;&nbsp;uint8\_t pin = 0;      // pin is always 1 Byte  
&nbsp;&nbsp;uint16\_t param1 = 0;  // the size of param1 can be altered e.g. uint32_t   
&nbsp;&nbsp;uint16\_t param2 = 0;  // the size of param2 can be altered e.g. uint32_t   
&nbsp;&nbsp;uint16\_t result = 0;  // the size of the result can be altered e.g. uint32_t   
&nbsp;&nbsp;uint8\_t guestStatus = 1;  // guestStatus is always 1 Byte   
&nbsp;&nbsp;uint8\_t checksum = 0;  // TODO   
\};   

The Host sends a variable length command message dependent upon the data required. 'command' is always sent and if necessary, 'pin', 'param1' and 'param2'. The default 'minimum' message length is 1 Byte and the 'maximum' default message length is 6 Bytes.

If it is necessary for the Guest to respond with data or an error message, a 3 Byte response is returned (requested by the Host) comprising the 'guestStatus' Byte and the 'result' word. This system keeps I2C data payload to a minimum.

The Guest compiles automatically dependent upon the AVR device being used. Upon startup, the Host requests the configuration data from the Guest as follows:

The configuration for an ATmega328

\#define PP\_IS\_CLASSIC\_AVR true  // uses DDR, PORT & PIN registers   
\#define PP\_MCU\_TYPE 0xA1        // the MCU type for internal use   
\#define PP\_DIG\_PINS 20          // total number of digital pins available   
\#define PP\_ANA\_PINS 8           // total number opf analog pins available   
\#define PP\_EEPROM\_SIZE  1024    // the EEPROM size in Bytes   
\#define PP\_INT\_TRIGGER\_PIN 2    // the local pin used to trigger interrupt on the Host   
\#define PP\_NUM\_SERVOS 12        // fixed by the Servo library   
// exclude Pins 0,1,2,18 and 19   
\#define PP_DIGITALMAP 0b0001111111111111110000000000000000000000000000000000000000000000LL   
// exclude Analog channels 4 and 5   
\#define PP_ANALOGMAP  0b1111001100000000L   
// enable Ports B,C and D   
\#define PP_PORTMAP    0b0111000000000000L   
\#define TRIGGER_INT_DELAY 0 // delay the Interrupt Trigger pin going LOW to HIGH in mS   

The data retrieved by the Host is stored in a structure which is user accessible:

struct \{   
&nbsp;&nbsp;uint8\_t mcuType;  
&nbsp;&nbsp;uint8\_t mcuFreq;   
&nbsp;&nbsp;uint8\_t maxPins;   
&nbsp;&nbsp;uint8\_t analogChannels;   
&nbsp;&nbsp;uint8\_t servoPins;   
&nbsp;&nbsp;uint16\_t eepromSize;   
&nbsp;&nbsp;uint8\_t ledPin;   
&nbsp;&nbsp;uint64\_t digitalMap;   
&nbsp;&nbsp;uint16\_t analogMap;   
&nbsp;&nbsp;uint16\_t portMap;   
\} info;

The configuration for each AVR type includes a 64bit pin map for digital pins that allows certain digital pins to be excluded. For instance, the Arduino UNO with the ATmega328P AVR provides only 20 digital pins numbered 0 - 19. Therefore, pins greater than 19 are not required and their corresponding bits in the map are set to zero.

**The maps are written left to right with the left-most bit being 0 (zero). 1 = enabled, 0 = disabled**

&nbsp;&nbsp;`|----------111111111122222222223333333333444444444455555555556666|`   
&nbsp;&nbsp;`|0123456789012345678901234567890123456789012345678901234567890123|`   

`0b0001111111111111110000000000000000000000000000000000000000000000LL`   


In the ATmega328 Digital pin map shown above, pins 0,1,2,18 and 19 are also excluded when the map is read left to right, the left-most bit being pin 0. These excluded pins are Serial RX, Serial TX, D2 used for the Signal line, SDA and SCL.


&nbsp;&nbsp;`|----------111111|`   
&nbsp;&nbsp;`|0123456789012345|`   
&nbsp;&nbsp;&nbsp;&nbsp;`----------------`   
`0b1111001100000000L`   

The 16bit Analog map defines which analog channels can be used. In the example above, analog channels 4 and 5 are excluded as these are used by SDA and SCL. Analog channels 8 - 15 are excluded because they are not available on the ATnega328.

&nbsp;&nbsp;`|ABCDEFGHIJKLMNOP|`   
&nbsp;&nbsp;&nbsp;&nbsp;`----------------`   
`0b0111000000000000L`   

The 16bit Port map defines which Port registers are available in the AVR. Ports range from 'A' to 'P' so, in the example above, only Ports B,C and D are enabled. Other definitions are self evident.

### Commands:

All the commands, with the exception of SERVO, PORT and EEPROM commands, are identical to the Arduino Wiring commands and therefore, the Arduino reference should be used for these commands.

### Servos:

The original plan for Esduino was to have separate classes for Servos and EEPROM but difficulties implementing a single 'shared' interrupt handler resulted in a single class for all methods. Therefore, certain Arduino/Wiring commands have slightly different names. This may change in future versions!

| Wiring | Esduino |
| :----- | :------- |
| attach() | svoAttach() |
| write() | svoWrite{} |
| writeMicroseconds() | svoWriteMicroseconds() |
| read() | svoRead() |
| attached() | svoAttached() |
| detach() | svoDetach() |

The Arduino Servo library ties a servo to an object created as required in the sketch. Esduino ties a servo to the pin number that the servo is connected to on the Guest and does not require multiple objects to be created.

Therefore, the commands in an Arduino sketch:

`#include <Servo.h>`

Servo myservo1;  // the Servo objects   
Servo myservo2;  
Servo myservo3;  

void setup()   
{   
&nbsp;&nbsp;&nbsp;&nbsp;myservo1.attach(9);   
&nbsp;&nbsp;&nbsp;&nbsp;myservo2.attach(10);   
&nbsp;&nbsp;&nbsp;&nbsp;myservo3.attach(11);   
}   

would be like this using Esduino:

`#include "Esduino.h"`

Esduino guest;  // an object for Esduino

void setup()   
{   
&nbsp;&nbsp;&nbsp;&nbsp;guest.begin(9,2)	// start Esduino on I2C 9, Interrupt pin 2   

&nbsp;&nbsp;&nbsp;&nbsp;guest.servoAttach(9);   
&nbsp;&nbsp;&nbsp;&nbsp;guest.servoAttach(10);   
&nbsp;&nbsp;&nbsp;&nbsp;guest.servoAttach(11);   
}

All subsequent Servo commands refer to the pin number, not the servo instance.

### EEPROM:

EEPROM commands also use a slightly different syntax:

| Wiring | Esduino |
| :----- | :------- |
| write() | epmWrite() |
| read() | epmRead() |
| update() | epmUpdate() |
| put() | epmPut() |
| get() | epmGet() |

epmWite(), epmRead() and epmUpdate() write/read single unsigned Bytes. epmPut writes multiple Bytes which can be from a variable or an array. epmGet() retrieves multiple Bytes which are put into a target variable or array.

Example:

`uint16_t eepromAddress = 0;`  
`float myFloat1 = 12345.12345;`  
`float myFloat2 = 0.0;`  

// write myFloat1 to the EEPROM at address 0  
&nbsp;&nbsp;`epmPut(eepromAddress, &myFloat1, sizeof(myFloat1));`   
// read myFloat2 from the EEPROM at address 0  
&nbsp;&nbsp;`epmGet(eepromAddress, &myFloat2, sizeof(myFloat2));`  

**Note the use of the '&' (ampersand) before the variable name! as we are passing the address of the variable.**

### Direct PORT access:

Because Esduino is controlling a remote AVR device the local macros provided by Arduino for direct Port access cannot be used on the Host. A simple command set is used to read/write the Port registers:

**Port read/write for Guest Port registers only:**

| Arduino | Esduino |
| :------ | :------- |
| DDRB = 0xFF | regWrite(DDRB,0xFF) |
| PORTB = 0xFF | regWrite(PORTB,0xFF) |
| uint8_t reg = PINB | uint8_t reg = regRead(PINB) |

**Bit set/clear for Guest Port registers only:**

| Arduino Statement | Esduino Statement |
| :---------------- | :----------------- |
| bitWrite(DDRB,5,1) or DDRB \|= (1 << 5) | regBitWrite(DDRB,5,1) |   
| bitWrite(DDRB,5,0) or DDRB &= ~(1 << 5) | regBitWrite(DDRB,5,0) |

### Installation and use:

#### `The Host and Guest use a common header file named 'EsduinoConfig.h'.`

If you are working with both ESP8266 and AVR devices, I really recommend using seperate installations of the Arduino IDE, one for AVR and other Arduino boards and one for ESP8266 only. If you prefer to use a single instance of the Arduino IDE with both AVR and ESP8266 cores installed, the 'Esduino' library takes care of the coorrect compilation for the device.

The ESP8266 Host sketches are compiled using the Arduino IDE (1.8.13) with the ESP8266 core package installed (https://github.com/esp8266/Arduino)

The AVR Guest sketch is also compiled using the Arduino IDE and the AVR device is loaded with a specific version of the Guest code for the device MCU.

#### `The 'Esduino' class is contained in the 'Esduino' library which will work with both the ESP8266 and the AVR Guest`

**Guest Arduino board or AVR device**

#### `The 'Esduino' library is installed in the 'libraries' folder of your 'Sketchbook' folder in the normal way.`

The Guest Arduino board makes use of a standard Arduino sketch which automatically compiles for the correct AVR type. Once the Guest sketch is uploaded to the chosen Arduino board, you're ready to go.

The Guest Arduino board uses SCL, SDA and an additional signalling pin preset to D2. DO NOT FORGET THE COMMON GROUND CONNECTION!

**ESP8266 Host**

The ESP8266 board/device chosen must have the full pin set which includes SCL, SDA and a pin for use as an interrupt input. Remember that the GPIO 16 pin on the ESP8266 cannot be used as an interrupt. The default connections are as follows:

| Host ESP8266 | Guest AVR | Notes |
| :----------- | :-------- | :---- |
| SDA (GPIO 4) | SDA | 2K2 resistor pull-up to ESP8266 3.3V |
| SCL (GPIO 5) | SCL | 2K2 resistor pull-up to ESP8266 3.3V |
| INT (GPIO 2) | D2 | ** DIODE REQUIRED -->\|-- |
| GND | GND |

** **A diode such as a 1N4148 must be used in the connection between the ESP8266 and the Arduino board in the INT line.**

**ANODE towards the ESP8266, CATHODE towards the Arduino board. This is necessary to prevent damage because the ESP8266 is a 3.3V device and the AVR is a 5V device. If the Guest Arduino board or AVR device is running on 3.3V, the diode is not required.**

INT is an Interrupt input pin, Arduino pin D2 is an output pin used to trigger the INT input upon completion of a command on the Guest AVR. Both INT and the Guest AVR output pin are user selectable.