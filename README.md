# Esduino

### A project to allow an ESP8266 module to control an Arduino board to add additional pins/ADC/PWM etc.

Esduino, a simple way to add an ATmega328, ATmega32u4 or ATmega2560 AVR or Arduino board to an ESP8266 to provide additional IO and other functions using the I2C bus plus one additional signal line. Works with Arduino NANO, UNO, LEONARDO, MEGA2560 etc. or any board with an AVR device of the above type.

The project is written in C/C++ using Arduino IDE 1.8.13. The Arduino ESP8266 sketches use a standard C++ shared library and a common configuration file 'EsduinoConfig.h' in the 'utils' folder of the library. This config file is also used for the Guest AVR/Arduino board sketch.

The interface between the ESP8266 Host and the Guest AVR/Arduino board requires just four lines, SCL, SDA, Signal and Ground. The Signal line is an interrupt signal used by the Guest AVR to signal that it has finished processing the last command. Typical processing times for commands are circa 1-3 milliseconds. The only components required to connect the ESP8266, which is a 3.3V device, with the Arduino board, which is a 5V device, are 2 pull-up resistors (ESP8266 only) for the I2C signals and a diode, such as a 1N4148 for the Signal line.

The Guest Arduino board or AVR device is loaded with an Arduino sketch which uses only 311 Bytes of SRAM and 6644 Bytes of Flash. User functions for reading sensors etc. can be easily added to the sketch.

The Host device, ESP8266, makes use of a class contained in a library. Both the Host and Guest also use a common configuration file.

#### The code and other files provided here are in their 'raw' state. A lot of polishing is my next aim but, for now, the code works and is presented for demonstration and test purposes. The author accepts no responsibility for any harm, panic, wars or other damage resulting from the use of them!

### Additional functionality for the ESP8266:

* Full access and control of the Digital IO Pins
* Full access to the Analog Pins
* Full control of the AREF source/input
* PWM on Arduino defined Pins
* Tone functions
* Direct Port Access using the DDRx, PORTx and PINx registers (on most boards)
* Servo control of up to 48 servos (depends on the AVR device being used)
* EEPROM read, write, update, put and get

#### The ESP8266 interface is as follows:

| ESP8266 | Guest AVR | Notes |
| :------ | :-------- | :---- |
| SDA (GPIO 4) | SDA | 
| SCL (GPIO 5) | SCL |
| INT (GPIO 2) | D2 | Both user selectable |
| GND | GND |


INT is an Interrupt input pin, Arduino pin D2 is an output pin used to trigger the INT input upon completion of a command on the Guest AVR. Both INT and the Guest AVR output pin are user selectable.

The commands provided by Esduino are the same as those available in the Arduino/Wiring environment with the exception of the Servo commands.

The Arduino Servo library ties a servo to an object created as required in the sketch. Esduino ties a servo to a pin and does not require multiple objects to be created.

Therefore, the command:

`include <Servo.h>`

Servo myservo1;   
Servo myservo2;   
Servo myservo3;   

void setup()   
{   
&nbsp;&nbsp;&nbsp;&nbsp;myservo1.attach(9);   
&nbsp;&nbsp;&nbsp;&nbsp;myservo2.attach(10);   
&nbsp;&nbsp;&nbsp;&nbsp;myservo3.attach(11);   
}   

would be like this under Esduino:

`include "Esduino.h"`

Esduino pp;

void setup()   
{   
&nbsp;&nbsp;&nbsp;&nbsp;pp.servoAttach(9);   
&nbsp;&nbsp;&nbsp;&nbsp;pp.servoAttach(10);   
&nbsp;&nbsp;&nbsp;&nbsp;pp.servoAttach(11);   
}

All subsequent Servo commands refer to the pin number, not the servo instance.
 