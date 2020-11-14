#include <Wire.h>
#include <EEPROM.h>
#include <Esduino.h>

// define the I2C address of this Guest

#define PP_IIC_ADDRESS 0x09

// print out incomming commands and their results
// DEBUG will use a lot of Flash (34%) and RAM (52%) so be aware!
// without DEBUG the use is: Flash (21%) and RAM (15%)

#define DEBUG  false

#if DEBUG
#include "printF.h"
#endif

// set to false if full PWM is required and servos are not required

#define PP_SERVO_ENABLE true    

//*********************************//
// USER CONFIGURATION FOLLOWS HERE //
//*********************************//
