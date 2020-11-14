void setup() {
  // configure the signal pin
  //digitalWrite(PP_INT_TRIGGER_PIN, HIGH);
  pinMode(PP_INT_TRIGGER_PIN, OUTPUT);

#if DEBUG
  Serial.begin(1000000);
  printf("DEBUG MODE! AVR ID: 0x%02X\n", PP_MCU_TYPE);
#endif

  Wire.begin(PP_IIC_ADDRESS);
  // turn OFF the pullup resitors applied by the Wire library
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
  // setup the Wire interrupt handlers
  Wire.onReceive(receiveISR);
  Wire.onRequest(requestISR);

  //**********************//
  // USER SETUP GOES HERE //
  //**********************//

  // tell the Host we're ready
  triggerInterrupt();
}// END OF setup()
