/*
  The classic Blink sketch using Port manipulation

  (C) October 2020 by Rev Phil
*/
#include <ESP8266WiFi.h>
#include <Esduino.h>

#define USE_WIFI false  // startup with or without WiFi connection

Esduino pp;  // an object for the Esduino library


void setup()
{
  Serial.begin(115200); // default Serial configuration
  Serial.setDebugOutput(true);  // for printf


#if USE_WIFI
  // WiFi security details
  const char* ssid = "<username>";
  const char* password = "<password>";
  // begin WiFi
  WiFi.disconnect();  // disconnect WiFi
  WiFi.persistent(false); //do not save WiFi credentials to Flash
  WiFi.mode(WIFI_STA);  // set ESP8266 as a WorkSTAtion & Server
  WiFi.begin(ssid, password);
  // wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("*");  // print a '*' on the monitor
    delay(500); // wait 500 mS
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("  IP: ");
  Serial.println(WiFi.localIP());
#else
  // no WiFi
  WiFi.disconnect();  // disconnect WiFi
  WiFi.mode(WIFI_OFF);    // turn WiFi OFF
  WiFi.forceSleepBegin(); // put WiFi to sleep
  yield();  // hand control back
#endif
  delay(1000);
  printf("\r\n\r\nEsduino ESP8266 Host...\r\n");
  // start Esduino, (I2C address, Interrupt pin)
  if (pp.begin(9, 2) < 0) pp.perror("Esduino begin:");

  // OK, we've started up correctly, let's print out the details of the Guest MCU
  pp.printDetails();
}

void loop() {
  // Make a loop for the available number of Analog Channels
  // Excluded channels will not be read e.g. on ATmega328 channels 4 & 5 will fail
  // as they are asigned to I2C SCL & SDA
  for (uint8_t x = 0; x < pp.info.analogChannels; x++) {
    // do the Analog read on channel 'x'
    int result = pp.analogRead(x);
    // was the read successful ?, -1 = error
    if (result < 0) pp.perror();  // NO, so print out the error
    else {                        // YES, so print out the Analog value
      // print out the channel details
      Serial.print("Analog Channel: ");
      Serial.print(x);
      Serial.print("\t");
      Serial.println(result);  
    }
    // wait a bit
    delay(1000);
  }
  Serial.println();
}
