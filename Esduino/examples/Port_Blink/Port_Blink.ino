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

  // set the Guest LED pin as an OUTPUT, DDRB bit PB5 as OUTPUT
  pp.regBitWrite(DDRB,5,OUTPUT);

}

void loop() {
  // Guest LED pin HIGH, PORTB bit PB5 HIGH
  pp.regBitWrite(PORTB,5,HIGH);
  delay(1000);
  // Guest LED pin LOW, PORTB bit PB5 LOW
  pp.regBitWrite(PORTB,5,LOW);
  delay(1000);
}
