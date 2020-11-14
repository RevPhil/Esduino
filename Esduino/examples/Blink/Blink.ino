/*
  The classic Blink sketch

  Modified for 'PinsPlus' use October 2020 by Rev Phil
*/
#include <ESP8266WiFi.h>
#include <Esduino.h>

#define USE_WIFI false  // startup with or without WiFi connection

PinsPlus pp;  // an object for the PinsPlus library

#define GUEST_I2C_ADDRESS 9
#define INTERRUPT_PIN 2

void setup()
{
  Serial.begin(115200); // default Serial configuration
  Serial.setDebugOutput(true);  // for printf
#if USE_WIFI
  // WiFi security details
  const char* ssid = "<SSID>";
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
  printf("\r\n\r\nPinsPlus ESP8266 Host...\r\n");
  // start PinsPlus, (I2C address, Interrupt pin)
  if (pp.begin(GUEST_I2C_ADDRESS, INTERRUPT_PIN) < 0) pp.perror("PinsPlus begin:");

  // OK, we've started up correctly, let's print out the details of the Guest MCU
  pp.printDetails();

  // set the Guest LED pin as an OUTPUT
  pp.pinMode(pp.info.ledPin, OUTPUT);

}

void loop() {
  // Guest LED pin HIGH
  pp.digitalWrite(pp.info.ledPin, HIGH);
  delay(1000);
  // Guest LED pin LOW
  pp.digitalWrite(pp.info.ledPin, LOW);
  delay(1000);
}
