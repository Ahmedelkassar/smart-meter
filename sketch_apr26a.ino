#include <ESP8266WiFi.h>

#include <ThingSpeak.h>

 float p= 0;
float sum= 15.3; 

WiFiClient client;

long myChannelNumber = 1415987;
const char myWriteAPIKey[] = "GTJ7H6WS0Z17WIL0";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.begin("kassar12", "ahmed1812930");
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    Serial.print("..");
  }
  Serial.println();
  Serial.println("NodeMCU is connected!");
  Serial.println(WiFi.localIP());

  ThingSpeak.begin(client);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  ThingSpeak.writeField(myChannelNumber, 1, p, myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 2, sum, myWriteAPIKey);
  delay(2000);

}
