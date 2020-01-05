#include <FastLED.h>
#include <Wire.h>

#define NUM_LEDS 300 
#define DATA_PIN 17
CRGB leds[NUM_LEDS];

byte mode = 0;

void setup() { 
  delay(100);
  Serial.begin(57600);
  Serial.println("resetting");
  LEDS.addLeds<WS2812B,DATA_PIN,RGB>(leds,NUM_LEDS);
  LEDS.setBrightness(84);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(9);                // join i2c bus with address #9
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.println("ready.");
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }

void loop() { 
  static uint8_t hue = 0;
  //Serial.print("x");
  // First slide the led in one direction
  for(int i = 0; i < NUM_LEDS; i++) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show(); 
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
  //Serial.print("x");

  // Now go in the other direction.  
  for(int i = (NUM_LEDS)-1; i >= 0; i--) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show();
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
}
void receiveEvent(int howMany)
{
  digitalWrite(LED_BUILTIN, HIGH);       // briefly flash the LED
  byte bytes[32];
  for(int i = 0; i < 32; i++)
  {
    byte b = 0;
    if(Wire.available()>0)
    {
      b = Wire.read();
    }
    bytes[i] = b;
    Serial.print(b);
    Serial.print(":");
  }
  Serial.println();
  digitalWrite(LED_BUILTIN, LOW);
}
void requestEvent()
{
  digitalWrite(LED_BUILTIN, HIGH);  // briefly flash the LED
  Wire.write(mode);
  digitalWrite(LED_BUILTIN, LOW);
}
