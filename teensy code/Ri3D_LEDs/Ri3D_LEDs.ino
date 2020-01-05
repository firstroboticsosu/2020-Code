#include <FastLED.h>
#include <Wire.h>

#define NUM_LEDS 300 
#define DATA_PIN 17
CRGB leds[NUM_LEDS];

byte mode = 0;
boolean weRed = true;
byte toMode = 0;
boolean readyToTrans = false;
int transitionVal = 255;
long modeTime = 0;

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

void loop() 
{ 
  toMode = (millis()/50000)%2;
  if(readyToTrans && mode!=toMode)
  {
    mode = toMode;
    modeTime = millis();
  }
  if(mode == 0)
  {
    byte val = (byte)((sin((millis()-modeTime)/1000.0+PI*3/2)+1)/2*255);
    readyToTrans = val == 0;
    for(int i = 0; i < NUM_LEDS; i++) {
      // Set the i'th led to red 
      leds[i] = CHSV(0,0,val);
      // Show the leds
    }
  }
  else if(mode == 1)
  {
    if(toMode==1)
    {
      transitionVal = (millis()-modeTime)/1000.0*255;
      if(transitionVal>255)
      {
        transitionVal = 255;
      }
    }
    else
    {
      transitionVal -= 3;
      transitionVal = max(transitionVal, 0);
    }
    Serial.println(transitionVal);
    CHSV c = CHSV((millis()/100)%255, 255, transitionVal);
    for(int i = 0; i < NUM_LEDS; i++) {
      // Set the i'th led to red 
      leds[i] = c;
      // Show the leds
    }
    readyToTrans = transitionVal==0;
  }
  //Serial.println(toMode+":"+mode);
  FastLED.show(); 
  delay(10);
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
      if(i==0 && b==18)
      {
        b = Wire.read();
      }
    }
    bytes[i] = b;
    Serial.print(b);
    Serial.print(":");
  }
  if(bytes[0] == 0)
  {
    mode = bytes[1];
  }
  if(bytes[0] == 1)
  {
    weRed = bytes[1]==true;
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
