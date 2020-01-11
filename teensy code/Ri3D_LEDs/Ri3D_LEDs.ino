#include <FastLED.h>
#include <Wire.h>

#define NUM_LEDS 71
#define DATA_PIN 17
CRGB leds[NUM_LEDS];

byte mode = 0;
byte toMode = 0;
boolean readyToTrans = false;
int transitionVal = 255;
long modeTime = 0;
int alliance = -1;
byte redColor = 96;
byte blueColor = 160;
double percBalls = .5;
byte yellowHue = 70;
double off = 0;

void setup() { 
  delay(100);
  Serial.begin(57600);
  Serial.println("resetting");
  LEDS.addLeds<WS2812B,DATA_PIN,RGB>(leds,NUM_LEDS);
  LEDS.setBrightness(127);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(9);                // join i2c bus with address #9
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.println("ready.");
}

void loop() 
{ 
  if(readyToTrans && mode!=toMode)
  {
    mode = toMode;
    modeTime = millis();
    if(alliance==-1)
    {
      blueColor = 160;
      redColor = 96;
    }
  }
  if(mode == 0)//off
  {
    byte val = (byte)((sin((millis()-modeTime)/500.0+PI*3/2)+1)/2*255);
    readyToTrans = val == 0;
    for(int i = 0; i < NUM_LEDS; i++) {
      // Set the i'th led to red 
      leds[i] = CHSV(0,0,val);
      // Show the leds
    }
  }
  else if(mode == 1 || mode == 3)//disabled or auto just change the speed
  {
    if(toMode==1 && mode == 1)//disabled
    {
      transitionVal = (millis()-modeTime)/1000.0*255;
      if(transitionVal>255)
      {
        transitionVal = 255;
      }
    }
    else if(toMode==3 && mode == 3)//auto
    {
      transitionVal = (millis()-modeTime)/1000.0*255;
      if(transitionVal>255)
      {
        transitionVal = 255;
      }
    }
    else
    {
      transitionVal -= 8;
      transitionVal = max(transitionVal, 0);
    }
    double offSpeed = .1;
    if(mode == 3)
    {
      offSpeed = .7;
    }
    boolean red = false;
//    double off = (millis()-modeTime)/offSpeed;
    off += offSpeed;
    if(alliance == 0)
    {
      if(blueColor>96)
      {
        blueColor--;
      }
      if(redColor>96)
      {
        redColor--;
      }
    }
    else if(alliance == 1)
    {
      if(blueColor<160)
      {
        blueColor++;
      }
      if(redColor<160)
      {
        redColor++;
      }
    }
    for(int i = 0; i < NUM_LEDS; i++) 
    {
      double perc = ((double)i)/NUM_LEDS;
      double sinIn = (perc*10)*(2*PI)+off;//10 times over 2 pi
      byte hue = 0;
      if(((int)(perc*10+(off+PI/2)/(2*PI)))%2<1)
      {
        hue = redColor;
      }
      else
      {
        hue = blueColor;
      }
      CHSV c = CHSV(hue, 255, min((sin(sinIn)+1)/2*255,transitionVal));
      leds[i] = c;
    }
    readyToTrans = transitionVal==0;
    if((toMode == 3 && mode == 1) || (toMode == 1 && mode == 3))
    {
      mode = toMode;
    }
  }
  else if(mode == 2)//telop
  {
    byte hue = 96;
    if(alliance == 1)
    {
      hue = 160;
    }
    if(toMode==2)
    {
      transitionVal = (millis()-modeTime)/500.0*255;
      if(transitionVal>255)
      {
        transitionVal = 255;
      }
    }
    else
    {
      transitionVal -= 16;
      transitionVal = max(transitionVal, 0);
    }
    CHSV c = CHSV(hue, 255, transitionVal);
    for(int i = 0; i < NUM_LEDS; i++) 
    {
      leds[i] = c;
    }
    readyToTrans = transitionVal == 0;
  }
  else if(mode == 4)//balls
  {
    for(int i = 0; i < NUM_LEDS; i++) 
    {
      double perc = ((double)i)/NUM_LEDS;
      CHSV c = CHSV(yellowHue, 255, perc<percBalls ? 255 : 0);
      leds[i] = c;
    }
    readyToTrans = true;
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
    toMode = bytes[1];
  }
  if(bytes[0] == 1)
  {
    boolean weRed = bytes[1]==true;
    if(weRed)
    {
      alliance = 0;
    }
    else
    {
      alliance = 1;
    }
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
