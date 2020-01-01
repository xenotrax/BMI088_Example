#include <Arduino.h>
#include <FastLED.h>
#include "BMI088.h"


float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;


// How many leds in your strip?
#define NUM_LEDS 13

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 13
#define CLOCK_PIN 13
int counter = 0;
int counter2 = 0;
bool order = true;
bool order2 = true;
bool alerte = false;
byte r = random(0, 255);
byte g = random(0, 255);
byte b = random(0, 255);
byte joyPinZ = 35;
byte joyPinX = 34;           // slider variable connecetd to analog pin 0
byte joyPinY = 33;           // slider variable connecetd to analog pin 1
int X = 0;                  // variable to read the value from the analog pin 0
int Y = 0;                  // variable to read the value from the analog pin 1
int Z = 0;

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 

    int status;
  /* USB Serial to print data */
     Wire.begin();
    Serial.begin(115200);
    
    while(!Serial);
    Serial.println("BMI088 Raw Data");

   while(1)
    {
        if(bmi088.isConnection())
        {
            bmi088.initialize();
            Serial.println("BMI088 is connected");
            break;
        }
        else Serial.println("BMI088 is not connected");
        
        delay(2000);
    }


      pinMode(joyPinZ, INPUT);
      // Uncomment/edit one of the following lines for your leds arrangement.
      // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  	  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
      // FastLED.addLeds<APA104, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<UCS1903B, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<GW6205, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<GW6205_400, DATA_PIN, RGB>(leds, NUM_LEDS);
      
      // FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<SM16716, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<LPD8806, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<P9813, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<APA102, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<DOTSTAR, RGB>(leds, NUM_LEDS);

      // FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<P9813, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
}

void loop() { 

  X = map(analogRead(joyPinX),0,4095,0,255);
  Y = map(analogRead(joyPinY),0,4095,0,255);
//  X = analogRead(joyPinX);
//  Y = analogRead(joyPinY);
  Z = digitalRead(joyPinZ);
  //Serial.print(" X: "); Serial.print(X);
  //Serial.print(" Y: "); Serial.print(Y);
  //Serial.print(" Z: "); Serial.println(Z);
  
  if(Z){
    for(byte i = 0; i < NUM_LEDS;i++){
      leds[i] = CRGB::Red;
     }
     FastLED.show();
     delay(100);
     alerte = true;
 }
 else
 {
   /* code */
 
 
  // Turn the LED on, then pause
  if(alerte){
    alerte = false;
        for(byte i = 0; i < NUM_LEDS;i++){
      leds[i] = CRGB::Black;
     }
     FastLED.show();
     delay(100);
  }
  leds[counter] = CRGB(X,Y,map(counter2, 0, 13, 0, 255));
  FastLED.show();
  if(X < 128) X = 256 - X;
  byte temp = 256 - X;
  delay(temp + 10);
  // Now turn the LED off, then pause
  leds[counter] = CRGB::Black;
  FastLED.show();
  delay(temp + 10);
  if(order)counter++;
  else counter--;
  if(counter == 12 || counter == 0){
    if(order2)counter2++;
    else counter--;
    if(counter2 == 12 || counter2 == 0){
      order2 = !order2;
    }
    order = !order;
  }
 }
 
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();
    
    Serial.print(az);
    Serial.print(",");
    //Serial.print(ay);
    //Serial.print(",");
    //Serial.print(az);
    //Serial.print(",");
    

    //Serial.print(gx);
    //Serial.print(",");
    //Serial.print(gy);
    //Serial.print(",");
    //Serial.print(gz);
    //Serial.print(",");
    
    //Serial.print(temp);
    
    Serial.println();
    
    delay(50);
}
