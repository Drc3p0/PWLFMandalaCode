/* Use a Piezo sensor (percussion / drum) to send USB MIDI note on
   messages, where the "velocity" represents how hard the Piezo was
   tapped.

   Connect a Pieze sensor to analog pin.  This example was tested
   with Murata 7BB-27-4L0.  Almost any piezo sensor (not a buzzer with
   built-in oscillator electronics) may be used.  However, Piezo
   sensors are easily damaged by excessive heat if soldering.  It
   is highly recommended to buy a Piezo with wires already attached!

   Use a 100K resistor between A0 to GND, to give the sensor a "load".
   The value of this resistor determines how "sensitive" the circuit is.

   A pair of 1N4148 diodes are recommended to protect the analog pin.
   The first diode connects to A0 with its stripe (cathode) and the other
   side to GND.  The other diode connects its non-stripe (anode) side to
   A0, and its stripe (cathode) side to 3.3V.

   Sensitivity may also be tuned with the map() function.  Uncomment
   the Serial.print lines to see the actual analog measurements in the
   Arduino Serial Monitor.

   You must select MIDI from the "Tools > USB Type" menu

This is a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop 
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required 
to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution
**********************************************************/

//sketch in progress.  trying to read i2c from mpr121 and send midi message from teensy in response to which pin is touched. 
//important links: teensy wire library: https://www.pjrc.com/teensy/td_libs_Wire.html
// teensy midi library: https://www.pjrc.com/teensy/td_midi.html

//Drum stuff
int state[] = {0,0,0,0,0,0}; // 0=idle, 1=looking for peak, 2=ignore aftershocks
int peak[] = {0,0,0,0,0,0};    // remember the highest reading
elapsedMillis msec[] = {0,0,0,0,0,0}; // timer to end states 1 and 2
const int numDrumPins = 6;
const int drumNotes[] = {38,39,40,41,42,43};
const int drumPins[] = {A0, A1, A2, A7, A8, A9};
const int channel = 1;
const int thresholdMin = 12;  // minimum reading, avoid "noise"
const int aftershockMillis = 60; // time of aftershocks & vibration


#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Ticker.h>
#include <FastLED.h>

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif
//This is the duration between sends of midi signal in Milliseconds.
#define TRIGGER_DURATION 30

// LED Stuff
#define DATA_PIN    3
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    300
CRGB leds[NUM_LEDS];
#define BRIGHTNESS          100
#define FRAMES_PER_SECOND  120
#define LED_DECAY 5000 // leds will decay in brightness for 5 seconds before going dark
#define AMBIENT_DELAY 4000 // leds will start doing something after the keys are untouched for this duration.

bool ambient_leds = true;
bool trigger_leds = false;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns from Demo Reel 100 example

 CRGBPalette16 custom_palette_1 = 
    { 0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33, 
      0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF };

 CRGBPalette16 custom_palette_2 = //first variable has to be black or the colors wont continue while holding?
    { 
      CRGB::Black, CRGB::Maroon, CRGB::Maroon, CRGB::DarkViolet, CRGB::Maroon, CRGB::Maroon, CRGB::DarkRed, CRGB::Maroon,
      CRGB::Maroon, CRGB::Maroon, CRGB::DarkRed, CRGB::Maroon, CRGB::Maroon, CRGB::OrangeRed, CRGB::Maroon, CRGB::Maroon
    };

     CRGBPalette16 black_palette = 
    { 
      CRGB::Black,CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, 
      CRGB::Black,CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black
    };

  CRGBPalette16 currentPalette( black_palette);
  CRGBPalette16 targetPalette( OceanColors_p );

void triggerLoop(); // hoisted, defined below.
void ledFrameLoop();
void startAmbient();
void stopPiezo();

void triggerLoop(); // hoisted, defined below.
// Ticker Library sets up timers and a function to call when the timer elapses
           //(functioncalled, timertime, number to repeat(0is forever, RESOLUTION)
Ticker repeatTimer(triggerLoop, TRIGGER_DURATION, 0, MILLIS);
Ticker ledFrameTimer(ledFrameLoop, 1000/FRAMES_PER_SECOND, 0, MILLIS);
Ticker ambientLEDs(startAmbient, AMBIENT_DELAY, 0 , MILLIS);
Ticker PiezoEffect(stopPiezo, 1000, 0 , MILLIS);

#define MPR121_TOUCH_THRESHOLD_DEFAULT 20  ///< default touch threshold value (was 12)
#define MPR121_RELEASE_THRESHOLD_DEFAULT 6 ///< default relese threshold value (was 6)
// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 capA = Adafruit_MPR121();
Adafruit_MPR121 capB = Adafruit_MPR121(); 
// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouchedA = 0;
uint16_t currtouchedA = 0;
uint16_t lasttouchedB = 0;
uint16_t currtouchedB = 0;
const uint8_t numElectrodes = 12; //added by drc
//MPR121 MIDI output values
const uint8_t controlNumA[] = {88,88,87,87,13,13,5,5,0,0,3,3}; //Change to #'s stefan is using. 88, 87, 13, 5, 0, 3, 50 is mode shift
const uint8_t controlNumB[] = {28,29,30,31,20,21,22,23,24,25,26,27}; //Change to #'s stefan is using. 88, 87, 13, 5, 0, 3, 50 is mode shift
const uint8_t controlValA[] = {127,0,127,0,127,0,127,0,127,0,127,0}; //Change to #'s stefan is using
const uint8_t controlValB[] = {127,127,127,127,127,127,127,127,127,127,127,127}; //Change to #'s stefan is using
uint8_t ElectrodeTouchedA[numElectrodes] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t ElectrodeTouchedB[numElectrodes] = {0,0,0,0,0,0,0,0,0,0,0,0};

//old MIDI output notes ?? can  delete?
const uint8_t notes[numElectrodes] = {36, 38, 40, 43, 45, 47, 48, 50, 52, 55, 57, 60}; //added by drc


void setup() {
  
  Wire.begin(0x5A); //added by drc
  Wire.begin(0x5B); //added by drc

  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); //added by drc
  Wire.setSDA(18); //use a 4.7k pullup resistor //added by drc
  Wire.setSCL(19); //use a 4.7k pullup resistor //added by drc
  Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!capA.begin(0x5A)) {
    Serial.println("MPR121 0x5A not found, check wiring?");
    while (1); 
  }
  Serial.println("MPR121 found!");
  if (!capB.begin(0x5B)) {
    Serial.println("MPR121 0x5B not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 0x5B found!");
  
repeatTimer.start();

//from drum code.  not sure if its even necessary
  while (!Serial && millis() < 2500) /* wait for serial monitor */ ;
  Serial.println("Piezo Peak Capture");  

  repeatTimer.start();
  ledFrameTimer.start();

    // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {

//drum stuff
while (usbMIDI.read()) { } // ignore incoming messages??
  for (uint8_t x=0; x<numDrumPins; x++) { //changed i<0 to i<numElectrodes
    int value = analogRead(drumPins[x]);

    if (state[x] == 0) {
      // IDLE state: if any reading is above a threshold, begin peak
      if (value > thresholdMin) {
        //Serial.println("begin state 1");
        state[x] = 1;
        peak[x] = value;
        msec[x] = 0;
      }
    } else if (state[x] == 1) {
      // Peak Tracking state: for 10 ms, capture largest reading
      if (value > peak[x]) {
        peak[x] = value;
      }
      if (msec[x] >= 10) {
        Serial.print("peak = ");
        Serial.println(peak[x]);
        trigger_leds = true;
        ambient_leds = false;
        PiezoEffect.interval(peak[x]*10);
        //starts the piezo decay timer and the timer to start ambient after no action
        PiezoEffect.start();
        ambientLEDs.start(); 
        
        //Serial.println("begin state 2");
        int velocity = map(peak[x], thresholdMin, 1023, 1, 127);
        usbMIDI.sendNoteOn(drumNotes[x], velocity, channel);
        state[x] = 2;
        msec[x] = 0;
      }
    } else {
      // Ignore Aftershock state: wait for things to be quiet again
      if (value > thresholdMin) {
        msec[x] = 0; // keep resetting timer if above threshold
      } else if (msec[x] > 30) {
        //Serial.println("begin state 0");
        usbMIDI.sendNoteOff(drumNotes[x], 0, channel);
        state[x] = 0; // go back to idle after 30 ms below threshold
      }
    }
  // key repeat timer
  repeatTimer.update();

  // led frame render timer
  ledFrameTimer.update();

  // turn on ambient LEDs timer
  ambientLEDs.update();
  FastLED.show(); 
  PiezoEffect.update();

} 

  // Get the currently touched pads
  currtouchedA = capA.touched();
  currtouchedB = capB.touched();

  repeatTimer.update();
  // timer is runnig as long as this is getting called.
  checkElectrodes();
}

void checkElectrodes(){

  for (uint8_t i=0; i<numElectrodes; i++) { //changed i<0 to i<numElectrodes
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouchedA & _BV(i)) && !(lasttouchedA & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, HIGH); //added by drc
      //Serial.print(i); Serial.println(" touched of A");
      // set the array value to 1 on touch
      ElectrodeTouchedA[i] = 1;

      // on touch, turn off ambient leds and turn on trigger_leds
      ambient_leds = false;
      trigger_leds = true;
      // assigns palette back to black for fade in later.
      currentPalette = black_palette;
    }
    // if it *was* touched and now *isnt*, alert! 
    if (!(currtouchedA & _BV(i)) && (lasttouchedA & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, LOW);
      //Serial.print(i); Serial.println(" released of A");
      // set it back to 0 on release
      ElectrodeTouchedA[i] = 0;  

      trigger_leds = false;
      ambient_leds = false;
      ambientLEDs.start();
      //start ambient w black
      

    }

    //For mpr121 0x5B--------------------
    // it if *is* touched and *wasnt* touched before, alert!
    
    if ((currtouchedB & _BV(i)) && !(lasttouchedB & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, HIGH); //added by drc
      //Serial.print(i); Serial.println(" touched of B");
      // set the array value to 1 on touch
      ElectrodeTouchedB[i] = 1;

      // on touch, turn off ambient leds and turn on trigger_leds
      ambient_leds = false;
      trigger_leds = true;
            // assigns palette back to black for fade in later.
      currentPalette = black_palette;
    }
    // if it *was* touched and now *isnt*, alert! 
    if (!(currtouchedB & _BV(i)) && (lasttouchedB & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, LOW);
      //Serial.print(i); Serial.println(" released of B");
      // set it back to 0 on release
      ElectrodeTouchedB[i] = 0;  

      trigger_leds = false;
      ambient_leds = false;
      ambientLEDs.start();  
      //start ambient w black
      //CRGBPalette16 currentPalette( black_palette);

    }
    

  }

  // reset our state
  lasttouchedA = currtouchedA;
  lasttouchedB = currtouchedB;
  return; //Added back by DRC trying to debug??}
}

void triggerLoop(){
  // this fires on a timer and anything pressed triggers midi
  for (uint8_t i=0; i<numElectrodes; i++) { 
    if (ElectrodeTouchedA[i]) {
      triggerMidiA(i);
    }
     if (ElectrodeTouchedB[i]) {
      triggerMidiB(i);
    }
  }
  //Serial.print(ElectrodeTouched[0]);
  //Serial.print("trigger Loop");

}

void startAmbient(){
  // this fires after the ambient timer has elapsed.
  // it will start the leds doing the ambient pattern until set to false.
  ambient_leds = true;
}

void stopPiezo(){
  // after piezo timer finishes turn off effect.
  trigger_leds = false;
}

void ledFrameLoop(){
  // calls bpm from DEMO REEL 100
  // This is a single 'frame' of leds for the whole strip.
  // it gets called based on the timer to set framerate
  // any function that writes a full strip worth of colors will work.

  gHue++;
  if (trigger_leds == true) {
    bpm();
    //pacifica_add_whitecaps();
    //Serial.println("leds triggered");
  }
  else if (ambient_leds == true && trigger_leds == false) {
    
    // added this modulo to slow how much the blend gets called for a slower fade in.  
    if (gHue % 4 == 0 ){

      // this blends the just blacked out palette to target which is Ocean. 
      //12 is default. play with this one 
      nblendPaletteTowardPalette(currentPalette, targetPalette, 12);
      // form docs: a visually smoother transition: in the middle of the cross-fade your current palette will actually contain some colors from the old palette, a few blended colors, and some colors from the new palette.
      //The maximum number of possible palette changes per call is 48 (sixteen color entries time three channels each).//
      //The default 'maximim number of changes' here is 12, meaning that only approximately a quarter of the palette entries will be changed per call.


      //Serial.println("ambient mode");
    }
    // also added this sin8 that I think slows it a bit and makes it bounce, remove sin38 and it should not bounce?
    sin8(gHue) /2 ;
    fill_palette(leds, NUM_LEDS, gHue, 15, currentPalette,50, LINEARBLEND );
   
  }
  else {
    fadeToBlackBy( leds, NUM_LEDS, 1);
  }
  
}

void bpm(){
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 6; //was 18
  CRGBPalette16 palette = custom_palette_2;
  
  //const pal
	//uint8_t index
	//uint8_t brightness 
	//TBlendType blendType = LINEARBLEND
  
  /// you could probaby give a really quick ramp here with
  
  //nblendPaletteTowardPalette(currentPalette, palette, 100);
  
  uint8_t beat = beatsin8( BeatsPerMinute, 18, 255,0,1);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    
    //the third index argument (gHue + i*2) is determining the index along the palette 0-254 through the range. 
    // so slowing down would probably be remove that *2 ? 
    
    //.so changing the formula here with gHue always incrementing 0-255 can control how fast the colors scan through the palette
    leds[i] = ColorFromPalette(palette, gHue+(i*6), beat-gHue+(i*10));  //was gHue+(i*6)
  }
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void triggerMidiA(int i){
   usbMIDI.sendControlChange(controlNumA[i], controlValA[i], channel); //(control#, controlval, channel)
   Serial.print("triggered midi on: A ");
   Serial.println(i);
}
void triggerMidiB(int i){
   usbMIDI.sendControlChange(controlNumB[i], controlValB[i], channel); //(control#, controlval, channel)
   Serial.print("triggered midi on: B ");
   Serial.println(i);
}
