#define Button_Color_Red 2
#define Button_Color_Green 4
#define Button_Color_Blue 7
#define BatteryWatcher_Red 5
#define BatteryWatcher_Green 6
#define voltage 7
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
DFRobotDFPlayerMini myMP3;

#include <Adafruit_NeoPixel.h>
#define PINLeft             3                                                                             //sets the pin on which the neopixels are connected
#define NUMPIXELSLeft       39 //defines the number of pixels in the strip
int intervalLeft = 20;       //defines the delay interval between running the functions
#define PINRight            9//sets the pin on which the neopixels are connected
#define NUMPIXELSRight      39 //defines the number of pixels in the strip
int intervalRight = 20;        //defines the delay interval between running the functions
#define PINMiddle1             12//sets the pin on which the neopixels are connected
#define NUMPIXELSMiddle1      39 //defines the number of pixels in the strip
int intervalMiddle1 = 20;        //defines the delay interval between running the functions
#define PINMiddle2            8 //sets the pin on which the neopixels are connected
#define NUMPIXELSMiddle2      39 //defines the number of pixels in the strip
int intervalMiddle2 = 20;        //defines the delay interval between running the functions

Adafruit_NeoPixel pixelsLeft = Adafruit_NeoPixel(NUMPIXELSLeft, PINLeft, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsRight = Adafruit_NeoPixel(NUMPIXELSRight, PINRight, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsMiddle1 = Adafruit_NeoPixel(NUMPIXELSMiddle1, PINMiddle1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsMiddle2 = Adafruit_NeoPixel(NUMPIXELSMiddle2, PINMiddle2, NEO_GRB + NEO_KHZ800);

//Variables for the Left side
uint32_t redLeft = pixelsLeft.Color(255, 0, 0);
uint32_t blueLeft = pixelsLeft.Color(0, 0, 255);
uint32_t greenLeft = pixelsLeft.Color(0, 255, 0);
uint32_t pixelColourLeft;
uint32_t lastColorLeft;
float activeColorLeft[] = {255, 0, 0};//sets the default color to red // used by modes 10, 12, and 13.
boolean NeoStateLeft[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeLeft = 0; //sets a mode to run each of the functions
long previousMillisLeft = 0; // a long value to store the millis()
long lastAllCycleLeft = 0; // last cycle in the ALL() function
long previousColorMillisLeft = 0; // timer for the last color change
int iLeft = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorLeft = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jLeft; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleLeft = 0;//sets the cycle number in newTheatreChase()
int TCColorLeft = 0;//sets the color in newTheatreChase()
int lLeft = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mLeft = 0; //sets the color value in newRainbowCycle()
int nLeft = 2; //sets the pixel number in cyclonChaser()
int breatherLeft = 0; //sets the brightness value in breather()
boolean dirLeft = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatLeft = true; //sets the beat cycle in heartbeat()
int beatsLeft = 0; //sets the beat number in heartbeat()
int brightnessLeft = 200; //sets the default brightness value
int oLeft = 0; //christmas LED value
int qLeft = 5; // values for the All() function
uint32_t lastAllColorLeft = 0; // last color displayed in the All() function

//Variables for the Right side
uint32_t redRight = pixelsRight.Color(255, 0, 0);
uint32_t blueRight = pixelsRight.Color(0, 0, 255);
uint32_t greenRight = pixelsRight.Color(0, 255, 0);
uint32_t pixelColourRight;
uint32_t lastColorRight;
float activeColorRight[] = {255, 0, 0};//sets the default color to red
boolean NeoStateRight[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeRight = 0; //sets a mode to run each of the functions
long previousMillisRight = 0; // a long value to store the millis()
long lastAllCycleRight = 0; // last cycle in the ALL() function
long previousColorMillisRight = 0; // timer for the last color change
int iRight = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorRight = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jRight; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleRight = 0;//sets the cycle number in newTheatreChase()
int TCColorRight = 0;//sets the color in newTheatreChase()
int lRight = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mRight = 0; //sets the color value in newRainbowCycle()
int nRight = 2; //sets the pixel number in cyclonChaser()
int breatherRight = 0; //sets the brightness value in breather()
boolean dirRight = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatRight = true; //sets the beat cycle in heartbeat()
int beatsRight = 0; //sets the beat number in heartbeat()
int brightnessRight = 200; //sets the default brightness value
int oRight = 0; //christmas LED value
int qRight = 5; // values for the All() function
uint32_t lastAllColorRight = 0; // last color displayed in the All() function

//Variables for the Middle2 side
uint32_t redMiddle2 = pixelsMiddle2.Color(255, 0, 0);
uint32_t blueMiddle2 = pixelsMiddle2.Color(0, 0, 255);
uint32_t greenMiddle2 = pixelsMiddle2.Color(0, 255, 0);
uint32_t pixelColourMiddle2;
uint32_t lastColorMiddle2;
float activeColorMiddle2[] = {255, 0, 0};//sets the default color to red
boolean NeoStateMiddle2[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeMiddle2 = 0; //sets a mode to run each of the functions
long previousMillisMiddle2 = 0; // a long value to store the millis()
long lastAllCycleMiddle2 = 0; // last cycle in the ALL() function
long previousColorMillisMiddle2 = 0; // timer for the last color change
int iMiddle2 = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorMiddle2 = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jMiddle2; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleMiddle2 = 0;//sets the cycle number in newTheatreChase()
int TCColorMiddle2 = 0;//sets the color in newTheatreChase()
int lMiddle2 = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mMiddle2 = 0; //sets the color value in newRainbowCycle()
int nMiddle2 = 2; //sets the pixel number in cyclonChaser()
int breatherMiddle2 = 0; //sets the brightness value in breather()
boolean dirMiddle2 = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatMiddle2 = true; //sets the beat cycle in heartbeat()
int beatsMiddle2 = 0; //sets the beat number in heartbeat()
int brightnessMiddle2 = 255; //sets the default brightness value
int oMiddle2 = 0; //christmas LED value
int qMiddle2 = 5; // values for the All() function
uint32_t lastAllColorMiddle2 = 0; // last color displayed in the All() function
byte rr = 0xff; //Meteor Red
byte gg = 0xff; //Meteor Green
byte bb = 0xff; //Meteor Blue

byte ledStateLeft;
byte ledStateRight;
byte ledStateMiddle1;
byte ledStateMiddle2;

boolean alarm = false;

boolean readyFromPC = false; // set once main PC sends the ready byte over hardware Serial
boolean bereitAnnounced = false; // ensures "Ich bin bereit" plays only once

// State color reported by the PC based on the mission_control state machine.
// 0 = none received yet (fallback: white, state machine not active), 2/3/16/17/18/19/20 = see the ledStateLeft switch in f_loop.ino
byte rosColorState = 0;
