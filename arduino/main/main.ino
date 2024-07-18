#define Taster_Farbe_Rot 2
#define Taster_Farbe_Gruen 4
#define Taster_Farbe_Blau 7
#define Battarieweachter_ROT 5
#define Battarieweachter_GRUEN 6
#define voltage 7
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
DFRobotDFPlayerMini myMP3;

#include <Adafruit_NeoPixel.h>
#define PINLinks             3                                                                             //sets the pin on which the neopixels are connected
#define NUMPIXELSLinks       39 //defines the number of pixels in the strip
int intervalLinks = 20;       //defines the delay interval between running the functions
#define PINRechts            9//sets the pin on which the neopixels are connected
#define NUMPIXELSRechts      39 //defines the number of pixels in the strip
int intervalRechts = 20;        //defines the delay interval between running the functions
#define PINMitte1             12//sets the pin on which the neopixels are connected
#define NUMPIXELSMitte1      39 //defines the number of pixels in the strip
int intervalMitte1 = 20;        //defines the delay interval between running the functions
#define PINMitte2            8 //sets the pin on which the neopixels are connected
#define NUMPIXELSMitte2      39 //defines the number of pixels in the strip
int intervalMitte2 = 20;        //defines the delay interval between running the functions

Adafruit_NeoPixel pixelsLinks = Adafruit_NeoPixel(NUMPIXELSLinks, PINLinks, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsRechts = Adafruit_NeoPixel(NUMPIXELSRechts, PINRechts, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsMitte1 = Adafruit_NeoPixel(NUMPIXELSMitte1, PINMitte1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsMitte2 = Adafruit_NeoPixel(NUMPIXELSMitte2, PINMitte2, NEO_GRB + NEO_KHZ800);

//Variablen für Seite Links
uint32_t redLinks = pixelsLinks.Color(255, 0, 0);
uint32_t blueLinks = pixelsLinks.Color(0, 0, 255);
uint32_t greenLinks = pixelsLinks.Color(0, 255, 0);
uint32_t pixelColourLinks;
uint32_t lastColorLinks;
float activeColorLinks[] = {255, 0, 0};//sets the default color to red // Menü 10, 12, und 13.
boolean NeoStateLinks[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeLinks = 0; //sets a mode to run each of the functions
long previousMillisLinks = 0; // a long value to store the millis()
long lastAllCycleLinks = 0; // last cycle in the ALL() function
long previousColorMillisLinks = 0; // timer for the last color change
int iLinks = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorLinks = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jLinks; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleLinks = 0;//sets the cycle number in newTheatreChase()
int TCColorLinks = 0;//sets the color in newTheatreChase()
int lLinks = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mLinks = 0; //sets the color value in newRainbowCycle()
int nLinks = 2; //sets the pixel number in cyclonChaser()
int breatherLinks = 0; //sets the brightness value in breather()
boolean dirLinks = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatLinks = true; //sets the beat cycle in heartbeat()
int beatsLinks = 0; //sets the beat number in heartbeat()
int brightnessLinks = 200; //sets the default brightness value
int oLinks = 0; //christmas LED value
int qLinks = 5; // values for the All() function
uint32_t lastAllColorLinks = 0; // last color displayed in the All() function

//Variablen für Seite Rechts
uint32_t redRechts = pixelsRechts.Color(255, 0, 0);
uint32_t blueRechts = pixelsRechts.Color(0, 0, 255);
uint32_t greenRechts = pixelsRechts.Color(0, 255, 0);
uint32_t pixelColourRechts;
uint32_t lastColorRechts;
float activeColorRechts[] = {255, 0, 0};//sets the default color to red
boolean NeoStateRechts[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeRechts = 0; //sets a mode to run each of the functions
long previousMillisRechts = 0; // a long value to store the millis()
long lastAllCycleRechts = 0; // last cycle in the ALL() function
long previousColorMillisRechts = 0; // timer for the last color change
int iRechts = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorRechts = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jRechts; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleRechts = 0;//sets the cycle number in newTheatreChase()
int TCColorRechts = 0;//sets the color in newTheatreChase()
int lRechts = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mRechts = 0; //sets the color value in newRainbowCycle()
int nRechts = 2; //sets the pixel number in cyclonChaser()
int breatherRechts = 0; //sets the brightness value in breather()
boolean dirRechts = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatRechts = true; //sets the beat cycle in heartbeat()
int beatsRechts = 0; //sets the beat number in heartbeat()
int brightnessRechts = 200; //sets the default brightness value
int oRechts = 0; //christmas LED value
int qRechts = 5; // values for the All() function
uint32_t lastAllColorRechts = 0; // last color displayed in the All() function

//Variablen für Seite Mitte2
uint32_t redMitte2 = pixelsMitte2.Color(255, 0, 0);
uint32_t blueMitte2 = pixelsMitte2.Color(0, 0, 255);
uint32_t greenMitte2 = pixelsMitte2.Color(0, 255, 0);
uint32_t pixelColourMitte2;
uint32_t lastColorMitte2;
float activeColorMitte2[] = {255, 0, 0};//sets the default color to red
boolean NeoStateMitte2[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, true}; //Active Neopixel Function (off by default)
int neopixModeMitte2 = 0; //sets a mode to run each of the functions
long previousMillisMitte2 = 0; // a long value to store the millis()
long lastAllCycleMitte2 = 0; // last cycle in the ALL() function
long previousColorMillisMitte2 = 0; // timer for the last color change
int iMitte2 = 0; //sets the pixel number in newTheatreChase() and newColorWipe()
int CWColorMitte2 = 0; //sets the newColorWipe() color value 0=Red, 1=Green, 2=Blue
int jMitte2; //sets the pixel to skip in newTheatreChase() and newTheatreChaseRainbow()
int cycleMitte2 = 0;//sets the cycle number in newTheatreChase()
int TCColorMitte2 = 0;//sets the color in newTheatreChase()
int lMitte2 = 0; //sets the color value to send to Wheel in newTheatreChaseRainbow() and newRainbow()
int mMitte2 = 0; //sets the color value in newRainbowCycle()
int nMitte2 = 2; //sets the pixel number in cyclonChaser()
int breatherMitte2 = 0; //sets the brightness value in breather()
boolean dirMitte2 = true; //sets the direction in breather()-breathing in or out, and cylonChaser()-left or right
boolean beatMitte2 = true; //sets the beat cycle in heartbeat()
int beatsMitte2 = 0; //sets the beat number in heartbeat()
int brightnessMitte2 = 255; //sets the default brightness value
int oMitte2 = 0; //christmas LED value
int qMitte2 = 5; // values for the All() function
uint32_t lastAllColorMitte2 = 0; // last color displayed in the All() function
byte rr = 0xff; //Meteor Rot
byte gg = 0xff; //Meteor Grün
byte bb = 0xff; //Metior Blau

byte zahla;
byte zahlb;
byte zahlc;
byte zahld;

boolean alarm = false;
