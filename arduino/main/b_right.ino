uint32_t WheelRechts(byte WheelPosRechts) { //neopixel wheel function, pick from 256 colors
  WheelPosRechts = 255 - WheelPosRechts;
  if (WheelPosRechts < 85) {
    return pixelsRechts.Color(255 - WheelPosRechts * 3, 0, WheelPosRechts * 3);
  }
  if (WheelPosRechts < 170) {
    WheelPosRechts -= 85;
    return pixelsRechts.Color(0, WheelPosRechts * 3, 255 - WheelPosRechts * 3);
  }
  WheelPosRechts -= 170;
  return pixelsRechts.Color(WheelPosRechts * 3, 255 - WheelPosRechts * 3, 0);
}
void writeLEDSRechts(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsRechts.numPixels(); i ++)
  {
    pixelsRechts.setPixelColor(i, pixelsRechts.Color(R, G, B));
  }
  pixelsRechts.show();
}
void writeLEDSRechts(byte R, byte G, byte B, byte bright) { //same as above with brightness added
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  for (int i = 0; i < pixelsRechts.numPixels(); i ++)
  {
    pixelsRechts.setPixelColor(i, pixelsRechts.Color(R, G, B));
  }
  pixelsRechts.show();
}
void writeLEDSRechts(byte R, byte G, byte B, byte bright, byte LED) { // same as above only with individual LEDS
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  pixelsRechts.setPixelColor(LED, pixelsRechts.Color(R, G, B));
  pixelsRechts.show();
}
unsigned int RGBValueRechts(const char * s) { //converts the value to an RGB value
  unsigned int result = 0;
  int c ;
  if ('0' == *s && 'x' == *(s + 1)) {
    s += 2;
    while (*s) {
      result = result << 4;
      if (c = (*s - '0'), (c >= 0 && c <= 9)) result |= c;
      else if (c = (*s - 'A'), (c >= 0 && c <= 5)) result |= (c + 10);
      else if (c = (*s - 'a'), (c >= 0 && c <= 5)) result |= (c + 10);
      else break;
      ++s;
    }
  }
  return result;
}
uint8_t splitColorRechts ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

void ALLRechts() {
  if (millis() - lastAllCycleRechts > 60000)
  {
    qRechts ++;
    if ((qRechts < 5) || (qRechts > 14))
    {
      qRechts = 5;

    }
    lastAllCycleRechts = millis();
  }
  if (qRechts == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeRechts();
  }
  if (qRechts == 6)
  {
    newTheatreChaseRechts();
  }
  if (qRechts == 7)
  {
    newRainbowRechts();
  }
  if (qRechts == 8)
  {
    newTheatreChaseRainbowRechts();
  }
  if (qRechts == 9)
  {
    colorCyclerRechts();
    cylonChaserRechts();
  }
  if (qRechts == 10)
  {
    newRainbowCycleRechts();
  }
  if (qRechts == 11)
  {
    colorCyclerRechts();
    breathingRechts();
  }
  if (qRechts == 12)
  {
    colorCyclerRechts();
    heartbeatRechts();
  }
  if (qRechts == 13)
  {
    christmasChaseRechts();
  }
  if (qRechts == 14)
  {
    FireRechts();
  }
}
void colorCyclerRechts() {
  if (millis() - previousColorMillisRechts > intervalRechts)
  {
    lastColorRechts ++;
    if (lastColorRechts > 255)
    {
      lastColorRechts = 0;
    }
    uint32_t newColor = WheelRechts(lastColorRechts);
    activeColorRechts[0] = splitColorRechts(newColor, 'r');
    activeColorRechts[1] = splitColorRechts(newColor, 'g');
    activeColorRechts[2] = splitColorRechts(newColor, 'b');
    previousColorMillisRechts = millis();
  }
}
void christmasChaseRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qRechtsRechts = 0; qRechtsRechts < NUMPIXELSRechts + 4; qRechtsRechts ++)
    {
      pixelsRechts.setPixelColor(qRechtsRechts, pixelsRechts.Color(255, 0, 0));
    }
    if (oRechts < 4)
    {
      for (int p = oRechts; p < NUMPIXELSRechts + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsRechts.setPixelColor(p, pixelsRechts.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSRechts + 4 ))
        {
          pixelsRechts.setPixelColor(p, pixelsRechts.Color(0, 255, 0));
          pixelsRechts.setPixelColor(p - 1, pixelsRechts.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSRechts % 4) == 2) {
          pixelsRechts.setPixelColor(NUMPIXELSRechts - 1, pixelsRechts.Color(0, 255, 0));
        }
      }
      pixelsRechts.show();
      oRechts++;
    }
    if (oRechts >= 4)
      oRechts = 0;
    previousMillisRechts = millis();
  }
}
void heartbeatRechts() {
#if defined DEBUG
  Serial.print("testintervalRechts");
  Serial.println(millis() - previousMillisRechts);
#endif
  if (millis() - previousMillisRechts > intervalRechts * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatRechts == true) && (beatsRechts == 0) && (millis() - previousMillisRechts > intervalRechts * 7)) //if the beatRechts is on and it's the first beatRechts (beatRechtss==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSRechts((activeColorRechts[0] / 255) * h, (activeColorRechts[1] / 255) * h, (activeColorRechts[2] / 255) * h);
        delay(3);
      }
      beatRechts = false;//sets the next beatRechts to off
      previousMillisRechts = millis();//starts the timer again


    }
    else if ((beatRechts == false) && (beatsRechts == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSRechts((activeColorRechts[0] / 255) * h, (activeColorRechts[1] / 255) * h, (activeColorRechts[2] / 255) * h);
        delay(3);
      }
      beatRechts = true;//sets the beatRechts to On
      beatsRechts = 1;//sets the next beat to the second beat
      previousMillisRechts = millis();
    }
    else if ((beatRechts == true) && (beatsRechts == 1) && (millis() - previousMillisRechts > intervalRechts * 2))//if the beatRechts is on and it's the second beatRechts and the intervalRechts is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSRechts((activeColorRechts[0] / 255) * h, (activeColorRechts[1] / 255) * h, (activeColorRechts[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatRechts = false;//sets the next beatRechts to off
      previousMillisRechts = millis();
    }
    else if ((beatRechts == false) && (beatsRechts == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSRechts((activeColorRechts[0] / 255) * h, (activeColorRechts[1] / 255) * h, (activeColorRechts[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatRechts = true;//sets the next beat to on
      beatsRechts = 0;//starts the sequence again
      previousMillisRechts = millis();
    }
#if defined DEBUG
    Serial.print("previousMillisRechts:");
    Serial.println(previousMillisRechts);
#endif
  }
}
void breathingRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2) //if the timer has reached its delay value
  {
    writeLEDSRechts((activeColorRechts[0] / 255) * breatherRechts, (activeColorRechts[1] / 255) * breatherRechts, (activeColorRechts[2] / 255) * breatherRechts); //write the leds to the color and brightness level
    if (dirRechts == true)//if the lights are coming on
    {
      if (breatherRechts < 255)//once the value is less than 255
      {
        breatherRechts = breatherRechts + 15;//adds 15 to the brightness level for the next time
      }
      else if (breatherRechts >= 255)//if the brightness is greater or equal to 255
      {
        dirRechts = false;//sets the direction to false
      }
    }
    if (dirRechts == false)//if the lights are going off
    {
      if (breatherRechts > 0)
      {
        breatherRechts = breatherRechts - 15;//takes 15 away from the brightness level
      }
      else if (breatherRechts <= 0)//if the brightness level is nothing
        dirRechts = true;//changes the direction again to on
    }
    previousMillisRechts = millis();
  }
}
void cylonChaserRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 5 / 3) //intervalRechts * 2 / 3)
  {
    for (int h = 0; h < pixelsRechts.numPixels(); h++)
    {
      pixelsRechts.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsRechts.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsRechts.setPixelColor(nRechts, pixelsRechts.Color(activeColorRechts[0], activeColorRechts[1], activeColorRechts[2]));//sets the main pixel to full brightness
      pixelsRechts.setPixelColor(nRechts + 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsRechts.setPixelColor(nRechts - 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50));
      if (dirRechts == true)//if the pixels are going up in value
      {
        if (nRechts <  (pixelsRechts.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nRechts++;//increase N ie move one more forward the next time
        }
        else if (nRechts >= (pixelsRechts.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirRechts = false;//change the direction
        }
      }
      if (dirRechts == false)//if the pixels are going down in value
      {
        if (nRechts > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nRechts--; //decrease the active pixel number
        }
        else if (nRechts <= 1)//if the pixel number has reached 1
        {
          dirRechts = true;//change the direction
        }
      }
    }
    if ((pixelsRechts.numPixels() > 10) && (pixelsRechts.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsRechts.setPixelColor(nRechts, pixelsRechts.Color(activeColorRechts[0], activeColorRechts[1], activeColorRechts[2]));//same as above only with 2 pixels either side
      pixelsRechts.setPixelColor(nRechts + 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 150, (activeColorRechts[1] / 255) * 150, (activeColorRechts[2] / 255) * 150));
      pixelsRechts.setPixelColor(nRechts + 2, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50));
      pixelsRechts.setPixelColor(nRechts - 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 150, (activeColorRechts[1] / 255) * 150, (activeColorRechts[2] / 255) * 150));
      pixelsRechts.setPixelColor(nRechts - 2, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50));
      if (dirRechts == true)
      {
        if (nRechts <  (pixelsRechts.numPixels() - 2))
        {
          nRechts++;
        }
        else if (nRechts >= (pixelsRechts.numPixels() - 2))
        {
          dirRechts = false;
        }
      }
      if (dirRechts == false)
      {
        if (nRechts > 2)
        {
          nRechts--;
        }
        else if (nRechts <= 2)
        {
          dirRechts = true;
        }
      }
    }
    if (pixelsRechts.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsRechts.setPixelColor(nRechts, pixelsRechts.Color((activeColorRechts[0] / 255) * 255, (activeColorRechts[1] / 255) * 255, (activeColorRechts[2] / 255) * 255));
      pixelsRechts.setPixelColor(nRechts + 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 150, (activeColorRechts[1] / 255) * 150, (activeColorRechts[2] / 255) * 150));
      pixelsRechts.setPixelColor(nRechts + 2, pixelsRechts.Color((activeColorRechts[0] / 255) * 100, (activeColorRechts[1] / 255) * 100, (activeColorRechts[2] / 255) * 100));
      pixelsRechts.setPixelColor(nRechts + 3, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50));
      pixelsRechts.setPixelColor(nRechts - 1, pixelsRechts.Color((activeColorRechts[0] / 255) * 150, (activeColorRechts[1] / 255) * 150, (activeColorRechts[2] / 255) * 150));
      pixelsRechts.setPixelColor(nRechts - 2, pixelsRechts.Color((activeColorRechts[0] / 255) * 100, (activeColorRechts[1] / 255) * 100, (activeColorRechts[2] / 255) * 100));
      pixelsRechts.setPixelColor(nRechts - 3, pixelsRechts.Color((activeColorRechts[0] / 255) * 50, (activeColorRechts[1] / 255) * 50, (activeColorRechts[2] / 255) * 50));
      if (dirRechts == true)
      {
        if (nRechts <  (pixelsRechts.numPixels() - 3))
        {
          nRechts++;
        }
        else if (nRechts >= (pixelsRechts.numPixels() - 3))
        {
          dirRechts = false;
        }
      }
      if (dirRechts == false)
      {
        if (nRechts > 3)
        {
          nRechts--;
        }
        else if (nRechts <= 3)
        {
          dirRechts = true;
        }
      }
    }
    pixelsRechts.show();//show the pixels
    previousMillisRechts = millis();
  }
}
void newTheatreChaseRainbowRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2)
  {
    for (int h = 0; h < pixelsRechts.numPixels(); h = h + 3) {
      pixelsRechts.setPixelColor(h + (jRechts - 1), 0);    //turn every third pixel off from the last cycle
      pixelsRechts.setPixelColor(NUMPIXELSRechts - 1, 0);
    }
    for (int h = 0; h < pixelsRechts.numPixels(); h = h + 3)
    {
      pixelsRechts.setPixelColor(h + jRechts, WheelRechts( ( h + lRechts) % 255));//turn every third pixel on and cycle the color
    }
    pixelsRechts.show();
    jRechts++;
    if (jRechts >= 3)
      jRechts = 0;
    lRechts++;
    if (lRechts >= 256)
      lRechts = 0;
    previousMillisRechts = millis();
  }
}
void newRainbowCycleRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2)
  {
    for (int h = 0; h < pixelsRechts.numPixels(); h++)
    {
      pixelsRechts.setPixelColor(h, WheelRechts(((h * 256 / pixelsRechts.numPixels()) + mRechts) & 255));
    }
    mRechts++;
    if (mRechts >= 256 * 5)
      mRechts = 0;
    pixelsRechts.show();
    previousMillisRechts = millis();
  }
}
void newRainbowRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2)
  {
    for (int h = 0; h < pixelsRechts.numPixels(); h++)
    {
      pixelsRechts.setPixelColor(h, WheelRechts((h + lRechts) & 255));
    }
    lRechts++;
    if (lRechts >= 256)
      lRechts = 0;
    pixelsRechts.show();
    previousMillisRechts = millis();
  }
}
void newTheatreChaseRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2)
  {
    uint32_t color;
    int k = jRechts - 3;
    jRechts = iRechts;
    while (k >= 0)
    {
      pixelsRechts.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorRechts == 0)
    {
      color = pixelsRechts.Color(255, 0, 0);
    }
    else if (TCColorRechts == 1)
    {
      color = pixelsRechts.Color(0, 255, 0);
    }
    else if (TCColorRechts == 2)
    {
      color = pixelsRechts.Color(0, 0, 255);
    }
    else if (TCColorRechts == 3)
    {
      color = pixelsRechts.Color(255, 255, 255);
    }
    while (jRechts < NUMPIXELSRechts)
    {
      pixelsRechts.setPixelColor(jRechts, color);
      jRechts = jRechts + 3;
    }
    pixelsRechts.show();
    if (cycleRechts == 10)
    {
      TCColorRechts ++;
      cycleRechts = 0;
      if (TCColorRechts == 4)
        TCColorRechts = 0;
    }
    iRechts++;
    if (iRechts >= 3)
    {
      iRechts = 0;
      cycleRechts ++;
    }
    previousMillisRechts = millis();
  }
}
void newColorWipeRechts() {
  if (millis() - previousMillisRechts > intervalRechts * 2)
  {
    uint32_t color;
    if (CWColorRechts == 0)
    {
      color = pixelsRechts.Color(255, 0, 0);
    }
    else if (CWColorRechts == 1)
    {
      color = pixelsRechts.Color(0, 255, 0);
    }
    else if (CWColorRechts == 2)
    {
      color = pixelsRechts.Color(0, 0, 255);
    }
    pixelsRechts.setPixelColor(iRechts, color);
    pixelsRechts.show();
    iRechts++;
    if (iRechts == NUMPIXELSRechts)
    {
      iRechts = 0;
      CWColorRechts++;
      if (CWColorRechts == 3)
        CWColorRechts = 0;
    }
    previousMillisRechts = millis();
  }
}
void FireRechts()
{
  FireR(55,120);
}  

  void FireR(int Cooling, int Sparking) {
  static int heat[NUMPIXELSRechts];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSRechts; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSRechts) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSRechts - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSRechts; j++) {
    setPixelHeatColorRechts(j, heat[j] );
  }

  showStripRechts();
  delay(intervalRechts);
}

void setPixelHeatColorRechts (int PixelR, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelRechts(PixelR, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelRechts(PixelR, 255, heatramp, 0);
  } else {                               // coolest
    setPixelRechts(PixelR, heatramp, 0, 0);
  }
}
// *** REPLACE TO HERE ***

void showStripRechts() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsRechts.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixelRechts(int PixelR, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsRechts.setPixelColor(PixelR, pixelsRechts.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelR].r = red;
   leds[PixelR].g = green;
   leds[PixelR].b = blue;
 #endif
}

void setAllRechts(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSRechts; i++ ) {
    setPixelRechts(i, red, green, blue); 
  }
  showStripRechts();
}
