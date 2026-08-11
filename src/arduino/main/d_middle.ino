uint32_t WheelMitte2(byte WheelPosMitte2) { //neopixel wheel function, pick from 256 colors

  WheelPosMitte2 = 255 - WheelPosMitte2;
  if (WheelPosMitte2 < 85) {
    return pixelsMitte2.Color(255 - WheelPosMitte2 * 3, 0, WheelPosMitte2 * 3);
  }
  if (WheelPosMitte2 < 170) {
    WheelPosMitte2 -= 85;
    return pixelsMitte2.Color(0, WheelPosMitte2 * 3, 255 - WheelPosMitte2 * 3);
  }
  WheelPosMitte2 -= 170;
  return pixelsMitte2.Color(WheelPosMitte2 * 3, 255 - WheelPosMitte2 * 3, 0);
}
void writeLEDSMitte2(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsMitte2.numPixels(); i ++)
  {
    pixelsMitte2.setPixelColor(i, pixelsMitte2.Color(R, G, B));
  }
  pixelsMitte2.show();
}
void writeLEDSMitte2(byte R, byte G, byte B, byte bright) { //same as above with brightness added
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  for (int i = 0; i < pixelsMitte2.numPixels(); i ++)
  {
    pixelsMitte2.setPixelColor(i, pixelsMitte2.Color(R, G, B));
  }
  pixelsMitte2.show();
}
void writeLEDSMitte2(byte R, byte G, byte B, byte bright, byte LED) { // same as above only with individual LEDS
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  pixelsMitte2.setPixelColor(LED, pixelsMitte2.Color(R, G, B));
  pixelsMitte2.show();
}
unsigned int RGBValueMitte2(const char * s) { //converts the value to an RGB value
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
uint8_t splitColorMitte2 ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

void ALLMitte2() {
  if (millis() - lastAllCycleMitte2 > 60000)
  {
    qMitte2 ++;
    if ((qMitte2 < 5) || (qMitte2 > 14))
    {
      qMitte2 = 5;

    }
    lastAllCycleMitte2 = millis();
  }
  if (qMitte2 == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeMitte2();
  }
  if (qMitte2 == 6)
  {
    newTheatreChaseMitte2();
  }
  if (qMitte2 == 7)
  {
    newRainbowMitte2();
  }
  if (qMitte2 == 8)
  {
    newTheatreChaseRainbowMitte2();
  }
  if (qMitte2 == 9)
  {
    colorCyclerMitte2();
    cylonChaserMitte2();
  }
  if (qMitte2 == 10)
  {
    newRainbowCycleMitte2();
  }
  if (qMitte2 == 11)
  {
    colorCyclerMitte2();
    breathingMitte2();
  }
  if (qMitte2 == 12)
  {
    colorCyclerMitte2();
    heartbeatMitte2();
  }
  if (qMitte2 == 13)
  {
    christmasChaseMitte2();
  }
  if (qMitte2 == 14)
  {
    FireMitte2();
  }
}
void colorCyclerMitte2() {
  if (millis() - previousColorMillisMitte2 > intervalMitte2)
  {
    lastColorMitte2 ++;
    if (lastColorMitte2 > 255)
    {
      lastColorMitte2 = 0;
    }
    uint32_t newColor = WheelMitte2(lastColorMitte2);
    activeColorMitte2[0] = splitColorMitte2(newColor, 'r');
    activeColorMitte2[1] = splitColorMitte2(newColor, 'g');
    activeColorMitte2[2] = splitColorMitte2(newColor, 'b');
    previousColorMillisMitte2 = millis();
  }
}
void christmasChaseMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qMitte2Mitte2 = 0; qMitte2Mitte2 < NUMPIXELSMitte2 + 4; qMitte2Mitte2 ++)
    {
      pixelsMitte2.setPixelColor(qMitte2Mitte2, pixelsMitte2.Color(255, 0, 0));
    }
    if (oMitte2 < 4)
    {
      for (int p = oMitte2; p < NUMPIXELSMitte2 + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsMitte2.setPixelColor(p, pixelsMitte2.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSMitte2 + 4 ))
        {
          pixelsMitte2.setPixelColor(p, pixelsMitte2.Color(0, 255, 0));
          pixelsMitte2.setPixelColor(p - 1, pixelsMitte2.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSMitte2 % 4) == 2) {
          pixelsMitte2.setPixelColor(NUMPIXELSMitte2 - 1, pixelsMitte2.Color(0, 255, 0));
        }
      }
      pixelsMitte2.show();
      oMitte2++;
    }
    if (oMitte2 >= 4)
      oMitte2 = 0;
    previousMillisMitte2 = millis();
  }
}
void heartbeatMitte2() {
#if defined DEBUG
  Serial.print("testintervalMitte2");
  Serial.println(millis() - previousMillisMitte2);
#endif
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatMitte2 == true) && (beatsMitte2 == 0) && (millis() - previousMillisMitte2 > intervalMitte2 * 7)) //if the beatMitte2 is on and it's the first beatMitte2 (beatMitte2s==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSMitte2((activeColorMitte2[0] / 255) * h, (activeColorMitte2[1] / 255) * h, (activeColorMitte2[2] / 255) * h);
        delay(3);
      }
      beatMitte2 = false;//sets the next beatMitte2 to off
      previousMillisMitte2 = millis();//starts the timer again


    }
    else if ((beatMitte2 == false) && (beatsMitte2 == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSMitte2((activeColorMitte2[0] / 255) * h, (activeColorMitte2[1] / 255) * h, (activeColorMitte2[2] / 255) * h);
        delay(3);
      }
      beatMitte2 = true;//sets the beatMitte2 to On
      beatsMitte2 = 1;//sets the next beat to the second beat
      previousMillisMitte2 = millis();
    }
    else if ((beatMitte2 == true) && (beatsMitte2 == 1) && (millis() - previousMillisMitte2 > intervalMitte2 * 2))//if the beatMitte2 is on and it's the second beatMitte2 and the intervalMitte2 is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSMitte2((activeColorMitte2[0] / 255) * h, (activeColorMitte2[1] / 255) * h, (activeColorMitte2[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatMitte2 = false;//sets the next beatMitte2 to off
      previousMillisMitte2 = millis();
    }
    else if ((beatMitte2 == false) && (beatsMitte2 == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSMitte2((activeColorMitte2[0] / 255) * h, (activeColorMitte2[1] / 255) * h, (activeColorMitte2[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatMitte2 = true;//sets the next beat to on
      beatsMitte2 = 0;//starts the sequence again
      previousMillisMitte2 = millis();
    }
#if defined DEBUG
    Serial.print("previousMillisMitte2:");
    Serial.println(previousMillisMitte2);
#endif
  }
}
void breathingMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2) //if the timer has reached its delay value
  {
    writeLEDSMitte2((activeColorMitte2[0] / 255) * breatherMitte2, (activeColorMitte2[1] / 255) * breatherMitte2, (activeColorMitte2[2] / 255) * breatherMitte2); //write the leds to the color and brightness level
    if (dirMitte2 == true)//if the lights are coming on
    {
      if (breatherMitte2 < 255)//once the value is less than 255
      {
        breatherMitte2 = breatherMitte2 + 15;//adds 15 to the brightness level for the next time
      }
      else if (breatherMitte2 >= 255)//if the brightness is greater or equal to 255
      {
        dirMitte2 = false;//sets the direction to false
      }
    }
    if (dirMitte2 == false)//if the lights are going off
    {
      if (breatherMitte2 > 0)
      {
        breatherMitte2 = breatherMitte2 - 15;//takes 15 away from the brightness level
      }
      else if (breatherMitte2 <= 0)//if the brightness level is nothing
        dirMitte2 = true;//changes the direction again to on
    }
    previousMillisMitte2 = millis();
  }
}
void cylonChaserMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 5 / 3) //intervalMitte2 * 2 / 3)
  {
    for (int h = 0; h < pixelsMitte2.numPixels(); h++)
    {
      pixelsMitte2.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsMitte2.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsMitte2.setPixelColor(nMitte2, pixelsMitte2.Color(activeColorMitte2[0], activeColorMitte2[1], activeColorMitte2[2]));//sets the main pixel to full brightness
      pixelsMitte2.setPixelColor(nMitte2 + 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsMitte2.setPixelColor(nMitte2 - 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50));
      if (dirMitte2 == true)//if the pixels are going up in value
      {
        if (nMitte2 <  (pixelsMitte2.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nMitte2++;//increase N ie move one more forward the next time
        }
        else if (nMitte2 >= (pixelsMitte2.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirMitte2 = false;//change the direction
        }
      }
      if (dirMitte2 == false)//if the pixels are going down in value
      {
        if (nMitte2 > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nMitte2--; //decrease the active pixel number
        }
        else if (nMitte2 <= 1)//if the pixel number has reached 1
        {
          dirMitte2 = true;//change the direction
        }
      }
    }
    if ((pixelsMitte2.numPixels() > 10) && (pixelsMitte2.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsMitte2.setPixelColor(nMitte2, pixelsMitte2.Color(activeColorMitte2[0], activeColorMitte2[1], activeColorMitte2[2]));//same as above only with 2 pixels either side
      pixelsMitte2.setPixelColor(nMitte2 + 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 150, (activeColorMitte2[1] / 255) * 150, (activeColorMitte2[2] / 255) * 150));
      pixelsMitte2.setPixelColor(nMitte2 + 2, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50));
      pixelsMitte2.setPixelColor(nMitte2 - 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 150, (activeColorMitte2[1] / 255) * 150, (activeColorMitte2[2] / 255) * 150));
      pixelsMitte2.setPixelColor(nMitte2 - 2, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50));
      if (dirMitte2 == true)
      {
        if (nMitte2 <  (pixelsMitte2.numPixels() - 2))
        {
          nMitte2++;
        }
        else if (nMitte2 >= (pixelsMitte2.numPixels() - 2))
        {
          dirMitte2 = false;
        }
      }
      if (dirMitte2 == false)
      {
        if (nMitte2 > 2)
        {
          nMitte2--;
        }
        else if (nMitte2 <= 2)
        {
          dirMitte2 = true;
        }
      }
    }
    if (pixelsMitte2.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsMitte2.setPixelColor(nMitte2, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 255, (activeColorMitte2[1] / 255) * 255, (activeColorMitte2[2] / 255) * 255));
      pixelsMitte2.setPixelColor(nMitte2 + 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 150, (activeColorMitte2[1] / 255) * 150, (activeColorMitte2[2] / 255) * 150));
      pixelsMitte2.setPixelColor(nMitte2 + 2, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 100, (activeColorMitte2[1] / 255) * 100, (activeColorMitte2[2] / 255) * 100));
      pixelsMitte2.setPixelColor(nMitte2 + 3, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50));
      pixelsMitte2.setPixelColor(nMitte2 - 1, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 150, (activeColorMitte2[1] / 255) * 150, (activeColorMitte2[2] / 255) * 150));
      pixelsMitte2.setPixelColor(nMitte2 - 2, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 100, (activeColorMitte2[1] / 255) * 100, (activeColorMitte2[2] / 255) * 100));
      pixelsMitte2.setPixelColor(nMitte2 - 3, pixelsMitte2.Color((activeColorMitte2[0] / 255) * 50, (activeColorMitte2[1] / 255) * 50, (activeColorMitte2[2] / 255) * 50));
      if (dirMitte2 == true)
      {
        if (nMitte2 <  (pixelsMitte2.numPixels() - 3))
        {
          nMitte2++;
        }
        else if (nMitte2 >= (pixelsMitte2.numPixels() - 3))
        {
          dirMitte2 = false;
        }
      }
      if (dirMitte2 == false)
      {
        if (nMitte2 > 3)
        {
          nMitte2--;
        }
        else if (nMitte2 <= 3)
        {
          dirMitte2 = true;
        }
      }
    }
    pixelsMitte2.show();//show the pixels
    previousMillisMitte2 = millis();
  }
}
void newTheatreChaseRainbowMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)
  {
    for (int h = 0; h < pixelsMitte2.numPixels(); h = h + 3) {
      pixelsMitte2.setPixelColor(h + (jMitte2 - 1), 0);    //turn every third pixel off from the last cycle
      pixelsMitte2.setPixelColor(NUMPIXELSMitte2 - 1, 0);
    }
    for (int h = 0; h < pixelsMitte2.numPixels(); h = h + 3)
    {
      pixelsMitte2.setPixelColor(h + jMitte2, WheelMitte2( ( h + lMitte2) % 255));//turn every third pixel on and cycle the color
    }
    pixelsMitte2.show();
    jMitte2++;
    if (jMitte2 >= 3)
      jMitte2 = 0;
    lMitte2++;
    if (lMitte2 >= 256)
      lMitte2 = 0;
    previousMillisMitte2 = millis();
  }
}
void newRainbowCycleMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)
  {
    for (int h = 0; h < pixelsMitte2.numPixels(); h++)
    {
      pixelsMitte2.setPixelColor(h, WheelMitte2(((h * 256 / pixelsMitte2.numPixels()) + mMitte2) & 255));
    }
    mMitte2++;
    if (mMitte2 >= 256 * 5)
      mMitte2 = 0;
    pixelsMitte2.show();
    previousMillisMitte2 = millis();
  }
}
void newRainbowMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)
  {
    for (int h = 0; h < pixelsMitte2.numPixels(); h++)
    {
      pixelsMitte2.setPixelColor(h, WheelMitte2((h + lMitte2) & 255));
    }
    lMitte2++;
    if (lMitte2 >= 256)
      lMitte2 = 0;
    pixelsMitte2.show();
    previousMillisMitte2 = millis();
  }
}
void newTheatreChaseMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)
  {
    uint32_t color;
    int k = jMitte2 - 3;
    jMitte2 = iMitte2;
    while (k >= 0)
    {
      pixelsMitte2.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorMitte2 == 0)
    {
      color = pixelsMitte2.Color(255, 0, 0);
    }
    else if (TCColorMitte2 == 1)
    {
      color = pixelsMitte2.Color(0, 255, 0);
    }
    else if (TCColorMitte2 == 2)
    {
      color = pixelsMitte2.Color(0, 0, 255);
    }
    else if (TCColorMitte2 == 3)
    {
      color = pixelsMitte2.Color(255, 255, 255);
    }
    while (jMitte2 < NUMPIXELSMitte2)
    {
      pixelsMitte2.setPixelColor(jMitte2, color);
      jMitte2 = jMitte2 + 3;
    }
    pixelsMitte2.show();
    if (cycleMitte2 == 10)
    {
      TCColorMitte2 ++;
      cycleMitte2 = 0;
      if (TCColorMitte2 == 4)
        TCColorMitte2 = 0;
    }
    iMitte2++;
    if (iMitte2 >= 3)
    {
      iMitte2 = 0;
      cycleMitte2 ++;
    }
    previousMillisMitte2 = millis();
  }
}
void newColorWipeMitte2() {
  if (millis() - previousMillisMitte2 > intervalMitte2 * 2)
  {
    uint32_t color;
    if (CWColorMitte2 == 0)
    {
      color = pixelsMitte2.Color(255, 0, 0);
    }
    else if (CWColorMitte2 == 1)
    {
      color = pixelsMitte2.Color(0, 255, 0);
    }
    else if (CWColorMitte2 == 2)
    {
      color = pixelsMitte2.Color(0, 0, 255);
    }
    pixelsMitte2.setPixelColor(iMitte2, color);
    pixelsMitte2.show();
    iMitte2++;
    if (iMitte2 == NUMPIXELSMitte2)
    {
      iMitte2 = 0;
      CWColorMitte2++;
      if (CWColorMitte2 == 3)
        CWColorMitte2 = 0;
    }
    previousMillisMitte2 = millis();
  }
}
void FireMitte2()
{
  FireM2(55,120);
}  

  void FireM2(int Cooling, int Sparking) {
  static int heat[NUMPIXELSMitte2];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSMitte2; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSMitte2) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSMitte2 - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSMitte2; j++) {
    setPixelHeatColorMitte2(j, heat[j] );
  }

  showStripMitte2();
  delay(intervalMitte2);
}

void setPixelHeatColorMitte2 (int PixelM2, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelMitte2(PixelM2, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelMitte2(PixelM2, 255, heatramp, 0);
  } else {                               // coolest
    setPixelMitte2(PixelM2, heatramp, 0, 0);
  }
}
// *** REPLACE TO HERE ***

void showStripMitte2() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsMitte2.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixelMitte2(int PixelM2, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsMitte2.setPixelColor(PixelM2, pixelsMitte2.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelM2].r = red;
   leds[PixelM2].g = green;
   leds[PixelM2].b = blue;
 #endif
}

void setAllMitte2(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSMitte2; i++ ) {
    setPixelMitte2(i, red, green, blue); 
  }
  showStripMitte2();
}
