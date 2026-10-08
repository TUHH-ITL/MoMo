uint32_t WheelRight(byte WheelPosRight) { //neopixel wheel function, pick from 256 colors
  WheelPosRight = 255 - WheelPosRight;
  if (WheelPosRight < 85) {
    return pixelsRight.Color(255 - WheelPosRight * 3, 0, WheelPosRight * 3);
  }
  if (WheelPosRight < 170) {
    WheelPosRight -= 85;
    return pixelsRight.Color(0, WheelPosRight * 3, 255 - WheelPosRight * 3);
  }
  WheelPosRight -= 170;
  return pixelsRight.Color(WheelPosRight * 3, 255 - WheelPosRight * 3, 0);
}
void writeLEDSRight(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsRight.numPixels(); i ++)
  {
    pixelsRight.setPixelColor(i, pixelsRight.Color(R, G, B));
  }
  pixelsRight.show();
}
void writeLEDSRight(byte R, byte G, byte B, byte bright) { //same as above with brightness added
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  for (int i = 0; i < pixelsRight.numPixels(); i ++)
  {
    pixelsRight.setPixelColor(i, pixelsRight.Color(R, G, B));
  }
  pixelsRight.show();
}
void writeLEDSRight(byte R, byte G, byte B, byte bright, byte LED) { // same as above only with individual LEDS
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  pixelsRight.setPixelColor(LED, pixelsRight.Color(R, G, B));
  pixelsRight.show();
}
unsigned int RGBValueRight(const char * s) { //converts the value to an RGB value
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
uint8_t splitColorRight ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

void ALLRight() {
  if (millis() - lastAllCycleRight > 60000)
  {
    qRight ++;
    if ((qRight < 5) || (qRight > 14))
    {
      qRight = 5;

    }
    lastAllCycleRight = millis();
  }
  if (qRight == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeRight();
  }
  if (qRight == 6)
  {
    newTheatreChaseRight();
  }
  if (qRight == 7)
  {
    newRainbowRight();
  }
  if (qRight == 8)
  {
    newTheatreChaseRainbowRight();
  }
  if (qRight == 9)
  {
    colorCyclerRight();
    cylonChaserRight();
  }
  if (qRight == 10)
  {
    newRainbowCycleRight();
  }
  if (qRight == 11)
  {
    colorCyclerRight();
    breathingRight();
  }
  if (qRight == 12)
  {
    colorCyclerRight();
    heartbeatRight();
  }
  if (qRight == 13)
  {
    christmasChaseRight();
  }
  if (qRight == 14)
  {
    FireRight();
  }
}
void colorCyclerRight() {
  if (millis() - previousColorMillisRight > intervalRight)
  {
    lastColorRight ++;
    if (lastColorRight > 255)
    {
      lastColorRight = 0;
    }
    uint32_t newColor = WheelRight(lastColorRight);
    activeColorRight[0] = splitColorRight(newColor, 'r');
    activeColorRight[1] = splitColorRight(newColor, 'g');
    activeColorRight[2] = splitColorRight(newColor, 'b');
    previousColorMillisRight = millis();
  }
}
void christmasChaseRight() {
  if (millis() - previousMillisRight > intervalRight * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qRightRight = 0; qRightRight < NUMPIXELSRight + 4; qRightRight ++)
    {
      pixelsRight.setPixelColor(qRightRight, pixelsRight.Color(255, 0, 0));
    }
    if (oRight < 4)
    {
      for (int p = oRight; p < NUMPIXELSRight + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsRight.setPixelColor(p, pixelsRight.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSRight + 4 ))
        {
          pixelsRight.setPixelColor(p, pixelsRight.Color(0, 255, 0));
          pixelsRight.setPixelColor(p - 1, pixelsRight.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSRight % 4) == 2) {
          pixelsRight.setPixelColor(NUMPIXELSRight - 1, pixelsRight.Color(0, 255, 0));
        }
      }
      pixelsRight.show();
      oRight++;
    }
    if (oRight >= 4)
      oRight = 0;
    previousMillisRight = millis();
  }
}
void heartbeatRight() {
#if defined DEBUG
  Serial.print("testintervalRight");
  Serial.println(millis() - previousMillisRight);
#endif
  if (millis() - previousMillisRight > intervalRight * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatRight == true) && (beatsRight == 0) && (millis() - previousMillisRight > intervalRight * 7)) //if the beatRight is on and it's the first beatRight (beatRights==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSRight((activeColorRight[0] / 255) * h, (activeColorRight[1] / 255) * h, (activeColorRight[2] / 255) * h);
        delay(3);
      }
      beatRight = false;//sets the next beatRight to off
      previousMillisRight = millis();//starts the timer again


    }
    else if ((beatRight == false) && (beatsRight == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSRight((activeColorRight[0] / 255) * h, (activeColorRight[1] / 255) * h, (activeColorRight[2] / 255) * h);
        delay(3);
      }
      beatRight = true;//sets the beatRight to On
      beatsRight = 1;//sets the next beat to the second beat
      previousMillisRight = millis();
    }
    else if ((beatRight == true) && (beatsRight == 1) && (millis() - previousMillisRight > intervalRight * 2))//if the beatRight is on and it's the second beatRight and the intervalRight is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSRight((activeColorRight[0] / 255) * h, (activeColorRight[1] / 255) * h, (activeColorRight[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatRight = false;//sets the next beatRight to off
      previousMillisRight = millis();
    }
    else if ((beatRight == false) && (beatsRight == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSRight((activeColorRight[0] / 255) * h, (activeColorRight[1] / 255) * h, (activeColorRight[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatRight = true;//sets the next beat to on
      beatsRight = 0;//starts the sequence again
      previousMillisRight = millis();
    }
#if defined DEBUG
    Serial.print("previousMillisRight:");
    Serial.println(previousMillisRight);
#endif
  }
}
void breathingRight() {
  // Step 10 (was 5) halves the boot pulse period vs the previous tuning,
  // same gate. (Actual update rate is still capped
  // by the main loop()'s own blocking delay(50)/delay(100) calls elsewhere,
  // so this is the achievable smoothness without touching that.)
  if (millis() - previousMillisRight > intervalRight * 2 / 3) //if the timer has reached its delay value
  {
    writeLEDSRight((activeColorRight[0] / 255) * breatherRight, (activeColorRight[1] / 255) * breatherRight, (activeColorRight[2] / 255) * breatherRight); //write the leds to the color and brightness level
    if (dirRight == true)//if the lights are coming on
    {
      if (breatherRight < 255)//once the value is less than 255
      {
        breatherRight = breatherRight + 10;//adds 10 to the brightness level for the next time
        if (breatherRight > 255) breatherRight = 255;//clamp: step 10 doesn't divide 255 evenly, would overshoot and byte-wrap
      }
      else if (breatherRight >= 255)//if the brightness is greater or equal to 255
      {
        dirRight = false;//sets the direction to false
      }
    }
    if (dirRight == false)//if the lights are going off
    {
      if (breatherRight > 0)
      {
        breatherRight = breatherRight - 10;//takes 10 away from the brightness level
        if (breatherRight < 0) breatherRight = 0;//clamp: step 10 doesn't divide 255 evenly, would undershoot and byte-wrap
      }
      else if (breatherRight <= 0)//if the brightness level is nothing
        dirRight = true;//changes the direction again to on
    }
    previousMillisRight = millis();
  }
}
void cylonChaserRight() {
  if (millis() - previousMillisRight > intervalRight * 5 / 3) //intervalRight * 2 / 3)
  {
    for (int h = 0; h < pixelsRight.numPixels(); h++)
    {
      pixelsRight.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsRight.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsRight.setPixelColor(nRight, pixelsRight.Color(activeColorRight[0], activeColorRight[1], activeColorRight[2]));//sets the main pixel to full brightness
      pixelsRight.setPixelColor(nRight + 1, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsRight.setPixelColor(nRight - 1, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50));
      if (dirRight == true)//if the pixels are going up in value
      {
        if (nRight <  (pixelsRight.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nRight++;//increase N ie move one more forward the next time
        }
        else if (nRight >= (pixelsRight.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirRight = false;//change the direction
        }
      }
      if (dirRight == false)//if the pixels are going down in value
      {
        if (nRight > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nRight--; //decrease the active pixel number
        }
        else if (nRight <= 1)//if the pixel number has reached 1
        {
          dirRight = true;//change the direction
        }
      }
    }
    if ((pixelsRight.numPixels() > 10) && (pixelsRight.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsRight.setPixelColor(nRight, pixelsRight.Color(activeColorRight[0], activeColorRight[1], activeColorRight[2]));//same as above only with 2 pixels either side
      pixelsRight.setPixelColor(nRight + 1, pixelsRight.Color((activeColorRight[0] / 255) * 150, (activeColorRight[1] / 255) * 150, (activeColorRight[2] / 255) * 150));
      pixelsRight.setPixelColor(nRight + 2, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50));
      pixelsRight.setPixelColor(nRight - 1, pixelsRight.Color((activeColorRight[0] / 255) * 150, (activeColorRight[1] / 255) * 150, (activeColorRight[2] / 255) * 150));
      pixelsRight.setPixelColor(nRight - 2, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50));
      if (dirRight == true)
      {
        if (nRight <  (pixelsRight.numPixels() - 2))
        {
          nRight++;
        }
        else if (nRight >= (pixelsRight.numPixels() - 2))
        {
          dirRight = false;
        }
      }
      if (dirRight == false)
      {
        if (nRight > 2)
        {
          nRight--;
        }
        else if (nRight <= 2)
        {
          dirRight = true;
        }
      }
    }
    if (pixelsRight.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsRight.setPixelColor(nRight, pixelsRight.Color((activeColorRight[0] / 255) * 255, (activeColorRight[1] / 255) * 255, (activeColorRight[2] / 255) * 255));
      pixelsRight.setPixelColor(nRight + 1, pixelsRight.Color((activeColorRight[0] / 255) * 150, (activeColorRight[1] / 255) * 150, (activeColorRight[2] / 255) * 150));
      pixelsRight.setPixelColor(nRight + 2, pixelsRight.Color((activeColorRight[0] / 255) * 100, (activeColorRight[1] / 255) * 100, (activeColorRight[2] / 255) * 100));
      pixelsRight.setPixelColor(nRight + 3, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50));
      pixelsRight.setPixelColor(nRight - 1, pixelsRight.Color((activeColorRight[0] / 255) * 150, (activeColorRight[1] / 255) * 150, (activeColorRight[2] / 255) * 150));
      pixelsRight.setPixelColor(nRight - 2, pixelsRight.Color((activeColorRight[0] / 255) * 100, (activeColorRight[1] / 255) * 100, (activeColorRight[2] / 255) * 100));
      pixelsRight.setPixelColor(nRight - 3, pixelsRight.Color((activeColorRight[0] / 255) * 50, (activeColorRight[1] / 255) * 50, (activeColorRight[2] / 255) * 50));
      if (dirRight == true)
      {
        if (nRight <  (pixelsRight.numPixels() - 3))
        {
          nRight++;
        }
        else if (nRight >= (pixelsRight.numPixels() - 3))
        {
          dirRight = false;
        }
      }
      if (dirRight == false)
      {
        if (nRight > 3)
        {
          nRight--;
        }
        else if (nRight <= 3)
        {
          dirRight = true;
        }
      }
    }
    pixelsRight.show();//show the pixels
    previousMillisRight = millis();
  }
}
void newTheatreChaseRainbowRight() {
  if (millis() - previousMillisRight > intervalRight * 2)
  {
    for (int h = 0; h < pixelsRight.numPixels(); h = h + 3) {
      pixelsRight.setPixelColor(h + (jRight - 1), 0);    //turn every third pixel off from the last cycle
      pixelsRight.setPixelColor(NUMPIXELSRight - 1, 0);
    }
    for (int h = 0; h < pixelsRight.numPixels(); h = h + 3)
    {
      pixelsRight.setPixelColor(h + jRight, WheelRight( ( h + lRight) % 255));//turn every third pixel on and cycle the color
    }
    pixelsRight.show();
    jRight++;
    if (jRight >= 3)
      jRight = 0;
    lRight++;
    if (lRight >= 256)
      lRight = 0;
    previousMillisRight = millis();
  }
}
void newRainbowCycleRight() {
  if (millis() - previousMillisRight > intervalRight * 2)
  {
    for (int h = 0; h < pixelsRight.numPixels(); h++)
    {
      pixelsRight.setPixelColor(h, WheelRight(((h * 256 / pixelsRight.numPixels()) + mRight) & 255));
    }
    mRight++;
    if (mRight >= 256 * 5)
      mRight = 0;
    pixelsRight.show();
    previousMillisRight = millis();
  }
}
void newRainbowRight() {
  if (millis() - previousMillisRight > intervalRight * 2)
  {
    for (int h = 0; h < pixelsRight.numPixels(); h++)
    {
      pixelsRight.setPixelColor(h, WheelRight((h + lRight) & 255));
    }
    lRight++;
    if (lRight >= 256)
      lRight = 0;
    pixelsRight.show();
    previousMillisRight = millis();
  }
}
void newTheatreChaseRight() {
  if (millis() - previousMillisRight > intervalRight * 2)
  {
    uint32_t color;
    int k = jRight - 3;
    jRight = iRight;
    while (k >= 0)
    {
      pixelsRight.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorRight == 0)
    {
      color = pixelsRight.Color(255, 0, 0);
    }
    else if (TCColorRight == 1)
    {
      color = pixelsRight.Color(0, 255, 0);
    }
    else if (TCColorRight == 2)
    {
      color = pixelsRight.Color(0, 0, 255);
    }
    else if (TCColorRight == 3)
    {
      color = pixelsRight.Color(255, 255, 255);
    }
    else if (TCColorRight == 4)
    {
      color = pixelsRight.Color(255, 255, 0);
    }
    while (jRight < NUMPIXELSRight)
    {
      pixelsRight.setPixelColor(jRight, color);
      jRight = jRight + 3;
    }
    pixelsRight.show();
    if (cycleRight == 10)
    {
      TCColorRight ++;
      cycleRight = 0;
      if (TCColorRight == 4)
        TCColorRight = 0;
    }
    iRight++;
    if (iRight >= 3)
    {
      iRight = 0;
      cycleRight ++;
    }
    previousMillisRight = millis();
  }
}
void flashingRight() { // rapid on/off RED blink, no held state needed
  if ((millis() / 150) % 2 == 0) {
    writeLEDSRight(255, 0, 0);
  } else {
    writeLEDSRight(0, 0, 0);
  }
}
void flashingGreenRight() { // rapid on/off GREEN blink, no held state needed
  if ((millis() / 150) % 2 == 0) {
    writeLEDSRight(0, 255, 0);
  } else {
    writeLEDSRight(0, 0, 0);
  }
}
void newColorWipeRight() {
  if (millis() - previousMillisRight > intervalRight * 2)
  {
    uint32_t color;
    if (CWColorRight == 0)
    {
      color = pixelsRight.Color(255, 0, 0);
    }
    else if (CWColorRight == 1)
    {
      color = pixelsRight.Color(0, 255, 0);
    }
    else if (CWColorRight == 2)
    {
      color = pixelsRight.Color(0, 0, 255);
    }
    pixelsRight.setPixelColor(iRight, color);
    pixelsRight.show();
    iRight++;
    if (iRight == NUMPIXELSRight)
    {
      iRight = 0;
      CWColorRight++;
      if (CWColorRight == 3)
        CWColorRight = 0;
    }
    previousMillisRight = millis();
  }
}
void FireRight()
{
  FireR(55,120);
}  

  void FireR(int Cooling, int Sparking) {
  static int heat[NUMPIXELSRight];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSRight; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSRight) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSRight - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSRight; j++) {
    setPixelHeatColorRight(j, heat[j] );
  }

  showStripRight();
  delay(intervalRight);
}

void setPixelHeatColorRight (int PixelR, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelRight(PixelR, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelRight(PixelR, 255, heatramp, 0);
  } else {                               // coolest
    setPixelRight(PixelR, heatramp, 0, 0);
  }
}
// *** REPLACE TO HERE ***

void showStripRight() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsRight.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixelRight(int PixelR, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsRight.setPixelColor(PixelR, pixelsRight.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelR].r = red;
   leds[PixelR].g = green;
   leds[PixelR].b = blue;
 #endif
}

void setAllRight(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSRight; i++ ) {
    setPixelRight(i, red, green, blue); 
  }
  showStripRight();
}
