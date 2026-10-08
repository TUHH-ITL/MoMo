uint32_t WheelMiddle2(byte WheelPosMiddle2) { //neopixel wheel function, pick from 256 colors

  WheelPosMiddle2 = 255 - WheelPosMiddle2;
  if (WheelPosMiddle2 < 85) {
    return pixelsMiddle2.Color(255 - WheelPosMiddle2 * 3, 0, WheelPosMiddle2 * 3);
  }
  if (WheelPosMiddle2 < 170) {
    WheelPosMiddle2 -= 85;
    return pixelsMiddle2.Color(0, WheelPosMiddle2 * 3, 255 - WheelPosMiddle2 * 3);
  }
  WheelPosMiddle2 -= 170;
  return pixelsMiddle2.Color(WheelPosMiddle2 * 3, 255 - WheelPosMiddle2 * 3, 0);
}
void writeLEDSMiddle2(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsMiddle2.numPixels(); i ++)
  {
    pixelsMiddle2.setPixelColor(i, pixelsMiddle2.Color(R, G, B));
  }
  pixelsMiddle2.show();
}
void writeLEDSMiddle2(byte R, byte G, byte B, byte bright) { //same as above with brightness added
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  for (int i = 0; i < pixelsMiddle2.numPixels(); i ++)
  {
    pixelsMiddle2.setPixelColor(i, pixelsMiddle2.Color(R, G, B));
  }
  pixelsMiddle2.show();
}
void writeLEDSMiddle2(byte R, byte G, byte B, byte bright, byte LED) { // same as above only with individual LEDS
  float fR = (R / 255) * bright;
  float fG = (G / 255) * bright;
  float fB = (B / 255) * bright;
  pixelsMiddle2.setPixelColor(LED, pixelsMiddle2.Color(R, G, B));
  pixelsMiddle2.show();
}
unsigned int RGBValueMiddle2(const char * s) { //converts the value to an RGB value
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
uint8_t splitColorMiddle2 ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

void ALLMiddle2() {
  if (millis() - lastAllCycleMiddle2 > 60000)
  {
    qMiddle2 ++;
    if ((qMiddle2 < 5) || (qMiddle2 > 14))
    {
      qMiddle2 = 5;

    }
    lastAllCycleMiddle2 = millis();
  }
  if (qMiddle2 == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeMiddle2();
  }
  if (qMiddle2 == 6)
  {
    newTheatreChaseMiddle2();
  }
  if (qMiddle2 == 7)
  {
    newRainbowMiddle2();
  }
  if (qMiddle2 == 8)
  {
    newTheatreChaseRainbowMiddle2();
  }
  if (qMiddle2 == 9)
  {
    colorCyclerMiddle2();
    cylonChaserMiddle2();
  }
  if (qMiddle2 == 10)
  {
    newRainbowCycleMiddle2();
  }
  if (qMiddle2 == 11)
  {
    colorCyclerMiddle2();
    breathingMiddle2();
  }
  if (qMiddle2 == 12)
  {
    colorCyclerMiddle2();
    heartbeatMiddle2();
  }
  if (qMiddle2 == 13)
  {
    christmasChaseMiddle2();
  }
  if (qMiddle2 == 14)
  {
    FireMiddle2();
  }
}
void colorCyclerMiddle2() {
  if (millis() - previousColorMillisMiddle2 > intervalMiddle2)
  {
    lastColorMiddle2 ++;
    if (lastColorMiddle2 > 255)
    {
      lastColorMiddle2 = 0;
    }
    uint32_t newColor = WheelMiddle2(lastColorMiddle2);
    activeColorMiddle2[0] = splitColorMiddle2(newColor, 'r');
    activeColorMiddle2[1] = splitColorMiddle2(newColor, 'g');
    activeColorMiddle2[2] = splitColorMiddle2(newColor, 'b');
    previousColorMillisMiddle2 = millis();
  }
}
void christmasChaseMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qMiddle2Middle2 = 0; qMiddle2Middle2 < NUMPIXELSMiddle2 + 4; qMiddle2Middle2 ++)
    {
      pixelsMiddle2.setPixelColor(qMiddle2Middle2, pixelsMiddle2.Color(255, 0, 0));
    }
    if (oMiddle2 < 4)
    {
      for (int p = oMiddle2; p < NUMPIXELSMiddle2 + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsMiddle2.setPixelColor(p, pixelsMiddle2.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSMiddle2 + 4 ))
        {
          pixelsMiddle2.setPixelColor(p, pixelsMiddle2.Color(0, 255, 0));
          pixelsMiddle2.setPixelColor(p - 1, pixelsMiddle2.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSMiddle2 % 4) == 2) {
          pixelsMiddle2.setPixelColor(NUMPIXELSMiddle2 - 1, pixelsMiddle2.Color(0, 255, 0));
        }
      }
      pixelsMiddle2.show();
      oMiddle2++;
    }
    if (oMiddle2 >= 4)
      oMiddle2 = 0;
    previousMillisMiddle2 = millis();
  }
}
void heartbeatMiddle2() {
#if defined DEBUG
  Serial.print("testintervalMiddle2");
  Serial.println(millis() - previousMillisMiddle2);
#endif
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatMiddle2 == true) && (beatsMiddle2 == 0) && (millis() - previousMillisMiddle2 > intervalMiddle2 * 7)) //if the beatMiddle2 is on and it's the first beatMiddle2 (beatMiddle2s==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSMiddle2((activeColorMiddle2[0] / 255) * h, (activeColorMiddle2[1] / 255) * h, (activeColorMiddle2[2] / 255) * h);
        delay(3);
      }
      beatMiddle2 = false;//sets the next beatMiddle2 to off
      previousMillisMiddle2 = millis();//starts the timer again


    }
    else if ((beatMiddle2 == false) && (beatsMiddle2 == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSMiddle2((activeColorMiddle2[0] / 255) * h, (activeColorMiddle2[1] / 255) * h, (activeColorMiddle2[2] / 255) * h);
        delay(3);
      }
      beatMiddle2 = true;//sets the beatMiddle2 to On
      beatsMiddle2 = 1;//sets the next beat to the second beat
      previousMillisMiddle2 = millis();
    }
    else if ((beatMiddle2 == true) && (beatsMiddle2 == 1) && (millis() - previousMillisMiddle2 > intervalMiddle2 * 2))//if the beatMiddle2 is on and it's the second beatMiddle2 and the intervalMiddle2 is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSMiddle2((activeColorMiddle2[0] / 255) * h, (activeColorMiddle2[1] / 255) * h, (activeColorMiddle2[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatMiddle2 = false;//sets the next beatMiddle2 to off
      previousMillisMiddle2 = millis();
    }
    else if ((beatMiddle2 == false) && (beatsMiddle2 == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSMiddle2((activeColorMiddle2[0] / 255) * h, (activeColorMiddle2[1] / 255) * h, (activeColorMiddle2[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatMiddle2 = true;//sets the next beat to on
      beatsMiddle2 = 0;//starts the sequence again
      previousMillisMiddle2 = millis();
    }
#if defined DEBUG
    Serial.print("previousMillisMiddle2:");
    Serial.println(previousMillisMiddle2);
#endif
  }
}
void breathingMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2) //if the timer has reached its delay value
  {
    writeLEDSMiddle2((activeColorMiddle2[0] / 255) * breatherMiddle2, (activeColorMiddle2[1] / 255) * breatherMiddle2, (activeColorMiddle2[2] / 255) * breatherMiddle2); //write the leds to the color and brightness level
    if (dirMiddle2 == true)//if the lights are coming on
    {
      if (breatherMiddle2 < 255)//once the value is less than 255
      {
        breatherMiddle2 = breatherMiddle2 + 15;//adds 15 to the brightness level for the next time
      }
      else if (breatherMiddle2 >= 255)//if the brightness is greater or equal to 255
      {
        dirMiddle2 = false;//sets the direction to false
      }
    }
    if (dirMiddle2 == false)//if the lights are going off
    {
      if (breatherMiddle2 > 0)
      {
        breatherMiddle2 = breatherMiddle2 - 15;//takes 15 away from the brightness level
      }
      else if (breatherMiddle2 <= 0)//if the brightness level is nothing
        dirMiddle2 = true;//changes the direction again to on
    }
    previousMillisMiddle2 = millis();
  }
}
void cylonChaserMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 5 / 3) //intervalMiddle2 * 2 / 3)
  {
    for (int h = 0; h < pixelsMiddle2.numPixels(); h++)
    {
      pixelsMiddle2.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsMiddle2.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsMiddle2.setPixelColor(nMiddle2, pixelsMiddle2.Color(activeColorMiddle2[0], activeColorMiddle2[1], activeColorMiddle2[2]));//sets the main pixel to full brightness
      pixelsMiddle2.setPixelColor(nMiddle2 + 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsMiddle2.setPixelColor(nMiddle2 - 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50));
      if (dirMiddle2 == true)//if the pixels are going up in value
      {
        if (nMiddle2 <  (pixelsMiddle2.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nMiddle2++;//increase N ie move one more forward the next time
        }
        else if (nMiddle2 >= (pixelsMiddle2.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirMiddle2 = false;//change the direction
        }
      }
      if (dirMiddle2 == false)//if the pixels are going down in value
      {
        if (nMiddle2 > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nMiddle2--; //decrease the active pixel number
        }
        else if (nMiddle2 <= 1)//if the pixel number has reached 1
        {
          dirMiddle2 = true;//change the direction
        }
      }
    }
    if ((pixelsMiddle2.numPixels() > 10) && (pixelsMiddle2.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsMiddle2.setPixelColor(nMiddle2, pixelsMiddle2.Color(activeColorMiddle2[0], activeColorMiddle2[1], activeColorMiddle2[2]));//same as above only with 2 pixels either side
      pixelsMiddle2.setPixelColor(nMiddle2 + 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 150, (activeColorMiddle2[1] / 255) * 150, (activeColorMiddle2[2] / 255) * 150));
      pixelsMiddle2.setPixelColor(nMiddle2 + 2, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50));
      pixelsMiddle2.setPixelColor(nMiddle2 - 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 150, (activeColorMiddle2[1] / 255) * 150, (activeColorMiddle2[2] / 255) * 150));
      pixelsMiddle2.setPixelColor(nMiddle2 - 2, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50));
      if (dirMiddle2 == true)
      {
        if (nMiddle2 <  (pixelsMiddle2.numPixels() - 2))
        {
          nMiddle2++;
        }
        else if (nMiddle2 >= (pixelsMiddle2.numPixels() - 2))
        {
          dirMiddle2 = false;
        }
      }
      if (dirMiddle2 == false)
      {
        if (nMiddle2 > 2)
        {
          nMiddle2--;
        }
        else if (nMiddle2 <= 2)
        {
          dirMiddle2 = true;
        }
      }
    }
    if (pixelsMiddle2.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsMiddle2.setPixelColor(nMiddle2, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 255, (activeColorMiddle2[1] / 255) * 255, (activeColorMiddle2[2] / 255) * 255));
      pixelsMiddle2.setPixelColor(nMiddle2 + 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 150, (activeColorMiddle2[1] / 255) * 150, (activeColorMiddle2[2] / 255) * 150));
      pixelsMiddle2.setPixelColor(nMiddle2 + 2, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 100, (activeColorMiddle2[1] / 255) * 100, (activeColorMiddle2[2] / 255) * 100));
      pixelsMiddle2.setPixelColor(nMiddle2 + 3, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50));
      pixelsMiddle2.setPixelColor(nMiddle2 - 1, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 150, (activeColorMiddle2[1] / 255) * 150, (activeColorMiddle2[2] / 255) * 150));
      pixelsMiddle2.setPixelColor(nMiddle2 - 2, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 100, (activeColorMiddle2[1] / 255) * 100, (activeColorMiddle2[2] / 255) * 100));
      pixelsMiddle2.setPixelColor(nMiddle2 - 3, pixelsMiddle2.Color((activeColorMiddle2[0] / 255) * 50, (activeColorMiddle2[1] / 255) * 50, (activeColorMiddle2[2] / 255) * 50));
      if (dirMiddle2 == true)
      {
        if (nMiddle2 <  (pixelsMiddle2.numPixels() - 3))
        {
          nMiddle2++;
        }
        else if (nMiddle2 >= (pixelsMiddle2.numPixels() - 3))
        {
          dirMiddle2 = false;
        }
      }
      if (dirMiddle2 == false)
      {
        if (nMiddle2 > 3)
        {
          nMiddle2--;
        }
        else if (nMiddle2 <= 3)
        {
          dirMiddle2 = true;
        }
      }
    }
    pixelsMiddle2.show();//show the pixels
    previousMillisMiddle2 = millis();
  }
}
void newTheatreChaseRainbowMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)
  {
    for (int h = 0; h < pixelsMiddle2.numPixels(); h = h + 3) {
      pixelsMiddle2.setPixelColor(h + (jMiddle2 - 1), 0);    //turn every third pixel off from the last cycle
      pixelsMiddle2.setPixelColor(NUMPIXELSMiddle2 - 1, 0);
    }
    for (int h = 0; h < pixelsMiddle2.numPixels(); h = h + 3)
    {
      pixelsMiddle2.setPixelColor(h + jMiddle2, WheelMiddle2( ( h + lMiddle2) % 255));//turn every third pixel on and cycle the color
    }
    pixelsMiddle2.show();
    jMiddle2++;
    if (jMiddle2 >= 3)
      jMiddle2 = 0;
    lMiddle2++;
    if (lMiddle2 >= 256)
      lMiddle2 = 0;
    previousMillisMiddle2 = millis();
  }
}
void newRainbowCycleMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)
  {
    for (int h = 0; h < pixelsMiddle2.numPixels(); h++)
    {
      pixelsMiddle2.setPixelColor(h, WheelMiddle2(((h * 256 / pixelsMiddle2.numPixels()) + mMiddle2) & 255));
    }
    mMiddle2++;
    if (mMiddle2 >= 256 * 5)
      mMiddle2 = 0;
    pixelsMiddle2.show();
    previousMillisMiddle2 = millis();
  }
}
void newRainbowMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)
  {
    for (int h = 0; h < pixelsMiddle2.numPixels(); h++)
    {
      pixelsMiddle2.setPixelColor(h, WheelMiddle2((h + lMiddle2) & 255));
    }
    lMiddle2++;
    if (lMiddle2 >= 256)
      lMiddle2 = 0;
    pixelsMiddle2.show();
    previousMillisMiddle2 = millis();
  }
}
void newTheatreChaseMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)
  {
    uint32_t color;
    int k = jMiddle2 - 3;
    jMiddle2 = iMiddle2;
    while (k >= 0)
    {
      pixelsMiddle2.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorMiddle2 == 0)
    {
      color = pixelsMiddle2.Color(255, 0, 0);
    }
    else if (TCColorMiddle2 == 1)
    {
      color = pixelsMiddle2.Color(0, 255, 0);
    }
    else if (TCColorMiddle2 == 2)
    {
      color = pixelsMiddle2.Color(0, 0, 255);
    }
    else if (TCColorMiddle2 == 3)
    {
      color = pixelsMiddle2.Color(255, 255, 255);
    }
    while (jMiddle2 < NUMPIXELSMiddle2)
    {
      pixelsMiddle2.setPixelColor(jMiddle2, color);
      jMiddle2 = jMiddle2 + 3;
    }
    pixelsMiddle2.show();
    if (cycleMiddle2 == 10)
    {
      TCColorMiddle2 ++;
      cycleMiddle2 = 0;
      if (TCColorMiddle2 == 4)
        TCColorMiddle2 = 0;
    }
    iMiddle2++;
    if (iMiddle2 >= 3)
    {
      iMiddle2 = 0;
      cycleMiddle2 ++;
    }
    previousMillisMiddle2 = millis();
  }
}
void newColorWipeMiddle2() {
  if (millis() - previousMillisMiddle2 > intervalMiddle2 * 2)
  {
    uint32_t color;
    if (CWColorMiddle2 == 0)
    {
      color = pixelsMiddle2.Color(255, 0, 0);
    }
    else if (CWColorMiddle2 == 1)
    {
      color = pixelsMiddle2.Color(0, 255, 0);
    }
    else if (CWColorMiddle2 == 2)
    {
      color = pixelsMiddle2.Color(0, 0, 255);
    }
    pixelsMiddle2.setPixelColor(iMiddle2, color);
    pixelsMiddle2.show();
    iMiddle2++;
    if (iMiddle2 == NUMPIXELSMiddle2)
    {
      iMiddle2 = 0;
      CWColorMiddle2++;
      if (CWColorMiddle2 == 3)
        CWColorMiddle2 = 0;
    }
    previousMillisMiddle2 = millis();
  }
}
void FireMiddle2()
{
  FireM2(55,120);
}  

  void FireM2(int Cooling, int Sparking) {
  static int heat[NUMPIXELSMiddle2];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSMiddle2; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSMiddle2) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSMiddle2 - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSMiddle2; j++) {
    setPixelHeatColorMiddle2(j, heat[j] );
  }

  showStripMiddle2();
  delay(intervalMiddle2);
}

void setPixelHeatColorMiddle2 (int PixelM2, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelMiddle2(PixelM2, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelMiddle2(PixelM2, 255, heatramp, 0);
  } else {                               // coolest
    setPixelMiddle2(PixelM2, heatramp, 0, 0);
  }
}
// *** REPLACE TO HERE ***

void showStripMiddle2() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsMiddle2.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixelMiddle2(int PixelM2, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsMiddle2.setPixelColor(PixelM2, pixelsMiddle2.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelM2].r = red;
   leds[PixelM2].g = green;
   leds[PixelM2].b = blue;
 #endif
}

void setAllMiddle2(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSMiddle2; i++ ) {
    setPixelMiddle2(i, red, green, blue); 
  }
  showStripMiddle2();
}
