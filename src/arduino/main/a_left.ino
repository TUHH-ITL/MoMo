uint32_t WheelLinks(byte WheelPosLinks) { //neopixel wheel function, pick from 256 colors
  WheelPosLinks = 255 - WheelPosLinks;
  if (WheelPosLinks < 85) {
    return pixelsLinks.Color(255 - WheelPosLinks * 3, 0, WheelPosLinks * 3);
  }
  if (WheelPosLinks < 170) {
    WheelPosLinks -= 85;
    return pixelsLinks.Color(0, WheelPosLinks * 3, 255 - WheelPosLinks * 3);
  }
  WheelPosLinks -= 170;
  return pixelsLinks.Color(WheelPosLinks * 3, 255 - WheelPosLinks * 3, 0);
}
void writeLEDSLinks(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsLinks.numPixels(); i ++)
  {
    pixelsLinks.setPixelColor(i, pixelsLinks.Color(R, G, B));
  }
  pixelsLinks.show();
}

unsigned int RGBValueLinks(const char * s) { //converts the value to an RGB value
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
uint8_t splitColorLinks ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}


void colorCyclerLinks() {
  if (millis() - previousColorMillisLinks > intervalLinks)
  {
    lastColorLinks ++;
    if (lastColorLinks > 255)
    {
      lastColorLinks = 0;
    }
    uint32_t newColor = WheelLinks(lastColorLinks);
    activeColorLinks[0] = splitColorLinks(newColor, 'r');
    activeColorLinks[1] = splitColorLinks(newColor, 'g');
    activeColorLinks[2] = splitColorLinks(newColor, 'b');
    previousColorMillisLinks = millis();
  }
}
void christmasChaseLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qLinksLinks = 0; qLinksLinks < NUMPIXELSLinks + 4; qLinksLinks ++)
    {
      pixelsLinks.setPixelColor(qLinksLinks, pixelsLinks.Color(255, 0, 0));
    }
    if (oLinks < 4)
    {
      for (int p = oLinks; p < NUMPIXELSLinks + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsLinks.setPixelColor(p, pixelsLinks.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSLinks + 4 ))
        {
          pixelsLinks.setPixelColor(p, pixelsLinks.Color(0, 255, 0));
          pixelsLinks.setPixelColor(p - 1, pixelsLinks.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSLinks % 4) == 2) {
          pixelsLinks.setPixelColor(NUMPIXELSLinks - 1, pixelsLinks.Color(0, 255, 0));
        }
      }
      pixelsLinks.show();
      oLinks++;
    }
    if (oLinks >= 4)
      oLinks = 0;
    previousMillisLinks = millis();
  }
}
void heartbeatLinks() {
#if defined DEBUG
#endif
  if (millis() - previousMillisLinks > intervalLinks * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatLinks == true) && (beatsLinks == 0) && (millis() - previousMillisLinks > intervalLinks * 7)) //if the beatLinks is on and it's the first beatLinks (beatLinkss==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSLinks((activeColorLinks[0] / 255) * h, (activeColorLinks[1] / 255) * h, (activeColorLinks[2] / 255) * h);
        delay(3);
      }
      beatLinks = false;//sets the next beatLinks to off
      previousMillisLinks = millis();//starts the timer again


    }
    else if ((beatLinks == false) && (beatsLinks == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSLinks((activeColorLinks[0] / 255) * h, (activeColorLinks[1] / 255) * h, (activeColorLinks[2] / 255) * h);
        delay(3);
      }
      beatLinks = true;//sets the beatLinks to On
      beatsLinks = 1;//sets the next beat to the second beat
      previousMillisLinks = millis();
    }
    else if ((beatLinks == true) && (beatsLinks == 1) && (millis() - previousMillisLinks > intervalLinks * 2))//if the beatLinks is on and it's the second beatLinks and the intervalLinks is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSLinks((activeColorLinks[0] / 255) * h, (activeColorLinks[1] / 255) * h, (activeColorLinks[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatLinks = false;//sets the next beatLinks to off
      previousMillisLinks = millis();
    }
    else if ((beatLinks == false) && (beatsLinks == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSLinks((activeColorLinks[0] / 255) * h, (activeColorLinks[1] / 255) * h, (activeColorLinks[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatLinks = true;//sets the next beat to on
      beatsLinks = 0;//starts the sequence again
      previousMillisLinks = millis();
    }
#if defined DEBUG
#endif
  }
}
void breathingLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2) //if the timer has reached its delay value
  {
    writeLEDSLinks((activeColorLinks[0] / 255) * breatherLinks, (activeColorLinks[1] / 255) * breatherLinks, (activeColorLinks[2] / 255) * breatherLinks); //write the leds to the color and brightness level
    if (dirLinks == true)//if the lights are coming on
    {
      if (breatherLinks < 255)//once the value is less than 255
      {
        breatherLinks = breatherLinks + 15;//adds 15 to the brightness level for the next time
      }
      else if (breatherLinks >= 255)//if the brightness is greater or equal to 255
      {
        dirLinks = false;//sets the direction to false
      }
    }
    if (dirLinks == false)//if the lights are going off
    {
      if (breatherLinks > 0)
      {
        breatherLinks = breatherLinks - 15;//takes 15 away from the brightness level
      }
      else if (breatherLinks <= 0)//if the brightness level is nothing
        dirLinks = true;//changes the direction again to on
    }
    previousMillisLinks = millis();
  }
}
void cylonChaserLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 5 / 3) //intervalLinks * 2 / 3)
  {
    for (int h = 0; h < pixelsLinks.numPixels(); h++)
    {
      pixelsLinks.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsLinks.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsLinks.setPixelColor(nLinks, pixelsLinks.Color(activeColorLinks[0], activeColorLinks[1], activeColorLinks[2]));//sets the main pixel to full brightness
      pixelsLinks.setPixelColor(nLinks + 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsLinks.setPixelColor(nLinks - 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50));
      if (dirLinks == true)//if the pixels are going up in value
      {
        if (nLinks <  (pixelsLinks.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nLinks++;//increase N ie move one more forward the next time
        }
        else if (nLinks >= (pixelsLinks.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirLinks = false;//change the direction
        }
      }
      if (dirLinks == false)//if the pixels are going down in value
      {
        if (nLinks > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nLinks--; //decrease the active pixel number
        }
        else if (nLinks <= 1)//if the pixel number has reached 1
        {
          dirLinks = true;//change the direction
        }
      }
    }
    if ((pixelsLinks.numPixels() > 10) && (pixelsLinks.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsLinks.setPixelColor(nLinks, pixelsLinks.Color(activeColorLinks[0], activeColorLinks[1], activeColorLinks[2]));//same as above only with 2 pixels either side
      pixelsLinks.setPixelColor(nLinks + 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 150, (activeColorLinks[1] / 255) * 150, (activeColorLinks[2] / 255) * 150));
      pixelsLinks.setPixelColor(nLinks + 2, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50));
      pixelsLinks.setPixelColor(nLinks - 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 150, (activeColorLinks[1] / 255) * 150, (activeColorLinks[2] / 255) * 150));
      pixelsLinks.setPixelColor(nLinks - 2, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50));
      if (dirLinks == true)
      {
        if (nLinks <  (pixelsLinks.numPixels() - 2))
        {
          nLinks++;
        }
        else if (nLinks >= (pixelsLinks.numPixels() - 2))
        {
          dirLinks = false;
        }
      }
      if (dirLinks == false)
      {
        if (nLinks > 2)
        {
          nLinks--;
        }
        else if (nLinks <= 2)
        {
          dirLinks = true;
        }
      }
    }
    if (pixelsLinks.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsLinks.setPixelColor(nLinks, pixelsLinks.Color((activeColorLinks[0] / 255) * 255, (activeColorLinks[1] / 255) * 255, (activeColorLinks[2] / 255) * 255));
      pixelsLinks.setPixelColor(nLinks + 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 150, (activeColorLinks[1] / 255) * 150, (activeColorLinks[2] / 255) * 150));
      pixelsLinks.setPixelColor(nLinks + 2, pixelsLinks.Color((activeColorLinks[0] / 255) * 100, (activeColorLinks[1] / 255) * 100, (activeColorLinks[2] / 255) * 100));
      pixelsLinks.setPixelColor(nLinks + 3, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50));
      pixelsLinks.setPixelColor(nLinks - 1, pixelsLinks.Color((activeColorLinks[0] / 255) * 150, (activeColorLinks[1] / 255) * 150, (activeColorLinks[2] / 255) * 150));
      pixelsLinks.setPixelColor(nLinks - 2, pixelsLinks.Color((activeColorLinks[0] / 255) * 100, (activeColorLinks[1] / 255) * 100, (activeColorLinks[2] / 255) * 100));
      pixelsLinks.setPixelColor(nLinks - 3, pixelsLinks.Color((activeColorLinks[0] / 255) * 50, (activeColorLinks[1] / 255) * 50, (activeColorLinks[2] / 255) * 50));
      if (dirLinks == true)
      {
        if (nLinks <  (pixelsLinks.numPixels() - 3))
        {
          nLinks++;
        }
        else if (nLinks >= (pixelsLinks.numPixels() - 3))
        {
          dirLinks = false;
        }
      }
      if (dirLinks == false)
      {
        if (nLinks > 3)
        {
          nLinks--;
        }
        else if (nLinks <= 3)
        {
          dirLinks = true;
        }
      }
    }
    pixelsLinks.show();//show the pixels
    previousMillisLinks = millis();
  }
}
void newTheatreChaseRainbowLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2)
  {
    for (int h = 0; h < pixelsLinks.numPixels(); h = h + 3) {
      pixelsLinks.setPixelColor(h + (jLinks - 1), 0);    //turn every third pixel off from the last cycle
      pixelsLinks.setPixelColor(NUMPIXELSLinks - 1, 0);
    }
    for (int h = 0; h < pixelsLinks.numPixels(); h = h + 3)
    {
      pixelsLinks.setPixelColor(h + jLinks, WheelLinks( ( h + lLinks) % 255));//turn every third pixel on and cycle the color
    }
    pixelsLinks.show();
    jLinks++;
    if (jLinks >= 3)
      jLinks = 0;
    lLinks++;
    if (lLinks >= 256)
      lLinks = 0;
    previousMillisLinks = millis();
  }
}
void newRainbowCycleLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2)
  {
    for (int h = 0; h < pixelsLinks.numPixels(); h++)
    {
      pixelsLinks.setPixelColor(h, WheelLinks(((h * 256 / pixelsLinks.numPixels()) + mLinks) & 255));
    }
    mLinks++;
    if (mLinks >= 256 * 5)
      mLinks = 0;
    pixelsLinks.show();
    previousMillisLinks = millis();
  }
}
void newRainbowLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2)
  {
    for (int h = 0; h < pixelsLinks.numPixels(); h++)
    {
      pixelsLinks.setPixelColor(h, WheelLinks((h + lLinks) & 255));
    }
    lLinks++;
    if (lLinks >= 256)
      lLinks = 0;
    pixelsLinks.show();
    previousMillisLinks = millis();
  }
}
void newTheatreChaseLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2)
  {
    uint32_t color;
    int k = jLinks - 3;
    jLinks = iLinks;
    while (k >= 0)
    {
      pixelsLinks.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorLinks == 0)
    {
      color = pixelsLinks.Color(255, 0, 0);
    }
    else if (TCColorLinks == 1)
    {
      color = pixelsLinks.Color(0, 255, 0);
    }
    else if (TCColorLinks == 2)
    {
      color = pixelsLinks.Color(0, 0, 255);
    }
    else if (TCColorLinks == 3)
    {
      color = pixelsLinks.Color(255, 255, 255);
    }
    while (jLinks < NUMPIXELSLinks)
    {
      pixelsLinks.setPixelColor(jLinks, color);
      jLinks = jLinks + 3;
    }
    pixelsLinks.show();
    if (cycleLinks == 10)
    {
      TCColorLinks ++;
      cycleLinks = 0;
      if (TCColorLinks == 4)
        TCColorLinks = 0;
    }
    iLinks++;
    if (iLinks >= 3)
    {
      iLinks = 0;
      cycleLinks ++;
    }
    previousMillisLinks = millis();
  }
}
void newColorWipeLinks() {
  if (millis() - previousMillisLinks > intervalLinks * 2)
  {
    uint32_t color;
    if (CWColorLinks == 0)
    {
      color = pixelsLinks.Color(255, 0, 0);
    }
    else if (CWColorLinks == 1)
    {
      color = pixelsLinks.Color(0, 255, 0);
    }
    else if (CWColorLinks == 2)
    {
      color = pixelsLinks.Color(0, 0, 255);
    }
    pixelsLinks.setPixelColor(iLinks, color);
    pixelsLinks.show();
    iLinks++;
    if (iLinks == NUMPIXELSLinks)
    {
      iLinks = 0;
      CWColorLinks++;
      if (CWColorLinks == 3)
        CWColorLinks = 0;
    }
    previousMillisLinks = millis();
  }
}

void setPixelLinks(int PixelL, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsLinks.setPixelColor(PixelL, pixelsLinks.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelL].r = red;
   leds[PixelL].g = green;
   leds[PixelL].b = blue;
 #endif
}
void showStripLinks() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsLinks.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}
void setPixelHeatColorLinks (int PixelL, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191); 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelLinks(PixelL, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelLinks(PixelL, 255, heatramp, 0);
  } else {                               // coolest
    setPixelLinks(PixelL, heatramp, 0, 0);
  }
}
void FireLinks(int Cooling, int Sparking) {
  static byte heat[NUMPIXELSLinks];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSLinks; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSLinks) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSLinks - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSLinks; j++) {
    setPixelHeatColorLinks(j, heat[j] );
  }

  showStripLinks();
  delay(intervalLinks);
}
void setAllLinks(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSLinks; i++ ) {
    setPixelLinks(i, red, green, blue); 
  }
  showStripLinks();
}
void FireLinks() {
  FireLinks(55,120);
}

void MeteorLinks() {// Meteor
  int zz = random(1, 40);
  Serial.println (zz); // rotb jergentwann auftaucht

if ( zz >= 30) { // rot jergentwann auftaucht
    rr = 0x00;
    gg = 0xff;
    bb = 0x00;
  }  
if ( zz < 30 && zz >= 20) { // gruen jergentwann auftaucht
    rr = 0x00;
    gg = 0xff;
    bb = 0x00;
  }  
if ( zz < 20 && zz >= 10) { // blau jergentwann auftaucht
    rr = 0x00;
    gg = 0x00;
    bb = 0xff;
  }  
if ( zz < 10) { // weiss jergentwann auftaucht
    rr = 0xff;
    gg = 0xff;
    bb = 0xff;
  }
  //MeteorL(random(1, 5), rr, gg, bb, random(5, 8), random(64, 75), true, random(20, 75));
  MeteorL(random(1, 5), rr, gg, bb, random(5, 8), random(64, 75), true, random(20, 75));
  //MeteorLinks(byte Rot, byte Gruen, byte Blau, byte interval_random2, byte interval_random3, boolean truefalse, int interval_random4)
  delay(random(100)); // 100
}
void MeteorL(int rx, byte Rot, byte Gruen, byte Blau, byte interval_random2, byte interval_random3, boolean truefalse, int interval_random4) {
  setAllL(0, 0, 0);
  for (int i = 0; i < NUMPIXELSLinks + NUMPIXELSLinks; i++) {
    //for (int i = 0; i < NUMPIXELSLinks; i++) {
    if(i == NUMPIXELSLinks + NUMPIXELSLinks){i = 0;}
    for (int j = 0; j < NUMPIXELSLinks; j++) {
      //if(j = NUMPIXELSLinks){j = 0;}
      if ( (!truefalse) || (random(15) > 8) ) {
        KeHitamLinks( rx, j, interval_random3 );
      }
    }

    // meteor
    for (int j = 0; j < interval_random2; j++) {
      if(j == NUMPIXELSLinks){j = 0;}
      if ( ( i - j < NUMPIXELSLinks) && (i - j >= 0) ) {
        setPixelLinks(rx, i - j, Rot, Gruen, Blau);
      }
    }

    pixelsLinks.show();
    delay(interval_random4);
  }
}
void KeHitamLinks(int rxx, int ledNo, byte fadeValue) {

  uint32_t oldColor;
  uint8_t r, g, b;
  int value;

  oldColor = pixelsLinks.getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
  g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
  b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);

  pixelsLinks.setPixelColor(ledNo, r, g, b);
}
void setPixelLinks(int rxxx, int Pixel, byte Rot, byte Gruen, byte Blau) {
  pixelsLinks.setPixelColor(Pixel, pixelsLinks.Color(Rot, Gruen, Blau));
}
void setAllL(byte Rot, byte Gruen, byte Blau) {

  for (int i = 0; i < NUMPIXELSLinks; i++ ) {
    if(i == NUMPIXELSLinks){i = 0;}
    pixelsLinks.setPixelColor(i, pixelsLinks.Color(Rot, Gruen, Blau));
  }
  pixelsLinks.show();
}

void ALLLinks() {
  if (millis() - lastAllCycleLinks > 60000)
  {
    qLinks ++;
    if ((qLinks < 5) || (qLinks > 14))
    {
      qLinks = 5;

    }
    lastAllCycleLinks = millis();
  }
  if (qLinks == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeLinks();
  }
  if (qLinks == 6)
  {
    newTheatreChaseLinks();
  }
  if (qLinks == 7)
  {
    newRainbowLinks();
  }
  if (qLinks == 8)
  {
    newTheatreChaseRainbowLinks();
  }
  if (qLinks == 9)
  {
    colorCyclerLinks();
    cylonChaserLinks();
  }
  if (qLinks == 10)
  {
    newRainbowCycleLinks();
  }
  if (qLinks == 11)
  {
    colorCyclerLinks();
    breathingLinks();
  }
  if (qLinks == 12)
  {
    colorCyclerLinks();
    heartbeatLinks();
  }
  if (qLinks == 13)
  {
    christmasChaseLinks();
  }
  if (qLinks == 14)
  {
    FireLinks();
  }
  if (qLinks == 15)
  {
    MeteorLinks();
  }
}
/*
void Blitz() {

  if (millis() - timer > 500) {//12000 dauert mp3
    timer = millis();
    zufall = random(1, NUMPIXELSLinks-1);
  }


  if (millis() - tasting > 2) 
  {
    tasting = millis();
    pixelsLinks.setBrightness(brightnessLinks);
    pixelsLinks.setPixelColor(zufall-2, pixelsLinks.Color(0,0,255)); // Moderately bright green color.
    pixelsLinks.setPixelColor(zufall-1, pixelsLinks.Color(0,0,255)); // Moderately bright green color.
    pixelsLinks.setPixelColor(zufall, pixelsLinks.Color(0,0,255)); // Moderately bright green color.
    pixelsLinks.setPixelColor(zufall+1, pixelsLinks.Color(0,0,255)); // Moderately bright green color.
    pixelsLinks.setPixelColor(zufall+2, pixelsLinks.Color(0,0,255)); // Moderately bright green color.
    pixelsLinks.show(); // This sends the updated pixel color to the hardware.
  }
}
*/
