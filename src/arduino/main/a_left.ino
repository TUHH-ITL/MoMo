uint32_t WheelLeft(byte WheelPosLeft) { //neopixel wheel function, pick from 256 colors
  WheelPosLeft = 255 - WheelPosLeft;
  if (WheelPosLeft < 85) {
    return pixelsLeft.Color(255 - WheelPosLeft * 3, 0, WheelPosLeft * 3);
  }
  if (WheelPosLeft < 170) {
    WheelPosLeft -= 85;
    return pixelsLeft.Color(0, WheelPosLeft * 3, 255 - WheelPosLeft * 3);
  }
  WheelPosLeft -= 170;
  return pixelsLeft.Color(WheelPosLeft * 3, 255 - WheelPosLeft * 3, 0);
}
void writeLEDSLeft(byte R, byte G, byte B) { //basic write colors to the neopixels with RGB values
  for (int i = 0; i < pixelsLeft.numPixels(); i ++)
  {
    pixelsLeft.setPixelColor(i, pixelsLeft.Color(R, G, B));
  }
  pixelsLeft.show();
}

unsigned int RGBValueLeft(const char * s) { //converts the value to an RGB value
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
uint8_t splitColorLeft ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}


void colorCyclerLeft() {
  if (millis() - previousColorMillisLeft > intervalLeft)
  {
    lastColorLeft ++;
    if (lastColorLeft > 255)
    {
      lastColorLeft = 0;
    }
    uint32_t newColor = WheelLeft(lastColorLeft);
    activeColorLeft[0] = splitColorLeft(newColor, 'r');
    activeColorLeft[1] = splitColorLeft(newColor, 'g');
    activeColorLeft[2] = splitColorLeft(newColor, 'b');
    previousColorMillisLeft = millis();
  }
}
void christmasChaseLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 10)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    for (int qLeftLeft = 0; qLeftLeft < NUMPIXELSLeft + 4; qLeftLeft ++)
    {
      pixelsLeft.setPixelColor(qLeftLeft, pixelsLeft.Color(255, 0, 0));
    }
    if (oLeft < 4)
    {
      for (int p = oLeft; p < NUMPIXELSLeft + 4; p = p + 4)
      {
        if (p == 0)
        {
          pixelsLeft.setPixelColor(p, pixelsLeft.Color(0, 255, 0));
        }
        else if ((p > 0) && (p < NUMPIXELSLeft + 4 ))
        {
          pixelsLeft.setPixelColor(p, pixelsLeft.Color(0, 255, 0));
          pixelsLeft.setPixelColor(p - 1, pixelsLeft.Color(0, 255, 0));
        }
        if ( (p == 2) && (NUMPIXELSLeft % 4) == 2) {
          pixelsLeft.setPixelColor(NUMPIXELSLeft - 1, pixelsLeft.Color(0, 255, 0));
        }
      }
      pixelsLeft.show();
      oLeft++;
    }
    if (oLeft >= 4)
      oLeft = 0;
    previousMillisLeft = millis();
  }
}
void heartbeatLeft() {
#if defined DEBUG
#endif
  if (millis() - previousMillisLeft > intervalLeft * 2)//if the time between the function being last run is greater than intervel * 2 - run it
  {
    if ((beatLeft == true) && (beatsLeft == 0) && (millis() - previousMillisLeft > intervalLeft * 7)) //if the beatLeft is on and it's the first beatLeft (beatLefts==0) and the time between them is enough
    {
      for (int h = 50; h <= 255; h = h + 15)//turn on the pixels at 50 and bring it up to 255 in 15 level increments
      {
        writeLEDSLeft((activeColorLeft[0] / 255) * h, (activeColorLeft[1] / 255) * h, (activeColorLeft[2] / 255) * h);
        delay(3);
      }
      beatLeft = false;//sets the next beatLeft to off
      previousMillisLeft = millis();//starts the timer again


    }
    else if ((beatLeft == false) && (beatsLeft == 0))//if the beat is off and the beat cycle is still in the first beat
    {
      for (int h = 255; h >= 0; h = h - 15)//turn off the pixels
      {
        writeLEDSLeft((activeColorLeft[0] / 255) * h, (activeColorLeft[1] / 255) * h, (activeColorLeft[2] / 255) * h);
        delay(3);
      }
      beatLeft = true;//sets the beatLeft to On
      beatsLeft = 1;//sets the next beat to the second beat
      previousMillisLeft = millis();
    }
    else if ((beatLeft == true) && (beatsLeft == 1) && (millis() - previousMillisLeft > intervalLeft * 2))//if the beatLeft is on and it's the second beatLeft and the intervalLeft is enough
    {
      for (int h = 50; h <= 255; h = h + 15)
      {
        writeLEDSLeft((activeColorLeft[0] / 255) * h, (activeColorLeft[1] / 255) * h, (activeColorLeft[2] / 255) * h); //turn on the pixels
        delay(3);
      }
      beatLeft = false;//sets the next beatLeft to off
      previousMillisLeft = millis();
    }
    else if ((beatLeft == false) && (beatsLeft == 1))//if the beat is off and it's the second beat
    {
      for (int h = 255; h >= 0; h = h - 15)
      {
        writeLEDSLeft((activeColorLeft[0] / 255) * h, (activeColorLeft[1] / 255) * h, (activeColorLeft[2] / 255) * h); //turn off the pixels
        delay(3);
      }
      beatLeft = true;//sets the next beat to on
      beatsLeft = 0;//starts the sequence again
      previousMillisLeft = millis();
    }
#if defined DEBUG
#endif
  }
}
void breathingLeft() {
  // Step 10 (was 5) halves the boot pulse period vs the previous tuning,
  // same gate. (Actual update rate is still capped
  // by the main loop()'s own blocking delay(50)/delay(100) calls elsewhere,
  // so this is the achievable smoothness without touching that.)
  if (millis() - previousMillisLeft > intervalLeft * 2 / 3) //if the timer has reached its delay value
  {
    writeLEDSLeft((activeColorLeft[0] / 255) * breatherLeft, (activeColorLeft[1] / 255) * breatherLeft, (activeColorLeft[2] / 255) * breatherLeft); //write the leds to the color and brightness level
    if (dirLeft == true)//if the lights are coming on
    {
      if (breatherLeft < 255)//once the value is less than 255
      {
        breatherLeft = breatherLeft + 10;//adds 10 to the brightness level for the next time
        if (breatherLeft > 255) breatherLeft = 255;//clamp: step 10 doesn't divide 255 evenly, would overshoot and byte-wrap
      }
      else if (breatherLeft >= 255)//if the brightness is greater or equal to 255
      {
        dirLeft = false;//sets the direction to false
      }
    }
    if (dirLeft == false)//if the lights are going off
    {
      if (breatherLeft > 0)
      {
        breatherLeft = breatherLeft - 10;//takes 10 away from the brightness level
        if (breatherLeft < 0) breatherLeft = 0;//clamp: step 10 doesn't divide 255 evenly, would undershoot and byte-wrap
      }
      else if (breatherLeft <= 0)//if the brightness level is nothing
        dirLeft = true;//changes the direction again to on
    }
    previousMillisLeft = millis();
  }
}
void cylonChaserLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 5 / 3) //intervalLeft * 2 / 3)
  {
    for (int h = 0; h < pixelsLeft.numPixels(); h++)
    {
      pixelsLeft.setPixelColor(h, 0);//sets all pixels to off
    }
    if (pixelsLeft.numPixels() <= 10)//if the number of pixels in the strip is 10 or less only activate 3 leds in the strip
    {
      pixelsLeft.setPixelColor(nLeft, pixelsLeft.Color(activeColorLeft[0], activeColorLeft[1], activeColorLeft[2]));//sets the main pixel to full brightness
      pixelsLeft.setPixelColor(nLeft + 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50)); //sets the surrounding pixels brightness to 50
      pixelsLeft.setPixelColor(nLeft - 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50));
      if (dirLeft == true)//if the pixels are going up in value
      {
        if (nLeft <  (pixelsLeft.numPixels() - 1))//if the pixels are moving forward and havent reach the end of the strip "-1" to allow for the surrounding pixels
        {
          nLeft++;//increase N ie move one more forward the next time
        }
        else if (nLeft >= (pixelsLeft.numPixels() - 1))//if the pixels have reached the end of the strip
        {
          dirLeft = false;//change the direction
        }
      }
      if (dirLeft == false)//if the pixels are going down in value
      {
        if (nLeft > 1)//if the pixel number is greater than 1 (to allow for the surrounding pixels)
        {
          nLeft--; //decrease the active pixel number
        }
        else if (nLeft <= 1)//if the pixel number has reached 1
        {
          dirLeft = true;//change the direction
        }
      }
    }
    if ((pixelsLeft.numPixels() > 10) && (pixelsLeft.numPixels() <= 20))//if there are between 11 and 20 pixels in the strip add 2 pixels on either side of the main pixel
    {
      pixelsLeft.setPixelColor(nLeft, pixelsLeft.Color(activeColorLeft[0], activeColorLeft[1], activeColorLeft[2]));//same as above only with 2 pixels either side
      pixelsLeft.setPixelColor(nLeft + 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 150, (activeColorLeft[1] / 255) * 150, (activeColorLeft[2] / 255) * 150));
      pixelsLeft.setPixelColor(nLeft + 2, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50));
      pixelsLeft.setPixelColor(nLeft - 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 150, (activeColorLeft[1] / 255) * 150, (activeColorLeft[2] / 255) * 150));
      pixelsLeft.setPixelColor(nLeft - 2, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50));
      if (dirLeft == true)
      {
        if (nLeft <  (pixelsLeft.numPixels() - 2))
        {
          nLeft++;
        }
        else if (nLeft >= (pixelsLeft.numPixels() - 2))
        {
          dirLeft = false;
        }
      }
      if (dirLeft == false)
      {
        if (nLeft > 2)
        {
          nLeft--;
        }
        else if (nLeft <= 2)
        {
          dirLeft = true;
        }
      }
    }
    if (pixelsLeft.numPixels() > 20)//if there are more than 20 pixels in the strip add 3 pixels either side of the main pixel
    {
      pixelsLeft.setPixelColor(nLeft, pixelsLeft.Color((activeColorLeft[0] / 255) * 255, (activeColorLeft[1] / 255) * 255, (activeColorLeft[2] / 255) * 255));
      pixelsLeft.setPixelColor(nLeft + 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 150, (activeColorLeft[1] / 255) * 150, (activeColorLeft[2] / 255) * 150));
      pixelsLeft.setPixelColor(nLeft + 2, pixelsLeft.Color((activeColorLeft[0] / 255) * 100, (activeColorLeft[1] / 255) * 100, (activeColorLeft[2] / 255) * 100));
      pixelsLeft.setPixelColor(nLeft + 3, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50));
      pixelsLeft.setPixelColor(nLeft - 1, pixelsLeft.Color((activeColorLeft[0] / 255) * 150, (activeColorLeft[1] / 255) * 150, (activeColorLeft[2] / 255) * 150));
      pixelsLeft.setPixelColor(nLeft - 2, pixelsLeft.Color((activeColorLeft[0] / 255) * 100, (activeColorLeft[1] / 255) * 100, (activeColorLeft[2] / 255) * 100));
      pixelsLeft.setPixelColor(nLeft - 3, pixelsLeft.Color((activeColorLeft[0] / 255) * 50, (activeColorLeft[1] / 255) * 50, (activeColorLeft[2] / 255) * 50));
      if (dirLeft == true)
      {
        if (nLeft <  (pixelsLeft.numPixels() - 3))
        {
          nLeft++;
        }
        else if (nLeft >= (pixelsLeft.numPixels() - 3))
        {
          dirLeft = false;
        }
      }
      if (dirLeft == false)
      {
        if (nLeft > 3)
        {
          nLeft--;
        }
        else if (nLeft <= 3)
        {
          dirLeft = true;
        }
      }
    }
    pixelsLeft.show();//show the pixels
    previousMillisLeft = millis();
  }
}
void newTheatreChaseRainbowLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 2)
  {
    for (int h = 0; h < pixelsLeft.numPixels(); h = h + 3) {
      pixelsLeft.setPixelColor(h + (jLeft - 1), 0);    //turn every third pixel off from the last cycle
      pixelsLeft.setPixelColor(NUMPIXELSLeft - 1, 0);
    }
    for (int h = 0; h < pixelsLeft.numPixels(); h = h + 3)
    {
      pixelsLeft.setPixelColor(h + jLeft, WheelLeft( ( h + lLeft) % 255));//turn every third pixel on and cycle the color
    }
    pixelsLeft.show();
    jLeft++;
    if (jLeft >= 3)
      jLeft = 0;
    lLeft++;
    if (lLeft >= 256)
      lLeft = 0;
    previousMillisLeft = millis();
  }
}
void newRainbowCycleLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 2)
  {
    for (int h = 0; h < pixelsLeft.numPixels(); h++)
    {
      pixelsLeft.setPixelColor(h, WheelLeft(((h * 256 / pixelsLeft.numPixels()) + mLeft) & 255));
    }
    mLeft++;
    if (mLeft >= 256 * 5)
      mLeft = 0;
    pixelsLeft.show();
    previousMillisLeft = millis();
  }
}
void newRainbowLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 2)
  {
    for (int h = 0; h < pixelsLeft.numPixels(); h++)
    {
      pixelsLeft.setPixelColor(h, WheelLeft((h + lLeft) & 255));
    }
    lLeft++;
    if (lLeft >= 256)
      lLeft = 0;
    pixelsLeft.show();
    previousMillisLeft = millis();
  }
}
void newTheatreChaseLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 2)
  {
    uint32_t color;
    int k = jLeft - 3;
    jLeft = iLeft;
    while (k >= 0)
    {
      pixelsLeft.setPixelColor(k, 0);
      k = k - 3;
    }
    if (TCColorLeft == 0)
    {
      color = pixelsLeft.Color(255, 0, 0);
    }
    else if (TCColorLeft == 1)
    {
      color = pixelsLeft.Color(0, 255, 0);
    }
    else if (TCColorLeft == 2)
    {
      color = pixelsLeft.Color(0, 0, 255);
    }
    else if (TCColorLeft == 3)
    {
      color = pixelsLeft.Color(255, 255, 255);
    }
    else if (TCColorLeft == 4)
    {
      color = pixelsLeft.Color(255, 255, 0);
    }
    while (jLeft < NUMPIXELSLeft)
    {
      pixelsLeft.setPixelColor(jLeft, color);
      jLeft = jLeft + 3;
    }
    pixelsLeft.show();
    if (cycleLeft == 10)
    {
      TCColorLeft ++;
      cycleLeft = 0;
      if (TCColorLeft == 4)
        TCColorLeft = 0;
    }
    iLeft++;
    if (iLeft >= 3)
    {
      iLeft = 0;
      cycleLeft ++;
    }
    previousMillisLeft = millis();
  }
}
void flashingLeft() { // rapid on/off RED blink, no held state needed
  if ((millis() / 150) % 2 == 0) {
    writeLEDSLeft(255, 0, 0);
  } else {
    writeLEDSLeft(0, 0, 0);
  }
}
void flashingGreenLeft() { // rapid on/off GREEN blink, no held state needed
  if ((millis() / 150) % 2 == 0) {
    writeLEDSLeft(0, 255, 0);
  } else {
    writeLEDSLeft(0, 0, 0);
  }
}
void newColorWipeLeft() {
  if (millis() - previousMillisLeft > intervalLeft * 2)
  {
    uint32_t color;
    if (CWColorLeft == 0)
    {
      color = pixelsLeft.Color(255, 0, 0);
    }
    else if (CWColorLeft == 1)
    {
      color = pixelsLeft.Color(0, 255, 0);
    }
    else if (CWColorLeft == 2)
    {
      color = pixelsLeft.Color(0, 0, 255);
    }
    pixelsLeft.setPixelColor(iLeft, color);
    pixelsLeft.show();
    iLeft++;
    if (iLeft == NUMPIXELSLeft)
    {
      iLeft = 0;
      CWColorLeft++;
      if (CWColorLeft == 3)
        CWColorLeft = 0;
    }
    previousMillisLeft = millis();
  }
}

void setPixelLeft(int PixelL, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsLeft.setPixelColor(PixelL, pixelsLeft.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[PixelL].r = red;
   leds[PixelL].g = green;
   leds[PixelL].b = blue;
 #endif
}
void showStripLeft() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixelsLeft.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}
void setPixelHeatColorLeft (int PixelL, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191); 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelLeft(PixelL, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixelLeft(PixelL, 255, heatramp, 0);
  } else {                               // coolest
    setPixelLeft(PixelL, heatramp, 0, 0);
  }
}
void FireLeft(int Cooling, int Sparking) {
  static byte heat[NUMPIXELSLeft];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELSLeft; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELSLeft) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELSLeft - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELSLeft; j++) {
    setPixelHeatColorLeft(j, heat[j] );
  }

  showStripLeft();
  delay(intervalLeft);
}
void setAllLeft(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELSLeft; i++ ) {
    setPixelLeft(i, red, green, blue); 
  }
  showStripLeft();
}
void FireLeft() {
  FireLeft(55,120);
}

void MeteorLeft() {// Meteor
  int zz = random(1, 40);
  Serial.println (zz); // random color roll

if ( zz >= 30) { // sometimes red
    rr = 0x00;
    gg = 0xff;
    bb = 0x00;
  }
if ( zz < 30 && zz >= 20) { // sometimes green
    rr = 0x00;
    gg = 0xff;
    bb = 0x00;
  }
if ( zz < 20 && zz >= 10) { // sometimes blue
    rr = 0x00;
    gg = 0x00;
    bb = 0xff;
  }
if ( zz < 10) { // sometimes white
    rr = 0xff;
    gg = 0xff;
    bb = 0xff;
  }
  //MeteorLeftEffect(random(1, 5), rr, gg, bb, random(5, 8), random(64, 75), true, random(20, 75));
  MeteorLeftEffect(random(1, 5), rr, gg, bb, random(5, 8), random(64, 75), true, random(20, 75));
  //MeteorLeft(byte Red, byte Green, byte Blue, byte interval_random2, byte interval_random3, boolean truefalse, int interval_random4)
  delay(random(100)); // 100
}
void MeteorLeftEffect(int rx, byte Red, byte Green, byte Blue, byte interval_random2, byte interval_random3, boolean truefalse, int interval_random4) {
  setAllLeftMeteor(0, 0, 0);
  for (int i = 0; i < NUMPIXELSLeft + NUMPIXELSLeft; i++) {
    //for (int i = 0; i < NUMPIXELSLeft; i++) {
    if(i == NUMPIXELSLeft + NUMPIXELSLeft){i = 0;}
    for (int j = 0; j < NUMPIXELSLeft; j++) {
      //if(j = NUMPIXELSLeft){j = 0;}
      if ( (!truefalse) || (random(15) > 8) ) {
        fadeToBlackLeft( rx, j, interval_random3 );
      }
    }

    // meteor
    for (int j = 0; j < interval_random2; j++) {
      if(j == NUMPIXELSLeft){j = 0;}
      if ( ( i - j < NUMPIXELSLeft) && (i - j >= 0) ) {
        setPixelLeft(rx, i - j, Red, Green, Blue);
      }
    }

    pixelsLeft.show();
    delay(interval_random4);
  }
}
void fadeToBlackLeft(int rxx, int ledNo, byte fadeValue) {

  uint32_t oldColor;
  uint8_t r, g, b;
  int value;

  oldColor = pixelsLeft.getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
  g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
  b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);

  pixelsLeft.setPixelColor(ledNo, r, g, b);
}
void setPixelLeft(int rxxx, int Pixel, byte Red, byte Green, byte Blue) {
  pixelsLeft.setPixelColor(Pixel, pixelsLeft.Color(Red, Green, Blue));
}
void setAllLeftMeteor(byte Red, byte Green, byte Blue) {

  for (int i = 0; i < NUMPIXELSLeft; i++ ) {
    if(i == NUMPIXELSLeft){i = 0;}
    pixelsLeft.setPixelColor(i, pixelsLeft.Color(Red, Green, Blue));
  }
  pixelsLeft.show();
}

void ALLLeft() {
  if (millis() - lastAllCycleLeft > 60000)
  {
    qLeft ++;
    if ((qLeft < 5) || (qLeft > 14))
    {
      qLeft = 5;

    }
    lastAllCycleLeft = millis();
  }
  if (qLeft == 5) // if the option has been selected keep running the function for that option
  {
    newColorWipeLeft();
  }
  if (qLeft == 6)
  {
    newTheatreChaseLeft();
  }
  if (qLeft == 7)
  {
    newRainbowLeft();
  }
  if (qLeft == 8)
  {
    newTheatreChaseRainbowLeft();
  }
  if (qLeft == 9)
  {
    colorCyclerLeft();
    cylonChaserLeft();
  }
  if (qLeft == 10)
  {
    newRainbowCycleLeft();
  }
  if (qLeft == 11)
  {
    colorCyclerLeft();
    breathingLeft();
  }
  if (qLeft == 12)
  {
    colorCyclerLeft();
    heartbeatLeft();
  }
  if (qLeft == 13)
  {
    christmasChaseLeft();
  }
  if (qLeft == 14)
  {
    FireLeft();
  }
  if (qLeft == 15)
  {
    MeteorLeft();
  }
}
/*
void Blitz() {

  if (millis() - timer > 500) {//12000 dauert mp3
    timer = millis();
    randomPixel = random(1, NUMPIXELSLeft-1);
  }


  if (millis() - tasting > 2) 
  {
    tasting = millis();
    pixelsLeft.setBrightness(brightnessLeft);
    pixelsLeft.setPixelColor(randomPixel-2, pixelsLeft.Color(0,0,255)); // Moderately bright green color.
    pixelsLeft.setPixelColor(randomPixel-1, pixelsLeft.Color(0,0,255)); // Moderately bright green color.
    pixelsLeft.setPixelColor(randomPixel, pixelsLeft.Color(0,0,255)); // Moderately bright green color.
    pixelsLeft.setPixelColor(randomPixel+1, pixelsLeft.Color(0,0,255)); // Moderately bright green color.
    pixelsLeft.setPixelColor(randomPixel+2, pixelsLeft.Color(0,0,255)); // Moderately bright green color.
    pixelsLeft.show(); // This sends the updated pixel color to the hardware.
  }
}
*/
