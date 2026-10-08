void loop() {
  while (Serial.available() > 0) {
    byte cmd = Serial.read();
    switch (cmd) {
      case 'R': readyFromPC = true; break; // main PC finished booting
      case 'G': rosColorState = 2;  break; // GREEN: autonomous mission underway
      case 'Y': rosColorState = 17; break; // YELLOW: paused / standing by
      case 'B': rosColorState = 3;  break; // BLUE: under manual/teleop control
      case 'E': rosColorState = 1;  break; // RED: gentle-stop / software emergency
      case 'W': rosColorState = 18; break; // WHITE: state machine not active yet
      case 'P': rosColorState = 19; break; // pulsing: motors booting
      case 'F': rosColorState = 20; break; // RED-ORANGE: motor driver crashed / boot failed
      case 'S': rosColorState = 21; break; // pulsing PURPLE: arm searching/scanning for a grasp
      case 'A': rosColorState = 22; break; // chasing YELLOW: arm approaching/executing a grasp
      case 'X': rosColorState = 23; break; // flashing RED: grasp failed / slipped / unreachable
      case 'K': rosColorState = 24; break; // flashing GREEN: verified grasp success
      case 'M': rosColorState = 25; break; // solid CYAN: arm under manual/joystick remote control
      default: break;
    }
  }
  if (readyFromPC && !bereitAnnounced) {
    myMP3.play(8); // "Ich bin bereit"
    bereitAnnounced = true;
  }
  pixelsLeft.setBrightness(brightnessLeft); // sets the inital brightness of the neopixels
  pixelsRight.setBrightness(brightnessRight); // sets the inital brightness of the neopixels
  int analog_BatteryWatcher_Green = analogRead(BatteryWatcher_Green);
  int analog_BatteryWatcher_Red = analogRead(BatteryWatcher_Red);
  int analog_voltage = analogRead(voltage);
  //Serial.println(analog_voltage);
  delay(50);
  if ((analog_BatteryWatcher_Red>500) && (analog_BatteryWatcher_Green<500)){ // Shutdown
    digitalWrite(Button_Color_Red, HIGH); digitalWrite(Button_Color_Green, LOW); digitalWrite(Button_Color_Blue, LOW);
    ledStateLeft = 1; ledStateRight = 1;
    delay(100);
     }
  if ((analog_BatteryWatcher_Green>500) && (analog_BatteryWatcher_Red<500) && (analog_voltage<500)){ // no alarm
     digitalWrite(Button_Color_Red, LOW); digitalWrite(Button_Color_Blue, HIGH);
     myMP3.stop();
     ledStateLeft = (rosColorState != 0) ? rosColorState : 18; // mission state color from PC, WHITE until the state machine is active
     ledStateRight = ledStateLeft;
     alarm = true;
     delay(100);
     }
  if ((analog_BatteryWatcher_Red>500) && (analog_BatteryWatcher_Green>500) && (analog_voltage<500)){ // battery warning
     digitalWrite(Button_Color_Red, HIGH); digitalWrite(Button_Color_Green, LOW); digitalWrite(Button_Color_Blue, LOW);
     ledStateLeft = 16; ledStateRight = 16; // ORANGE: battery warning
     //delay(20); alarm = true;
     myMP3.loop(3);
     delay(100);
    }
  if ((analog_BatteryWatcher_Red<500) && (analog_BatteryWatcher_Green>500) && (analogRead(voltage)>500)){ // e-stop active
     digitalWrite(Button_Color_Red, HIGH); digitalWrite(Button_Color_Blue, LOW);
     if (alarm==true){myMP3.play(9); alarm = false;}
     ledStateLeft = 1; ledStateRight = 1;
     delay(100);
    }
  switch (ledStateLeft) { // LED LEFT
    case 1:
      flashingLeft();//flash RED (gentle-stop / emergency) -- accessibility: never rely on solid red/green alone
      break;
    case 2:
      writeLEDSLeft(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSLeft(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLLeft();
      break;
    case 5:
      newColorWipeLeft();
      break;
    case 6:
      newTheatreChaseLeft();
      break;
    case 7:
      newRainbowLeft();
      break;
    case 8:
      writeLEDSLeft(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowLeft();
      break;
    case 10:
      colorCyclerLeft();
      cylonChaserLeft();
      break;
    case 11:
      newRainbowCycleLeft();
      break;
    case 12:
      colorCyclerLeft();
      breathingLeft();
      break;
    case 13:
      colorCyclerLeft();
      heartbeatLeft();
      break;
    case 14:
      christmasChaseLeft();
      break;
    case 15:
      FireLeft();
      break;
    case 16:
      writeLEDSLeft(255, 140, 0);//write ORANGE to all pixels (battery warning)
      break;
    case 17:
      writeLEDSLeft(255, 255, 0);//write YELLOW to all pixels (paused/standby)
      break;
    case 18:
      writeLEDSLeft(255, 255, 255);//write WHITE to all pixels (state machine not active yet)
      break;
    case 19:
      activeColorLeft[0] = 255; activeColorLeft[1] = 255; activeColorLeft[2] = 255;
      breathingLeft(); //pulse WHITE (motors booting)
      break;
    case 20:
      writeLEDSLeft(255, 60, 0);//write RED-ORANGE to all pixels (motor driver crashed / boot failed)
      break;
    case 21:
      activeColorLeft[0] = 150; activeColorLeft[1] = 0; activeColorLeft[2] = 255;
      breathingLeft(); //pulse PURPLE (arm searching/scanning)
      break;
    case 22:
      TCColorLeft = 4;
      newTheatreChaseLeft(); //chase YELLOW (arm approaching/executing)
      break;
    case 23:
      flashingLeft(); //flash RED (grasp failed/slipped/unreachable)
      break;
    case 24:
      flashingGreenLeft(); //flash GREEN (verified grasp success)
      break;
    case 25:
      writeLEDSLeft(0, 255, 255); //write CYAN (arm under manual/joystick remote control)
      break;

    default:
      writeLEDSLeft(0, 0, 0); //sets all the pixels to off;
      break;
  }
  switch (ledStateRight) { // LED RIGHT
    case 1:
      flashingRight();//flash RED (gentle-stop / emergency) -- accessibility: never rely on solid red/green alone
      break;
    case 2:
      writeLEDSRight(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSRight(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLRight();
      break;
    case 5:
      newColorWipeRight();
      break;
    case 6:
      newTheatreChaseRight();
      break;
    case 7:
      newRainbowRight();
      break;
    case 8:
      writeLEDSRight(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowRight();
      break;
    case 10:
      colorCyclerRight();
      cylonChaserRight();
      break;
    case 11:
      newRainbowCycleRight();
      break;
    case 12:
      colorCyclerRight();
      breathingRight();
      break;
    case 13:
      colorCyclerRight();
      heartbeatRight();
      break;
    case 14:
      christmasChaseRight();
      break;
    case 15:
      FireRight();
      break;
    case 16:
      writeLEDSRight(255, 140, 0);//write ORANGE to all pixels (battery warning)
      break;
    case 17:
      writeLEDSRight(255, 255, 0);//write YELLOW to all pixels (paused/standby)
      break;
    case 18:
      writeLEDSRight(255, 255, 255);//write WHITE to all pixels (state machine not active yet)
      break;
    case 19:
      activeColorRight[0] = 255; activeColorRight[1] = 255; activeColorRight[2] = 255;
      breathingRight(); //pulse WHITE (motors booting)
      break;
    case 20:
      writeLEDSRight(255, 60, 0);//write RED-ORANGE to all pixels (motor driver crashed / boot failed)
      break;
    case 21:
      activeColorRight[0] = 150; activeColorRight[1] = 0; activeColorRight[2] = 255;
      breathingRight(); //pulse PURPLE (arm searching/scanning)
      break;
    case 22:
      TCColorRight = 4;
      newTheatreChaseRight(); //chase YELLOW (arm approaching/executing)
      break;
    case 23:
      flashingRight(); //flash RED (grasp failed/slipped/unreachable)
      break;
    case 24:
      flashingGreenRight(); //flash GREEN (verified grasp success)
      break;
    case 25:
      writeLEDSRight(0, 255, 255); //write CYAN (arm under manual/joystick remote control)
      break;

    default:
      writeLEDSRight(0, 0, 0); //sets all the pixels to off;
      break;
  }
  /*
  switch (ledStateMiddle1) { // LED MIDDLE1
    case 1:
      writeLEDSMiddle1(255, 0, 0);//write RED to all pixels
      Serial.println("writeLEDSMiddle1ROT");
      break;
    case 2:
      writeLEDSMiddle1(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSMiddle1(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLMiddle1();
      break;
    case 5:
      newColorWipeMiddle1();
      break;
    case 6:
      newTheatreChaseMiddle1();
      break;
    case 7:
      newRainbowMiddle1();
      break;
    case 8:
      writeLEDSMiddle1(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowMiddle1();
      break;
    case 10:
      colorCyclerMiddle1();
      cylonChaserMiddle1();
      break;
    case 11:
      newRainbowCycleMiddle1();
      break;
    case 12:
      colorCyclerMiddle1();
      breathingMiddle1();
      break;
    case 13:
      colorCyclerMiddle1();
      heartbeatMiddle1();
      break;
    case 14:
      christmasChaseMiddle1();
      break;
    case 15:
      FireMiddle1();
      break;
    default:
      writeLEDSMiddle1(0, 0, 0); //sets all the pixels to off;
      break;
  }
  */
  switch (ledStateMiddle2) { // LED MIDDLE2
    case 1:
      writeLEDSMiddle2(255, 0, 0);//write RED to all pixels
      //Serial.println("writeLEDSMiddle2ROT");
      break;
    case 2:
      writeLEDSMiddle2(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSMiddle2(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLMiddle2();
      break;
    case 5:
      newColorWipeMiddle2();
      break;
    case 6:
      newTheatreChaseMiddle2();
      break;
    case 7:
      newRainbowMiddle2();
      break;
    case 8:
      writeLEDSMiddle2(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowMiddle2();
      break;
    case 10:
      colorCyclerMiddle2();
      cylonChaserMiddle2();
      break;
    case 11:
      newRainbowCycleMiddle2();
      break;
    case 12:
      colorCyclerMiddle2();
      breathingMiddle2();
      break;
    case 13:
      colorCyclerMiddle2();
      heartbeatMiddle2();
      break;
    case 14:
      christmasChaseMiddle2();
      break;
    case 15:
      FireMiddle2();
      break;
    default:
      writeLEDSMiddle2(0, 0, 0); //sets all the pixels to off;
      break;
  }
}
