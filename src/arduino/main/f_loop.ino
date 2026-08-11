void loop() {
  pixelsLinks.setBrightness(brightnessLinks); // sets the inital brightness of the neopixels
  pixelsRechts.setBrightness(brightnessRechts); // sets the inital brightness of the neopixels
  int analog_Battarieweachter_GRUEN = analogRead(Battarieweachter_GRUEN);
  int analog_Battarieweachter_ROT = analogRead(Battarieweachter_ROT);
  int analog_voltage = analogRead(voltage);
  //Serial.println(analog_voltage);
  delay(50);
  if ((analog_Battarieweachter_ROT>500) && (analog_Battarieweachter_GRUEN<500)){ // Abschaltung
    digitalWrite(Taster_Farbe_Rot, HIGH); digitalWrite(Taster_Farbe_Gruen, LOW); digitalWrite(Taster_Farbe_Blau, LOW); 
    zahla = 1; zahlb = 1;
    delay(100);
     }
  if ((analog_Battarieweachter_GRUEN>500) && (analog_Battarieweachter_ROT<500) && (analog_voltage<500)){ // kein Alarm
     digitalWrite(Taster_Farbe_Rot, LOW); digitalWrite(Taster_Farbe_Blau, HIGH);
     myMP3.stop();
     zahla = 11; zahlb = 11;
     if (alarm==false){myMP3.play(8); alarm = true;}
     delay(100);
     }
  if ((analog_Battarieweachter_ROT>500) && (analog_Battarieweachter_GRUEN>500) && (analog_voltage<500)){ // battarie Warnung
     digitalWrite(Taster_Farbe_Rot, HIGH); digitalWrite(Taster_Farbe_Gruen, LOW); digitalWrite(Taster_Farbe_Blau, LOW); 
     zahla = 3; zahlb = 3; 
     //delay(20); alarm = true;
     myMP3.loop(3);
     delay(100);
    }
  if ((analog_Battarieweachter_ROT<500) && (analog_Battarieweachter_GRUEN>500) && (analogRead(voltage)>500)){ // NotAUS aktiv
     digitalWrite(Taster_Farbe_Rot, HIGH); digitalWrite(Taster_Farbe_Blau, LOW); 
     if (alarm==true){myMP3.play(9); alarm = false;}
     zahla = 1; zahlb = 1;
     delay(100);
    }
  switch (zahla) { // LED LINKS
    case 1:
      writeLEDSLinks(255, 0, 0);//write RED to all pixels
      break;
    case 2:
      writeLEDSLinks(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSLinks(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLLinks();
      break;
    case 5:
      newColorWipeLinks();
      break;
    case 6:
      newTheatreChaseLinks();
      break;
    case 7:
      newRainbowLinks();
      break;
    case 8:
      writeLEDSLinks(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowLinks();
      break;
    case 10:
      colorCyclerLinks();
      cylonChaserLinks();
      break;
    case 11:
      newRainbowCycleLinks();
      break;
    case 12:
      colorCyclerLinks();
      breathingLinks();
      break;
    case 13:
      colorCyclerLinks();
      heartbeatLinks();
      break;
    case 14:
      christmasChaseLinks();
      break;
    case 15:
      FireLinks();
      break;

    default:
      writeLEDSLinks(0, 0, 0); //sets all the pixels to off;
      break;
  }
  switch (zahlb) { // LED RECHTS
    case 1:
      writeLEDSRechts(255, 0, 0);//write RED to all pixels
      break;
    case 2:
      writeLEDSRechts(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSRechts(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLRechts();
      break;
    case 5:
      newColorWipeRechts();
      break;
    case 6:
      newTheatreChaseRechts();
      break;
    case 7:
      newRainbowRechts();
      break;
    case 8:
      writeLEDSRechts(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowRechts();
      break;
    case 10:
      colorCyclerRechts();
      cylonChaserRechts();
      break;
    case 11:
      newRainbowCycleRechts();
      break;
    case 12:
      colorCyclerRechts();
      breathingRechts();
      break;
    case 13:
      colorCyclerRechts();
      heartbeatRechts();
      break;
    case 14:
      christmasChaseRechts();
      break;
    case 15:
      FireRechts();
      break;

    default:
      writeLEDSRechts(0, 0, 0); //sets all the pixels to off;
      break;
  }
  /*
  switch (zahlc) { // LED MITTE1
    case 1:
      writeLEDSMitte1(255, 0, 0);//write RED to all pixels
      Serial.println("writeLEDSMitte1ROT");
      break;
    case 2:
      writeLEDSMitte1(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSMitte1(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLMitte1();
      break;
    case 5:
      newColorWipeMitte1();
      break;
    case 6:
      newTheatreChaseMitte1();
      break;
    case 7:
      newRainbowMitte1();
      break;
    case 8:
      writeLEDSMitte1(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowMitte1();
      break;
    case 10:
      colorCyclerMitte1();
      cylonChaserMitte1();
      break;
    case 11:
      newRainbowCycleMitte1();
      break;
    case 12:
      colorCyclerMitte1();
      breathingMitte1();
      break;
    case 13:
      colorCyclerMitte1();
      heartbeatMitte1();
      break;
    case 14:
      christmasChaseMitte1();
      break;
    case 15:
      FireMitte1();
      break;
    default:
      writeLEDSMitte1(0, 0, 0); //sets all the pixels to off;
      break;
  }
  */
  switch (zahld) { // LED MITTE2
    case 1:
      writeLEDSMitte2(255, 0, 0);//write RED to all pixels
      //Serial.println("writeLEDSMitte2ROT");
      break;
    case 2:
      writeLEDSMitte2(0, 255, 0);//write GREEN to all pixels
      break;
    case 3:
      writeLEDSMitte2(0, 0, 255);//write BLUE to all pixels
      break;
    case 4:
      ALLMitte2();
      break;
    case 5:
      newColorWipeMitte2();
      break;
    case 6:
      newTheatreChaseMitte2();
      break;
    case 7:
      newRainbowMitte2();
      break;
    case 8:
      writeLEDSMitte2(85, 85, 85);
      break;
    case 9:
      newTheatreChaseRainbowMitte2();
      break;
    case 10:
      colorCyclerMitte2();
      cylonChaserMitte2();
      break;
    case 11:
      newRainbowCycleMitte2();
      break;
    case 12:
      colorCyclerMitte2();
      breathingMitte2();
      break;
    case 13:
      colorCyclerMitte2();
      heartbeatMitte2();
      break;
    case 14:
      christmasChaseMitte2();
      break;
    case 15:
      FireMitte2();
      break;
    default:
      writeLEDSMitte2(0, 0, 0); //sets all the pixels to off;
      break;
  }
}
