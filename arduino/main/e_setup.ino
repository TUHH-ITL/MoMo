void setup() {
  // put your setup code here, to run once:
  pixelsLinks.begin(); //starts the neopixels
  pixelsRechts.begin(); //starts the neopixels
  pixelsMitte2.begin(); //starts the neopixels
  writeLEDSLinks(0, 0, 0); //sets all the pixels to off
  writeLEDSRechts(0, 0, 0); //sets all the pixels to off
  writeLEDSMitte2(0, 0, 0); //sets all the pixels to off
  Serial.begin(9600);
  mySerial.begin(9600);
  myMP3.begin(mySerial, true);
  if (!myMP3.begin(mySerial, true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  myMP3.volume(30);
  //myMP3.play(8);
  pinMode(Taster_Farbe_Rot, OUTPUT);
  pinMode(Taster_Farbe_Gruen, OUTPUT);
  pinMode(Taster_Farbe_Blau, OUTPUT);

  pinMode(Battarieweachter_ROT, INPUT);
  pinMode(Battarieweachter_GRUEN, INPUT);

  
  digitalWrite(Taster_Farbe_Rot, LOW);
  digitalWrite(Taster_Farbe_Gruen, LOW);
  digitalWrite(Taster_Farbe_Blau, LOW);
  delay(1000);
}
