void setup() {
  // put your setup code here, to run once:
  pixelsLeft.begin(); //starts the neopixels
  pixelsRight.begin(); //starts the neopixels
  pixelsMiddle2.begin(); //starts the neopixels
  writeLEDSLeft(0, 0, 0); //sets all the pixels to off
  writeLEDSRight(0, 0, 0); //sets all the pixels to off
  writeLEDSMiddle2(0, 0, 0); //sets all the pixels to off
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
  pinMode(Button_Color_Red, OUTPUT);
  pinMode(Button_Color_Green, OUTPUT);
  pinMode(Button_Color_Blue, OUTPUT);

  pinMode(BatteryWatcher_Red, INPUT);
  pinMode(BatteryWatcher_Green, INPUT);

  
  digitalWrite(Button_Color_Red, LOW);
  digitalWrite(Button_Color_Green, LOW);
  digitalWrite(Button_Color_Blue, LOW);
  delay(1000);
}
