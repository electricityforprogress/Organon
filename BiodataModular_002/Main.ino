// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void setup() {

  if (debugSerial || rawSerial) Serial.begin(115200); // Serial baud for debugging and raw

 
      //load from EEPROM memory
  //    EEPROM.begin(EEPROM_SIZE);

 //DAC setup
  mcp.begin();
  
  //pinMode(buttonPin, INPUT_PULLUP); //button managed by PinButton
  pinMode(interruptPin, INPUT_PULLUP); //pulse input

  button.begin();

  pinMode(potPin, INPUT);

  pinMode(gate1Pin, OUTPUT);
  pinMode(gate2Pin, OUTPUT);
  pinMode(gate3Pin, OUTPUT);
  pinMode(rawBio, OUTPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
  pinMode(led4Pin, OUTPUT);

     //welcome message
     delay(150);
    if(debugSerial) Serial.println(); Serial.println();
    if(debugSerial) Serial.println(F("Welcome to Biodata Modular"));

    TXLED1; 
    RXLED1;

    //LED light show
    //turn them on
    for(byte i=0;i<ledCount;i++) {
        ledFaders[i].Setup(i);
        ledFaders[i].Set(ledFaders[i].maxBright,500*(i+1)); 
    }
    unsigned long prevMillis = millis();
    //let em fade up over time
    while(prevMillis+1000>millis()) {
        for(byte i=0;i<ledCount;i++) ledFaders[i].Update();
    }
    //turn them off
    for(byte i=0;i<ledCount;i++) {
        ledFaders[i].Set(0,500*(i+1)); 
    }
    //fade out over time
    prevMillis = millis();
    while(prevMillis+3000>millis()) {
        for(byte i=0;i<ledCount;i++) ledFaders[i].Update();
    }
   
//reset device device if button is held 'after light show'
 if(!digitalRead(buttonPin)) {
     if (debugSerial) Serial.println("Button Held at Bootup - Reset!");
     ledFaders[0].Set(255, 1000);
     ledFaders[0].Update();
     while(ledFaders[0].isRunning) {
        ledFaders[0].Update();
     }
     //reset memory - chromatic scale, channel 1, brightness 99%, MIDIcc 80
      //EEPROM.update(0, defScale); EEPROM.update(1, channel); EEPROM.update(2,99); EEPROM.update(3,80);
     // EEPROM.commit();
       //  maxBrightness = 99; //convert a two digit value to a float multiplier
       //  midiCC = 80; // change the MIDI CC wiggly channel
       //  channel = 1;  //declared at top
     //    scaleSelect = scalePenta;

     ledFaders[0].Set(0, 0); //does this set immediately?

}

//read from memory and load
//  byte scaleIndex = EEPROM.read(0);
//  byte midiChannel = EEPROM.read(1);
//  byte wifiPower = EEPROM.read(2);
//  byte blePower = EEPROM.read(3);
//  byte keybyte = EEPROM.read(4);
//  if(keybyte != 1) { //if not initialized first time - Scale,channel,wifi,bluetooth, key
//    //init for millersville
//    //EEPROM.write(0, 0); EEPROM.write(1, 1); EEPROM.write(2,1); EEPROM.write(3,0); EEPROM.write(4,1);
//     //normal init - ble ON, wifi OFF
//    //  EEPROM.update(0, defScale); EEPROM.update(1, channel); EEPROM.update(2,0); EEPROM.update(3,1); EEPROM.update(4,1);
//      //EEPROM.commit();
//      if (debugSerial) Serial.println("EEPROM Initialized - First time!");
//         scaleIndex = EEPROM.read(0);
//         midiChannel = EEPROM.read(1);
//         wifiPower = EEPROM.read(2);
//         blePower = EEPROM.read(3);
//  }
  
//  channel = midiChannel; //need two bytes to hold up to 16 channels!!
  
//                       if(scaleIndex == 0) scaleSelect = scaleChrom; 
//                       if(scaleIndex == 1) scaleSelect = scaleMinor; 
//                       if(scaleIndex == 2) scaleSelect = scaleMajor; 
//                       if(scaleIndex == 3) scaleSelect = scalePenta; 
//                       if(scaleIndex == 4) scaleSelect = scaleIndian; 
                       
  //   wifiMIDI = wifiPower;
  //   bleMIDI = blePower;
  

  if (serialMIDI)  setupSerialMIDI(); // MIDI hardware serial output
  
 // if (wifiMIDI)    setupWifi(); 
 // else { WiFi.disconnect(true); delay(1);   WiFi.mode(WIFI_OFF); delay(1);} //turn wifi radio off
 // if (bleMIDI)     bleSetup();

  //setup pulse input pin
  attachInterrupt(interruptPin, sample, RISING);  //begin sampling from interrupt
  //attachInterrupt(interruptPin, sampleFall, FALLING);  //begin sampling from interrupt

//for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,2000); ledFaders[i].Update(); } //all fade off

} //end setup(){}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void loop() {

// turn off ugly red/green LEDs on proMicro Tx/Rx pins, 
// TXLED0 is ON - TXLED1 is Off
    TXLED1; 
    RXLED1;
    
  //manage time
  currentMillis = millis();
 // MIDI.read();
  
  //analyze data when the buffer is full
  if (sampleIndex >= samplesize)  {
    analyzeSample();
  }

  // Manage MIDI
  checkNote();  //turn off expired notes
  checkControl();  //update control value

  //update Raw output toggle
//  if(rawToggle!=prevToggle) {
//    if(rawToggle) { digitalWriteFast(rawBio, HIGH); digitalWriteFast(led4Pin, HIGH);}
//    else { digitalWriteFast(rawBio, LOW); digitalWriteFast(led4Pin, LOW); }
//    prevToggle = rawToggle;
//  }

  // Mange LEDs
  for(byte i=0;i<ledCount;i++) ledFaders[i].Update();
  
  //Manage pot and button
   checkKnob();  // updates threshold in main biodata mode
  // checkButton();

} //end loop(){}
