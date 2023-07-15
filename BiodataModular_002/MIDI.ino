

void setNote(int value, int velocity, long duration, int notechannel)
{
  //find available note in array (velocity = 0);
  for(int i=0;i<polyphony;i++){
    if(!noteArray[i].velocity){
      //if velocity is 0, replace note in array
      noteArray[i].type = 0;
      noteArray[i].value = value;
      noteArray[i].velocity = velocity;
      noteArray[i].duration = currentMillis + duration;
      noteArray[i].channel = notechannel;
      
//*************
      if(serialMIDI) midiSerial(144, channel, value, velocity); //play note

//set gates
        if(i==0) digitalWrite(gate1Pin, HIGH);
        if(i==1) digitalWrite(gate2Pin, HIGH);
        if(i==2) digitalWrite(gate3Pin, HIGH);
//set Note Voltage from oldcrow https://modwiggler.com/forum/viewtopic.php?t=159413
//        int noteVolt = value * 273; //(note - Transpose) * (4095/60)
//        noteVolt = (X>>2)+((x>>1)& 1); //round to nearest integer
//   could have a lookup table of notes to voltages?
//what about calibration... hmm ... just need to be equal tempered
        int noteVolt = map(value, noteMin, noteMax, 0.0, 4095.0); //or just use map?
        
        if(i==0) mcp.setChannelValue(MCP4728_CHANNEL_A, noteVolt);
        if(i==1) mcp.setChannelValue(MCP4728_CHANNEL_B, noteVolt);
        if(i==2) mcp.setChannelValue(MCP4728_CHANNEL_C, noteVolt);
        
        //setLED
        ledFaders[i].Set(ledFaders[i].maxBright,350);
      
      break; // exit for loop and continue once new note is placed
    }
  }
}

void setControl(int type, int value, int velocity, long duration)
{
  controlMessage.type = type;
  controlMessage.value = value;
  controlMessage.velocity = velocity; //MIDI CC value, just reusing functions ;)
  controlMessage.period = duration;
  controlMessage.duration = currentMillis + duration; //schedule for update cycle
//  if(debugSerial) { 
//    Serial.print("setControl CC"); Serial.print(controlNumber); Serial.print(" value "); Serial.println(velocity); 
//  }

}


void checkControl()
{
  //need to make this a smooth slide transition, using high precision 
  //distance is current minus goal
  signed int distance =  controlMessage.velocity - controlMessage.value; 
  //if still sliding
  if(distance != 0) {
    //check timing
    if(currentMillis>controlMessage.duration) { //and duration expired
        controlMessage.duration = currentMillis + controlMessage.period; //extend duration
        //update value
       if(distance > 0) { controlMessage.value += 1; } else { controlMessage.value -=1; }
       
       //send MIDI control message after ramp duration expires, on each increment
//*************
      //determine serial, wifi, or ble
       if(serialMIDI) midiSerial(176, channel, controlMessage.type, controlMessage.value); 

     //update control CV output -- Could increase granularity here by sending a larger value
     //     from the change algorithm and the map down to a MIDI CC lower grain value... 
     mcp.setChannelValue(MCP4728_CHANNEL_D, map(controlMessage.value,0,127,0, 4095));
    
     //use LED to display CV value
     //analogWrite(led4Pin, map(controlMessage.value,0,127,0,ledBrightness[3]));
     ledFaders[3].Set(map(controlMessage.value,0,127,0,ledFaders[3].maxBright),0);

    }
  }
}

void checkNote()
{
  for (int i = 0;i<polyphony;i++) {
    if(noteArray[i].velocity) {
      long durr = noteArray[i].duration;

      //*** this won't work for short notes....      
      if((noteArray[i].duration-300 <= currentMillis) && (ledFaders[i].destinationLevel!=0)) {
        //update LEDs a couple mS before the end of the note, only do it once
        ledFaders[i].Set(0,400);//turn off leds fade out
      }
      if (noteArray[i].duration <= currentMillis) {
        //send noteOff for all notes with expired duration    
        if(serialMIDI) midiSerial(144, channel, noteArray[i].value, 0); 

        //set Gate to OFF
        if(i==0) digitalWrite(gate1Pin, LOW);
        if(i==1) digitalWrite(gate2Pin, LOW);
        if(i==2) digitalWrite(gate3Pin, LOW);
                                
        noteArray[i].velocity = 0;
        //ledFaders[i].Set(0,800);//turn off leds fade out

        
//        if (debugSerial) { 
//           Serial.print("Note "); Serial.print(i); Serial.print(" Off: "); 
//          Serial.println(noteArray[i].value);
//        }
      }
    }
  }

}


//Control different MIDI output methods - message type, channel, note/number, velcoity/value
void midiSerial(int type, int channel, int data1, int data2) {

//Hardware Serial 31250 MIDI data
  if(serialMIDI) {
    //  Note type = 144
    //  Control type = 176  
    // remove MSBs on data
    data1 &= 0x7F;  //number
    data2 &= 0x7F;  //velocity
    byte statusbyte = (type | ((channel-1) & 0x0F));
    Serial1.write(statusbyte);
    Serial1.write(data1);
    Serial1.write(data2);
  }
}




int scaleSearch(int note, int scale[], int scalesize) {
 for(byte i=1;i<scalesize;i++) {
  if(note == scale[i]) { return note; }
  else { if(note < scale[i]) { return scale[i]; } } //highest scale value less than or equal to note
  //otherwise continue search
 }
 //didn't find note and didn't pass note value, uh oh!
 return 6;//give arbitrary value rather than fail
}


int scaleNote(int note, int scale[], int root) {
  //input note mod 12 for scaling, note/12 octave
  //search array for nearest note, return scaled*octave
  int scaled = note%12;
  int octave = note/12;
  int scalesize = (scale[0]);
  //search entire array and return closest scaled note
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + root; //apply octave and root

  return scaled;
}
