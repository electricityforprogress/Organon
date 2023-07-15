//Electricity for Progress
// electricityforprogress.com
// store - Biodata - Modules - Custom
// **-----------------------------------------------------------------------------
/*
   Biodata Modular
   ProMicro 32u4 at 5v with
   mcp4728 Quad 12bit DAC
   555 timer Biodata input
   Three Gate, Three Note CV
   PWM Raw Out, Control CV
   MIDI Output
   Electrode Input

Features
1. Button and Knob handling
2. Menus - Note Scaling, MIDI Channel, Brightness, MIDI CC Change
3. Biodata input
4. Scaling Notes
5. Tuning/adjust? ?? tracking 0-5v, make it easy linear map(); or other algorithm

   
*/
// **-----------------------------------------------------------------------------

// DAC declaration Quad output
#include <Adafruit_MCP4728.h>
#include <Wire.h>

Adafruit_MCP4728 mcp;

//save settings to EEPROM for Scale, MIDI channel, poly or 'qy8 mode', bright,
#include <EEPROM.h>
#define EEPROM_SIZE 5 
  
//MIDI Note and Controls
const byte polyphony = 3; // 1; //mono  // number of notes to track at a given time

byte channel = 1; //MIDI channel can be set using menus

//******************************
//set scaled values, sorted array, first element scale length
//the whole scaling algorithm needs to be refactored ;)
//int scaleDiaMinor[]  = {7, 0, 2, 3, 5, 7, 8, 10};
int scalePenta[]  = {5, 0, 3, 5, 7, 9};
int scaleMajor[]  = {7, 0, 2, 4, 5, 7, 9, 11};
int scaleIndian[]  = {7, 0, 1, 1, 4, 5, 8, 10};
int scaleMinor[]  = {7, 0, 2, 3, 5, 7, 8, 10};
int scaleChrom[] = {13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
     //enter the scale name here to use
int *scaleSelect = scaleChrom; //initialize scaling
byte defScale = 3; 
//   if(scaleIndex == 0) scaleSelect = scaleChrom; 
//   if(scaleIndex == 1) scaleSelect = scaleMinor; 
//   if(scaleIndex == 2) scaleSelect = scaleMajor; 
//   if(scaleIndex == 3) scaleSelect = scalePenta; 
//   if(scaleIndex == 4) scaleSelect = scaleIndian; 
int root = 0; //initialize for root
//*******************************

//Debug and MIDI output Settings ********
byte debugSerial = 1; //debugging serial messages
byte rawSerial = 1; // raw biodata stream via serial data
bool rawToggle = 0; //toggle for raw data output
bool prevToggle = 0; //actual value toggling

byte serialMIDI = 1; //write serial data to MIDI hardware output

byte midiMode = 0;  //change mode for serial, ble, wifi, usb

int noteMin = 36; //C2  - keyboard note minimum
int noteMax = 96; //C7  - keyboard note maximum
byte controlNumber = 80; //set to mappable control, low values may interfere with other soft synth controls!!

unsigned long rawSerialTime = 0;
int rawSerialDelay = 0;
// **************************************

#include <EEPROM.h>
#define EEPROM_SIZE 5 // scaleindex, midi channel, wifi, bluetooth, key

// I/O Pin declarations

#define interruptPin INT2  //biodata input pin
byte buttonPin = 15;   //for some reason it needed to be a byte, sigh
#define potPin A0
#define gate1Pin 16
#define gate2Pin 4
#define gate3Pin 7
#define rawBio 8  //not used 
#define led1Pin 10
#define led2Pin 5
#define led3Pin 6
#define led4Pin 9


//Potentiometer management
int knobValue = 0;
int knobPrev = 0;
unsigned long knobPrevTime = 0;
int knobTime = 300; //read pot every x milliseconds

// LED definition
#define ledCount 4
byte leds[ledCount] = {10,5,6,9};
byte maxBrightness = 50; //multipler for brightness
//8,5,1,8 -->  map(maxBrightness,0,100,
byte ledBrightness[ledCount] = {50,50,80,60};   //{80,50,10,80};  //adjust max per colour
//
unsigned long blinkTime = 0;
bool blinkToggle = 0;

//fading leds asynchronously
/*
 * pinNumber, maxBright,
 * currentLevel, destinationLevel, startTime, 
 * duration, stepSize, stepTime, isRunning
 * set stepSize as rate of fade based on maxBrightness and duration
 *      stepSize = duration/destinationLevel
 * if isRunning
 *   if currTime-startTime<duration //fade running
 *     if currTime=stepTime>stepSize
 *       stepTime=currTime
 *       if curr<dest, curr++
 *       if curr>dest, cur--
 *   else isRunning = FALSE //fade ended
 */
class samFader {
  
  public:
  byte pinNumber;
  byte espPWMchannel;
  int maxBright;
  int currentLevel = 0;
  int destinationLevel = 0;
  unsigned long startTime; //time at which a fade begins
  int duration = 0;
  unsigned long stepSize;  //computed duration across brightness 
  unsigned long stepTime; //last time a step was called
  bool isRunning = 0;

  samFader(byte pin, byte pwmChannel, byte maxB) {
    pinNumber=pin;
    espPWMchannel=pwmChannel;
    maxBright=maxB;
  }

  void Set(int dest, int dur) {
    startTime = millis();
    destinationLevel = dest;
    duration = dur;
    stepTime = 0;
    int difference = abs(destinationLevel - currentLevel);
    stepSize = duration/(difference+1);
    isRunning = 1;
    if(dur == 0) { // do it right away!
    //  ledcWrite(espPWMchannel, dest);  //this change is for ESP32 or other AVRs
      analogWrite(pinNumber,destinationLevel);
      isRunning = 0;
    }
  }
  
  void Update() {
    if(isRunning) {
      if(stepTime+stepSize<millis()) {
        //update to the next level
        if(currentLevel>destinationLevel) currentLevel--;
        if(currentLevel<destinationLevel) currentLevel++;
        //update variables for tracking
        stepTime = millis();
        //write brightness to LED
        //ledcWrite(espPWMchannel, currentLevel);
        analogWrite(pinNumber,currentLevel);
      }
      if(startTime+duration<millis()) {
     //   ledcWrite(espPWMchannel, destinationLevel);
        analogWrite(pinNumber,destinationLevel);
        isRunning = 0;
      }
    }

  }

  void Setup(byte chan) {
   // ledcSetup(chan,5000,13);    //ESP32 setup
   // ledcAttachPin(pinNumber,chan);  //ESP32 setup
   
  }
  
};

samFader ledFaders[] = {  samFader(leds[0],0,ledBrightness[0]),
                          samFader(leds[1],1,ledBrightness[1]),
                          samFader(leds[2],2,ledBrightness[2]),
                          samFader(leds[3],3,ledBrightness[3])
};



//Timing and tracking
unsigned long currentMillis = 0;
unsigned long prevMillis = 0;


//****** sample size sets the 'grain' of the detector
// a larger size will smooth over small variations
// a smaller size will excentuate small changes
const byte samplesize = 10; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array,

volatile unsigned long microseconds; //sampling timer
volatile byte sampleIndex = 0;
volatile unsigned long samples[samplesize];

float threshold = 1.71; //threshold multiplier
float threshMin = 1.11; //1.61; //scaling threshold min
float threshMax = 4.01  ; //scaling threshold max
float prevThreshold = 0;



typedef struct _MIDImessage { //build structure for Note and Control MIDImessages
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
}
MIDImessage;
MIDImessage noteArray[polyphony]; //manage MIDImessage data as an array with size polyphony
int noteIndex = 0;
MIDImessage controlMessage; //manage MIDImessage data for Control Message (CV out)

//setups for each MIDI type, provide led display output
void setupSerialMIDI() {
   if (debugSerial) Serial.println("MIDI set on Serial1 31250");
  Serial1.begin(31250); //MIDI data out TX pins
}

void checkKnob() {
  //manage time
  currentMillis = millis();
  
  threshold = analogRead(potPin); 

  prevThreshold = threshold; //remember last value
  //set threshold to knobValue mapping
  threshold = mapfloat(threshold, 0, 1023, threshMin, threshMax);


    knobValue = analogRead(potPin);
    
  //fance timed code to poll the pot less frequently
    if(knobPrevTime + knobTime > currentMillis) {
      knobPrevTime = currentMillis;
      //smooth the knob readings with .01uf to GND
      knobPrev = knobValue;
        
      knobValue = analogRead(potPin);
    
    if(knobValue > knobPrev) {  } //do something
  }

}

//provide float map function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//samButton2 code for simple button mangement .. still incomplete

class samButton {
  public:

    unsigned int doubleClickTime = 300; //milliseconds within which a double click event could occur
    unsigned int debounce = 35; //debounce time
    bool changed = false; //short lived indicator that a change happened, will see after an update call

    samButton(byte buttonPin, bool pUp); //constructor

    void begin(); // initialize button pin with pullup
    void update(); // update button and led
    bool read(); // read current button state 'raw' value
    unsigned long changedTime(); // returns last changed time
    bool pressed(); // is the button currently pressed?
    bool wasPressed(); // was the button just depressed?
    bool wasReleased(); // was the button just released?
    bool longPress(); // if(read() && currentMillis - pressStart > _longTime)
    bool doubleClick(); // if(_clickCount == 2)

  private:
    unsigned int _buttonPin;
    unsigned int _buttonIndex = 0;
    bool _pullup = 1; // resistor pullup = 1
    unsigned int _clickTime = 100; //time to detect true click
    unsigned int _longTime = 1000; //time to detect long press
    unsigned int _clickCount; // number of times button was clicked within a double click time
    bool _state = 0;
    bool _prevState = 0;
    unsigned long _changeTime = 0; //denotes change after toggle or update to button state
    unsigned long _prevDebounce = 0; // previous debounce toggle in real time
};



//****************
// Simple Button debouncing
//****************
//#include "samButton.h"

samButton::samButton(byte buttonPin, bool pUp) {
  _buttonPin = buttonPin;
  _pullup = pUp;
}




void samButton::begin() {
  if (_pullup == 1) pinMode(_buttonPin, INPUT_PULLUP); //setup button input
  else pinMode(_buttonPin, INPUT); // no pullup
  _state = !digitalRead(_buttonPin);
  //set button prev state
  _prevState = _state;
  //set button time
  _prevDebounce = millis();
  //change time is current time
  _changeTime = _prevDebounce;
  //change is false
  changed = 0;
}

void samButton::update() {
  unsigned long milli = millis(); //get current time

  //bool reading = MPR121.getTouchData();  //
  bool reading = !digitalRead(_buttonPin);  //get current reading
  if (reading != _prevState) { // if current reading is change from last secure state
    _prevState = reading; // reset bouncing reading
    _prevDebounce = milli; // update bouncing time
  }

  if (milli - _prevDebounce > debounce) { //evaulate debounce time
    _prevDebounce = milli; //reset time
    if (reading != _state) { // debounced and changed
      changed = 1;
      _state = reading;
      _prevState = _state;
      _changeTime = milli;
    }
  }
  else changed = 0; //has not changed or still under debounce time
  return _state;
}

bool samButton::read() {
  return !digitalRead(_buttonPin); ;
}


bool samButton::pressed() { //returns true if the button is pressed currently
  return _state;
}

unsigned long samButton::changedTime() {  // used to determine how long button has been in current state
  return _changeTime;
}

bool samButton::wasPressed() {
  return changed && _state; // if both just changed and button is pressed
}

bool samButton::wasReleased() {
  return changed && !_state;
}

//Button declaration
samButton button(buttonPin, 1); //create button with internal pullup



void checkButton() {
//update the button and evaluate menu modes
// **Bug** crash after scale change when notes are running 
    byte modeValue = 0;
    button.update();
  

  if(button.wasPressed()) {   //first button 'was pressed'
       if (debugSerial) {
          Serial.println("---***---***---ButtonClick***---***---***");
          Serial.print("MIDI Channel "); Serial.println(channel);
          Serial.print("ScaleIndex "); Serial.println(EEPROM.read(0));
          Serial.print("Threshold "); Serial.println(threshold);
          Serial.print("Note Scale: "); Serial.println(modeValue);
       }

        unsigned long menuTimer = millis();
        int menu = 1; //main biodata play mode
        int prevMenu = 0;

          //****could be the cause of flickering
          for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off
        while(menuTimer + 10000 > millis()) {
            //manage time
            currentMillis = millis();
            //turn on Menu LED
               //blink led during selection
               if((blinkTime ) < millis()) { blinkToggle = !blinkToggle; blinkTime = millis(); }   
               if(blinkToggle) ledFaders[menu].Set(ledFaders[menu].maxBright,0);
                        
            //reset timer
            menuTimer = millis();
           
           //check for click and enter Menu mode
             button.update();
             modeValue = 0;
             if(button.wasPressed()) { //second 'was pressed'
                if (debugSerial) { Serial.print("Enter Menu "); 
                Serial.println(menu); }

                //****could be the cause of flickering
                for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off

                menuTimer = millis();
                int prevModeValue = 0;
                while(menuTimer + 20000 > millis()){
                    //Loop for management of menu mode selection
                    //main biodata routine does not run during menu selection

                      //manage time
                    currentMillis = millis();
                    //check the button for clicks
                    button.update();

                    prevModeValue = modeValue;
                    //knob turn to extend time and select menu???
                    checkKnob();
    
                      if(menu == 1) { // MIDI Scaling
                        //only 4 modes 0-3 on modular version (could do more similar to binary)
                       modeValue = map(knobValue, 0, 1023, 0, 4);
                        //****could be the cause of flickering
                          if(modeValue != prevModeValue) {
                            Serial.print("ModeValue Change "); Serial.println(modeValue);
                            for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off
                          }
                       //blink led during selection
                       if((blinkTime + 250) < millis()) { blinkToggle = !blinkToggle; blinkTime = millis(); }   
                       if(blinkToggle) ledFaders[modeValue].Set(ledFaders[modeValue].maxBright,0);
                       if(menu!=modeValue) ledFaders[menu].Set(ledFaders[menu].maxBright,0); //red0
                       
                       if(modeValue == 0) scaleSelect = scaleChrom; 
                       if(modeValue == 1) scaleSelect = scaleMinor; 
                       if(modeValue == 2) scaleSelect = scaleMajor; 
                       if(modeValue == 3) scaleSelect = scalePenta; 
                       //if(modeValue == 4) scaleSelect = scaleIndian; 
                       
                      }
                      

                     //Menu has been entered, value has been selected
                     //click to save a value from the submenu       
                      if(button.wasPressed()) { ///select value sub menu
                        //light show fast flash
                        for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off
                        ledFaders[menu].Set(ledFaders[menu].maxBright,0);
                        delay(75);
                        ledFaders[menu].Set(0,0);
                        delay(75);
                        ledFaders[menu].Set(ledFaders[menu].maxBright,0);
                        delay(75);
                        ledFaders[menu].Set(0,0);
                        delay(75);
                        ledFaders[menu].Set(ledFaders[menu].maxBright,0);
                        delay(75);
                        ledFaders[menu].Set(0,0);
                        
                        for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off

                        if(menu == 1){
                         // EEPROM.update(0,modeValue);
                         // EEPROM.commit();
                          if (debugSerial) { Serial.print("MIDI Scale "); Serial.println(modeValue); }
                        }
                        
                     //   EEPROM.commit(); //save all the values!!

                        return; //break; //save here              
                      }

                      //unsure if button clicks get here...shouldn't tho!
                   if(button.wasPressed()) {
                     for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off
                     if (debugSerial) Serial.println("shouldnt get here in menus?");
                     return; //save here
                   }
 
                }
            
             } // end second button 'was pressed'
        } //while in menu loop
        
        for(byte i=0;i<ledCount;i++) { ledFaders[i].Set(0,0); } //all off
  } //end first button 'was pressed'
  //if double, if long click, etc
  
} //end checkButton
