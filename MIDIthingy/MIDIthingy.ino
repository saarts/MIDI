/*
 *
 * Created: 9/Nov/2018
 *
 */ 

#include "MIDIUSB.h"

// Event type (0x09 = note on, 0x08 = note off, 0x0B = control change CC, 0x0C = program change  ).
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Pitch is the note number (48 = middle C).
// Velocity (64 = normal, 127 = fastest).
// Control is the CC command number.

char states[7];       //Array for current button states
char prevstates[7];   //Array for previous button states


void noteon(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteon = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteon);
}

void noteoff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteoff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteoff);
}

void programchange(byte channel, byte program) {
  midiEventPacket_t change = {0x0C, 0xC0 | channel, program};
  MidiUSB.sendMIDI(change);
}


void controlchange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void setup() {
  Serial.begin(115200);   //I do not think I actually used Serial in this code... lol

  //Setting inputpins
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
}

void loop() {
  char i=0;

//Reading input states of the buttons
//I think this can also be for looped
  states[0]=digitalRead(2);
  states[1]=digitalRead(3);
  states[2]=digitalRead(4);
  states[3]=digitalRead(5);
  states[4]=digitalRead(6);
  states[5]=digitalRead(7);
  states[6]=digitalRead(8);


//I use first 4 buttons from left to right for CC commands
  for(i=0;i<4;i++){

    //Compare previous state
  if(states[i]!= prevstates[i]){

    //If state is LOW, because of Pullup
      if(states[i]== 0){
       controlchange(0, 17+i, 100);
       MidiUSB.flush();
      }    
    prevstates[i]=states[i];
    delay(50); //Button debounce
    }
  }

//And last 3 buttons for ProgramChange commands
  for(i=4;i<7;i++){
  if(states[i]!= prevstates[i]){
    if(states[i]== 0){
       programchange(0, i);
       MidiUSB.flush();
    }
    prevstates[i]=states[i];
    delay(50); //Button debounce
    }
  }
 
  
}
