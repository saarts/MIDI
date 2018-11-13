/*
 *
 * Created: 13/Nov/2018
 *
 */ 
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#include "MIDIUSB.h"

// Event type (0x09 = note on, 0x08 = note off, 0x0B = control change CC, 0x0C = program change  ).
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Pitch is the note number (48 = middle C).
// Velocity (64 = normal, 127 = fastest).
// Control is the CC command number.


//
//void noteon(byte channel, byte pitch, byte velocity) {
//  midiEventPacket_t noteon = {0x09, 0x90 | channel, pitch, velocity};
//  MidiUSB.sendMIDI(noteon);
//}
//
//void noteoff(byte channel, byte pitch, byte velocity) {
//  midiEventPacket_t noteoff = {0x08, 0x80 | channel, pitch, velocity};
//  MidiUSB.sendMIDI(noteoff);
//}

void programchange(byte channel, byte program) {
  midiEventPacket_t change = {0x0C, 0xC0 | channel, program};
  MidiUSB.sendMIDI(change);
  MidiUSB.flush();
}


void controlchange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}



void setup() {

  //Port B pins on micro
  // PB0 = SS     PCINT0
  // PB1 = SCLK   PCINT1
  // PB2 = MOSI   PCINT2
  // PB3 = MISO   PCINT3
  // PB4 = D8     PCINT4
  // PB5 = D9     PCINT5
  // PB6 = D10    PCINT6
  // PB7 = D11    PCINT7
  
                 // turn off global interrupts  I-bit in the Status Register (SREG)
  DDRB = B00000000;    //Port B inputs
  PORTB = B11111110;   //Port B pullups

  
  PCICR |= B00000001;    // turn on pin change interrupts  

//When the PCIE0 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt 0 is
//enabled. Any change on any enabled PCINT7..0 pin will cause an interrupt. The corresponding interrupt of Pin
//Change Interrupt Request is executed from the PCI0 Interrupt Vector. PCINT7..0 pins are enabled individually
//by the PCMSK0 Register.

  PCMSK0 |= B11111110;  // Choose which pins output interrupt
//Each PCINT7..0 bit selects whether pin change interrupt is enabled on the corresponding I/O pin. If PCINT7..0
//is set and the PCIE0 bit in PCICR is set, pin change interrupt is enabled on the corresponding I/O pin. If
//PCINT7..0 is cleared, pin change interrupt on the corresponding I/O pin is disabled.

  
                   // turn on global interrupts  I-bit in the Status Register (SREG)


  Serial.begin(115200);
}





void loop() {
  //TROLOLOLOOOO
  //Nothing to do here
}




//Routine for pin change interrupt
ISR(PCINT0_vect){ 
//IF button was pressed
uint8_t pinstates = PINB;
  if(CHECK_BIT(pinstates, 1)){
        if(CHECK_BIT(pinstates, 2)){
                  if(CHECK_BIT(pinstates, 3)){
                            if(CHECK_BIT(pinstates, 4)){
                                      if(CHECK_BIT(pinstates, 5)){
                                                if(CHECK_BIT(pinstates, 6)){
                                                          if(CHECK_BIT(pinstates, 7)){
    }else{
      programchange(0, 3);
      }
    }else{
      programchange(0, 2);
      }
    }else{
      programchange(0, 1);
      }
    }else{
      controlchange(0, 20, 100);
      }
    }else{
      controlchange(0, 18, 100);
      }
    }else{
      controlchange(0, 19, 100);
      }
    }else{
      controlchange(0, 17, 100);
      }
  }

 
