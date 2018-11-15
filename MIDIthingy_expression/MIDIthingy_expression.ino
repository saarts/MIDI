/*
 *
 * Created: 13/Nov/2018
 * Modified: 15/Nov/2018
 *
 */ 

//*****************************************************************************//

//#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#include "MIDIUSB.h"

//*****************************************************************************//

//Pin change related
uint8_t pinstates = 0b11111110;
uint8_t dowork = 0;

//software debounce related
uint16_t curmillis = 0;
uint16_t prevmillis = 0;
uint16_t debmillis = 70;

//Analog related
const int threshold = 4;
static int lastpotvalue = 0;
static int lastmappedvalue = 0;

//*****************************************************************************//

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


//*****************************************************************************//

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


//*****************************************************************************//

void setup() {

  //Port B pins on micro
  // PB0 = SS     PCINT0   There is RX LED conneced to this pin, therefore I do not use it. One could remove it from the board.
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
//The 0 bit is 0, because of the RX LED.
  
                   // turn on global interrupts  I-bit in the Status Register (SREG)


  Serial.begin(115200);
}


//*****************************************************************************//


void loop() {


   int currentpotvalue = analogRead(A0);
    if(abs(currentpotvalue - lastpotvalue) > threshold){
    lastpotvalue = currentpotvalue;
    
   int mappedvalue = map(currentpotvalue, 0, 1023, 0, 127);  // Map the value to 0-127
    if(mappedvalue != lastmappedvalue){
       lastmappedvalue = mappedvalue;
       controlchange(0, 21, mappedvalue);
      }
   
    }


if(dowork>0){
  curmillis = millis();

  if((prevmillis-curmillis)>debmillis){
  //If button is pressed, the pin is pulled LOW. For example, if pin 4 is pressed, PINB = 11101111
  //Check pin state against bits. Here the logic, is, when true, check the next one, if not, button is pressed.
  if(pinstates & 0b00000010){
        if(pinstates & 0b00000100){
                  if(pinstates & 0b00001000){
                            if(pinstates & 0b00010000){
                                      if(pinstates & 0b00100000){
                                                if(pinstates & 0b01000000){
                                                          if(pinstates & 0b10000000){
    }else{
      programchange(0, 3);
     // Serial.println(7);
      }
    }else{
      programchange(0, 2);
    //  Serial.println(6);
      }
    }else{
      programchange(0, 1);
    //  Serial.println(5);
      }
    }else{
      controlchange(0, 20, 100);
    //  Serial.println(4);
      }
    }else{
      controlchange(0, 18, 100);
    //  Serial.println(2);
      }
    }else{
      controlchange(0, 19, 100); // I messed up the connections on hardware level
    //  Serial.println(3);
      }
    }else{
      controlchange(0, 17, 100);
    //  Serial.println(1);
      }
      //debounce end
      prevmillis=curmillis;
  }
      
      //Work is done
      dowork = 0; 
}
  
}


//*****************************************************************************//

//Routine for pin change interrupt
ISR(PCINT0_vect){ 
//Record current pin states on port B.
pinstates = PINB;
//Flag for loop
dowork = 1;

  }

