/*
 *
 * Created: 22/Dec/2018
 * Modified: 24/Dec/2018
 * 
 * NB! this code is using arduino micro with Atmega32u4
 *
 */ 

//*****************************************************************************//

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define PitchBend 0xE0 

#include "MIDIUSB.h"

//*****************************************************************************//

//Pin change related
uint8_t pinstates = 0b11111110;
uint8_t dowork = 0;



//software debounce related
unsigned long curmillis = 0;
unsigned long prevmillis = 0;
unsigned long debmillis = 70;


//
unsigned long timemillis = 0;
unsigned long delaymillis = 500;

//Analog related
const int threshold = 4;
const uint8_t highspeed = 210;
const uint8_t lowspeed = 120;
const uint8_t lowspeedHR = 138;
const uint8_t difference = 110;
static int lastpotvalue = 0;
static int lastmappedvalue = 0;
int gotovalue=0;

static uint8_t low, high;




  int faderChannel = 1;
  bool positionUpdated = false;

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


 //THIS SECTION IS FOR PIN CHANGE INTERRUPT

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
  DDRB = B00000110;    //Port B inputs
  PORTB = B00000000;   //Port B pullups

  
  PCICR |= B00000000;    // turn on pin change interrupts  

//When the PCIE0 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt 0 is
//enabled. Any change on any enabled PCINT7..0 pin will cause an interrupt. The corresponding interrupt of Pin
//Change Interrupt Request is executed from the PCI0 Interrupt Vector. PCINT7..0 pins are enabled individually
//by the PCMSK0 Register.

  PCMSK0 |= B00000000;  // Choose which pins output interrupt
//Each PCINT7..0 bit selects whether pin change interrupt is enabled on the corresponding I/O pin. If PCINT7..0
//is set and the PCIE0 bit in PCICR is set, pin change interrupt is enabled on the corresponding I/O pin. If
//PCINT7..0 is cleared, pin change interrupt on the corresponding I/O pin is disabled.
//The 0 bit is 0, because of the RX LED.
  
// turn on global interrupts  I-bit in the Status Register (SREG)


 //THIS SECTION IS FOR ADC FUNCTIONS

//ADMUX contains following 8 bits:
//  REFS1 REFS0 ADLAR MUX4 MUX3  MUX2  MUX1  MUX0

//REFS1 and REFS0 assign voltage reference arduino uni default is REFS1 = 0, REFS2 = 1 AV CC with external capacitor on AREF pin
//ADLAR sets the presentation of the ADC conversion result. If it is 1, the ADC conversion result is left adjusted. If 0, right adjusted. It is set to 0, right adjusted, by the Arduino software.
//MUX0 to MUX4 selects the analog pin, gain, diferential mode of internal temp sensor
//I chose adc0, therefore MUX0-MUX4 00000


ADMUX = B01000000;


//ADCSRA contains following 8 bits:
//ADEN  ADSC  ADATE ADIF  ADIE  ADPS2 ADPS1 ADPS0
//ADEN ADC enable with writing 1. writing 0 kills conversions
//Setting theADSC(ADC Start Conversion) to 1, the chip begins AD conversion. While AD conversion is executed, this bit is 1. After the conversion, it becomes 0.
//The ADATE(ADC Auto Trigger Enable) controls automatic trigger of AD conversion. The bit is not used by the Arduino software.
//The ADIF(ADC Interrupt Flag) and ADIE(ADC Interrupt Enable) controls the interruption. The bits are not used by the Arduino software.
//ADIE When this bit is written to one and the I-bit in SREG is set, the ADC Conversion Complete Interrupt is activated.
//The ADPS are the bits to determine the division factor between the system clock frequency and the input clock to the AD converter.
//Arduino Uno uses 16Mhz system clock, the clock to the AD convert is 125kHz(=16MHz/128). 

ADCSRA= B11000111;

ADCSRB= B00000000;

DIDR0 = B00000001;







 
 // PORTB = B00000010; //DOWN
  //delay(100);
  //PORTB = B00000100; //UP
  //delay(100);

  
  analogWrite(5, 140);



 // Serial.begin(115200);
}


//*****************************************************************************//
int analogvalue(){
  
 // uint8_t low, high;
  
  ADMUX = B01000000;
  //ADCSRA |= B01000000;

  if(CHECK_BIT(ADCSRA,6)){
    }else{
        low  = ADCL;
  high = ADCH;
  ADCSRA |= B01000000;
      }



  
  return (high << 8) | low;
  
  }



//*****************************************************************************//


void loop() {
 // delay(100);
 // PORTB = B00000000;
 
  
  

    
   if(millis()-timemillis>=delaymillis){

       gotovalue=gotovalue+100;
    
                if(gotovalue>1000){
      gotovalue=20;
      }

   // Serial.print("gotoval: ");
  //  Serial.println(gotovalue);
  //  Serial.print(" potval: ");
   // Serial.println(analogvalue());
    timemillis=millis();


    }

  int currentpotvalue = analogvalue();

    
   if(currentpotvalue>(gotovalue+threshold)){

    PORTB = B00000010; //DOWN

    if((currentpotvalue-gotovalue)>difference){
      analogWrite(5, highspeed);
      }else{
        analogWrite(5, lowspeed);
         
        }
   // Serial.println("DOWN");
   // delay(1);
    }else{
            if(currentpotvalue<(gotovalue-threshold)){
    PORTB = B00000100; //DOWN

        if((gotovalue-currentpotvalue)>difference){
      analogWrite(5, highspeed);
      }else{
        analogWrite(5, lowspeed);
          
        }
   // delay(1);
    }else{
    PORTB = B00000000;  
   // delay(1);
      } 
      }


   //Serial.println(currentpotvalue);
  // delay(500);
 

if(dowork>0){
  curmillis = millis();

  if((prevmillis-curmillis)>debmillis){
  //If button is pressed, the pin is pulled LOW. For example, if pin 4 is pressed, PINB = 11101111
  //Check pin state against bits. Here the logic, is, when true, check the next one, if not, button is pressed.
  if(pinstates & 0b00000010){
    }else{
      controlchange(0, 30, 100);
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
