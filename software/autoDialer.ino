////////////////////////////////
/// Dial rotation/combination
/// Uses stepper.h as provided in the Electrical folder
////////////////////////////////

/*Instructions for wire connections:
Connect the Anode of an LED to pin 8, and the cathode to ground. 
Connect the SIG terminal of the photogate to pin 2 (and PWR to 5V, GRND to ground)
Connect the motor's PUL+ to 5V (power); connect PUL- to pin 7, DIR- to pin 6, and ENA- to pin 5.

*/
#include <math.h>
#include <stdio.h>
#include "stepper.h"
#include <SoftwareSerial.h>

//for displaying to LCD
SoftwareSerial LCD(10, 11); // Arduino SS_RX = pin 10 (unused), Arduino SS_TX = pin 11 

//LCD Instructions:
void clear_display()
{
     LCD.write(0xFE); //command flag
     LCD.write(0x01); 
}

void cursor_to_line_1()
{
    LCD.write(0xFE); //command flag
    LCD.write(128); //position
}

//Put cursor to beginning of line 2:
void cursor_to_line_2()
 {
    LCD.write(0xFE); //command flag
    LCD.write(192); //position
 }

//Move cursor right by one:
void cursor_right_1()
{
    LCD.write(254);
    LCD.write(20);
}

void print_combo(int x,int y,int z)
{
  LCD.write("Trying: ");
  cursor_right_1();
  LCD.print(x,DEC);
  LCD.print(" ");
  LCD.print(y,DEC);
  LCD.print(" ");
  LCD.print(z,DEC);
  LCD.write(254);
  LCD.write(128);
  cursor_to_line_1();
}
//#define DEBUG  //put debug code in #ifdef block so we just need to remove this to remove debugging

#define DBG_DELAY 200 /// debug delay b/w combo digits in tryCombo

#define NUM_DIGITS 100  /// dial digit count
#define STEP_DIGIT 3    /// dial step (2*tolerance of dial)

//req for stepper.h
#define SPEED 800  /// not yet sure what units this has, but bigger is still faster
#define MICROSTEPS 32
#define ENABLE_PIN 5
#define PULSE_PIN 7
#define DIR_PIN 6
#define INTERRUPT_PIN 2
#define LED_PIN 13

volatile int32_t pos = 0; //value will be changed in the ISR, so should be declared volatile
bool zeroTrigger = false;

//variable below is purely for user debugging
//------------------------------------
volatile byte ledState = HIGH;
//----------------------------------


//should be tied to photogate with Arduino interrupt
//will want trigger to happen on rising edge, not while input is HIGH (infinite trigger)
void onZeroTriggered()
{
  zeroTrigger = true;
  pos = 0;

  //the LED will change value each time photogate is interrupted. can delete two lines of code below after satisfied with results
  digitalWrite(LED_PIN, ledState);
  ledState = !ledState;


  
  return;
}

/// Spins dial a number of increments
// delta > 0 => ccw, delta < 0 => cw (looking down at motor shaft)
// Sets pos to a value between 0 and 99 which corresponds to a position on the dial. Also spins the motor delta increments.
void spin( int32_t delta )
{
  pos = pos + delta;
  int mult = (int)(pos / NUM_DIGITS);
  pos = pos - mult*NUM_DIGITS; 
  if(pos < 0)
  {
    pos = pos + NUM_DIGITS;
  }
  
  //call library step function
  step(delta*4);
  
  return;
}

void toZero()
{
    //interrupt when photogate triggered
  while(digitalRead(2) == HIGH)
  {
    step(-4);
  }
  delay(1000);
  pos = 0;
    //for testing, just spin until pos = 0;
    //spin( -pos );
    
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
#ifdef DEBUG 
  Serial.println("Reset and try combo:");
  Serial.println(c0);Serial.println(c1);Serial.println(c2);
  delay(DBG_DELAY*2);
#endif

  toZero();

  //dial combination digits
  
#ifdef DEBUG 
  Serial.println("Going to first number...");
  delay(DBG_DELAY);
#endif

  spin( (c0 - pos > 0) ? c0 - pos : c0 - pos + NUM_DIGITS );
  spin( 3*NUM_DIGITS ); //stop when dial mark reaches c0 second time

#ifdef DEBUG 
  Serial.println("Going to second number");
  delay(DBG_DELAY);
#endif

  spin( (c1 - pos < 0) ? c1 - pos : c1 - pos - NUM_DIGITS );
  spin( -2*NUM_DIGITS ); //stop when dial mark reaches c1 third time

#ifdef DEBUG 
  Serial.println("Going to third number");
  delay(DBG_DELAY);
#endif

  spin( (c2 - pos > 0) ? c2 - pos : c2 - pos + NUM_DIGITS );
  spin( NUM_DIGITS ); //stop when dial mark reaches c2 second time
  
#ifdef DEBUG
  Serial.println("Checking combination...");
  delay(DBG_DELAY);
#endif

  //TODO: spin once to see if safe opens
  //just spin around for now
  spin( -NUM_DIGITS );
  return;
}

void setup()
{
  Stepper(NUM_DIGITS, MICROSTEPS, ENABLE_PIN, PULSE_PIN, DIR_PIN);
  setSpeed(SPEED);
  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP); //make pin 2 an interrupt pin
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onZeroTriggered, FALLING);  //associate pin 2 on UNO with interrupt function. onZeroTriggered gets called whenever pin 2 goes low. connect the SIG terminal of photogate to pin 2 on the UNO. 
  pinMode(LED_PIN, OUTPUT);
  
  LCD.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up

  clear_display(); 
  
  LCD.write("Beginning Safe Cracking Now...");
  delay(3000);

 clear_display();
  
 cursor_to_line_1();
  //code to run once goes here
}

void loop()
{
  toZero();

  for( uint32_t x = 0; x < NUM_DIGITS; x += STEP_DIGIT )
        for( uint32_t y = 0; y < NUM_DIGITS; y += STEP_DIGIT )
          for( uint32_t z = 0; z < NUM_DIGITS; z += STEP_DIGIT )
          {
                  
                  print_combo(x,y,z);
                  tryCombo( x, y, z );
                  clear_display();
                  
                  
          }
}
