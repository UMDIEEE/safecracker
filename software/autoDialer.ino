////////////////////////////////
/// Testbench for dial rotation/combination algorithm
/// Uses stepper.h as provided in the Electrical folder
////////////////////////////////

#include <math.h>
#include "stepper.h"
#include <stdio.h>

#define NUM_DIGITS 100  /// dial digit count
#define STEP_DIGIT 3  /// dial step (2*tolerance of dial)

//req for stepper.h
#define RPM 100
#define MICROSTEPS 32
#define ENABLE_PIN 5
#define PULSE_PIN 7
#define DIR_PIN 6

int32_t pos = 0;
bool zeroTrigger = false;

//should be tied to photogate with Arduino interrupt
//will want trigger to happen on rising edge, not while input is HIGH (infinite trigger)
void onZeroTriggered()
{
  zeroTrigger = true;
  pos = 0;
  return;
}

/// Spins dial a number of increments
// Sets pos to a value between 0 and 99 which corresponds to a position on the dial. Also spins the motor delta increments. 
// If delta>0, spins dial on safe counterclockwise (positive on unit circle)
void spin( int32_t delta )
{
  //pos gets updated to reflect the amount that the dial turns (delta amount) since the last value of pos
  pos = (pos - delta)%NUM_DIGITS;
  if( pos < 0 )      //if (pos - delta) is negative, then modulus will return a negative number (in C and Java). We need a positive number. ex: -3 mod 100 will return -3. Need it to return 97.
  {
      pos = pos + NUM_DIGITS;
  }
  
  if( (pos >= NUM_DIGITS) || (pos < 0) )
  {
      pos = pos % NUM_DIGITS;
  }
  
  
  //call library step function
  step(delta*4);
  Serial.println("Just left step()");
  Serial.println("Exiting spin().");
  return;
}

void toZero()
{
    //interrupt when photogate triggered
    //for testing, just spin until pos = 0;
    spin( -pos );
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
  toZero();

  // Wasn't revolving enough times b/w combo digits
  //  incremented multiples by 1, but hasn't been tested yet
 
  Serial.println("Beginning a new combination now. Rotating three times first.");
  //dial combination digits
  spin( 4*NUM_DIGITS );   //pickup 3 wheels
  delay(100);
 
  Serial.println("I just rotated three times. Going to first number now...");
  spin( c0 - pos );     //set wheel 0 to c0
  delay(100);
  
  Serial.println("Just got to first number. Going clockwise two turns now...");
  spin( -3*NUM_DIGITS );  //pickup next 2 wheels
  delay(100);
  
  Serial.println("Just rotated two times. Going to second number now...");
  spin( (c1 - pos <= 0) ? c1 - pos : c1 - pos - NUM_DIGITS ); //set wheel 1 to c1
  delay(100);
  
  Serial.println("Just got to second number. Going counter clockwise one turn now...");
  spin( 2*NUM_DIGITS );   //pickup last wheel
  delay(100);
  
  Serial.println("Just rotated one turn. Going to third number now...");
  spin( (c2 - pos >= 0) ? c2 - pos : c2 - pos - NUM_DIGITS ); //set wheel 2
  delay(100);
  
  Serial.println("Got to third number. Doing other stuff before starting again......"); 
 

  //spin once to see if safe opens
  //check for stall
  spin( -NUM_DIGITS );
  return;
}

void setup()
{
  Stepper(NUM_DIGITS, MICROSTEPS, ENABLE_PIN, PULSE_PIN, DIR_PIN);
  setSpeed(RPM);
  Serial.begin(9600);
}

void loop()
{
  toZero();
  
  bool halt = false;
 
  //step(NUM_DIGITS*4);
  //setSpeed(300);
  //step(-NUM_DIGITS*4);

  if(!halt)
  {
  for( uint32_t x = 0; x < NUM_DIGITS; x += STEP_DIGIT )
        for( uint32_t y = 0; y < NUM_DIGITS; y += STEP_DIGIT )
          for( uint32_t z = 0; z < NUM_DIGITS; z += STEP_DIGIT )
          {
                  tryCombo( x, y, z );
          }
  }

  halt = true;
}

