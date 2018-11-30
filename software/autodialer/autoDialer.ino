////////////////////////////////
/// Dial rotation/combination
/// Uses stepper.h as provided in the Electrical folder
////////////////////////////////

#include <math.h>
#include <stdio.h>
#include "stepper.h"

#define DEBUG	//put debug code in #ifdef block so we just need to remove this to remove debugging

#define DBG_DELAY 300 /// debug delay b/w combo digits in tryCombo

#define NUM_DIGITS 100  /// dial digit count
#define STEP_DIGIT 3  	/// dial step (2*tolerance of dial)

//req for stepper.h
#define SPEED 150	/// not yet sure what units this is, but bigger is still faster
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
// delta > 0 => ccw, delta < 0 => cw (looking down at motor shaft)
// Sets pos to a value between 0 and 99 which corresponds to a position on the dial. Also spins the motor delta increments.
void spin( int32_t delta )
{
  pos = pos + delta;
  int mult = (int)(pos / NUM_DIGITS);
  if(pos >= NUM_DIGITS)
  {
    pos = pos - mult*NUM_DIGITS; 
  }
  else if(pos < 0)
  {
    pos = pos + NUM_DIGITS*(1 - mult);
  }
  
  //call library step function
  step(delta*4);
  return;
}

void spinTo( int32_t c )
{
  
}
void toZero()
{
    //interrupt when photogate triggered
    //for testing, just spin until pos = 0;
    spin( -pos );
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
#ifdef DEBUG 
  Serial.println("Reset and try combo");
  Serial.println(c0);Serial.println(c1);Serial.println(c2);
  delay(DBG_DELAY);
#endif

  toZero();

#ifdef DEBUG 
  Serial.println("Going to first number, spin delta:");
  Serial.println(c0 - pos);
  delay(DBG_DELAY);
#endif
  //dial combination digits
  spin( 4*NUM_DIGITS );   //pickup 3 wheels
  spin( c0 - pos );     //set wheel 0 to c0

#ifdef DEBUG 
  Serial.println("Going to second number, spin delta:");
  Serial.println((c1 - pos <= 0) ? c1 - pos : c1 - pos - NUM_DIGITS);
  delay(DBG_DELAY);
#endif

  spin( -3*NUM_DIGITS );  //pickup next 2 wheels
  spin( (c1 - pos <= 0) ? c1 - pos : c1 - pos - NUM_DIGITS ); //set wheel 1 to c1

#ifdef DEBUG 
  Serial.println("Going to third number, spin delta:");
  Serial.println((c2 - pos >= 0) ? c2 - pos : c2 - pos - NUM_DIGITS);
  delay(DBG_DELAY);
#endif

  spin( 2*NUM_DIGITS );   //pickup last wheel
  spin( (c2 - pos >= 0) ? c2 - pos : c2 - pos - NUM_DIGITS ); //set wheel 3 to c3

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
  
  //code to run once goes here
  tryCombo(20, 30, 90);
}

void loop()
{
  toZero();
  
  //tryCombo does not function correctly, it is almost definitely the arguments to spin()
  //we need to redo the the function from scratch

  /*for( uint32_t x = 0; x < NUM_DIGITS; x += STEP_DIGIT )
        for( uint32_t y = 0; y < NUM_DIGITS; y += STEP_DIGIT )
          for( uint32_t z = 0; z < NUM_DIGITS; z += STEP_DIGIT )
          {
                  tryCombo( x, y, z );
          }*/
}

