////////////////////////////////
/// Testbench for dial rotation/combination algorithm
/// Uses stepper.h as provided in the Electrical folder
////////////////////////////////

#include <math.h>
#include "stepper.h"

#define NUM_DIGITS 100  /// dial digit count
#define STEP_DIGIT 3	/// dial step (2*tolerance of dial)

//req for stepper.h
#define RPM 300
#define MICROSTEPS 32
#define ENABLE_PIN 5
#define PULSE_PIN 7
#define DIR_PIN 6

int32_t pos = 0;
bool zeroTrigger = false;

//should be tied to photogate with Arduino interrupt
//will want trigger to happen on rising edge, not while input is HIGH (inifnite trigger)
void onZeroTriggered()
{
	zeroTrigger = true;
	pos = 0;
}

/// Spins dial a number of increments
/// \param delta  //< 0 for clockwise, > 0 is ccw (looking down shafter from top)
void spin( int32_t delta )
{
	pos += delta;
	if( pos >= NUM_DIGITS )
	{
    	pos = pos % NUM_DIGITS;
	}
	else if( pos < 0 )
	{
    	pos = NUM_DIGITS - abs(pos % NUM_DIGITS);
	}
    
	//call library step function
	step(delta*4);
}

void toZero()
{
	//interrupt when photogate triggered
	uint32_t count = 0;

	onZeroTriggered();
	/*for now just set pos to 0
	 
	while( !zeroTrigger )
	{
    	//perhaps spin continuously here, rather than increment?
    	//this is so we can more precisely set the dial at 0, if off increment initially
    	spin( 1, false );
    	count++;

    	if( count >= NUM_DIGITS )
    	{
        	//photogate never triggered => problem
        	//exception of some kind?
        	break;
    	}

    	//adjust if overshoot (if photogate not currently triggered)
	}*/

	zeroTrigger = false;
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
	toZero();

	// Wasn't revolving enough times b/w combo digits
	//  incremented multiples by 1, but hasn't been tested yet

	//dial combination digits
	spin( 4*NUM_DIGITS );   //pickup 3 wheels
	spin( c0 - pos );   	//set wheel 0 to c0

	//std::cout << "0 digit: " << c0 << " actual pos: "<< pos << std::endl;

	spin( -3*NUM_DIGITS );  //pickup next 2 wheels
	spin( (c1 - pos <= 0) ? c1 - pos : c1 - pos - NUM_DIGITS ); //set wheel 1 to c1

	//std::cout << "1 digit: " << c1 << " actual pos: "<< pos << std::endl;

	spin( 2*NUM_DIGITS ); 	//pickup last wheel
	spin( (c2 - pos <= 0) ? c2 - pos : c2 - pos - NUM_DIGITS ); //set wheel 2

	//std::cout << "2 digit: " << c2 << " actual pos: "<< pos << "\n" << std::endl;

	//spin once to see if safe opens
	//check for stall
	spin( -NUM_DIGITS );
}

void setup()
{
  Stepper(NUM_DIGITS, MICROSTEPS, ENABLE_PIN, PULSE_PIN, DIR_PIN);
  setSpeed(RPM);
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
	for( uint32_t z = 0; z < NUM_DIGITS; z += STEP_DIGIT )
    		for( uint32_t y = 0; y < NUM_DIGITS; y += STEP_DIGIT )
        	for( uint32_t x = 0; x < NUM_DIGITS; x += STEP_DIGIT )
        	{
            			tryCombo( x, y, z );
        	}
	}

	halt = true;
}
