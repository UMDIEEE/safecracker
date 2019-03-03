////////////////////////////////

/// Dial rotation/combination
/// Uses AccelStepper.h as provided in the Electrical folder
/// consider calling disableOutputs from library once safe has been opened. It will shut off power to motor pins and save power.
////////////////////////////////

/*Instructions for wire connections:
Connect the Anode of an LED to pin 8, and the cathode to ground. 
Connect the SIG terminal of the photogate to pin 2 (and PWR to 5V, GRND to ground)
Connect the motor's PUL+ to 5V (power); connect PUL- to pin 4, DIR- to pin 3, and ENA- to pin 5.
*/


/*The goal of the code is as follows: first, call call toZero(), which will keep rotating the motor clockwise until the photogate gets tripped. At this point, the motor will be stopped. Call setCurrentPosition(0), which will set the internal class variable that keeps track of the motor's position back to zero.
 * As a consequence of this function call, it sets the current speed to 0. I'm not sure if that will cause problems when we call this function every time the photogate gets tripped. We might have to follow that call with a call to a function which changes the current speed back to whatever value it was at when the motor was crossing the photogate (or a value close to it). This remains to be tested. 
 * Now, begin executing the triple for loop, starting with 0,0,0. Each time the photogate is interrupted, call currentPosition(), which returns the current position in microsteps, relative to the 0 position. positive values are clockwise past 0. Check if the currentPosition is greater than REVOLUTION/100 steps away from zero. If so, that means that the motor thought it was more than a digit off. If that's the case, call setCurrentPosition(0) and redo the combination. Otherwise, just call setCurrentPosition(0).
 * If, for some reason, the motor speed slows down as a result of resetting position to zero after tripping the photogate, we might have to live with only resetting the position to zero every n_th time the photogate gets tripped and hope the accumulated error is small enough. This is something that we need to test.
 * Also, the convention used in the first program is that positive rotations are in accordance with positive direction on the unit circle (i.e. a positive rotation would be the counterclockwise). The AccelStepper.h library uses the opposite convention. Rather than flip all of the signs in the code that we have already wrriten, try flipping the wiring of the phase A and B between the motor and driver.
 * If, for some reason, this doesn't work, then go back into the code and flip every sign in the argument to a stepper.move() call. You may need to do more than this. Finally, there is a chance thtat if all of the above causes no problems, something else might cause a problem- we don't know what exactly happens under the hood when the setCurrentPosition(0) function is called. One possible problem is this:
 * in our code, we calculate the amount of steps we would like to move to get from our current position to the next position in the tryCombo() function. Then, we call stepper.move() that amount of steps, and then keep on checking if we have reached that amount of steps in a while loop. And while we haven't reached that number of steps, we call stepper.run() in order to rotate the motor. At some point during the motor's rotation, the interrupt gets tripped and a function gets called which sets the current position to 0. The question is:
 * is the targetPosition variable (the one that keeps track of whether or not we have rotated the amount of steps that we said we would) going to change in order to reflect the fact that our current position was changed? For example, if we initially start at digit 5 and calculate that we would like to rotate the dial clockwise by 110 digits to end up at digit 95, which sets the internal targetPosition equal to [105 digits * (the # of steps per digit)], when we cross 0, there are still [5 digits * (# of steps per digit)] steps left to rotate. But
 * is it true that the targetPosition will be updated under the hood when we cross 0 to equal [5 * (# steps per digit)] steps left?
 * 
 */
 
#include <math.h>
#include <stdio.h>
//#include "stepper.h" //using AccelStepper.h now to control acceleration and velocity
#include <AccelStepper.h>
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

#define MICROSTEPS 32

#define ENABLE_PIN 5

#define STEP_PIN 4

#define DIR_PIN 3

#define INTERRUPT_PIN 2

//#define LED_PIN 13  //for LED display

#define REVOLUTION 6400  //macro representing the amount of steps/revolution that the motor achieves, which we configured on the driver. It, and STEPS_PER_DIGIT below, are used often as arguments to functions, which often require the argument as a number of steps

#define STEPS_PER_DIGIT 64


volatile int32_t pos = 0; //value will be changed in the ISR, so should be declared volatile
volatile int32_t delta = 0;  //value represents the amount of DIGITS to move
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  //object of class AccelStepper using library
bool zeroTrigger = false;
bool beenTriggeredBefore = false;
bool tryComboAgain = false;
//variable below is purely for user debugging

//------------------------------------

volatile byte ledState = HIGH;

//----------------------------------

//should be tied to photogate with Arduino interrupt

//will want trigger to happen on rising edge, not while input is HIGH (infinite trigger)

void onZeroTriggered()
{
  if(!beenTriggeredBefore)
  {
    stepper.move(0);
    zeroTrigger = true;
    
    //the LED will change value each time photogate is interrupted. can delete two lines of code below after satisfied with results
    //digitalWrite(LED_PIN, ledState);
    //ledState = !ledState;
  }
  else
  {
    if(stepper.currentPosition() > STEPS_PER_DIGIT || stepper.currentPosition() < -STEPS_PER_DIGIT) //if motor thinks it is more than a single digit away from where it actually is when dial passes 0
    {
      tryComboAgain = 1;
    }
    
  }

  stepper.setCurrentPosition(0);
  return;
}


//delta is the number of digits on the dial you wish to rotate. This function will update the global 'pos' variable to reflect where the new position should be after the motor rotates delta digits on the dial.
static inline void updatePosition(int32_t delta)
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
  
}

void toZero()
{
   //interrupt when photogate triggered
  while(digitalRead(INTERRUPT_PIN) == HIGH)
  {
    if(zeroTrigger)   //this condition is here so that if the above condition in the while loop is not being evaluated at the precise moment when the photogate is blocked, we will still be able to jump out of the loop and return as long as the ISR has been called.
    {
       pos = 0;
       return;
    }
    stepper.move(2*REVOLUTION);  //revolution/1000
    while(stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
    stepper.run();
  }

  pos = 0;
  return;

}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
#ifdef DEBUG 
  Serial.println("Reset and try combo:");
  Serial.println(c0);Serial.println(c1);Serial.println(c2);
  delay(DBG_DELAY*2);
#endif

  //dial combination digits

#ifdef DEBUG 
  Serial.println("Going to first number...");
  delay(DBG_DELAY);
#endif

  delta = (c0 - pos > 0) ? c0 - pos : c0 - pos + NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);

  delta = 3*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c0 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  
#ifdef DEBUG 
  Serial.println("Going to second number");
  delay(DBG_DELAY);
#endif

  delta = (c1 - pos < 0) ? c1 - pos : c1 - pos - NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);

  
  delta = -2*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c1 third time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);

#ifdef DEBUG 
  Serial.println("Going to third number");
  delay(DBG_DELAY);
#endif


  delta = (c2 - pos > 0) ? c2 - pos : c2 - pos + NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);

  delta = NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c2 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  
#ifdef DEBUG
  Serial.println("Checking combination...");
  delay(DBG_DELAY);
#endif

  // rotate around once to see if safe opens

  delta = -NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  return;
}

void setup()
{
  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP); //make pin 2 an interrupt pin
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  
  digitalWrite(ENABLE_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onZeroTriggered, FALLING);  //associate pin 2 on UNO with interrupt function. onZeroTriggered gets called whenever pin 2 goes low. connect the SIG terminal of photogate to pin 2 on the UNO. 

  //pinMode(LED_PIN, OUTPUT);

  stepper.setMaxSpeed(REVOLUTION*1.5);
  stepper.setAcceleration(REVOLUTION);
  stepper.moveTo(REVOLUTION);

  LCD.begin(9600); // set up serial port for 9600 baud

  delay(500); // wait for display to boot up

  digitalWrite(ENABLE_PIN, HIGH);

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

                  if(tryComboAgain) //if the amount of error between where the motor thinks it is when the dial crosses 0 and where it actually is is greater than a single digit, redo the combo again.
                  {z -= STEP_DIGIT; tryComboAgain = 0;}

                  clear_display();

          }

}
