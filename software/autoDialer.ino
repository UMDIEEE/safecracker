////////////////////////////////

/// Dial rotation/combination
/// Uses AccelStepper.h as provided in the Electrical folder
/// consider calling disableOutputs from library once safe has been opened. It will shut off power to motor pins and save power.
////////////////////////////////

//NOTE: Go into library and set velocity and acceleration to zero when we call the toZero function to fix the glitch!

/*Instructions for wire connections:
Connect the Anode of an LED to pin 8, and the cathode to ground. 
Connect the SIG terminal of the photogate to pin 2 (and PWR to 5V, GRND to ground)
Connect the motor's PUL+ to 5V (power); connect PUL- to pin 4, DIR- to pin 3, and ENA- to pin 5.
Connect the Rx terminal of the LCD display to pin 11 of Arduino (and PWR to 5V, GRND to ground)
*/


/*Left to do: figure out (calculate-yes it is possible) when the position variable will overflow. Before it does, have a condition in the triple for loop that says: "right before the position overflows and before I try the next combination,
 * return to 0 and call setCurrentPosition(0) again. If it will overflow twice throughout the entire safe cracking, then fix it a second (or third or fourth) time! But don't spend too much time doing things other than trying combinations. 
 */

 //Sign convention for this code: the DIAL will rotate clockwise if you provide a negative argument to stepper.move() and CCW for a positive argument. This follows the sign convention of the unit circle. Note that this means the motor itself rotates in the opposite direction- it's a matter of perspective.
 
#include <math.h>
#include <stdio.h>
//#include "stepper.h" //using AccelStepper.h now to control acceleration and velocity
#include <AccelStepper.h>
#include <SoftwareSerial.h>

//for displaying to LCD, connect Rx to pin 11 on Arduino (and power to 5V, ground to ground)

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

#define HALF_REV 3200


volatile int32_t pos = 0; //value will be changed in the ISR, so should be declared volatile
volatile int32_t delta = 0;  //value represents the amount of DIGITS to move
volatile float saved_speed;
volatile long saved_target;
volatile long saved_position;
volatile uint32_t opened = 0;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  //object of class AccelStepper using library
bool beenTriggeredBefore = false;
bool tryComboAgain = false;

bool finish_with_call = false;
//variable below is purely for user debugging

//------------------------------------

volatile byte ledState = HIGH;

//----------------------------------



//will want trigger to happen on rising edge, not while input is HIGH (infinite trigger)

void onZeroTriggered()
{
  opened = opened + 1;
  
  int dif = 0; long library_pos = 0; int modded_pos = 0;
   
  
  if(!beenTriggeredBefore)  //if the photogate is being tripped by the toZero() function (assuming the toZero() function is called prior to any other movements of the motor)
  {
    stepper.move(0);        //have the motor come to a stop. You MUST then call run() so the motor can physically decelerate to 0. 
    beenTriggeredBefore = 1;
    return;
  }
  else{
    library_pos = stepper.currentPosition();
    
    modded_pos = library_pos % REVOLUTION;
    
    dif = REVOLUTION - modded_pos;
    if( dif > HALF_REV)
    {
      stepper.setCurrentPosition(library_pos - modded_pos,0);
      
      
    }
    else
    {
      stepper.setCurrentPosition(library_pos + dif,0);
      //Serial.print("Set the current position to: ");
      //Serial.println(library_pos + dif);
    }
  }  
    
  return;
}


//delta is the number of digits on the dial you wish to rotate. This function will update the global 'pos' variable to reflect where the new position should be after the motor rotates delta digits on the dial.
static inline void updatePosition(int32_t delta)
{
  delta  = delta % 100;
 //pos gets updated to reflect the amount that the dial turns (delta amount) since the last value of pos
  pos = (delta + pos)%NUM_DIGITS;
  if( pos < 0 )      //if (pos - delta) is negative, then modulus will return a negative number (in C and Java). We need a positive number. ex: -3 mod 100 will return -3. Need it to return 97.
  {
      pos = pos + NUM_DIGITS;
  }
 return;
}

void toZero()
{
   //interrupt when photogate triggered
  
  while(digitalRead(INTERRUPT_PIN) == HIGH)
  {
    if(beenTriggeredBefore)   //this condition is here so that if the above condition in the while loop is not being evaluated at the precise moment when the photogate is blocked, we will still be able to jump out of the loop and return as long as the ISR has been called.
    {
       pos = 0;
       stepper.setCurrentPosition(0,1);
       delay(3000);
       return;
    }
    stepper.move(-3*REVOLUTION);  
    while(stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
  }

  pos = 0;
  stepper.setCurrentPosition(0,1);
  
  delay(3000);
  return;
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
  
  /*Serial.println("Beginning a new combination now.");
  Serial.println("Current position: ");
  Serial.println(pos);
  Serial.println("Going to first digit now");
  */
  delay(1000);
  delta = (c0 - pos >= 0) ? c0 - pos : c0 - pos + NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );    //going CCW to first digit. argument to stepper.move() better be positive.
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  delay(1000);

  delta = 3*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c0 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  delay(1000);

  Serial.println("Just stopped at the 1st digit which is at position: ");
  Serial.println(pos);
  Serial.println("Going to 2nd digit now.");
  
  delta = (c1 - pos < 0) ? c1 - pos : c1 - pos - NUM_DIGITS;    //going CW to second digit. argument to stepper.move() better be negative.
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  //delay(1000);

  
  delta = -2*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c1 third time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  Serial.println("Just stopped at the 2nd digit which is at position: ");
  Serial.println(pos);
  Serial.println("Going to third digit now.");
  delay(1000);

  delta = (c2 - pos > 0) ? c2 - pos : c2 - pos + NUM_DIGITS;    //going CCW to third digit. Argument to stepper.move() better be positive.
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  //delay(1000);

  delta = NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c2 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  Serial.println("Just stopped at 3rd digit which is at position: ");
  Serial.println(pos);
  Serial.println("Now rotating once around to see if safe opens");
  delay(1000);
  
  opened = 0;
  
  // rotate around once to see if safe opens

  delta = -NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  Serial.println("Just rotated 1 revolution around and stopped at position: ");
  Serial.println(pos);
  delay(1000);

  beenTriggeredBefore = 0;
  toZero(); 
  
  if(c2 == 0) //if the third digit is 0, then rotating around one REV at the end will increment opened to 1. And then going toZero() won't increment opened to 2 b/c the dial is already at 0. So increment opened by 1. Now, if it's still less than 2, we know that the dial never made it back to 0.
  {opened ++;}
  
  if(opened < 2)
  {
    Serial.println("We are about to cease the program");
    ceaseProgram();
  }
  pos = 0;
  return;
}

void ceaseProgram(void)
{
  while(1){Serial.println("We are in cease program");} 
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

  stepper.setMaxSpeed(REVOLUTION*0.5);
  stepper.setAcceleration(2*REVOLUTION);
  stepper.moveTo(REVOLUTION);

  LCD.begin(9600); // set up serial port for 9600 baud

  delay(500); // wait for display to boot up

  digitalWrite(ENABLE_PIN, HIGH);

  clear_display(); 

  LCD.write("Beginning Safe Cracking Now...");
  Serial.println("Delaying for three seconds...");
  delay(3000);
  clear_display();
  cursor_to_line_1();
  //code to run once goes here
}

void loop()
{
 
  Serial.println("Calling toZero() ");
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
