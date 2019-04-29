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

/*
 Includes
 */
#include <math.h>
#include <stdio.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>

 
/*
 defines
 */
////////// delays and debugs //////////
#define DEBUG_DELAY     0 // boolean, 0 for no forced delays, 1 for forced delays
#define DBG_DELAY       200 /// debug delay b/w combo digits in tryCombo

////////// pin definitions ///////////
#define INTERRUPT_PIN   2 // wire photointerruptor to this pin here
#define ENABLE_PIN      5 // arduino digital pin for ENA (enable, on holds motor, off releases the motor
#define STEP_PIN        4 // arduino digital pin for STEP (pulse moves stepper motor by one step or microstep)
#define DIR_PIN         3 // arduino digital pin for DIR (sets direction)
#define ONBOARD_LED     13

////////// motor parameters //////////
#define MICROSTEPS      32 // microsteps per step

////////// safe parameters //////////
#define NUM_DIGITS      100  /// dial digit count
#define STEP_DIGIT      3    /// dial step (2*tolerance of dial)

////////// control parameters //////////
#define REVOLUTION      6400  // = microsteps per step (driver config) * steps per revolution (motor spec) = pulses per revolution
#define HALF_REV        REVOLUTION / 2
#define STEPS_PER_DIGIT 64 //microsteps per step (driver config) * steps per digit (motor + safe spec)


/*
 global variables
 */
////////// position //////////
volatile int32_t pos = 0; //value will be changed in the ISR (interrupt handler), so should be declared volatile (can change at any time)
volatile int32_t delta = 0;  //value represents the amount of DIGITS to move
//volatile float saved_speed;
//volatile long saved_target;
volatile long saved_position;

////////// flags and states //////////
volatile uint32_t opened = 0; //incremented on interrupted
volatile bool tripped = 0;  //inverted on interrupt
bool beenTriggeredBefore = false;
bool finish_with_call = false;
volatile byte ledState = HIGH;

////////// objects ///////////
SoftwareSerial LCD(10, 11); // Arduino SS_RX = pin 10 (unused), Arduino SS_TX = pin 11
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  //object of class AccelStepper using library

//for displaying to LCD, connect Rx to pin 11 on Arduino (and power to 5V, ground to ground)
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



//will want trigger to happen on rising edge, not while input is HIGH (infinite trigger)

void onZeroTriggered()
{
  opened = opened + 1;
  tripped = !tripped;
  int dif = 0; long library_pos = 0; int modded_pos = 0;
   digitalWrite(13, tripped);
  
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
   int quit = 0;
   bool localTripped = tripped;
  while(digitalRead(INTERRUPT_PIN) == HIGH && !quit)
  {
    if(beenTriggeredBefore)   //this condition is here so that if the above condition in the while loop is not being evaluated at the precise moment when the photogate is blocked, we will still be able to jump out of the loop and return as long as the ISR has been called.
    {
       pos = 0;
       stepper.setCurrentPosition(0,1);
       if(DEBUG_DELAY)
       {delay(3000);}
       return;
    }
    stepper.move( (long) -1.1*REVOLUTION);  
    while(tripped == localTripped)
    {
      stepper.run();
    }
    quit = 1;   //have the function quit once it gets to the beginning of the while loop again. We only want this function to rotate 1.1 times times once. If photogate isn't tripped, something is wrong (or safe has been cracked). We don't want to be stuck in this loop if beenTriggeredBefore never gets set to one in the IRS.
  }

  pos = 0;
  stepper.setCurrentPosition(0,1);
  
  if(DEBUG_DELAY)
  {delay(3000);}
  return;
}

void tryCombo( int32_t c0, int32_t c1, int32_t  c2 )
{
  
  /*Serial.println("Beginning a new combination now.");
  Serial.println("Current position: ");
  Serial.println(pos);
  Serial.println("Going to first digit now");
  */
  if(DEBUG_DELAY)
  {delay(1000);}
  delta = (c0 - pos >= 0) ? c0 - pos : c0 - pos + NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT  + 3*NUM_DIGITS*STEPS_PER_DIGIT );    //going CCW to first digit. argument to stepper.move() better be positive.
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  /*
  delay(1000);
  delta = 3*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c0 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  */
  if(DEBUG_DELAY)
  {delay(1000);}
 

  Serial.println("Just stopped at the 1st digit which is at position: ");
  Serial.println(pos);
  Serial.println("Going to 2nd digit now.");
  
  delta = (c1 - pos < 0) ? c1 - pos : c1 - pos - NUM_DIGITS;    //going CW to second digit. argument to stepper.move() better be negative.
  stepper.move( delta*STEPS_PER_DIGIT  -2*NUM_DIGITS*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);

  //delay(1000);
  /*delta = -2*NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c1 third time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  */
  Serial.println("Just stopped at the 2nd digit which is at position: ");
  Serial.println(pos);
  Serial.println("Going to third digit now.");
  if(DEBUG_DELAY)
  {delay(1000);}

  delta = (c2 - pos > 0) ? c2 - pos : c2 - pos + NUM_DIGITS;    //going CCW to third digit. Argument to stepper.move() better be positive.
  stepper.move( delta*STEPS_PER_DIGIT    +NUM_DIGITS*STEPS_PER_DIGIT );
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  //delay(1000);

  /*delta = NUM_DIGITS;
  stepper.move( delta*STEPS_PER_DIGIT ); //stop when dial mark reaches c2 second time
  while(stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  updatePosition(delta);
  */
  Serial.println("Just stopped at 3rd digit which is at position: ");
  Serial.println(pos);
  Serial.println("Now rotating once around to see if safe opens");
  if(DEBUG_DELAY)
  {delay(1000);}
  
  opened = 0;
 
  if(c2 == 0) //if the last digit is a 0, we want the ISR to call setCurrentPosition(0) as soon as the dial rotates once around, because it won't happen it toZero().
  {
    beenTriggeredBefore = 0;
  }
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
  if(DEBUG_DELAY)
  {delay(1000);}

  beenTriggeredBefore = 0;
  toZero(); 
  
  if(c2 == 0) //if the third digit is 0, then rotating around one REV at the end will increment opened to 1. And then going toZero() won't increment opened to 2 b/c the dial is already at 0. So increment opened by 1. Now, if it's still less than 2, we know that the dial never made it back to 0.
  {opened ++;}
  
  if(opened < 2)
  {
    Serial.println("We are about to cease the program");
    ceaseProgram(c0, c1, c2);
  }
  pos = 0;
  return;
}

void ceaseProgram(int c0, int c1, int c2)
{
while(1)
    {
        if(digitalRead(ENABLE_PIN) == HIGH )
          digitalWrite(ENABLE_PIN, LOW);
      
        clear_display();
        cursor_to_line_1();
      
      char *finishmessage = "CONGRATS. YOU HAVE OUTSMARTED A BRICK. THE COMBO IS:";
      for (int i = 0; i < strlen(finishmessage); i++) {
          LCD.write(finishmessage[i]);
          delay(500);
      }
      
//    LCD.write('C'); delay(500);  LCD.write('O'); delay(500); LCD.write('N'); delay(500); LCD.write('G'); delay(500); LCD.write('R'); delay(500); LCD.write('A'); delay(500); LCD.write('D'); delay(500); LCD.write('S'); delay(500); LCD.write('.'); delay(500); LCD.write(' '); delay(500); LCD.write('Y'); delay(500); LCD.write('O'); delay(500); LCD.write('U'); delay(500); LCD.write('V'); delay(500);
//    LCD.write('E'); delay(500); LCD.write(' '); delay(500); LCD.write('O'); delay(500); LCD.write('U'); delay(500); LCD.write('T'); delay(500); LCD.write('S'); delay(500); LCD.write('M'); delay(500); LCD.write('A'); delay(500); LCD.write('R'); delay(500); LCD.write('T'); delay(500); LCD.write('E'); delay(500); LCD.write('D'); delay(500); LCD.write(' '); delay(500);
//    LCD.write('A'); delay(500); LCD.write(' '); delay(500); LCD.write('B'); delay(500); LCD.write('R'); delay(500); LCD.write('I'); delay(500); LCD.write('C'); delay(500); LCD.write('K'); delay(500); LCD.write('.'); delay(500); LCD.write(' '); delay(500); LCD.write('T'); delay(500); LCD.write('H'); delay(500); LCD.write('E'); delay(500); LCD.write(' '); delay(500); LCD.write('C'); delay(500);
//    LCD.write('O'); delay(500); LCD.write('M'); delay(500); LCD.write('B'); delay(500); LCD.write('O'); delay(500); LCD.write(' '); delay(500); LCD.write('I'); delay(500); LCD.write('S'); delay(500); LCD.write(':'); delay(500); LCD.write(' ');
    LCD.print(c0,DEC);
    LCD.print(" ");
    LCD.print(c1,DEC);
    LCD.print(" ");
    LCD.print(c2,DEC);
    cursor_right_1();
    
    while(1)
    {
      //do nothing
    }
  } 

  
}



void setup()
{
  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP); //make pin 2 an interrupt pin
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(13, LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onZeroTriggered, FALLING);  //associate pin 2 on UNO with interrupt function. onZeroTriggered gets called whenever pin 2 goes low. connect the SIG terminal of photogate to pin 2 on the UNO. 

  //pinMode(LED_PIN, OUTPUT);

  stepper.setMaxSpeed(REVOLUTION*1);  //good speed: 0.5 * REVOLUTION
  stepper.setAcceleration(2*REVOLUTION); //good acceleration: 2 * REVOLUTION
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
  delay(3000); //helps with calibrating the photogate
  uint32_t x,y,z;


  //To jump to a specific combination, uncomment the next two lines of code
  //x = 66; y = 0; z = 93;
  //goto label;
  
  for( x = 0; x < NUM_DIGITS; x += STEP_DIGIT )

        for( y = 0; y < NUM_DIGITS; y += STEP_DIGIT )

          //for( z = 0; z < NUM_DIGITS; z += STEP_DIGIT )

          {

              label:
                  print_combo(x,y,3);                // print_combo(x,y,z);

                  tryCombo( x, y, 3);                // tryCombo( x, y, z);

                  clear_display();

          }

}
