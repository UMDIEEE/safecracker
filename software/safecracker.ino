    #include "AccelStepper.h"
  
  #define INTERRUPT_PIN 2 // wire photointerruptor to this pin here

  #define NUM_DIGITS 100
  #define REVOLUTION 6400
  #define STEPS_PER_DIGIT 64 //microsteps per step (driver config) * steps per digit (motor + safe spec)
  #define ENABLE_PIN      5 // arduino digital pin for ENA (enable, on holds motor, off releases the motor
  #define STEP_PIN        4 // arduino digital pin for STEP (pulse moves stepper motor by one step or microstep)
  #define DIR_PIN 3 // arduino digital pin for DIR (sets direction)
  
  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //AccelStepper library object, needed for interfacing with motor

uint32_t dialPos = 0, x = 0, y = 0, z = 0;
  
void photogateISR()
{
  dialPos = 0;
}

void calibrate()
{
    dialPos = 100;
    stepper.move(STEPS_PER_DIGIT * NUM_DIGITS);
    while(stepper.distanceToGo() != 0 && dialPos != 0)
    {
        stepper.run();
    }
    stepper.setCurrentPosition(0,0);
    stepper.setMaxSpeed(REVOLUTION*100);  //good speed: 0.5 * REVOLUTION
}

void spin(int spin_count,int spin_to,int dir){
  //as a part of callibrate, set to zero
    int delta;

    if(dir > 0)
      delta = (spin_to - dialPos >= 0) ? spin_to - dialPos : (spin_to - dialPos) + NUM_DIGITS;
    else
      delta = (spin_to - dialPos <= 0) ? spin_to - dialPos : (spin_to - dialPos) - NUM_DIGITS;
    
    stepper.move(delta*STEPS_PER_DIGIT + dir*spin_count*NUM_DIGITS*STEPS_PER_DIGIT);
    while(stepper.distanceToGo() != 0)
    {
       stepper.run();
    }

    dialPos = spin_to;
}
 
 void setup() {
  // put your setup code here, to run once:
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN,HIGH);
    
    stepper.setMaxSpeed(REVOLUTION*100);  //good speed: 0.5 * REVOLUTION
    stepper.setAcceleration(2*REVOLUTION); //good acceleration: 2 * REVOLUTION

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), photogateISR, FALLING);

    digitalWrite(ENABLE_PIN, HIGH);
}

void loop() {
    //calibrate();

     for( x = 0; x < NUM_DIGITS; x += 1 )
        for( y = 0; y < NUM_DIGITS; y += 1 )
            for( z = 0; z < NUM_DIGITS; z += 1 )
            {
              calibrate();
              spin(4,x,1);
              spin(3,y,-1);
              spin(2,z,1);
              delay(3000);
            }
   while(1){ //do nothing 
   }
}
