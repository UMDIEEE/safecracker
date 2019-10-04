  #include "AccelStepper.h"
  
  #define INTERRUPT_PIN 2 // wire photointerruptor to this pin here
  
  #define REVOLUTION 6400
  #define STEPS_PER_DIGIT 64 //microsteps per step (driver config) * steps per digit (motor + safe spec)
  #define ENABLE_PIN      5 // arduino digital pin for ENA (enable, on holds motor, off releases the motor
  #define STEP_PIN        4 // arduino digital pin for STEP (pulse moves stepper motor by one step or microstep)
  #define DIR_PIN 3 // arduino digital pin for DIR (sets direction)
  
  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //AccelStepper library object, needed for interfacing with motor
 
 void setup() {
  // put your setup code here, to run once:
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN,HIGH);
    
    stepper.setMaxSpeed(REVOLUTION*1);  //good speed: 0.5 * REVOLUTION
    stepper.setAcceleration(2*REVOLUTION); //good acceleration: 2 * REVOLUTION

    digitalWrite(ENABLE_PIN, HIGH);
}

void loop() {
    stepper.move(STEPS_PER_DIGIT);
    while(stepper.distanceToGo() != 0)
    {
        stepper.run();
    }
    delay(250);
}
