///////////
//begin library global definitions
// define a global variable of type struct stepperdriver called driver
struct stepperdriver {
  long enable; //enable pin number
  long pulse; //pulse pin number
  long dir; //direction pin number
  long steps; //steps per revolution
  long microsteps; //microsteps per step
  long rpm; //rotation speed in rpm
} driver;
//end library global definitions
///////////


///////////
//begin library function declarations
void Stepper(int steps, int microsteps, int enable, int pulse, int dir);
void setSpeed(long rpm);
void step(int steps);
//end library function declarations
///////////


///////////
//begin library definitions
// initialize the stepperdriver struct
void Stepper(int steps, int microsteps, int enable, int pulse, int dir) {
//  Set enable, pulse, and direction pin numbers
//  Set rpm to 0. NOTE: 'rpm' is not actually revolutions per minute!
//  Set microsteps per step and steps per revolution

  driver.enable = enable;
  driver.pulse = pulse;
  driver.dir = dir;
  driver.steps = steps;
  driver.microsteps = microsteps;

  driver.rpm = 60;

  pinMode(driver.enable, OUTPUT);
  pinMode(driver.pulse, OUTPUT);
  pinMode(driver.dir, OUTPUT);

  //driver is ALWAYS enabled
  digitalWrite(driver.enable, HIGH);
  return;
}

// set the speed of the stepper
void setSpeed(int rpm){
  driver.rpm = rpm;
}

// move the stepper motor a set amount of steps. pass a negative value to move in the opposite direction
void step(int steps) {
//Negative (low) is clockwise, positive (high) is ccw looking down the shaft from the top (pointy bit facing viewer)
/*
Check for 0 steps
Get the direction
Set the direction in the direction pin
Wait for the direction to change (find out how long to wait for experimentally)
For the steps we need to step for
  Set pulse to high
  Wait for (time)
  Set pulse to low
  Wait for (time)

*/
    Serial.println("step() just got called.");
  if(steps == 0)
    return;

  int absSteps = abs(steps);
  int i;
  int dir = steps / abs(steps);
  float delaytime = (float) 1000*60000 / (float) (driver.rpm * (float) driver.steps * (float) driver.microsteps * 2);
//  Serial.println((unsigned int) delaytime);

  if(dir == 1) { // spin ccw (high)
    digitalWrite(driver.dir, LOW);
  } else { // spin cw (low)
    digitalWrite(driver.dir, HIGH);
  }

  delayMicroseconds(300); // at LEAST 25 us setup time on one tested driver

  for(i = 0; i < absSteps * driver.microsteps; i++) {
    digitalWrite(driver.enable, HIGH);
    digitalWrite(driver.pulse, HIGH);
    delayMicroseconds((unsigned int) delaytime);
//    Serial.println(delaytime);
    digitalWrite(driver.enable, HIGH);
    digitalWrite(driver.pulse, LOW);
    delayMicroseconds((unsigned int) delaytime);
//    Serial.println(delaytime);
  }

/*
Int i;
Int n = abs(steps);
delay(200);  // to make sure direction change registers
if(steps>0){
Set dir pin low
}
Else if(steps<0){
Set dir pin low
} // setting direction

for(i=0;i<n;i++){
// Actual moving
Set pulse pin high;
Delay (time);
Set pulse pin low;
delay(time);
}
*/
}
//end library definitions
///////////
