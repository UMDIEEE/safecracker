
static struct stepperdriver {
  int enable; //enable pin number
  int pulse; //pulse pin number
  int dir; //direction pin number
  int steps; //steps per revolution
  int microsteps; //microsteps per step
  int rpm; //rotation speed in rpm
  
} driver;
void setup() {
 /* pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
for (int i=0; i<10; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,HIGH);
    
    digitalWrite(PUL,HIGH);
    delay(500/2);
    digitalWrite(PUL,LOW);
    delay(500/2);  // full step per second
  }
  digitalWrite(DIR,HIGH);
// Change direction and wait before pulsing.
// Or else direction change will not register / steps will be in old direction
  delayMicroseconds(25);
  for (int i=0; i<10; i++)   //Backward 5000 steps
  {
    digitalWrite(ENA,HIGH);
    
    digitalWrite(PUL,HIGH);
    delay(500/2);
    digitalWrite(PUL,LOW);
    delay(500/2); // full step
  } */

  int
}

// initialize the stepperdriver struct
int Stepper(int steps, int microsteps, int enable, int pulse, int dir) {
  /* Set enable, pulse, and direction pin numbers
  Set rpm to 0
  Set microsteps per step and steps per revolution */
  pinMode(driver.enable, OUTPUT);
  pinMode(driver.pulse, OUTPUT);
  pinMode(driver.dir, OUTPUT);
  digitalWrite(driver.enable, HIGH);
  return 0;
}

void setSpeed(long rpm){
driver.rpm = rpm;
}    


void step(int steps){
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
  if(steps == 0)
    return;

  int dir = steps / abs(steps);
  if(dir == 1) { // spin ccw (high)
    digitalWrite(driver.dir, HIGH);
  }
  else { // spin cw (low)
  digitalWrite(driver.dir, LOW);
  }

  delayMicroseconds(30); // at LEAST 25 us

  for(int i = 0; i < steps * driver.microsteps; i++) {
    digitalWrite(driver.pulse, HIGH);
    delay(30000/(driver.rpm * driver.microsteps));
    digitalWrite(driver.pulse, LOW);
    delay(30000/(driver.rpm * driver.microsteps));
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

void loop(){
  
  
  
}
