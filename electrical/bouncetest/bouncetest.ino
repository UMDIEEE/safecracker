// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <AccelStepper.h>

/*
 * revolution set to 1600 (microstepping)
 * max speed at rev/4
 * max accel at rev/2
 * current set to 1.5A (max 1.7)
 */

#define REVOLUTION 1600

#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
void setup()
{  
  // Change these to suit your stepper if you want
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  stepper.setMaxSpeed(REVOLUTION/4);
  stepper.setAcceleration(REVOLUTION/2);
  stepper.moveTo(REVOLUTION);
}

void delayfun(int delaytime) {
    while(stepper.distanceToGo() != 0)
      stepper.run();
    delay(delaytime);
}

void loop()
{
    digitalWrite(ENA_PIN, HIGH);
    // If at the end of travel go to the other end
//    if (stepper.distanceToGo() == 0)
//      delay(1000);
//      stepper.moveTo(REVOLUTION+stepper.currentPosition());
//    stepper.run();
  static long pos = 0;
  for(int i = 0; i < 5; i++) {
    pos += REVOLUTION;
    stepper.moveTo(pos);
    delayfun(1000);
  }

  for(int i = 0; i < 5; i++) {
    pos -= REVOLUTION;
    stepper.moveTo(pos);
    delayfun(1000);
  }
/*
  stepper.moveTo(REVOLUTION/2);
  delayfun(1000);
  stepper.moveTo(-REVOLUTION/2);
  delayfun(1000);
  stepper.moveTo(0);
  delayfun(1000);

  stepper.moveTo(REVOLUTION/10);
  delayfun(0);
  stepper.moveTo(-REVOLUTION/10);
  delayfun(0);
  stepper.moveTo(0);
  delayfun(0);
 
  for(int i = 0; i < 5; i++) {
      stepper.moveTo(REVOLUTION/100);
      delayfun(500);
      stepper.moveTo(-REVOLUTION/100);
      delayfun(500);
      stepper.moveTo(0);
      delayfun(500);
  }

    for(int i = 0; i < 20; i++) {
      stepper.moveTo(REVOLUTION/100);
      delayfun(00);
      stepper.moveTo(-REVOLUTION/100);
      delayfun(00);
      stepper.moveTo(0);
      delayfun(00);
  }
  */
}
