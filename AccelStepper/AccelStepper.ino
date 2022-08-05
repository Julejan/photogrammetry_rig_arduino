#include <AccelStepper.h>
#include <MultiStepper.h>
#include <RotaryEncoder.h>

#define PIN_IN1 2
#define PIN_IN2 3

int encoderPositionOld = 0;

RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}


AccelStepper stepper(AccelStepper::FULL2WIRE, 8, 9);

void setup(){
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);
  
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  stepper.moveTo(5000);
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(5000);
  // setup the rotary encoder functionality
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
  encoderPositionOld = encoder->getPosition();
  Serial.println((encoderPositionOld - encoder->getPosition())*50);
  }


void loop(){
  if (stepper.distanceToGo() == 0)
    {
      if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld){
      Serial.println((encoderPositionOld - encoder->getPosition())*50);
      stepper.move((encoderPositionOld - encoder->getPosition())*50);
      encoderPositionOld = encoder->getPosition();
      }
    }
  stepper.run();
  }
