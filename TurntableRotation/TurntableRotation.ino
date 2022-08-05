#include <AccelStepper.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120


#define TURNTABLE_GEAR_RATIO 5 // 80 Zähne / 16 Zähne
#define TURN_ACCEL 3000


#define TURNTABLE_DIR 40
#define TURNTABLE_STEP 7
#define TURNTABLE_ENABLE 42

AccelStepper turntableStepper(AccelStepper::FULL2WIRE, TURNTABLE_DIR, TURNTABLE_STEP);

uint16_t turntableStepperSpeed = 100;

void setup(){
    turntableStepper.setMaxSpeed(turntableStepperSpeed * MOTOR_STEPS);
    turntableStepper.setAcceleration(TURN_ACCEL);

     turntableStepper.move(MOTOR_STEPS * TURNTABLE_GEAR_RATIO);
}

void loop(){
    turntableStepper.run();
}
