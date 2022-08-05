#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <TimerOne.h>
#include <AccelStepper.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// Parameters for tuning the Motors as needed
#define TURNTABLE_GEAR_RATIO 5
#define SLIDER_MULTIPLIER 30
#define CAM_TURN_MULTIPLIER 2
#define CAM_ACCEL 3000
#define TURN_ACCEL 3000
#define SLIDER_ACCEL 3000

// All the wires needed for full functionality
#define SLIDER_DIR 8
#define SLIDER_STEP 9
#define SLIDER_ENABLE 40
#define CAMERA_DIR 10
#define CAMERA_STEP 18
#define CAMERA_ENABLE 41
#define TURNTABLE_DIR 40
#define TURNTABLE_STEP 7
#define TURNTABLE_ENABLE 42

#define PIN_IN1 2
#define PIN_IN2 3
#define PIN_PUSH 4
#define CAMERA_PIN 11
#define CAMERA_TIME_MULTIPLIER 100

#define LCD_COLS 20
#define LCD_ROWS 4

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

// 2-wire basic config, microstepping is hardwired on the driver
AccelStepper sliderStepper(AccelStepper::FULL2WIRE, SLIDER_DIR, SLIDER_STEP);

// 2-wire basic config, microstepping is hardwired on the driver
AccelStepper cameraStepper(AccelStepper::FULL2WIRE, CAMERA_DIR, CAMERA_STEP);

// 2-wire basic config, microstepping is hardwired on the driver
AccelStepper turntableStepper(AccelStepper::FULL2WIRE, TURNTABLE_DIR, TURNTABLE_STEP);

static const char* mainMenu[4] = {"Start Recording", "Settings", "Calibrate", "Info"};
static const char* settingsMenu[5] = {"Camera Stepper Speed", "Turntable speed", "Slider stepper speed", "Time per photo", "Back"};

// Menu handling variables
uint8_t mainMenuCursorLine = 0;
uint8_t mainMenuFirstLine = 0;
uint8_t settingsMenuCursorLine = 0;
uint8_t settingsMenuFirstLine = 0;

int pushButtonCount;
int encoderPositionOld = 0;

bool bu_pressed = false;
bool pushButtonOld;

int state = 0; // "MainMenu";
uint8_t substate;
int actDir;
bool up = true;
int num_photos;
int act_photos;
bool recordingFinished;
bool recordingStarted;
bool testSettingsFinished;

// Variables for the Stepper
uint16_t sliderSingleSteps;
uint16_t cameraSingleSteps;
uint16_t turntableSingleSteps;
int16_t bottomSteps = 0;
int16_t topSteps = 0;
int16_t bottomRotationSteps = 0;
int16_t topRotationSteps = 0;
uint16_t cameraStepperSpeed = 100;
uint16_t turntableStepperSpeed = 100;
uint16_t sliderStepperSpeed = 100;
int16_t turntableSteps = 0;
int16_t sliderSteps = 0;
int16_t cameraSteps = 0;
uint8_t photosPerSliderDiv[10] = {2, 4, 5, 10, 11, 20, 22, 44, 55, 110};
uint8_t sliderDivs[6] = {2, 3, 5, 7, 9, 11};
uint8_t sliderDivCount;
uint8_t sliderDivSize = sizeof(sliderDivs)/sizeof(sliderDivs[0]);
uint8_t photosPerSliderDivSize = sizeof(photosPerSliderDiv)/sizeof(photosPerSliderDiv[0]);
uint8_t photosPerSliderDivCount;
uint16_t cameraTime = 2500;

int mainMenuSize = sizeof(mainMenu)/sizeof(mainMenu[0]);
int settingsMenuSize = sizeof(settingsMenu)/sizeof(settingsMenu[0]);

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

void moveStepper(AccelStepper& stepper){
  if (stepper.distanceToGo() == 0)
    {
    if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
      {
      stepper.move((encoderPositionOld - encoder->getPosition())*50);
      encoderPositionOld = encoder->getPosition();
      }
    }
  stepper.run();
  }

void checkPosition()
  {
  encoder->tick(); // just call tick() to check the state.
  }

// Interrupt Service Routine for a change to pushpin
void isrp ()
  {
  sliderStepper.run();
  if (!digitalRead(PIN_PUSH) && !pushButtonOld)
    {
    bu_pressed = true;
    pushButtonOld = true;
    }
  
  if(digitalRead(PIN_PUSH) && pushButtonOld)
    {
    pushButtonOld = false;
    }
    
  }  // end of isr

void setup()
  {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);
  pinMode(CAMERA_PIN, OUTPUT);
  Serial.begin(9600);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(isrp); // blinkLED to run every 15 milliseconds
  
  // initialize the LCD
  lcd.begin();
  
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.cursor_on();
  printMenu(mainMenu, mainMenuCursorLine, mainMenuFirstLine, mainMenuSize);
  
  sliderStepper.setMaxSpeed(sliderStepperSpeed * MOTOR_STEPS);
  sliderStepper.setAcceleration(SLIDER_ACCEL);
  cameraStepper.setMaxSpeed(cameraStepperSpeed * MOTOR_STEPS);
  cameraStepper.setAcceleration(CAM_ACCEL);
  turntableStepper.setMaxSpeed(turntableStepperSpeed * MOTOR_STEPS);
  turntableStepper.setAcceleration(TURN_ACCEL);

  // setup the rotary encoder functionality
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
  
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  } // setup()

void printMenu(const char* menu[], uint8_t menuCursorLine, uint8_t menuFirstLine, int menuSize){
  lcd.clear();
  
  if (menuSize > LCD_ROWS)
    {
    for (int i=0;i<LCD_ROWS;i++) 
      {
      lcd.setCursor(0,i);
      lcd.print(menu[menuFirstLine + i]);
      }
    }else{
      for (int i=0;i<menuSize;i++) 
        {
        lcd.setCursor(0,i);
        lcd.print(menu[menuFirstLine + i]);
        }
      }
  
  lcd.setCursor(0,(menuCursorLine-menuFirstLine));
  }

void moveMenuUp(const char* menu[], uint8_t* menuCursorLine, uint8_t* menuFirstLine, int menuSize){
  if ((*menuFirstLine == 0) & (*menuCursorLine == 0)) 
    {
    *menuFirstLine = menuSize-LCD_ROWS;   
    } 
    
    else if (*menuFirstLine == *menuCursorLine) 
    {
    *menuFirstLine--;
    }
    
    if (*menuCursorLine == 0) 
    {
    *menuCursorLine = menuSize-1;
    }
    
    else {
    *menuCursorLine=*menuCursorLine-1;
    }
  
  printMenu(menu, *menuCursorLine, *menuFirstLine, menuSize);
  }

void moveMenuDown(const char* menu[], uint8_t* menuCursorLine, uint8_t* menuFirstLine, int menuSize)
  {
  if (*menuCursorLine == (*menuFirstLine+LCD_ROWS-1)) 
    {
    *menuFirstLine++;
    }
  if (*menuCursorLine == (menuSize-1)) 
    {
    *menuCursorLine = 0;
    *menuFirstLine = 0;
    }
  else 
    {
    *menuCursorLine=*menuCursorLine+1;
    }
  printMenu(menu, *menuCursorLine, *menuFirstLine, menuSize);
  }

// Read the current position of the encoder and print out when changed.
void loop()
{
switch(state){
  case 0: //"MainMenu":
    {
      if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
        {
        actDir = (int)(encoder->getDirection());
        encoderPositionOld = encoder->getPosition();
        if (actDir == 1)
          {
          moveMenuUp(mainMenu, &mainMenuCursorLine, &mainMenuFirstLine, mainMenuSize);
          }

        if(actDir == -1)
          {
          moveMenuDown(mainMenu, &mainMenuCursorLine, &mainMenuFirstLine, mainMenuSize);
          }  
        }

      else if((bu_pressed == true) && (mainMenuCursorLine == 1))
        {
        state = mainMenuCursorLine+1;
        bu_pressed = false;
        printMenu(settingsMenu, settingsMenuCursorLine, settingsMenuFirstLine, settingsMenuSize);
        }

      else if (bu_pressed)
        {
        state = mainMenuCursorLine+1;
        bu_pressed = false;
        }

    }break;
  case 1: //"Start Recording":
    {
      switch (substate)
      {
      case 0:
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Check if subdivision");
        lcd.setCursor(0,1);
        lcd.print("is set correct!");
        lcd.setCursor(0,2);
        lcd.print("Turntable: ");
        lcd.setCursor(12,2);
        lcd.print(photosPerSliderDiv[photosPerSliderDivCount]);
        lcd.setCursor(0,3);
        lcd.print("Slider: ");
        lcd.setCursor(12,3);
        lcd.print(sliderDivs[sliderDivCount]);
        delay(1200);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Adjust or");
        lcd.setCursor(0, 1);
        lcd.print("confirm value");
        lcd.setCursor(0, 2);
        lcd.print("Slider : ");
        lcd.setCursor(0, 3);
        lcd.print("Turntable : ");
        lcd.setCursor(8, 2);
        lcd.print("    ");
        lcd.setCursor(8, 2);
        lcd.print(sliderDivs[sliderDivCount]);
        lcd.setCursor(11, 3);
        lcd.print("    ");
        lcd.setCursor(11, 3);
        lcd.print(photosPerSliderDiv[photosPerSliderDivCount]);
        substate++;
        }break;
      case 1:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld){ 
          encoderPositionOld = encoder->getPosition();
          actDir = (int)(encoder->getDirection());
          if (actDir == 1)
            {
            if (sliderDivCount < sliderDivSize-1) sliderDivCount++;;
            lcd.setCursor(8, 2);
            lcd.print("    ");
            lcd.setCursor(8, 2);
            lcd.print(sliderDivs[sliderDivCount]);
            }
          else if(actDir == -1)
            {
            if (sliderDivCount > 0) sliderDivCount--;
            lcd.setCursor(8, 2);
            lcd.print("    ");
            lcd.setCursor(8, 2);
            lcd.print(sliderDivs[sliderDivCount]);
            }
          }
        if(bu_pressed == true)
          {
          substate++;
          bu_pressed = false;  
          }
        }break;
      case 2:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld){ 
          encoderPositionOld = encoder->getPosition();
          actDir = (int)(encoder->getDirection());
          if (actDir == 1)
            {
            if (photosPerSliderDivCount < photosPerSliderDivSize-1) photosPerSliderDivCount++;
            lcd.setCursor(11, 3);
            lcd.print("    ");
            lcd.setCursor(11, 3);
            lcd.print(photosPerSliderDiv[photosPerSliderDivCount]);
            }
          else if(actDir == -1)
            {
            if (photosPerSliderDivCount > 0) photosPerSliderDivCount--;
            lcd.setCursor(11, 3);
            lcd.print("    ");
            lcd.setCursor(11, 3);
            lcd.print(photosPerSliderDiv[photosPerSliderDivCount]);
            }
          if(bu_pressed == true)
            {
            substate++;
            act_photos = 0;
            sliderSingleSteps = (int)((topSteps - bottomSteps) / sliderDivs[sliderDivCount]);
            Serial.print("SliderSingleSteps: ");
            Serial.println(sliderSingleSteps);
            turntableSingleSteps = (int)(MOTOR_STEPS * TURNTABLE_GEAR_RATIO / photosPerSliderDiv[photosPerSliderDivCount]);
            cameraSingleSteps = (int)((topRotationSteps - bottomRotationSteps) / sliderDivs[sliderDivCount]);
            bu_pressed = false;  
            lcd.clear();
            lcd.setCursor(0, 2);
            lcd.print("Progress: ");
            num_photos = (int)(photosPerSliderDiv[photosPerSliderDivCount] * sliderDivs[sliderDivCount] + photosPerSliderDiv[photosPerSliderDivCount]);
            lcd.setCursor(5, 3);
            lcd.print("0 of ");
            lcd.print(num_photos);
            }
          }  
        }break;
      case 3:
        {
        cameraStepper.moveTo(bottomRotationSteps);
        sliderStepper.moveTo(bottomSteps);
        lcd.setCursor(0,0);
        lcd.print("Camera Value: ");
        lcd.setCursor(15,0);
        lcd.print(cameraStepper.currentPosition());
        lcd.setCursor(0,1);
        lcd.print("Slider Value: ");
        lcd.setCursor(15,1);
        lcd.print(sliderStepper.currentPosition());       
        int displayMillis = millis();
        int displayUpdateTime = 150; 
        while( (cameraStepper.distanceToGo() != 0) || (sliderStepper.distanceToGo() != 0))
          {
          cameraStepper.run();
          sliderStepper.run();
          lcd.setCursor(15,0);
          lcd.print("      ");
          lcd.setCursor(15,0);
          lcd.print(cameraStepper.currentPosition());
          lcd.setCursor(15,1);
          lcd.print("    ");
          lcd.setCursor(15,1);
          lcd.print(sliderStepper.currentPosition());
          }
        for(int j = 0; j < photosPerSliderDiv[photosPerSliderDivCount]; j++)
              {
                turntableStepper.move(turntableSingleSteps);
                while (turntableStepper.distanceToGo() != 0)turntableStepper.run();                
                lcd.setCursor(5, 3);
                act_photos++;
                lcd.print(act_photos);
                lcd.print(" of ");
                lcd.print(num_photos);
                digitalWrite(CAMERA_PIN, HIGH);
                delay(cameraTime);
                digitalWrite(CAMERA_PIN, LOW);
              }
        for(int i = 0; i < sliderDivs[sliderDivCount]; i++)
          {
            cameraStepper.move(cameraSingleSteps);
            sliderStepper.move(sliderSingleSteps);
            while( (cameraStepper.distanceToGo() != 0) || (sliderStepper.distanceToGo() != 0))
              {
              cameraStepper.run();
              sliderStepper.run();
              //if((millis() - displayMillis) > displayUpdateTime)
              //{
              lcd.setCursor(15,0);
              lcd.print("      ");
              lcd.setCursor(15,0);
              lcd.print(cameraStepper.currentPosition());
              lcd.setCursor(15,1);
              lcd.print("    ");
              lcd.setCursor(15,1);
              lcd.print(sliderStepper.currentPosition());
              //}
              }
            // Rotate Turntable Stepper by one increment
            for(int j = 0; j < photosPerSliderDiv[photosPerSliderDivCount]; j++)
              {
                turntableStepper.move(turntableSingleSteps);
                while (turntableStepper.distanceToGo() != 0)turntableStepper.run();                
                lcd.setCursor(5, 3);
                act_photos++;
                lcd.print(act_photos);
                lcd.print(" of ");
                lcd.print(num_photos);
                digitalWrite(CAMERA_PIN, HIGH);
                delay(cameraTime);
                digitalWrite(CAMERA_PIN, LOW);
              }
          }
          substate++;  
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Recording finished");    
        }break;
      case 4:
        {  
        if (bu_pressed == true)
          {
          substate = 0;
          state = 0; //"MainMenu";
          bu_pressed = false;
          printMenu(mainMenu, mainMenuCursorLine, mainMenuFirstLine, mainMenuSize);
          }
        }break;
      }
    }break;
  case 2: //"Settings":
    {
    if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
      {
      encoderPositionOld = encoder->getPosition();
      if ((int)(encoder->getDirection()) == 1)
        moveMenuUp(settingsMenu, &settingsMenuCursorLine, &settingsMenuFirstLine, settingsMenuSize);
      else
        moveMenuDown(settingsMenu, &settingsMenuCursorLine, &settingsMenuFirstLine, settingsMenuSize);
      }
    else if((bu_pressed == true)&& (settingsMenuCursorLine == 4))
      {
        state = 0; //"MainMenu";
        bu_pressed = false;
        printMenu(mainMenu, mainMenuCursorLine, mainMenuFirstLine, mainMenuSize);
      }
    else if(bu_pressed)
      {
        state = settingsMenuCursorLine+5;
        bu_pressed = false;
      }
    }break;
  case 3: //"Calibrate":
    {
    switch (substate)
    {
    case 0:
      {
      sliderStepper.moveTo(topSteps);
      while(sliderStepper.distanceToGo() > 0) sliderStepper.run();
      sliderStepper.setMaxSpeed(200*120);
      sliderStepper.setAcceleration(200*120);
      int steps;
      encoderPositionOld = sliderSteps;
      encoder->setPosition(sliderSteps);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set slider to");
      lcd.setCursor(0, 1);
      lcd.print("top position!");
      lcd.setCursor(0, 2);
      lcd.print("Slider steps: ");  
      lcd.setCursor(13, 2);
      lcd.print(sliderStepper.currentPosition());
      substate++;
      }break;
    case 1:
      {
      if (sliderStepper.distanceToGo() == 0)
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          sliderStepper.move((encoderPositionOld - encoder->getPosition())*SLIDER_MULTIPLIER);
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(13, 2);
          lcd.print("    ");
          lcd.setCursor(13, 2);
          lcd.print(sliderStepper.currentPosition());
          }
        }
      while(sliderStepper.distanceToGo() > 0 || sliderStepper.distanceToGo() < 0) sliderStepper.run();
        if(bu_pressed == true)
          {
          cameraStepper.moveTo(topRotationSteps);
          while(cameraStepper.distanceToGo() > 0 || cameraStepper.distanceToGo() < 0) cameraStepper.run();
          substate++;
          bu_pressed = false;
          topSteps = sliderStepper.currentPosition(); 
          encoderPositionOld = cameraSteps;
          encoder->setPosition(cameraSteps);
          lcd.clear();
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Set camera to");
          lcd.setCursor(0, 1);
          lcd.print("the right angle!");
          lcd.setCursor(0, 2);
          lcd.print("Rotation steps: ");
          lcd.print(cameraStepper.currentPosition());
          } 
        }break;
    case 2:
      {
      if (cameraStepper.distanceToGo() == 0)
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          cameraStepper.move((encoderPositionOld - encoder->getPosition())*CAM_TURN_MULTIPLIER);
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(15, 2);
          lcd.print("    ");
          lcd.setCursor(15, 2);
          lcd.print(cameraStepper.currentPosition());
          }    
        while(cameraStepper.distanceToGo() > 0 || cameraStepper.distanceToGo() < 0) cameraStepper.run();
        }
        if(bu_pressed == true)
          {
          substate++;
          bu_pressed = false; 
          encoderPositionOld = sliderSteps;
          topRotationSteps = cameraStepper.currentPosition();
          encoder->setPosition(sliderSteps);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Set camera to");
          lcd.setCursor(0, 1);
          lcd.print("bottom position!");
          lcd.setCursor(0, 2);
          lcd.print("Slider steps: ");
          sliderStepper.setMaxSpeed(sliderStepperSpeed * MOTOR_STEPS);
          sliderStepper.setAcceleration(SLIDER_ACCEL);
          sliderStepper.moveTo(bottomSteps);
          lcd.print(sliderStepper.currentPosition());
          while(sliderStepper.distanceToGo() > 0 || sliderStepper.distanceToGo() < 0) sliderStepper.run();
          } 
          sliderStepper.setMaxSpeed(120 * MOTOR_STEPS);
          sliderStepper.setAcceleration(120 * MOTOR_STEPS); 
      }break;
    case 3:
      {
      if (sliderStepper.distanceToGo() == 0)
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          sliderStepper.move((encoderPositionOld - encoder->getPosition())*SLIDER_MULTIPLIER);
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(13, 2);
          lcd.print("    ");
          lcd.setCursor(13, 2);
          lcd.print(sliderStepper.currentPosition());
          }
          while(sliderStepper.distanceToGo() > 0 || sliderStepper.distanceToGo() < 0) sliderStepper.run();
        }
        if(bu_pressed == true)
          {
          substate++;
          bu_pressed = false; 
          encoderPositionOld = bottomRotationSteps;
          bottomSteps = sliderStepper.currentPosition();
          encoder->setPosition(bottomRotationSteps);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Set camera to");
          lcd.setCursor(0, 1);
          lcd.print("the right angle!");
          lcd.setCursor(0, 2);
          lcd.print("Rotation steps: "); 
          lcd.print(cameraStepper.currentPosition());
          cameraStepper.moveTo(bottomRotationSteps);
          while(cameraStepper.distanceToGo() > 0) cameraStepper.run();
          }  
      }break;
    case 4:
      {
      if (cameraStepper.distanceToGo() == 0)
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          cameraStepper.move((encoderPositionOld - encoder->getPosition())*CAM_TURN_MULTIPLIER);
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(15, 2);
          lcd.print("    ");
          lcd.setCursor(15, 2);
          lcd.print(cameraStepper.currentPosition());
          }
          while(cameraStepper.distanceToGo() > 0 || cameraStepper.distanceToGo() < 0) cameraStepper.run();
        }
        if(bu_pressed == true)
          {
          substate = 0;
          bu_pressed = false; 
          state = 0;
          lcd.clear();
          sliderStepper.setMaxSpeed(sliderStepperSpeed * MOTOR_STEPS);
          sliderStepper.setAcceleration(SLIDER_ACCEL);
          lcd.print("Calibration finished");
          delay(1000);
          printMenu(mainMenu, mainMenuCursorLine, mainMenuFirstLine, mainMenuSize);
          }  
        }break;
      }
    }break;
  case 4:
    {
      switch (substate)
      {
      case 0:
        {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Scan Software V1.0 ");
        lcd.setCursor(9, 1);
        lcd.print("by");
        lcd.setCursor(3, 2);
        lcd.print("Julian Haasis");
        substate++;
        }break;
      
      case 1:
        {
        if(bu_pressed == true){
        state = 0;
        bu_pressed = false;
        substate = 0;
        printMenu(mainMenu, mainMenuCursorLine, mainMenuFirstLine, mainMenuSize);
        }
        }break;
      }
    }break;  
  case 5: //"Camera Stepper Speed":
    {
    switch (substate)
      {
      case 0:
        {
        encoder->setPosition(cameraStepperSpeed);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Camera Stepper Speed");
        lcd.setCursor(0,1);
        lcd.print("Speed in RPM: ");
        substate++;
        }break;
      case 1:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(14, 1);
          lcd.print("     ");
          lcd.setCursor(14,1);
          lcd.print(encoder->getPosition());
          }
        if(bu_pressed == true){
        state = 2; //"Settings";  
        bu_pressed = false;
        substate = 0;
        cameraStepperSpeed = encoder->getPosition();
        cameraStepper.setMaxSpeed(cameraStepperSpeed * MOTOR_STEPS);
        printMenu(settingsMenu, settingsMenuCursorLine, settingsMenuFirstLine, settingsMenuSize);
        }
        }break;
      }
    }break;
  case 6: //"Turntable speed":
    {
    switch (substate)
      {
      case 0:
        {
        encoder->setPosition(turntableStepperSpeed);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Turntable Stepper Speed");
        lcd.setCursor(0,1);
        lcd.print("Speed in RPM: ");
        substate++;
        }break;
      case 1:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(14, 1);
          lcd.print("     ");
          lcd.setCursor(14,1);
          lcd.print(encoder->getPosition());
          }
        if(bu_pressed == true){
        state = 2; //"Settings";  
        bu_pressed = false;
        substate = 0;
        turntableStepperSpeed = encoder->getPosition();
        turntableStepper.setMaxSpeed(turntableStepperSpeed * MOTOR_STEPS);
        printMenu(settingsMenu, settingsMenuCursorLine, settingsMenuFirstLine, settingsMenuSize);
        }
        }break;
      }
    }break;
  case 7: //"Slider stepper speed":
    {
    switch (substate)
      {
      case 0:
        {
        encoder->setPosition(sliderStepperSpeed);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Slider Stepper Speed");
        lcd.setCursor(0,1);
        lcd.print("Speed in RPM: ");
        substate++;
        }break;
      case 1:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(14, 1);
          lcd.print("     ");
          lcd.setCursor(14,1);
          lcd.print(encoder->getPosition());
          }
        if(bu_pressed == true){
        state = 2; //"Settings";  
        bu_pressed = false;
        substate = 0;
        sliderStepperSpeed = encoder->getPosition();
        sliderStepper.setMaxSpeed(sliderStepperSpeed * MOTOR_STEPS);
        printMenu(settingsMenu, settingsMenuCursorLine, settingsMenuFirstLine, settingsMenuSize);
        }
        }break;
      }
    }break;
    case 8: //"Time per photo":
      {
      switch (substate)
      {
      case 0:
        {
        encoder->setPosition(cameraTime);
        encoderPositionOld = encoder->getPosition();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Time per photo");
        lcd.setCursor(0,1);
        lcd.print("Time in ms: ");
        lcd.print(cameraTime);
        substate++;
        }break;
      case 1:
        {
        if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
          {
          cameraTime += (encoderPositionOld - encoder->getPosition())*CAMERA_TIME_MULTIPLIER;  
          encoderPositionOld = encoder->getPosition();
          lcd.setCursor(12, 1);
          lcd.print("     ");
          lcd.setCursor(12,1);
          lcd.print(cameraTime);
          }
        if(bu_pressed == true){
        state = 2; //"Settings";  
        bu_pressed = false;
        substate = 0;
        printMenu(settingsMenu, settingsMenuCursorLine, settingsMenuFirstLine, settingsMenuSize);
        }
        }break;
      }
    }break;
  }
    
}// loop ()
