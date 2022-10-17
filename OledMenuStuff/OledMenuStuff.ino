#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RotaryEncoder.h>
#include <TimerOne.h>

#define PIN_IN1 2
#define PIN_IN2 3
#define PIN_PUSH 4
#define LCD_ROWS 7

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const char* mainMenu[10] = {"Start Recording", "Test Settings", "Settings", "Calibrate", "Dummy1", "Dummy2", "Dummy3", "Dummy4", "Dummy5", "Dummy6"};
uint8_t mainMenuSize = sizeof(mainMenu)/sizeof(mainMenu[0]);

int CursorLine = 0;
int DisplayFirstLine = 0;
int settingsMenuCursorLine = 0;
int settingsMenuFirstLine = 0;

int pushButtonCount;
int encoderPositionOld;

bool fired;
bool pushButtonOld;

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

// Interrupt Service Routine for a change to pushpin
void isrp ()
  {
  if (!digitalRead(PIN_PUSH) && !pushButtonOld)
    {
    fired = true;
    pushButtonOld = true;
    }
  
  if(digitalRead(PIN_PUSH) && pushButtonOld)
    {
    pushButtonOld = false;
    }
    
  }  // end of isr  // end of isr


void setup(){
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(9600);
  Serial.print("Init!");
  display.begin(i2c_Address, false);
  delay(250);
  Timer1.initialize(15000);
  Timer1.attachInterrupt(isrp); // blinkLED to run every 15 milliseconds

  // setup the rotary encoder functionality
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  Serial.println("Before menu");
  print_menue();
  Serial.println("Printed Menu");
}

void selection(){}

void print_menue() {
  display.setCursor(0, 0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  for (int i=0;i<LCD_ROWS;i++) {
    //  display.setCursor(0, i);
    if (i == (CursorLine-DisplayFirstLine)){    
      display.setTextColor(SH110X_BLACK, SH110X_WHITE);
      display.println(mainMenu[DisplayFirstLine + i]);
      display.setTextColor(SH110X_WHITE);
      }
     else{
      display.println(mainMenu[DisplayFirstLine + i]);
      }
    
  }
  //  display.setCursor(0,(CursorLine-DisplayFirstLine));
  display.display();
}
 
void move_down() {
  if (CursorLine == (DisplayFirstLine+LCD_ROWS-1)) {
    DisplayFirstLine++;
  }
  if (CursorLine == (mainMenuSize-1)) {
    CursorLine = 0;
    DisplayFirstLine = 0;
  }
  else {
    CursorLine=CursorLine+1;
  }
  print_menue();
} 

void move_up() {
  if ((DisplayFirstLine == 0) & (CursorLine == 0)) {
    DisplayFirstLine = mainMenuSize-LCD_ROWS;   
  } 
  else if (DisplayFirstLine == CursorLine) {
    DisplayFirstLine--;
  }
  if (CursorLine == 0) {
    CursorLine = mainMenuSize-1;
  }
  else {
    CursorLine=CursorLine-1;
  }
  print_menue();
}

void loop(){
  if (encoder->getPosition() > encoderPositionOld || encoder->getPosition() < encoderPositionOld)
    {
    encoderPositionOld = encoder->getPosition();
    if ((int)(encoder->getDirection()) == 1){
      move_up();
      Serial.print("Current encoder value: ");
      Serial.println(encoder->getPosition());
    }else{
      Serial.print("Current encoder value: ");
      Serial.println(encoder->getPosition());
      move_down();
    }
    }
  else if (fired)
    {
    selection();
    Serial.println("Fired!");
    fired = false;
    }  // end if fired
}
