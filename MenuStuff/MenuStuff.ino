#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <TimerOne.h>

#define PIN_IN1 2
#define PIN_IN2 3
#define PIN_PUSH 4

#define LCD_COLS 20
#define LCD_ROWS 4

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

static const char* mainMenu[4] = {"Start Recording", "Test Settings", "Settings", "Calibrate"};
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
  if (!digitalRead(PIN_PUSH) && !pushButtonOld){
    fired = true;
    pushButtonOld = true;
    pushButtonCount++;
    }
  if(digitalRead(PIN_PUSH) && pushButtonOld){
    pushButtonOld = false;
    }
    
}  // end of isr


void setup(){
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  Timer1.initialize(20000);
  Timer1.attachInterrupt(isrp); // blinkLED to run every 15 milliseconds
  
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.cursor_on();
  // setup the rotary encoder functionality
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  lcd.print("Push Button Count");
}

void selection(){}

void print_menue() {
  lcd.clear();
  for (int i=0;i<LCD_ROWS;i++) {
    lcd.setCursor(0,i);
    lcd.print(mainMenu[DisplayFirstLine + i]);
  }
  lcd.setCursor(0,(CursorLine-DisplayFirstLine));
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
    if ((int)(encoder->getDirection()) == 1)
      move_up();
    else
      move_down();
    }
  else if (fired)
    {
    selection();
    fired = false;
    }  // end if fired
  lcd.setCursor(0,1);
  lcd.print(pushButtonCount);
  delay(50);
}
