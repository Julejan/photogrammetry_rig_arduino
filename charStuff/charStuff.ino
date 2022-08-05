#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define LCD_COLS 20
#define LCD_ROWS 4

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);


static const char* settingsMenu[10] = {"Camera Stepper Speed", "Turntable speed", "Slider stepper speed" , "Slider divisions", "Turntable divisions", "Bottom position", "Bottom rotation", "Top position", "Top rotation", "Back"};

int mainMenuCursorLine = 0;
int mainMenuFirstLine = 0;

void printMainMenu(const char* menu[]) {
  lcd.clear();
  for (int i=0;i<LCD_ROWS;i++) {
    lcd.setCursor(0,i);
    lcd.print(menu[mainMenuFirstLine + i]);
  }
  lcd.setCursor(0,(mainMenuCursorLine-mainMenuFirstLine));
}

void setup()
  {
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.cursor_on();
  printMainMenu(settingsMenu);
  }

void loop() 
  {
    
  }
