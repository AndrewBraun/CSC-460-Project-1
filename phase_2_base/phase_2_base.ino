#include "scheduler.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

///////////// PINS /////////////
const int photoresistor_pin = A15; // pin of photoresistor
const int joystick_button_pin = 40; // pin of joystick button (on the servo joystick)

///////////// VALUES /////////////
int photoresistor_value = 512;
bool laser_ON = false; // current value of laser
int joystick_button_value = 1; // previous value of the joystick button

///////////// JOYSTICK CODE /////////////
const float k_joystickAlpha = 0.2;

struct Joystick_t
{
  int pinX;
  int pinY;

  int posX;
  int posY;
};

struct Joystick_t servo_joystick, roomba_joystick;

/**
 * Initialize a Joystick.
 */
void Joystick_init(struct Joystick_t* joy, int pinX, int pinY)
{
  pinMode(pinX, INPUT);
  pinMode(pinY, INPUT);

  joy->pinX = pinX;
  joy->pinY = pinY;
  joy->posX = analogRead(pinX);
  joy->posY = analogRead(pinY);
}

/**
 * Update joystick values using exponential averaging.
 */
void servo_joystick_update_task(){
  servo_joystick.posX = (1 - k_joystickAlpha) * analogRead(servo_joystick.pinX) + k_joystickAlpha * servo_joystick.posX;
  servo_joystick.posY = (1 - k_joystickAlpha) * analogRead(servo_joystick.pinY) + k_joystickAlpha * servo_joystick.posY;
}

void roomba_joystick_update_task(){
  roomba_joystick.posX =(1 - k_joystickAlpha) * analogRead(roomba_joystick.pinX) + k_joystickAlpha * roomba_joystick.posX;
  roomba_joystick.posY = (1 - k_joystickAlpha) * analogRead(roomba_joystick.pinY) + k_joystickAlpha * roomba_joystick.posY;
}

///////////// LCD Code /////////////

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/*
 * Initalize the LCD
 */
void lcd_setup() {
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Photoresistor:");
  lcd.setCursor(0,1);
  lcd.print("Joystick x axis:");
  lcd.setCursor(0,2);
  lcd.print("Joystick y axis:");
  lcd.setCursor(0,3);
  lcd.print("Joystick button:");
}

/*
 * Writes to the LCD: 
 * the light resistor value
 * the joystick's x postion
 * the joystick's y position 
 * the joystick button's value
 */
void write_to_lcd_task() {
  
  lcd.setCursor(14,0);
  printInt(photoresistor_value);
  
  lcd.setCursor(16,1);
  printInt(servo_joystick.posX);

  lcd.setCursor(16,2);
  printInt(servo_joystick.posY);

  lcd.setCursor(16,3);
  lcd.print(joystick_button_value);
}

/*
 * Helper function to print to LCD because Arduino doesn't have printf.
 */
void printInt(int integer) {
  lcd.print(integer);
  if (integer < 1000){
    lcd.print(' ');
    if (integer < 100) {
      lcd.print(' ');
      if (integer < 10) {
        lcd.print(' ');
      }
    }
  }
}

///////////// BUTTON/LASER CODE /////////////

/*
 * Read the joystick button value.
 * If the button is being pushed down, and it was not being pushed down earlier,
 * change the laser state.
 */
void read_laser_button_task() {
  int new_joystick_button_value = digitalRead(joystick_button_pin);
  
  if (!new_joystick_button_value && joystick_button_value){
    laser_ON = !laser_ON;
  }
  
  joystick_button_value = new_joystick_button_value;
}

///////////// PHOTORESISTOR CODE /////////////

void read_photoresistor_task() {
  photoresistor_value = analogRead(photoresistor_pin);
}

///////////// BLUETOOTH CODE /////////////

void write_to_bluetooth_task() {

  char msgBuf[3];
  
  sprintf(msgBuf, "%02d ", determine_direction(servo_joystick.posX));
  Serial.print(msgBuf);

  sprintf(msgBuf, "%02d ", determine_direction(servo_joystick.posY));
  Serial.print(msgBuf);

  sprintf(msgBuf, "%02d ", determine_direction(roomba_joystick.posX));
  Serial.print(msgBuf);

  sprintf(msgBuf, "%02d ", determine_direction(roomba_joystick.posY));
  Serial.print(msgBuf);

  Serial.println(String(laser_ON));
}

/*
 * Takes as input a joystick value and returns which direction the servo/roomba should move.
 */
int determine_direction (int value) {
  int direction = 0;
  
  if (value < 300) {
    direction = -1;
  }
  else if (value > 800) {
    direction = 1;
  }

  return direction;
}

///////////// MAIN CODE /////////////

void setup() {

  Joystick_init(&servo_joystick, A8, A9);
  Joystick_init(&roomba_joystick, A0, A1);

  lcd_setup();

  pinMode(photoresistor_pin, INPUT);
  pinMode(joystick_button_pin, INPUT_PULLUP);

  Serial.begin(9600);
  Serial1.begin(9600);

  Scheduler_Init();

  Scheduler_StartTask(0, 20, servo_joystick_update_task);
  Scheduler_StartTask(10, 20, roomba_joystick_update_task);
  Scheduler_StartTask(5, 10, read_photoresistor_task);
  Scheduler_StartTask(15, 10, read_laser_button_task);
  Scheduler_StartTask(50, 250, write_to_lcd_task);
  Scheduler_StartTask(25, 50, write_to_bluetooth_task);
}

void loop() {
  // put your main code here, to run repeatedly:
  Scheduler_Dispatch();
}
