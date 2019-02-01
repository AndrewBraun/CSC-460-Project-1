// LCD libraries and code from http://arduinoinfo.mywikis.net/wiki/LCD-Blue-I2C

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

///////////// JOYSTICK CODE /////////////
const float k_joystickAlpha = 0.2;

struct Joystick_t
{
  int pinX;
  int pinY;

  int posX;
  int posY;
};

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
void Joystick_update(struct Joystick_t* joy)
{
  joy->posX = (1 - k_joystickAlpha) * analogRead(joy->pinX) + k_joystickAlpha * joy->posX;
  joy->posY = (1 - k_joystickAlpha) * analogRead(joy->pinY) + k_joystickAlpha * joy->posY;
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
void write_to_lcd(int light_resistor, int joystick_x_value, int joystick_y_value, int joystick_button) {
  
  lcd.setCursor(14,0);
  printInt(light_resistor);
  
  lcd.setCursor(16,1);
  printInt(joystick_x_value);

  lcd.setCursor(16,2);
  printInt(joystick_y_value);

  lcd.setCursor(16,3);
  lcd.print(joystick_button);
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

///////////// SERVO CODE /////////////

struct Servo_t {
  int ctrlPin;
  Servo ctrl;

  int position;
};

void Servo_init(struct Servo_t* servo, int ctrlPin)
{
  servo->ctrlPin = ctrlPin;
  servo->ctrl.attach(ctrlPin);
  //servo.ctrl.writeMicroseconds(servo.posMs);
  servo->position = 90;
  servo->ctrl.write(90);
}

///////////// BUTTON/LASER CODE /////////////

int joystick_button_pin = 52; // pin of joystick button
int laserPin = LED_BUILTIN; // pin of laser
bool laser_ON = false; // current value of laser
int previous_joystick_button_value = 1; // previous value of the joystick button

void toggle_laser() {
  if (laser_ON) {
    analogWrite(laserPin, 0);
    laser_ON = false;
  }
  else {
    analogWrite(laserPin, 255);
    laser_ON = true;
  }
}

///////////// MAIN CODE /////////////

struct Servo_t pan, tilt;
struct Joystick_t joystick;

const unsigned int k_maxServoSpeed = 10; // microseconds/s

int photoresistor = A15; // pin of photoresistor

void setup() {
  
  Servo_init(&tilt, 8);
  Servo_init(&pan, 9);
  
  Joystick_init(&joystick, A0, A1);

  Serial.begin(9600);

  lcd_setup();

  pinMode(photoresistor, INPUT);

  pinMode(joystick_button_pin, INPUT_PULLUP);
  pinMode(laserPin, OUTPUT);
  analogWrite(laserPin, 0);
}

void loop() {
  
  Joystick_update(&joystick);

  if (joystick.posY < 300 && tilt.position > 0) {
    tilt.position--;
  }
  else if (joystick.posY > 800 && tilt.position < 180) {
    tilt.position++;
  }
  tilt.ctrl.write(tilt.position);
      
  if (joystick.posX < 300 && pan.position > 0) {
    pan.position--;
  }
  else if (joystick.posX > 800 && pan.position < 180) {
    pan.position++;
  }
  pan.ctrl.write(pan.position);

  int photoresistor_value = analogRead(photoresistor);
  
  int joystick_button_value = digitalRead(joystick_button_pin);
  if (!joystick_button_value && previous_joystick_button_value){
    toggle_laser();
  }
  previous_joystick_button_value = joystick_button_value;

  write_to_lcd(photoresistor_value, joystick.posX, joystick.posY, joystick_button_value);
}
