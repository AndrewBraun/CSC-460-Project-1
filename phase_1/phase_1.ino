// LCD libraries and code from http://arduinoinfo.mywikis.net/wiki/LCD-Blue-I2C

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

///// JOYSTICK CODE
const float k_joystickAlpha = 0.2;

struct Joystick_t
{
  int pinX;
  int pinY;

  int posX;
  int posY;

  // Some joysticks are offset from the center. Add this value to
  // compensate.
  int centerOffset = 137;

  int deadband = 50;
};

/**
 * Initialize a Joystick.
 */
void Joystick_init(struct Joystick_t joy, int pinX, int pinY)
{
  pinMode(pinX, INPUT);
  pinMode(pinY, INPUT);

  joy.pinX = pinX;
  joy.pinY = pinY;
  joy.posX = analogRead(pinX);
  joy.posY = analogRead(pinY);
}

/**
 * Update joystick values using exponential averaging.
 */
void Joystick_update(struct Joystick_t& joy)
{
  joy.posX = (1 - k_joystickAlpha) * analogRead(joy.pinX) + k_joystickAlpha * joy.posX;
  joy.posY = (1 - k_joystickAlpha) * analogRead(joy.pinY) + k_joystickAlpha * joy.posY;

  // Map to range [-512, 512]
  joy.posX += joy.centerOffset - 512;
  joy.posY += joy.centerOffset - 512;
}

bool Joystick_isInDeadband(struct Joystick_t joy, char axis)
{
  if (axis == 'x')
    return abs(joy.posX) < joy.deadband;
  if (axis == 'y')
    return abs(joy.posY) < joy.deadband;
}

///// LCD Code

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
  lcd.print("Stick x axis:");
  lcd.setCursor(0,2);
  lcd.print("Stick y axis:");
}

/*
 * Write the light resistor value, the joystick's x postion, and the joystick's y position to the LCD
 */
void write_to_lcd(int light_resistor, int joystick_x_value, int joystick_y_value) {

  lcd.setCursor(14,0);
  lcd.print(light_resistor);

  lcd.setCursor(13,1);
  lcd.print(joystick_x_value);

  lcd.setCursor(13,2);
  lcd.print(joystick_y_value);
}

///// SERVO CODE

const unsigned int k_minServoPosMs = 400;
const unsigned int k_maxServoPosMs = 1600;

struct Servo_t {
  int ctrlPin;
  Servo ctrl;

  unsigned int posMs = 1000;
};

void Servo_init(struct Servo_t& servo, int ctrlPin)
{
  servo.ctrlPin = ctrlPin;
  servo.ctrl.attach(ctrlPin);
  servo.ctrl.writeMicroseconds(servo.posMs);
}

void Servo_updateposms(struct Servo_t& servo, unsigned int ms)
{
  ms = max(ms, k_minServoPosMs);
  ms = min(ms, k_maxServoPosMs);

  servo.ctrl.writeMicroseconds(ms);
  servo.posMs = ms;
}

///// MAIN CODE

Servo_t pan, tilt;
Joystick_t joystick;

const unsigned int k_maxServoSpeed = 10; // microseconds/s

int photoresistor = A15;

void setup() {
  
  Servo_init(tilt, 3);
  Servo_init(pan, 2);
  Joystick_init(joystick, A0, A1);

  Serial.begin(9600);

  lcd_setup();

  pinMode(photoresistor, INPUT);
}

void loop() {
  
  Joystick_update(joystick);

  if (!Joystick_isInDeadband(joystick, 'y')){
    int nextTiltPos = tilt.posMs + (joystick.posY / 512.0f) * k_maxServoSpeed;
    Servo_updateposms(tilt, nextTiltPos);
    //Serial.println("Servo: " + (String) tilt.posMs);
  }

  if (!Joystick_isInDeadband(joystick, 'x')){
    int nextPanPos = pan.posMs + (joystick.posX / 512.0f) * k_maxServoSpeed;
    Servo_updateposms(pan, nextPanPos);
    //Serial.println("Servo: " + (String) pan.posMs);
  }

  int photoresistor_value = analogRead(photoresistor);

  write_to_lcd(photoresistor_value, joystick.posX, joystick.posY);
}
