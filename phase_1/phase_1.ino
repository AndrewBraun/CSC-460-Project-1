#include <Servo.h>

/**
 * Joystick module.
 */

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

/**
 * Servo module
 */

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

/**
 * Main code.
 */

Servo_t pan, tilt;
Joystick_t joystick;

const unsigned int k_maxServoSpeed = 10; // microseconds/s

void setup() {
  Servo_init(tilt, 3);
  Joystick_init(joystick, A0, A1);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Joystick_update(joystick);

  do
  {
    // Skip updating servo if in deadband
    if (Joystick_isInDeadband(joystick, 'y'))
      break;
    
    int nextTiltPos = tilt.posMs + (joystick.posY / 512.0f) * k_maxServoSpeed;
    Servo_updateposms(tilt, nextTiltPos);
    Serial.println("Servo: " + (String) tilt.posMs);
  }
  while (0);
}
