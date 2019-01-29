/**
 * Joystick module.
 */

const float k_joystickAlpha = 0.2;

typedef struct
{
  int pinX;
  int pinY;

  int posX;
  int posY;
} Joystick_t;

/**
 * Initialize a Joystick.
 */
void Joystick_init(Joystick_t& joy, int pinX, int pinY)
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
void Joystick_update(Joystick_t& joy)
{
  joy.posX = (1 - k_joystickAlpha) * analogRead(joy.pinX) + k_joystickAlpha * joy.posX;
  joy.posY = (1 - k_joystickAlpha) * analogRead(joy.pinY) + k_joystickAlpha * joy.posY;
}

/**
 * Main code.
 */

Joystick_t joystick;

void setup() {
  
  Joystick_init(joystick, A0, A1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Joystick_update(joystick);
  Serial.println("Joystick: " + (String) joystick.posX + "," + (String) joystick.posY);
}
