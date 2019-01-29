/**
 * Joystick module.
 */

typedef struct
{
  int pinX;
  int pinY;

  int posX;
  int posY;
} Joystick_t;

void Joystick_init(Joystick_t& joy, int pinX, int pinY)
{
  pinMode(pinX, INPUT);
  pinMode(pinY, INPUT);

  joy.pinX = pinX;
  joy.pinY = pinY;
}

void Joystick_update(Joystick_t& joy)
{
  joy.posX = analogRead(joy.pinX);
  joy.posY = analogRead(joy.pinY);
}

/**
 * Main code.
 */

Joystick_t joystick;

void setup() {
  Serial.begin(9600);
  
  Joystick_init(joystick, A0, A1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Joystick_update(joystick);
  Serial.println("Joystick: " + (String) joystick.posX + "," + (String) joystick.posY);
}
