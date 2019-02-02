#include <Servo.h>

///////////// SERVO CODE /////////////

struct Servo_t {
  int ctrlPin;
  Servo ctrl;

  int position;

  int minPosition = 0;
  int maxPosition = 180;
};

void Servo_init(struct Servo_t* servo, int ctrlPin)
{
  servo->ctrlPin = ctrlPin;
  servo->ctrl.attach(ctrlPin);
  //servo.ctrl.writeMicroseconds(servo.posMs);
  servo->position = 90;
  servo->ctrl.write(90);
}

void Servo_update(struct Servo_t* servo, int pos)
{
  pos = max(servo->minPosition, pos);
  pos = min(servo->maxPosition, pos);
  
  servo->position = pos;
  servo->ctrl.write(pos);
}

///////////// MAIN CODE /////////////

Servo_t panServo;
Servo_t tiltServo;

void setup() {
  Servo_init(&panServo, 2);
  Servo_init(&tiltServo, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
  Servo_update(&panServo, 100);
  Servo_update(&tiltServo, 100);
}
