#include <Servo.h>

const int k_maxServoSpeed = 10;

///////////// SERVO CODE /////////////

struct Servo_t {
  int ctrlPin;
  Servo ctrl;

  int position;

  int minPosition = 0;
  int maxPosition = 180;
};

int JoystickValToServoSpeed(int joystickVal)
{
  joystickVal -= 512;
  joystickVal = (int) ((joystickVal / 512.0f) * k_maxServoSpeed);
  return joystickVal;
}

void Servo_init(struct Servo_t* servo, int ctrlPin)
{
  servo->ctrlPin = ctrlPin;
  servo->ctrl.attach(ctrlPin);
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

///////////// BLUETOOTH CODE /////////////

struct BluetoothArgs_t {
  int servoX;
  int servoY;
  int roombaX;
  int roombaY;
  
  bool laserEnable = 0;
};

void Bluetooth_init()
{
  Serial.begin(9600);
  Serial.println("Starting bluetooth...");
  Serial1.begin(9600);
}

struct Servo_t g_panServo;
struct Servo_t g_tiltServo;
struct BluetoothArgs_t g_lastArgs;

///////////// TASKS /////////////////

void Task_updateBluetooth()
{
  // Message format (13 chars)
  // 00 00 00 00 0
  if (Serial1.available() >= 14)
  {
    g_lastArgs.servoX = Serial1.parseInt();
    g_lastArgs.servoY = Serial1.parseInt();
    g_lastArgs.roombaX = Serial1.parseInt();
    g_lastArgs.roombaY = Serial1.parseInt();
    g_lastArgs.laserEnable = Serial1.parseInt() == 0 ? false : true;

    char bytes[64];
    Serial1.readBytesUntil('\n', bytes, 64);
    Serial.println("ServoX: " + (String)g_lastArgs.servoX + " ServoY: " + (String)g_lastArgs.servoY);
  }
}

void Task_updateServoPan()
{
  int nextPos = g_panServo.position + g_lastArgs.servoX;
  Servo_update(&g_panServo, nextPos);
}

void Task_updateServoTilt()
{
  int nextPos = g_tiltServo.position + g_lastArgs.servoY;
  Servo_update(&g_tiltServo, g_lastArgs.servoY);
}

///////////// MAIN CODE /////////////

void setup() {
  Bluetooth_init();
  Servo_init(&g_panServo, 2);
  Servo_init(&g_tiltServo, 3);
}

void loop() {
  Task_updateBluetooth();
  Task_updateServoPan();
  Task_updateServoTilt();
}
