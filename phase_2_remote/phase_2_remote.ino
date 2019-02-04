#include "scheduler.h"
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
  
  bool laserEnable = false;
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

  String msg = Serial1.readStringUntil('\n');
  
  sscanf(msg.c_str(), "%02d %02d %02d %02d %d",
    &g_lastArgs.servoX,
    &g_lastArgs.servoY,
    &g_lastArgs.roombaX,
    &g_lastArgs.roombaY,
    &g_lastArgs.laserEnable);

  // Flush the stream if we're falling behind
  if (Serial1.available() >= 14)
  {
    Serial1.flush();
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
  Servo_update(&g_tiltServo, nextPos);
}

void Task_updateLaser()
{
  if (g_lastArgs.laserEnable)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);
}

///////////// MAIN CODE /////////////

void setup() {
  Scheduler_Init();
  
  Bluetooth_init();
  Servo_init(&g_panServo, 2);
  Servo_init(&g_tiltServo, 3);
  pinMode(13, OUTPUT);

  Scheduler_StartTask(0, 80, Task_updateServoPan);
  Scheduler_StartTask(10, 80, Task_updateServoTilt);
  Scheduler_StartTask(20, 80, Task_updateLaser);
  Scheduler_StartTask(30, 80, Task_updateBluetooth);
}

void loop() {
  Scheduler_Dispatch();
}
