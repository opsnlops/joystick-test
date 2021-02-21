#include <Arduino.h>

#include <ESP32Servo.h>
#include <PS4Controller.h>

int x1_val = 0;
int y1_val = 0;
int x2_val = 0;
int y2_val = 0;
int servo0_val = 0;
int servo1_val = 0;
int servo2_val = 0;
int servo3_val = 0;

#define USE_PS4
//#define USE_JOYSTICK

#define SERVO_US_MIN 1000
#define SERVO_US_MAX 2000

#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 4096

#define DS4_ANALOG_STICK_MIN -128
#define DS4_ANALOG_STICK_MAX 127

#define PERIOD_HZ 50

Servo servos[4];
const int servo0Pin = 13;
const int servo1Pin = 14;
const int servo2Pin = 12;
const int servo3Pin = 15;

void setup()
{

  Serial.begin(9600);
  while (!Serial)
    ;
  // Nice long delay to let minicom start
  delay(5000);

#ifdef USE_PS4
  log_i("Enabling Bluetooth...");
  if (!PS4.begin("e4:17:d8:9d:70:6b"))
  {
    log_e("PS4 init failed");
  }
#endif

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  log_i("attaching to servo 0");
  servos[0].setPeriodHertz(PERIOD_HZ);
  servos[0].attach(servo0Pin, SERVO_US_MIN, SERVO_US_MAX);
  log_i("done");

  log_i("attaching to servo 1");
  servos[1].setPeriodHertz(PERIOD_HZ);
  servos[1].attach(servo1Pin, SERVO_US_MIN, SERVO_US_MAX);
  log_i("done");

#ifdef USE_PS4
  log_i("attaching to servo 2");
  servos[2].setPeriodHertz(PERIOD_HZ);
  servos[2].attach(servo2Pin, SERVO_US_MIN, SERVO_US_MAX);
  log_i("done");

  log_i("attaching to servo 3");
  servos[3].setPeriodHertz(PERIOD_HZ);
  servos[3].attach(servo3Pin, SERVO_US_MIN, SERVO_US_MAX);
  log_i("done");
#endif
}

void loop()
{

#ifdef USE_JOYSTICK
  x1_val = analogRead(34);
  y1_val = analogRead(35);

  servo0_val = map(x1_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
  servo1_val = map(y1_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);

  log_v("x: %d, y: %d -> 0: %d, 1: %d\n", x1_val, y1_val, servo0_val, servo1_val);
  servos[0].writeMicroseconds(servo0_val);
  servos[1].writeMicroseconds(servo1_val);
#endif

#ifdef USE_PS4
  x1_val = PS4.RStickX();
  y1_val = PS4.RStickY();
  x2_val = PS4.LStickX();
  y2_val = PS4.LStickY();

  servo0_val = map(x1_val, DS4_ANALOG_STICK_MIN, DS4_ANALOG_STICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
  servo1_val = map(y1_val, DS4_ANALOG_STICK_MIN, DS4_ANALOG_STICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
  servo2_val = map(x2_val, DS4_ANALOG_STICK_MIN, DS4_ANALOG_STICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
  servo3_val = map(y2_val, DS4_ANALOG_STICK_MIN, DS4_ANALOG_STICK_MAX, SERVO_US_MIN, SERVO_US_MAX);

  log_v("x1: %d, y1: %d -> 0: %d, 1: %d   x2: %d, y2: %d -> 0: %d, 1: %d\n", x1_val, y1_val, servo0_val, servo1_val, x2_val, y2_val, servo2_val, servo3_val);

  servos[0].writeMicroseconds(servo0_val);
  servos[1].writeMicroseconds(servo1_val);
  servos[2].writeMicroseconds(servo2_val);
  servos[3].writeMicroseconds(servo3_val);
#endif

  // These servos use a 20ms pulse (50Hz)
  delay(1000 / PERIOD_HZ);
}