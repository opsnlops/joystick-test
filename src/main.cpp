#include <Arduino.h>

#include <ESP32Servo.h>

int x_val = 0;
int y_val = 0;

#define SERVO_MIN 0
#define SERVO_MAX 180

#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 4096

#define PERIOD_HZ 50

Servo servos[2];
const int servo0Pin = 13;
const int servo1Pin = 14;

void setup()
{

  Serial.begin(9600);
  while (!Serial)
    ;
  // Nice long delay to let minicom start
  delay(5000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.println("attaching to servo 0");
  servos[0].setPeriodHertz(PERIOD_HZ);
  servos[0].attach(servo0Pin, 1000, 2000);
  Serial.println("done");

  Serial.println("attaching to servo 1");
  servos[1].setPeriodHertz(PERIOD_HZ);
  servos[1].attach(servo1Pin, 1000, 2000);
  Serial.println("done");
}

void loop()
{

  x_val = analogRead(34);
  y_val = analogRead(35);

  int servo0_val = map(x_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_MIN, SERVO_MAX);
  int servo1_val = map(y_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_MIN, SERVO_MAX);

  //Serial.printf("x: %d, y: %d -> 0: %d, 1: %d\n", x_val, y_val, servo0_val, servo1_val);

  servos[0].write(servo0_val);
  servos[1].write(servo1_val);

  // These servos use a 20ms pulse (50Hz)
  delay(1000 / PERIOD_HZ);

}