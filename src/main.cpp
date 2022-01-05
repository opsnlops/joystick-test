#include <Arduino.h>
#include <ESP32Servo.h>

#include "MegunoLink.h"
#include "Filter.h"

int x1_val = 0;
int y1_val = 0;
int x2_val = 0;
int y2_val = 0;
int servo0_val = 0;
int servo1_val = 0;
int servo2_val = 0;
int servo3_val = 0;

#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500

#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 4095

#define DS4_ANALOG_STICK_MIN -128
#define DS4_ANALOG_STICK_MAX 127

const int PERIOD_HZ = 50;
const int SENSOR_OVERSAMPLING = 4;

Servo servos[4];
const int servo0Pin = 13;
const int servo1Pin = 14;
const int servo2Pin = 12;
const int servo3Pin = 15;

const int joystick0Pin = 34;
const int joystick1Pin = 35;

void TaskReadAnalogPin(void *pvParameters);
void TaskMoveServos(void *pvParameters);

void setup()
{

    Serial.begin(152000);
    while (!Serial)
        ;

    pinMode(joystick0Pin, INPUT);
    pinMode(joystick1Pin, INPUT);

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

    // Run on core 0
    xTaskCreatePinnedToCore(TaskReadAnalogPin, "TaskReadAnalogPin", 10240, NULL, 2, NULL, 0);

    // Run on core 1
    xTaskCreatePinnedToCore(TaskMoveServos, "TaskMoveServos", 2048, NULL, 1, NULL, 1);
}

void loop()
{
    // Kill this loop
    vTaskDelete(NULL);
}

void TaskReadAnalogPin(void *pvParameters)
{

    int readPeriod = (PERIOD_HZ * SENSOR_OVERSAMPLING);
    log_i("Reading the senor at %dHz", readPeriod);

    // Calculate the length of the period only once since we're going to
    // be doing this a lot
    TickType_t taskPeriod = pdMS_TO_TICKS(1000 / readPeriod);

    int count = 0;
    long ticks = 0;

    ExponentialFilter<int> Filter0(44, (JOYSTICK_MAX - JOYSTICK_MAX) / 2);
    ExponentialFilter<int> Filter1(44, (JOYSTICK_MAX - JOYSTICK_MAX) / 2);

    for (;;)
    {
        // Read the value of the pin
        Filter0.Filter(analogRead(joystick0Pin));
        Filter1.Filter(analogRead(joystick1Pin));

        if (count++ >= SENSOR_OVERSAMPLING)
        {
            int currentValue0 = Filter0.Current();
            int currentValue1 = Filter1.Current();
            servo0_val = map(currentValue0, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
            servo1_val = map(currentValue1, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);

            log_v("value: %d (%d)", currentValue0, servo0_val);

            count = 0;
        }

        if (ticks++ % 1000 == 0)
        {
            log_v("Tick %d", ticks++);
        }

        vTaskDelay(taskPeriod);
    }
}

void TaskMoveServos(void *pvParameters)
{

    TickType_t taskPeriod = pdMS_TO_TICKS(1000 / PERIOD_HZ);

    for (;;)
    {
        // Delay and wait for the sample to arrive
        vTaskDelay(taskPeriod);

        servos[0].writeMicroseconds(servo0_val);
        servos[1].writeMicroseconds(servo1_val);
    }
}