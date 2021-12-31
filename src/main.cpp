#include <Arduino.h>
#include <ESP32Servo.h>

#ifdef USE_PS4
    #include <PS4Controller.h>
#endif

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

#define SERVO_US_MIN 1000
#define SERVO_US_MAX 2000

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

void TaskReadAnalogPin(void *pvParameters);

void setup()
{

    Serial.begin(152000);
    while (!Serial)
        ;

#ifdef USE_PS4
    log_i("Enabling Bluetooth...");
    if (!PS4.begin("f4:d4:88:64:d7:2b"))
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

    xTaskCreate(TaskReadAnalogPin, "TaskReadAnalogPin", 10240, NULL, 1, NULL);
}

void loop()
{
    // Kill this loop
    vTaskDelete(NULL);

#ifdef USE_JOYSTICK
    x1_val = analogRead(joystick0Pin);
    y1_val = analogRead(35);

    servo0_val = map(x1_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);
    servo1_val = map(y1_val, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX);

    log_v("x: %d, y: %d -> 0: %d, 1: %d\n", x1_val, y1_val, servo0_val, servo1_val);
    servos[0].writeMicroseconds(servo0_val);
    servos[1].writeMicroseconds(servo1_val);
#endif

#ifdef USE_PS4

    if (PS4.isConnected())
    {

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
    }
#endif

    // These servos use a 20ms pulse (50Hz)
    delay(1000 / PERIOD_HZ);
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

    ExponentialFilter<int> Filter(44, (JOYSTICK_MAX - JOYSTICK_MAX) / 2);

    for (;;)
    {
        // Read the value of the pin
        Filter.Filter(analogRead(joystick0Pin));

        if (count++ >= SENSOR_OVERSAMPLING)
        {
            int value = Filter.Current();
            log_v(" value: %d (%d)", (int)value, map(value, JOYSTICK_MIN, JOYSTICK_MAX, SERVO_US_MIN, SERVO_US_MAX));

            count = 0;
        }

        if (ticks++ % 1000 == 0)
        {
            log_v("Tick %d", ticks++);
        }

        vTaskDelay(taskPeriod);
    }
}
