#include "motor.h"
#include "Arduino.h"

Motor::Motor(char _dir_pin_1, char _dir_pin_2, char _pwm_pin, char _enc_pin)
{
    dir_pin_1 = _dir_pin_1;
    dir_pin_2 = _dir_pin_2;
    pwm_pin = _pwm_pin;
    enc_pin = _enc_pin;

    pinMode(dir_pin_1, OUTPUT);
    pinMode(dir_pin_2, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    pinMode(enc_pin, INPUT);

    encoder_counter = 0L;
    encoder_timestamp = millis();
}

void Motor::forward()
{
    digitalWrite(dir_pin_1, HIGH);
    digitalWrite(dir_pin_2, LOW);
}

void Motor::backward()
{
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, HIGH);
}

void Motor::fast_stop()
{
    analogWrite(pwm_pin, 255); // ???
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, LOW);
}

void Motor::stop()
{
    analogWrite(pwm_pin, 0);
}

void Motor::pwm(unsigned char pwm)
{
    analogWrite(pwm_pin, pwm);
}

void Motor::interrupt()
{
    unsigned long interrupt_time = millis();
    if (interrupt_time - encoder_timestamp > 1)
    {
        encoder_counter++;
    }
    encoder_timestamp = interrupt_time;
}

void Motor::reset_encoder_counter()
{
    encoder_counter = 0L;
}