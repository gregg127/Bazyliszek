#include "motor.h"
#include "motors.h"
#include <Arduino.h>



//Konstruktor z ustawieniem wejść/wyjść dla silnika
Motor::Motor(char _direction_pin_1, char _direction_pin_2, char _pwm_pin, char _encoder_pin_1)
{
        direction_pin_1 = _direction_pin_1;
        direction_pin_2 = _direction_pin_2;
        pwm_pin = _pwm_pin;
        encoder_pin_1 = _encoder_pin_1;

        //Licznik zmian stanu enkodera
        encoder_counter = 0;

         //Czas ostatniej zmiany enkodera
        encoder_timestamp = millis();

        attachInterrupt(digitalPinToInterrupt(encoder_pin_1), interrupt_handler, CHANGE);
}

//Tryb jazdy do przodu
void Motor::forward()
{
    digitalWrite(direction_pin_1, HIGH);
    digitalWrite(direction_pin_2, LOW);
}

//Tryb jazdy do tyłu
void Motor::backward()
{
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, HIGH);
}
//Szybkie zatrzymanie
void Motor::fstop() {
    analogWrite(pwm_pin, 255);
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, LOW);
}
//zatrzymanie
void Motor::stop()
{
    analogWrite(pwm_pin, 0);
}
//Ustawienie mocy
void Motor::pwm(unsigned char new_pwm)
{
    analogWrite(pwm_pin, new_pwm);
}

//Obsłużenie przerwania
void Motor::interrupt()
{
    encoder_counter++;
}

//Wyzerowanie licznika obrotów
void Motor::reset_encoder_counter()
{
    encoder_counter = 0;
}

