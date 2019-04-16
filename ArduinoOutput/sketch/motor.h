#include "motors.h"
#include <Arduino.h>
struct Motor
{
    //Numer wejścia/wyjścia mikrokontrolera sterującego polaryzacją
    char direction_pin_1;

    //Numer wejścia/wyjścia mikrokontrolera sterującego polaryzacją
    char direction_pin_2;

    //Numer wejścia/wyjścia mikrokontrolera sterującego wypełnieniem PWM
    char pwm_pin;

    //Numer wejścia/wyjścia mikrokontrolera do odczytu wartości enkodera silnika
    char encoder_pin_1;

    //Licznik zmian stanu enkodera
    int encoder_counter;

    //Czas ostatniej zmiany enkodera
    unsigned long encoder_timestamp;

    //Konstruktor z ustawieniem wejść/wyjść dla silnika
    Motor(char, char, char, char);

    //Tryb jazdy do przodu
    void forward();

    //Tryb jazdy do tyłu
    void backward();

    //Szybkie zatrzymanie
    void fstop();

    //zatrzymanie
    void stop();

    //Ustawienie mocy
    void pwm(unsigned char);

    //Obsłużenie przerwania
    void interrupt();

    //Wyzerowanie licznika obrotów
    void reset_encoder_counter();

};