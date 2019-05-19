#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

//==============================================================================
//                          KONFIGURACJA (STAŁE)
//==============================================================================

#define INPUT_SIZE 4

#define MOTOR_LEFT_DIR_1 8
#define MOTOR_LEFT_DIR_2 7
#define MOTOR_LEFT_PWM 10
#define MOTOR_LEFT_ENC 2

#define MOTOR_RIGHT_DIR_1 13
#define MOTOR_RIGHT_DIR_2 12
#define MOTOR_RIGHT_PWM 11
#define MOTOR_RIGHT_ENC 3

#define BUMPER_LEFT A0
#define BUMPER_CENTER A1
#define BUMPER_RIGHT A2

#define PID_P_MOVE 1.51 // propocja - 255 -> 100 = 155 to jest zjazd
#define PID_I_MOVE 0.01 // calka - stala wartosc przez ktora mnozy sie sume
#define PID_D_MOVE 1.51 // pochodna - stala wartosc przez ktora mnozy sie roznice

#define PID_P_DRIVE -22      // propocja - 255 -> 100 = 155 to jest zjazd
#define PID_I_DRIVE -0.0149  // calka - stala wartosc przez ktora mnozy sie sume
#define PID_D_DRIVE -19      // pochodna - stala wartosc przez ktora mnozy sie roznice
#define DT_INCREASE_RATE 100 // funkja drive

//================================================================================
//                          MOTOR STRUKTURA
//================================================================================

struct Motor
{
  //Numer wejscia/wyjscia mikrokontrolera sterujacego polaryzacja
  int dir_pin_1;
  //Numer wejscia/wyjscia mikrokontrolera sterujacego polaryzacja
  int dir_pin_2;
  //Numer wejscia/wyjscia mikrokontrolera sterujacego wypelnieniem PWM
  int pwm_pin;
  //Numer wejscia/wyjscia mikrokontrolera do odczytu wartosci enkodera silnika
  int enc_pin;
  //Licznik zmian stanu enkodera
  unsigned long encoder_counter;
  //Czas ostatniej zmiany enkodera
  unsigned long encoder_timestamp;

  //Konstruktor z ustawieniem wejsc/wyjsc dla silnika
  Motor(int _dir_pin_1, int _dir_pin_2, int _pwm_pin, int _enc_pin)
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

  //Tryb jazdy do przodu
  void forward()
  {
    digitalWrite(dir_pin_1, HIGH);
    digitalWrite(dir_pin_2, LOW);
  }

  //Tryb jazdy do tylu
  void backward()
  {
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, HIGH);
  }

  //Szybkie zatrzymanie
  void fast_stop()
  {
    analogWrite(pwm_pin, 255);
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, LOW);
  }

  //Zatrzymanie
  void stop()
  {
    analogWrite(pwm_pin, 0);
  }

  //Ustawienie mocy
  void pwm(int pwm)
  {
    analogWrite(pwm_pin, pwm);
  }

  //Obsluzenie przerwania
  void interrupt()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - encoder_timestamp > 1)
    {
      encoder_counter++;
    }
    encoder_timestamp = interrupt_time;
  }

  // Wyzerowanie licznika obrotow
  void reset_encoder_counter()
  {
    encoder_counter = 0L;
  }
};

//================================================================================
//                          ZMIENNE GLOBALNE
//================================================================================

Motor right_motor = Motor(MOTOR_RIGHT_DIR_1, MOTOR_RIGHT_DIR_2, MOTOR_RIGHT_PWM, MOTOR_RIGHT_ENC);
Motor left_motor = Motor(MOTOR_LEFT_DIR_1, MOTOR_LEFT_DIR_2, MOTOR_LEFT_PWM, MOTOR_LEFT_ENC);

// Zmienne obslugujace komunikacje na porcie szeregowym
char bytes_read[INPUT_SIZE];            // tablica 4 bajtow do odczytywania danych
char control;                           // znak sterowania
int read_value;                         // odczytana wartosc
char read_value_chars[INPUT_SIZE] = ""; // tymczasowa tablica przechowujaca odczytana wartosc + '\0' (znak konca linii)

// Zmienne wspomagajace kontrolowanie silnikow robota
int turn_right = true;
int pwm_motors = 0;

//================================================================================
//                          SETUP PROGRAMU
//================================================================================

void setup()
{
  // Port do komunikacji przez Bluetooth
  Serial1.begin(115200);
  // Port wspomagajacy debugowanie
  Serial.begin(9600);

  attach_motors_interrupts();

  // Bumpers
  pinMode(BUMPER_LEFT, INPUT_PULLUP);
  pinMode(BUMPER_CENTER, INPUT_PULLUP);
  pinMode(BUMPER_RIGHT, INPUT_PULLUP);
}

//Przypisanie funkcji obsługującej przerwania z enkoderów
void attach_motors_interrupts()
{
  attachInterrupt(left_motor.enc_pin, left_motor_interrupt, RISING);
  attachInterrupt(right_motor.enc_pin, right_motor_interrupt, RISING);
}

void left_motor_interrupt()
{
  left_motor.interrupt();
}

void right_motor_interrupt()
{
  right_motor.interrupt();
}

//================================================================================
//                          GLOWNA PETLA PROGRAMU
//================================================================================

void loop()
{
  // Sprawdz czy roboty uderzyl w sciane
  bumpers_check();

  if (Serial1.available() >= INPUT_SIZE)
  {
    load_received_data();

    // Wyslij na port szeregowy Serial info o otrzymanej fladze i wartosci
    print_flag_info_serial();

    // Do sterowania nalezy korzystac ze zmiennych 'control' oraz 'read_value'
    switch (control)
    {
    case 'v': // ustawienie mocy silnikow
      set_velocity();
      break;
    case 'p': // procent obrocenia silnikow
      turn_motors();
      break;
    case 'b': // ustaw silniki na jazde do tylu
      set_motors_direction(false);
      break;
    case 'f': // ustaw silniki na jazde prostO
      set_motors_direction(true);
      break;
    case 's': // gwaltowne zatrzymanie silnikow
      stop_motors(true);
      break;
    }
  }
}

//================================================================================
//                          FUNKCJE STERUJACE
//================================================================================

void bumpers_check()
{
  if (digitalRead(BUMPER_LEFT) == LOW ||
      digitalRead(BUMPER_CENTER) == LOW ||
      digitalRead(BUMPER_RIGHT) == LOW)
  {
    right_motor.fast_stop();
    left_motor.fast_stop();
  }
}

void set_velocity()
{
  left_motor.pwm(read_value);
  right_motor.pwm(read_value);
  pwm_motors = read_value;
}

void turn_motors()
{
  if (read_value < 100) // lewo
  {
    left_motor.pwm(pwm_motors * read_value / 100);
    right_motor.pwm(pwm_motors);
  }
  else // prawo
  {
    right_motor.pwm(pwm_motors * (100 - (read_value - 100) / 100));
    left_motor.pwm(pwm_motors);
  }
}

void set_motors_direction(boolean is_forward)
{
  if (is_forward)
  {
    left_motor.forward();
    right_motor.forward();
  }
  else
  {
    left_motor.backward();
    right_motor.backward();
  }
}

void stop_motors(boolean fast_stop)
{
  if (fast_stop)
  {
    left_motor.fast_stop();
    right_motor.fast_stop();
  }
  else
  {
    left_motor.stop();
    right_motor.stop();
  }
}

//================================================================================
//                          FUNKCJE POMOCNICZE
//================================================================================

void print_flag_info_serial()
{
  Serial.print("Flag: ");
  Serial.print(control);
  Serial.print(" , value: ");
  Serial.print(read_value);
  Serial.println();
}

/*==============================================================================
                            PROTOKOL KOMUNIKACJI
  ==============================================================================

   --- Działanie protokołu:
   postac flagi: znak + trzy cyfry, no. f259, b000
   protokol umozliwia wysylanie dowolnej wartosci
   o maksymalnej liczbie cyfr rownej 3 oraz znak
   oznaczajacy akcje wykonywana przez robota
   Przyklad: jezeli chcesz wyslac flage ze znakiem r o wartosci 90
   flaga wyslana przez port szeregowy powinna miec postac: r090
   --- Zmienne:
   bytes_read - tablica 4 bajtow do ktorej sa pakowane dane z portu szeregowego
   control - char, znak sterujacy robotem
   read_value - integer, wartosc odczytana z portu szeregowego
*/
void load_received_data()
{
  // Odczytuje bajty i laduje je do tablicy bytes_read
  Serial1.readBytes(bytes_read, INPUT_SIZE);
  control = bytes_read[0];

  load_value();
}

// Zaladowanie do inta tablicy z liczba
void load_value()
{
  for (int i = 0; i < INPUT_SIZE - 1; i++)
  {
    read_value_chars[i] = bytes_read[i + 1];
  }
  read_value_chars[INPUT_SIZE - 1] = '\0';     // dodanie na koncu znaku konca ciagu zeby zrzutowac na inta
  sscanf(read_value_chars, "%d", &read_value); // czyta tablice znakow inta do zmiennej int korzystajac z adresu
}
