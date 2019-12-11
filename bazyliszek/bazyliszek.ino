#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

//==============================================================================
//                          KONFIGURACJA (STAŁE)
//==============================================================================
#define DEBUG 0

#define INPUT_SIZE 4
#define MOTOR_LEFT_DIR_1 22
#define MOTOR_LEFT_DIR_2 23
#define MOTOR_LEFT_PWM 6
#define MOTOR_LEFT_ENC_FIRST 20
#define MOTOR_LEFT_ENC_SECOND 50

#define MOTOR_RIGHT_DIR_1 24
#define MOTOR_RIGHT_DIR_2 25
#define MOTOR_RIGHT_PWM 7
#define MOTOR_RIGHT_ENC_FIRST 21
#define MOTOR_RIGHT_ENC_SECOND 51

#define BUMPERS 2

#define LED_RED 26
#define LED_GREEN 27
#define LED_BLUE 28

#define INTERRUPT_DEBUG_COUNTER_INTERVAL 1000

// Parametry enkoderow, przekladni i kol
#define INTERRUPTS_TO_MM 0.615998
#define ROBOT_WIDTH 295
#define LIN_DISPLACEMENT_RATIO INTERRUPTS_TO_MM / 2
#define THETA_RATIO INTERRUPTS_TO_MM / ROBOT_WIDTH
#define ODOMETRY_CHECK_INTERVAL 100 // in milisceonds
#define VELOCITY_MEASURE_INTERVAL 100 //in miliseconds

// Parametry komunikacji szeregowej
#define BAUDRATE 250000
#define READ_VALUE_TO_DISTANCE_RATIO 10


//konfiguracja PIDa
#define PROPORTIONAL_PARAM 0.0
#define INTEGRAL_PARAM 0.0
#define DERIVATIVE_PARAM 0.0

//================================================================================
//                          MOTOR STRUKTURA
//================================================================================

struct Motor
{
  //Wskaźnik na drugi silnik
  Motor* another_motor;
  //Numer wejscia/wyjscia mikrokontrolera sterujacego polaryzacja
  int dir_pin_1;
  //Numer wejscia/wyjscia mikrokontrolera sterujacego polaryzacja
  int dir_pin_2;
  //Numer wejscia/wyjscia mikrokontrolera sterujacego wypelnieniem PWM
  int pwm_pin;
  //Kierunek jazdy
  bool is_forward = true;

  //Numer wejscia/wyjscia mikrokontrolera do odczytu pierwszej wartosci enkodera silnika
  int enc_pin_first;
  //Numer wejscia/wyjscia mikrokontrolera do odczytu drugiej przesunietej w fazie wartosci enkodera silnika
  int enc_pin_second;
  //Licznik zmian stanu enkodera
  unsigned long encoder_counter;
  //Poprzedni przejechany dystans
  unsigned long prev_encoder_counter;
  //Czas ostatniej zmiany enkodera
  unsigned long encoder_timestamp;
  //Aktualna predkosc
  double velocity;

  Motor()
  {
  }

  //Konstruktor z ustawieniem wejsc/wyjsc dla silnika
  Motor(int _dir_pin_1, int _dir_pin_2, int _pwm_pin, int _enc_pin_first, int _enc_pin_second)
  {
    dir_pin_1 = _dir_pin_1;
    dir_pin_2 = _dir_pin_2;
    pwm_pin = _pwm_pin;
    enc_pin_first = _enc_pin_first;
    enc_pin_second = _enc_pin_second;

    pinMode(dir_pin_1, OUTPUT);
    pinMode(dir_pin_2, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
    pinMode(enc_pin_first, INPUT);
    pinMode(enc_pin_second, INPUT);

    reset_encoder_counter();
    encoder_timestamp = 0;
    velocity = 0;
    prev_encoder_counter = 0;
  }

  //Tryb jazdy do przodu
  void forward()
  {
    digitalWrite(dir_pin_1, HIGH);
    digitalWrite(dir_pin_2, LOW);
    is_forward = true;
    reset_prev_counters();
  }

  //Tryb jazdy do tylu
  void backward()
  {
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, HIGH);
    is_forward = false;
    reset_prev_counters();
  }

  //Szybkie zatrzymanie
  void fast_stop()
  {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, LOW);
  }

  void fast_stop_forward()
  {
    if (is_forward)
    {
      fast_stop();
    }
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

  double measure_velocity()
  {
    unsigned long current_millis = millis();
    if ((current_millis - encoder_timestamp) > VELOCITY_MEASURE_INTERVAL) {
      velocity = ((double)(encoder_counter - prev_encoder_counter)) * INTERRUPTS_TO_MM / (current_millis - encoder_timestamp);
      encoder_timestamp = current_millis;
      prev_encoder_counter = encoder_counter;
    }
    return velocity;
  }

  void interrupt()
  {
    encoder_counter++;
  }

  bool is_faster_than_another_motor()
  {
    return velocity > another_motor->velocity;
  }

  void reset_encoder_counter()
  {
    encoder_counter = 0L;
  }
  void reset_prev_counters()
  {
    encoder_timestamp = 0;
    prev_encoder_counter = 0;
  }
};

//================================================================================
//                          ZMIENNE GLOBALNE
//================================================================================

Motor right_motor = Motor(MOTOR_RIGHT_DIR_1, MOTOR_RIGHT_DIR_2, MOTOR_RIGHT_PWM, MOTOR_RIGHT_ENC_FIRST, MOTOR_RIGHT_ENC_SECOND);
Motor left_motor = Motor(MOTOR_LEFT_DIR_1, MOTOR_LEFT_DIR_2, MOTOR_LEFT_PWM, MOTOR_LEFT_ENC_FIRST, MOTOR_LEFT_ENC_SECOND);
void assign_motors() {
  right_motor.another_motor = &left_motor;
  left_motor.another_motor = &right_motor;
}

// Zmienne obslugujace komunikacje na porcie szeregowym
char bytes_read[INPUT_SIZE];            // tablica 4 bajtow do odczytywania danych
char control;                           // znak sterowania
int read_value;                         // odczytana wartosc
char read_value_chars[INPUT_SIZE] = ""; // tymczasowa tablica przechowujaca odczytana wartosc + '\0' (znak konca linii)

// Zmienne wspomagajace kontrolowanie silnikow robota
unsigned char pwm_motors = 255;
// Zmienna informująca, że wystąpiło zdarzenie bumpera, a nie zostało jeszcze odczytane przez funkcję ruchu
bool interrupt_has_recently_occured = false;


// Zmienna przechowująca wypełnienie PWM stosowane przy jeździe

//================================================================================
//                          SETUP PROGRAMU
//================================================================================

void setup()
{
  // Port do komunikacji z Raspberry PI
  Serial.begin(BAUDRATE);

  // Bumpery
  pinMode(BUMPERS, INPUT_PULLUP);

  attach_interrupts();

  // Ledy
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  //    right_motor = Motor(MOTOR_RIGHT_DIR_1, MOTOR_RIGHT_DIR_2, MOTOR_RIGHT_PWM, MOTOR_RIGHT_ENC_FIRST, MOTOR_RIGHT_ENC_SECOND);
  //    left_motor = Motor(MOTOR_LEFT_DIR_1, MOTOR_LEFT_DIR_2, MOTOR_LEFT_PWM, MOTOR_LEFT_ENC_FIRST, MOTOR_LEFT_ENC_SECOND);

  assign_motors();
}

void attach_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(BUMPERS), bumpers_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(left_motor.enc_pin_first), left_motor_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(right_motor.enc_pin_first), right_motor_interrupt, FALLING);
}

void bumpers_interrupt()
{
  interrupt_has_recently_occured = true;
  right_motor.fast_stop_forward();
  left_motor.fast_stop_forward();
  highlight(LED_RED);
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
bool mm = true;
void loop()
{
  if (Serial.available() >= INPUT_SIZE)
  {
    load_received_data();

    // Wyslij na port szeregowy Serial info o otrzymanej fladze i wartosci
    if (DEBUG) {
      print_flag_info_serial();
    }
    // Do sterowania nalezy korzystac ze zmiennych 'control' oraz 'read_value'
    switch (control)
    {
      case 'v': // ustawienie mocy silnikow
        set_velocity();
        break;
      case 'm': // ustaw silniki na jazde do przodu
        set_motors_direction(true);
        move();
        break;
      case 'b': // ustaw silniki na jazde do tylu
        set_motors_direction(false);
        move();
        break;
      case 'l': // wartosc odwrocenia silnikow
        left_motor.forward();
        right_motor.backward();
        move();
        break;
      case 'r': // wartosc odwrocenia silnikow
        left_motor.backward();
        right_motor.forward();
        move();
        break;
      case 'f': // ustaw silniki na jazde prostO
        set_motors_direction(true);
        break;
      case 's': // gwaltowne zatrzymanie silnikow
        stop_motors(true);
        break;
    }
  }
  measure_motors_velocity();
  set_motors_direction(true);
  if (mm) {
    move();
    mm = false;
    stop_motors(false);
  }
}

//================================================================================
//                          FUNKCJE STERUJACE
//================================================================================

void set_velocity()
{

  //  for(int i=0; i <= pwm; i+=20) {
  //    analogWrite(pwm_pin, i);
  //  }
  //set_motors_direction(true);
  //left_motor.pwm(read_value);
  //right_motor.pwm(read_value);
  //pwm_motors = read_value;
  pwm_motors = (unsigned char)read_value;
}
void move() {
  interrupt_has_recently_occured = false;
  if (!bumpers_not_active())
  {
    return;
  }
  highlight(LED_BLUE);
  reset_encoder_counters();
  double error_sum = 0;
  double previous_error = 0;
  //unsigned int distance = ((int)read_value) * READ_VALUE_TO_DISTANCE_RATIO;
  int a = 0;
  long long prev_millis = millis();
  double error_left = 0.0;
  double error_right = 0.0;
  double prev_error_left = 0.0;
  double prev_error_right = 0.0;
  double proportional = 255 / 18.0;
  double derrivative = 2.35;
  double integral = 0.5;
  double sum_error_left = 0.0;
  double sum_error_right = 0.0;
  for (a; a < 255; a++) {
    double normalized_l = left_motor.velocity * 20;
    double normalized_r = right_motor.velocity * 20;
    double normalized_pwm = ((double)a) * (18.0 / 255.0);
    error_left = (normalized_pwm - (normalized_l));
    error_right = (normalized_pwm - (normalized_r));
    sum_error_left += error_left;
    sum_error_right += error_right;
    unsigned char left_pwm = true || normalized_l > normalized_pwm ? (unsigned char)((abs(error_left) * proportional)+(prev_error_left - error_left)*derrivative+sum_error_left*integral) : a;
    unsigned char right_pwm = true || normalized_r > normalized_pwm ? (unsigned char)((abs(error_right) * proportional)+(prev_error_right - error_right)*derrivative+sum_error_right*integral) : a;
    left_motor.pwm(left_pwm);
    right_motor.pwm(right_pwm);
    prev_error_left = error_left;
    prev_error_right = error_right;

    //    Serial.print(millis());
    //    Serial.print('\t');
    Serial.print(((double)left_pwm) * (18.0 / 255.0));
    Serial.print('\t');
    Serial.print(((double)right_pwm) * (18.0 / 255.0));
    Serial.print('\t');
    Serial.print(normalized_l);
    Serial.print('\t');
    Serial.print(normalized_r);
    Serial.println();
    while (prev_millis + 66 > millis())
    {
      measure_motors_velocity();
//      error_left = (normalized_pwm - (normalized_l - normalized_pwm));
//    error_right = (normalized_pwm - (normalized_r - normalized_pwm));
//    sum_error_left += error_left;
//    sum_error_right += error_right;
//    left_pwm = true || normalized_l > normalized_pwm ? (unsigned char)((error_left * proportional)+(prev_error_left - error_left)*derrivative+sum_error_left*integral) : a;
//    right_pwm = true || normalized_r > normalized_pwm ? (unsigned char)((error_right * proportional)+(prev_error_right - error_right)*derrivative+sum_error_right*integral) : a;
//    left_motor.pwm(left_pwm);
//    right_motor.pwm(right_pwm);
//    prev_error_left = error_left;
//    prev_error_right = error_right;
    }
    prev_millis = millis();
  }


}
void old_v()
{
  double error_sum = 0;
  double prev_error = 0;
  double error = 0;
  highlight(LED_BLUE);
  unsigned int distance = ((unsigned int)read_value) * 10;
  reset_encoder_counters();
  long long mill = millis();
  long long last_mil = 0;
  unsigned char pwm = 5;
  interrupt_has_recently_occured = false;
  while ( bumpers_not_active() && !interrupt_has_recently_occured)
  {
    // softstart
    mill = millis();
    if (((mill - last_mil) > 10))
    {
      if (pwm < pwm_motors) {
        right_motor.pwm(pwm);
        left_motor.pwm(pwm - 12 > 0 ? pwm - 12 : pwm);
        pwm += 1;
      } else if ((pwm == pwm_motors))
        //        ^^^   (left_motor.velocity > 0) && left_motor.velocity - right_motor.velocity != 0 &&
      {
        error = left_motor.velocity - right_motor.velocity;
        //        double en1 = (right_motor.velocity / left_motor.velocity) * 255;
        //        double a = en1 / error;
        //double prop_pwm = (6038 * (left_motor.velocity - right_motor.velocity));
        double pwm_unranged = 603 * error - error_sum * 30 + (error - prev_error) * 2;
        int pwm_ranged = 255 - (int)pwm_unranged;
        if (pwm_unranged > 255) {
          pwm_ranged = 255;
        } else if (pwm_unranged < 0) {
          pwm_ranged = 0;
        }

        left_motor.pwm(pwm_ranged);
        Serial.print(pwm_ranged);
        Serial.print('\t');
        Serial.print(pwm_unranged);
        Serial.print('\t');
        Serial.print(left_motor.velocity);
        Serial.print('\t');
        Serial.println(right_motor.velocity);
      }
      last_mil = mill;
      measure_motors_velocity();
      error_sum += error;
      prev_error = error;
    }



  }
  if (!interrupt_has_recently_occured && bumpers_not_active()) {
    stop_motors(false);
    highlight(LED_GREEN);
    Serial.println("m0");
  } else {
    highlight(LED_RED);
    Serial.println("m1");
  }
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
    right_motor.pwm((pwm_motors * (100 - (read_value - 100))) / 100);
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
boolean has_reached ( unsigned int distance )
{
  return (left_motor.encoder_counter * INTERRUPTS_TO_MM < distance && right_motor.encoder_counter * INTERRUPTS_TO_MM < distance);
}
void measure_motors_velocity() {
  right_motor.measure_velocity();
  left_motor.measure_velocity();
  if (DEBUG && (millis() % 300 == 0)) {
    Serial.print("Motor R: ");
    Serial.print(right_motor.velocity);
    Serial.print('\t');
    Serial.print(left_motor.is_faster_than_another_motor() ? '<' : '>');
    Serial.print("\tMotor L: ");
    Serial.print(left_motor.velocity);
    Serial.print('\n');
  }
}
bool bumpers_not_active() {
  return digitalRead(BUMPERS);
}
void print_flag_info_serial()
{
  Serial.print("Flag: ");
  Serial.print(control);
  Serial.print(" , value: ");
  Serial.print(read_value);
  Serial.println();
}

void highlight(int led) {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(led, HIGH);
}

void reset_encoder_counters() {
  left_motor.reset_encoder_counter();
  right_motor.reset_encoder_counter();
}

int pwm (int error, int prev_error) {
}
// ==============================================================================
//                         PROTOKOL KOMUNIKACJI
// ==============================================================================
// --- Działanie protokołu:
// postac flagi: znak + trzy cyfry, no. f259, b000
// protokol umozliwia wysylanie dowolnej wartosci
// o maksymalnej liczbie cyfr rownej 3 oraz znak
// oznaczajacy akcje wykonywana przez robota
// Przyklad: jezeli chcesz wyslac flage ze znakiem r o wartosci 90
// flaga wyslana przez port szeregowy powinna miec postac: r090
// --- Zmienne:
// bytes_read - tablica 4 bajtow do ktorej sa pakowane dane z portu szeregowego
// control - char, znak sterujacy robotem
// read_value - integer, wartosc odczytana z portu szeregowego

void load_received_data()
{
  // Odczytuje bajty i laduje je do tablicy bytes_read
  Serial.readBytes(bytes_read, INPUT_SIZE);
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
