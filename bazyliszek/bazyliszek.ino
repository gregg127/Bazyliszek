#include <math.h>
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
#define INTERRUPTS_TO_CM 10*INTERRUPTS_TO_MM
#define ROBOT_WIDTH 295.0
#define LIN_DISPLACEMENT_RATIO INTERRUPTS_TO_MM / 2
#define THETA_RATIO INTERRUPTS_TO_MM / ROBOT_WIDTH
#define ODOMETRY_CHECK_INTERVAL 100 // w milisekundach
#define VELOCITY_MEASURE_INTERVAL 100 // w milisekundach

// Parametry komunikacji szeregowej
#define BAUDRATE 250000
#define READ_VALUE_TO_DISTANCE_RATIO 10


//konfiguracja PID
#define PROPORTIONAL 13.0769
#define INTEGRAL 3
#define DERIVATIVE 0.8

//komunikaty wykonania komendy
#define EXECUTION_SUCCESSFUL 0
#define EXECUTION_ABORTED_INVALID_PARAM 1
#define EXECUTION_ABORTED_BY_BUMPER_INTERRUPTION 2

//kierunki jazdy robota
#define LEFT_TURN false
#define RIGHT_TURN true
#define FORWARD_DRIVE true
#define BACKWARD_DRIVE false

//maksymalne wartości przyjomwane przez funckje sterujące
#define MAX_VELOCITY_PARAM_VALUE 255
#define MAX_TURN_PARAM_VALUE 360


//================================================================================
//                          MOTOR STRUKTURA
//================================================================================

struct Motor
{
  //Wskaźnik na drugi silnik
  Motor* another_motor;
  //Numer pinu mikrokontrolera sterujacego polaryzacja
  int dir_pin_1;
  //Numer pinu mikrokontrolera sterujacego polaryzacja
  int dir_pin_2;
  //Numer pinu mikrokontrolera sterujacego wypelnieniem PWM
  int pwm_pin;
  //Kierunek jazdy
  bool is_forward = true;

  //Numer pinu mikrokontrolera do odczytu pierwszej wartosci enkodera silnika
  int enc_pin_first;
  //Numer pinu mikrokontrolera do odczytu drugiej przesunietej w fazie wartosci enkodera silnika
  int enc_pin_second;
  //Licznik zmian stanu enkodera
  unsigned long encoder_counter;
  //Poprzedni przejechany dystans
  unsigned long prev_encoder_counter;
  //Czas ostatniej zmiany enkodera
  unsigned long encoder_timestamp;
  //Aktualna predkosc
  double velocity;
  //poprzedni pwm
  unsigned char prev_pwm;

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
    prev_pwm = 0;
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

  void stop_if_forward()
  {
    if (is_forward)
    {
      calm_stop();
    }
  }

  //Zatrzymanie
  void calm_stop()
  {
    analogWrite(pwm_pin, 0);
  }

  //Ustawienie mocy
  unsigned char pwm(int _pwm)
  {
    if (_pwm > 255) {
      _pwm = 255;
    } else if (_pwm < 0) {
      _pwm = 0;
    }
    analogWrite(pwm_pin, _pwm);
//    if (_pwm - prev_pwm > 25) {
//      prev_pwm = (unsigned char)prev_pwm-25;
//      analogWrite(pwm_pin, prev_pwm-25);
//    } else if(prev_pwm - _pwm > 25) {
//      prev_pwm = (unsigned char)prev_pwm+25;
//      analogWrite(pwm_pin, prev_pwm+25);
//    } else {
//      prev_pwm = (unsigned char)_pwm;
//      analogWrite(pwm_pin, _pwm);
//    }
    return 0;
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
unsigned char prev_pwm_motors = 0;
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
  right_motor.calm_stop();
  left_motor.calm_stop();
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
        assign_distance_by_cm();
        set_motors_direction(FORWARD_DRIVE);
        drive_robot();
        break;
      case 'b': // ustaw silniki na jazde do tylu
        assign_distance_by_cm();
        set_motors_direction(BACKWARD_DRIVE);
        drive_robot();
        break;
      case 'l': // wartosc odwrocenia silnikow
        turn(LEFT_TURN);
        break;
      case 'r': // wartosc odwrocenia silnikow
       turn(RIGHT_TURN);
        break;
      case 's': // gwaltowne zatrzymanie silnikow
        stop_motors(true);
        break;  
    }
  }
  measure_motors_velocity();
}

//================================================================================
//                          FUNKCJE STERUJACE
//================================================================================

void turn(boolean diretion) {
  if (read_value <= MAX_TURN_PARAM_VALUE)
  {
    assign_distance_by_degree();
    if (diretion)
    {
      left_motor.backward();
      right_motor.forward();
    } else {
      left_motor.forward();
      right_motor.backward();
    }
    drive_robot();
  } else {
    Serial.print(control);
    Serial.println(EXECUTION_ABORTED_INVALID_PARAM);
  }
}

void set_velocity()
{
  Serial.print(control);
  if (read_value <= MAX_VELOCITY_PARAM_VALUE)
  {
    pwm_motors = (unsigned char)read_value;
    Serial.println(EXECUTION_SUCCESSFUL);
  } else {
    Serial.println(EXECUTION_ABORTED_INVALID_PARAM);
  }
}
void debug_drive_robot() {  //TODO funkcja na podstawie której powstał wykres do pracy, chyba do usunięcia
  for (unsigned char in = 1; in>0; in++) {
    left_motor.pwm(in);
    right_motor.pwm(in);
    for (int abc=0; abc<20; abc++) {
      delay(1);
      measure_motors_velocity();
    }
    Serial.print((((double)in)*18)/255.0);
    Serial.print(',');
    Serial.print(left_motor.velocity*20);
    Serial.print(',');
    Serial.println(right_motor.velocity*20); 
  }
  left_motor.pwm(0);
  right_motor.pwm(0);
}
void drive_robot() {
  interrupt_has_recently_occured = false;
  if (!bumpers_not_active())
  {
    Serial.print(control);
    Serial.println(EXECUTION_ABORTED_BY_BUMPER_INTERRUPTION);
    return;
  }
  highlight(LED_BLUE);
  reset_encoder_counters();
  double error_sum = 0;
  double previous_error = 0;
  //unsigned int distance = ((int)read_value) * READ_VALUE_TO_DISTANCE_RATIO;
  int a = pwm_motors;
  int c = prev_pwm_motors;
  int b = 50;
  long long prev_millis = millis();
  double error_left = 0.0;
  double error_right = 0.0;
  double prev_error_left = 0.0;
  double prev_error_right = 0.0;
  double sum_error_left = 0.0;
  double sum_error_right = 0.0;
  unsigned int left_pwm = 0;
  unsigned int right_pwm = 0;
  while (!(has_reached()) && (bumpers_not_active() && !interrupt_has_recently_occured)) {
    if (a > c) {
      c+=5;
    }
    double normalized_l = left_motor.velocity * 20;
    double normalized_r = right_motor.velocity * 20;
    double normalized_pwm = ((double)c) * (18.0 / 255.0);
    error_left = (normalized_pwm - (normalized_l));
    error_right = (normalized_pwm - (normalized_r));
    sum_error_left += error_left;
    sum_error_right += error_right;
    left_pwm = (unsigned int)((abs(error_left) * PROPORTIONAL)+(prev_error_left - error_left)*DERIVATIVE+sum_error_left*INTEGRAL);
    right_pwm = (unsigned int)((abs(error_right) * PROPORTIONAL)+(prev_error_right - error_right)*DERIVATIVE+sum_error_right*INTEGRAL);
    left_motor.pwm(left_pwm);
    right_motor.pwm(right_pwm);
    prev_error_left = error_left;
    prev_error_right = error_right;

    if (left_pwm > 255) {
      left_pwm = 255;
    } else if (left_pwm < 0) {
      left_pwm = 0;
    }

    if (right_pwm > 255) {
      right_pwm = 255;
    } else if (left_pwm < 0) {
      right_pwm = 0;
    }
    if (DEBUG) {
      Serial.print(((double)left_pwm) * (18.0 / 255.0));
      Serial.print('\t');
      Serial.print(((double)right_pwm) * (18.0 / 255.0));
      Serial.print('\t');
      Serial.print(normalized_l);
      Serial.print('\t');
      Serial.print(normalized_r);
      Serial.println(); 
    }
    while (prev_millis + 66 > millis())
    {
      measure_motors_velocity();
    }
    prev_millis = millis();
  }
  prev_pwm_motors = 0;
  stop_motors(false);
  highlight(interrupt_has_recently_occured ? LED_RED : LED_GREEN);
  Serial.print(control);
  Serial.println(interrupt_has_recently_occured ? EXECUTION_ABORTED_BY_BUMPER_INTERRUPTION : EXECUTION_SUCCESSFUL);
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
    left_motor.calm_stop();
    right_motor.calm_stop();
  }
}

//================================================================================
//                          FUNKCJE POMOCNICZE
//================================================================================
void assign_distance_by_degree()
{
  read_value = read_value*PI*ROBOT_WIDTH/36;
}

void assign_distance_by_cm()
{
  read_value *= 100;
}
boolean has_reached ()
{
  return (left_motor.encoder_counter * INTERRUPTS_TO_CM > read_value || right_motor.encoder_counter * INTERRUPTS_TO_CM > read_value);
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
