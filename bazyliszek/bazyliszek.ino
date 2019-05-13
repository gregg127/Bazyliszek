#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

// #include "myoled.h"
// #include "motor.h"

//==============================================================================
//                          KONFIGURACJA (STAŁE)
//==============================================================================

#define MOTOR_LEFT_DIR_1 7
#define MOTOR_LEFT_DIR_2 5
#define MOTOR_LEFT_PWM 6
#define MOTOR_LEFT_ENC 2

#define MOTOR_RIGHT_DIR_1 9
#define MOTOR_RIGHT_DIR_2 8
#define MOTOR_RIGHT_PWM 11
#define MOTOR_RIGHT_ENC 3

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
  //Numer wejścia/wyjścia mikrokontrolera sterującego polaryzacją
  int dir_pin_1;
  //Numer wejścia/wyjścia mikrokontrolera sterującego polaryzacją
  int dir_pin_2;
  //Numer wejścia/wyjścia mikrokontrolera sterującego wypełnieniem PWM
  int pwm_pin;
  //Numer wejścia/wyjścia mikrokontrolera do odczytu wartości enkodera silnika
  int enc_pin;
  //Licznik zmian stanu enkodera
  long encoder_counter;
  //Czas ostatniej zmiany enkodera
  unsigned long encoder_timestamp;

  //Konstruktor z ustawieniem wejść/wyjść dla silnika
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
    
    Serial.print("Konstruktor");
  }

  //Tryb jazdy do przodu
  void forward()
  {
    digitalWrite(dir_pin_1, HIGH);
    digitalWrite(dir_pin_2, LOW);
  }

  //Tryb jazdy do tyłu
  void backward()
  {
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, HIGH);
  }

  //Szybkie zatrzymanie
  void fast_stop()
  {
    analogWrite(pwm_pin, 255); // ???
    digitalWrite(dir_pin_1, LOW);
    digitalWrite(dir_pin_2, LOW);
  }

  //Zatrzymanie
  void stop()
  {
    analogWrite(pwm_pin, 0);
  }

  //Ustawienie mocy
  void pwm(unsigned char pwm)
  {
    analogWrite(pwm_pin, pwm);
  }

  //Obsłużenie przerwania
  void interrupt()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - encoder_timestamp > 1)
    {
      encoder_counter++;
    }
    encoder_timestamp = interrupt_time;

    Serial.print("Test enkoderu, liczba przerwan: ");
    Serial.println(encoder_counter);
  }

  //Wyzerowanie licznika obrotów
  void reset_encoder_counter()
  {
    encoder_counter = 0L;
  }
};
//================================================================================
//                          ZMIENNE GLOBALNE
//================================================================================
Motor left_motor = Motor(MOTOR_LEFT_DIR_1, MOTOR_LEFT_DIR_2, MOTOR_LEFT_PWM, MOTOR_LEFT_ENC);
Motor right_motor = Motor(MOTOR_RIGHT_DIR_1, MOTOR_RIGHT_DIR_2, MOTOR_LEFT_PWM, MOTOR_RIGHT_ENC);

// Prędkość ustawiana w funkcji velocity, definuje jaki pwm ma być podawany na silniki w funkcji move
unsigned char target_velocity;

// Zmienne obslugujace komunikacje na porcie szeregowym
char bytes_read[4];            // tablica 4 bajtow do odczytywania danych
char control;                  // znak sterowania
int read_value;                // odczytana wartosc
char read_value_chars[4] = ""; // tymczasowa tablica przechowujaca odczytana wartosc + '\0' (znak konca linii)

// Zmienna ktora ustawia czy dane otrzymane przez port szeregowym powinny byc drukowane na wyswietlaczu OLED
boolean oled_info_debug_print = true;

//================================================================================
//                          SETUP PROGRAMU
//================================================================================
void setup()
{
  Serial.begin(115200);
  attach_motors_interrupts();
  //MyOled::setup_oled();
}

//Przypisanie funkcji obsługującej przerwania z enkoderów
void attach_motors_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(left_motor.enc_pin), left_motor_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(right_motor.enc_pin), right_motor_interrupt, RISING);
}

void left_motor_interrupt()
{
  //MyOled::display_char('t', true, "Test lewy interrupt");
  Serial.println("LEWY");
  left_motor.interrupt();
}

void right_motor_interrupt()
{
  //MyOled::display_char('p', true, "Test prawy interrupt");
  Serial.println("prawy");
  right_motor.interrupt();
}

//================================================================================
//                          GLOWNA PETLA PROGRAMU
//================================================================================
void loop()
{
  if (Serial.available() >= 4)
  {
    load_received_data();
    boolean command_available = true;
    String extra_info = " " + String(read_value);

    // Do sterowania nalezy korzystac ze zmiennych 'control' oraz 'read_value'
    switch (control)
    {
      case 'm':
        move_robot(read_value, true);
        break;
      case 'b':
        move_robot(read_value, false);
        break;
      case 'r':
        rotate(read_value);
        break;
      case 'v':
        velocity(read_value);
        break;
      case 's':
        left_motor.stop();
        right_motor.stop();
        break;
      case 'f':
        left_motor.fast_stop();
        right_motor.fast_stop();
        break;
      default:
        command_available = false;
        break;
    }
    if (oled_info_debug_print)
    {
      //MyOled::display_char(control, command_available, extra_info);
      Serial.println(extra_info);
      Serial.println(left_motor.enc_pin);
      Serial.println(right_motor.enc_pin);
      
    }
  }
}

//================================================================================
//                          FUNKCJE STERUJĄCE
//================================================================================

// FIX ME
//Move robot for the declared distance, measured in encoder readings
void move_robot(int cm, bool forward)
{
  double click_to_cm_ratio = 0.6; // dystans w cm przejechany przy jednym obrocie kolka
  double a_cm = 0;
  double b_cm = 0;
  left_motor.reset_encoder_counter();  // wyzerowanie licznika obrotow dla silnika lewego
  right_motor.reset_encoder_counter(); // wyzerowanie licznika obrotow dla silnika prawego

  // ustawienie kierunku jazdy
  if (forward)
  {
    left_motor.forward();
    right_motor.forward();
  }
  else
  {
    left_motor.backward();
    right_motor.backward();
  }

  double a_sum = 0;             // for integral part
  double a_previous_error = cm; // for derivative part
  unsigned long a_prev_millis = millis();
  unsigned long b_prev_millis = millis();
  double b_sum = 0;             // for integral part
  double b_previous_error = cm; // for derivative part

  // softstart
  for (int i = 0; i < 200; i += 1)
  {
    left_motor.pwm(i);
    right_motor.pwm(i);
    delay(1); // BLOKUJACE!!!
  }

  while (true)
  {
    //MyOled::write_oled_rotation_count(a_cm, b_cm, cm);

    a_cm = left_motor.encoder_counter * click_to_cm_ratio;
    b_cm = right_motor.encoder_counter * click_to_cm_ratio;

    //FIXME
    int pwm_a = pid_control(cm, a_cm, PID_P_MOVE, PID_I_MOVE, PID_D_MOVE, &a_sum, &a_previous_error, &a_prev_millis);
    //FIXME
    int pwm_b = pid_control(cm, b_cm, PID_P_MOVE, PID_I_MOVE, PID_D_MOVE, &b_sum, &b_previous_error, &a_prev_millis);

    if (pwm_a < 25 || pwm_b < 25)
    {
      break;
    }
    left_motor.pwm(pwm_a);
    right_motor.pwm(pwm_b);
  }
  left_motor.stop();
  right_motor.stop();
  //MyOled::print_oled_welcome_prompt();
}

// PID zwracający PWM
int pid_control(double cm_total, double cm_driven, double propotion, double integral, double derivative, double *sum, double *previous_error, unsigned long *prev_mils)
{
  double error = cm_total - cm_driven; // cm to end
  double delta = *previous_error - error;
  int time_delta = millis() - *prev_mils;
  *previous_error = error;
  *sum += error;
  int pwm = (propotion * error) + (integral * (*sum)) + (derivative * delta);
  Serial.println("PID Error\t" + String(int(error)) + "\t Delta:\t" + String(delta) + " Prev err\t" + String(*previous_error) + " Sum\t" + String(*sum) + " PWM \t" + String(pwm));
  if (pwm > 255)
  {
    pwm = 255;
  }
  *prev_mils = millis();
  return pwm;
}

//TODO
void rotate(int value)
{
  left_motor.reset_encoder_counter();
  right_motor.reset_encoder_counter();

  double desired = 320;
  int to_write_a = 0;
  int to_write_b = 0;
  int my_value = value;
  short max_pwm_value = 170;
  char offset = 50;
  if ((value > 0) && (value <= desired))
  {
    left_motor.backward();
    right_motor.forward();
    to_write_b = int((((double)(desired - value) / desired) * max_pwm_value)) + offset;
    right_motor.pwm(to_write_b);
    left_motor.pwm(0);
  }
  else if ((value > desired) && (value <= desired * 2))
  {
    left_motor.forward();
    right_motor.backward();
    value = value - desired;
    to_write_a = int((((double)(value) / desired) * max_pwm_value)) + offset;
    right_motor.pwm(0);
    left_motor.pwm(to_write_a);
  }
  else
  {
    left_motor.fast_stop();
    right_motor.fast_stop();
  }
  //MyOled::print_oled_rotation_input(my_value, to_write_a, to_write_b);
}

// Zmienia zmienna ktora jest wykorzystywana do ustawiania predkosci silnikow
void velocity(int value_pwm)
{
  target_velocity = value_pwm;
}

//????????????????????????????
void drive(int cm, bool direction)
{
  if (direction)
  {
    left_motor.forward();
    right_motor.forward();
  }
  else
  {
    left_motor.backward();
    right_motor.backward();
  }

  left_motor.reset_encoder_counter();
  right_motor.reset_encoder_counter();

  //TODO kalibracja
  int start_value = 30;
  unsigned long sofstart_offset = millis();
  int x = 30;

  //softstart
  while (x <= target_velocity)
  {
    if ((millis() - sofstart_offset) % 100 == 0)
    {
      left_motor.pwm(x);
      right_motor.pwm(x - start_value); // prawy silnik jezdzi szybciej przy tym samym napieciu co lewy
      x++;
    }
  }
  // ustawienie zmiennych pomocniczych do PIDa
  double a_sum = 0;        // integral part
  double a_previous_error; // derivative part
  double b_sum = 0;        // integral part
  double b_previous_error; // derivative part
  bool first_delta_error = true;
  unsigned long a_prev_millis = millis();
  unsigned long b_prev_millis = millis();
  // ustawienie zmiennych sluzacych do obliczania predkosci silnikow
  short interval = 250;

  double a_previous_rotation = 0;
  double b_previous_rotation = 0;
  int first_dt = (millis() - sofstart_offset) / 100;
  double a_vel_calibration = 0.9; //aby uniknąć skrętu w lewo na początku
  double a_vel = (left_motor.encoder_counter * (interval / DT_INCREASE_RATE)) * a_vel_calibration / (double)first_dt;
  double b_vel = 0;
  unsigned long offset = millis();
  unsigned int dt;
  unsigned int prev_dt = 0;
  unsigned long current_millis;

  // Zmienna przechowujaca wartosc wyliczona w PIDzie
  int pwn_b = 0;

  // Wyzerowanie wartosci licznikow rotacji enkoderow dzialajacych na przerwaniach
  left_motor.encoder_counter = 0;
  right_motor.encoder_counter = 0;
  boolean first_vel = true;

  while (true)
  {
    dt = millis() - offset;
    // Obliczanie predkosci obydwu silnikow co okreslony interwal
    if (check_interval(dt, prev_dt, interval))
    {
      if (!first_vel)
      {
        prev_dt = dt;
        current_millis = millis();
        a_vel = measure_velocity(&a_previous_rotation, left_motor.encoder_counter, current_millis, &a_prev_millis);
        b_vel = measure_velocity(&b_previous_rotation, right_motor.encoder_counter, current_millis, &b_prev_millis);
        //Serial.println(pwn_b);
      }
      else
      {
        //Serial.println("[FIRST VEL CHECK OMMITTED]");
        first_vel = false;
      }
    }

    // Pierwsze wywolanie PIDa, ustawienie odpowiednich bledow
    if (first_delta_error)
    {
      a_previous_error = a_vel - b_vel;
      b_previous_error = b_vel - a_vel;
      first_delta_error = false;
    }

    // Ustawienie wartosci PWM silnika B na podstawie obliczen z PIDa
    pwn_b = pid_control_velocity(a_vel, &b_vel, PID_P_DRIVE, PID_I_DRIVE, PID_D_DRIVE, &b_sum, &b_previous_error);
    right_motor.pwm(pwn_b);
  }

  // Zahamowanie obydwoma silnikami
  left_motor.stop();
  right_motor.stop();
}

bool distance_reached(int cm)
{
  return (cm >= calculate_distance());
}

float calculate_distance()
{
  //TODO zwróć odległość na podstawie średniej enkoderów
  return 0;
}

bool check_interval(unsigned int dt, unsigned int prev_dt, int interval)
{
  return (dt % interval == 0) && (dt != prev_dt) && (dt > 0);
}

// Mierzy predkosc na podstawie przyrostu licznika rotacji w czasie | przetestowane - dziala
double measure_velocity(double *previous_rotation, double current_rotation, unsigned long current_millis, unsigned long *previous_millis)
{
  double dt = (current_millis - *previous_millis) / DT_INCREASE_RATE;
  double velocity = (current_rotation - *previous_rotation) / dt;
  *previous_rotation = current_rotation;
  *previous_millis = current_millis;
  return velocity;
}

// INPUT : Predkosc drugiego silnika
// OUTPUT: PWM
double pid_control_velocity(double other_velocity, double *my_vel, double p, double i, double d, double *sum, double *previous_error)
{
  //Rożnica silnika na którym działa PID i drugiego silnika || rzedu max 1.2, ok. 0.8
  double error = *my_vel - other_velocity;
  double delta = error - *previous_error; // bardzo male, jakies 0.1
  *sum += error;

  *previous_error = error;

  float pid_output_pwm = (p * error) + (i * *sum) + (d * delta);

  if (pid_output_pwm > 255)
  {
    pid_output_pwm = 255;
  }
  else if (pid_output_pwm < 0)
  {
    pid_output_pwm = 0;
  }
  return int(pid_output_pwm);
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
  // odczytuje 4 bajty i laduje je do tablicy bytes_read
  Serial.readBytes(bytes_read, 4);
  Serial.flush();
  control = bytes_read[0];

  load_value();
}

// Zaladowanie do inta tablicy z liczba
void load_value()
{
  read_value_chars[0] = bytes_read[1];         // pierwsza cyfra
  read_value_chars[1] = bytes_read[2];         // druga cyfra
  read_value_chars[2] = bytes_read[3];         // trzecia cyfra
  read_value_chars[3] = '\0';                  // dodanie na koncu znaku konca ciagu zeby zrzutowac na inta
  sscanf(read_value_chars, "%d", &read_value); // czyta tablice znakow inta do zmiennej int korzystajac z adresu
}
