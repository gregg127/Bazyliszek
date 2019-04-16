#include <Arduino.h>
#line 1 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
#line 1 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
//================================================================================
//                                   DOŁĄCZANIE
//================================================================================
#include <math.h>
#include <SPI.h>
#include <Wire.h>

#include "myoled.h"
#include "motor.h"

//==============================================================================
//                          KONFIGURACJA (STAŁE)
//==============================================================================

#define MOTOR_LEFT_DIR_1 7
#define MOTOR_LEFT_DIR_2 5
#define MOTOR_LEFT_PWM 6
#define MOTOR_LEFT_ENC 2

#define MOTOR_RIGHT_DIR_1 9
#define MOTOR_RIGHT_DIR_2 8
#define MOTOR_RIGHT_PWM 6
#define MOTOR_RIGHT_ENC 3


//================================================================================
//                                   ZMIENNE GLOBALNE
//================================================================================

//Lewy silnik
Motor left_motor = Motor(MOTOR_LEFT_DIR_1, MOTOR_LEFT_DIR_2, MOTOR_LEFT_PWM, MOTOR_LEFT_ENC);

//Prawy silnik
Motor right_motor = Motor(MOTOR_RIGHT_DIR_1, MOTOR_RIGHT_DIR_2, MOTOR_LEFT_PWM, MOTOR_RIGHT_ENC);

//Prędkość ustawiana w funkcji velocity, definuje z jaki pwm ma być podawany na silniki w funkcji move
unsigned char target_velocity;

//serial port communication variables
char bytes_read[4];            // tablica 4 bajtow do odczytywania danych
char control;                  // znak sterowania - m,r,b,c,v,s,o
int read_value;                // odczytana wartosc
char read_value_chars[4] = ""; // tymczasowa tablica przechowujaca odczytana wartosc + '\0' (znak konca linii)

// TODO : check whats this
boolean wants_to_be_printed = true;

//Funkcje obsługujące przerwania lewego silnika, w przyszłości mają znaleźć się w strukturze Motor
#line 49 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void left_motor_interrupt();
#line 55 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void right_motor_interrupt();
#line 61 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void init_motors();
#line 68 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void attach_interrupts();
#line 76 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void setup();
#line 85 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void set_pin_modes();
#line 93 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void loop();
#line 133 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void move_robot(int cm, bool forward);
#line 200 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
int pid_control(double cm_total, double cm_driven, double propotion, double integral, double derivative, double *sum, double *previous_error, unsigned long *prev_mils);
#line 219 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void rotate(int value);
#line 258 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void velocity(int value_pwm);
#line 262 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void drive(int cm, bool direction);
#line 364 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
bool distance_reached(int cm);
#line 367 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
float calculate_distance();
#line 372 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
bool check_interval(unsigned int dt, unsigned int prev_dt, int interval);
#line 378 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
double measure_velocity(double *previous_rotation, double current_rotation, unsigned long current_millis, unsigned long *previous_millis);
#line 389 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
double pid_control_velocity(double other_velocity, double *my_vel, double p, double i, double d, double *sum, double *previous_error);
#line 414 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void encoder_a_interrupt_handler();
#line 424 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void encoder_b_interrupt_handler();
#line 435 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void analog_write_motors(int analog_pin, int analog_val);
#line 453 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void load_received_data();
#line 463 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void load_speed_value();
#line 49 "c:\\Users\\Krzysztof Filipow\\Documents\\dyplomowa\\bazyliszek\\bazyliszek\\bazyliszek.ino"
void left_motor_interrupt()
{
  left_motor.interrupt();
}

//Funkcje obsługujące przerwania prawego silnika, w przyszłości mają znaleźć się w strukturze Motor
void right_motor_interrupt()
{
  right_motor.interrupt();
}

//Utworzenie struktur dla obydwu silników
void init_motors()
{
  //left_motor =
  //right_motor = 
}

//Przypisanie funkcji obsługującej przerwania z enkoderów
void attach_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(left_motor.encoder_pin_1), left_motor_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(right_motor.encoder_pin_1), right_motor_interrupt, RISING);
}



void setup()
{
  Serial.begin(115200);
  set_pin_modes();
  init_motors();
  attach_interrupts();
  MyOled::setup_oled();
}

void set_pin_modes()
{
  pinMode(bluetooth_state, INPUT);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}


void loop()
{
  if (Serial.available() >= 4)
  {
    load_received_data();
    // Do sterowania nalezy korzystac ze zmiennych:
    // control - znak sterowania
    // read_value - wartosc odczytana, moze byc 0
    boolean listed = true;
    String extra_info = " " + String(read_value);

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
      wants_to_be_printed = false;
      break;
    case 'v':
      velocity(read_value);
      break;
    case 's': // stop motors
      MyMotors::stop_motors();
      break;
    default:
      listed = false;
      break;
    }
    if (wants_to_be_printed)
    {
      MyOled::display_char(control, listed, extra_info);
    }
  }
}

void move_robot(int cm, bool forward)
{ //Move robot for the delared distance, measured in encoder readings

  double click_to_cm_ratio = 0.6; // dystans w cm przejechany przy jednym obrocie kolka
  double a_cm = 0;
  double b_cm = 0;
  a_rotation_counter = 0; // wyzerowanie licznika obrotow dla silnika A
  b_rotation_counter = 0; // wyzerowanie licznika obrotow dla silnika B
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

  left_motor.pwm(target_velocity);
  right_motor.pwm(target_velocity);

  // propocja 255 -> 100 = 155 to jest zjazd
  double propotion = 1.51;  

  // calka - stala wartosc przez ktora mnozy sie sume
  double integral = 0.01;   

  // pochodona - stala wartosc przez ktora mnozy sie roznice
  double derivative = 1.51; 

  double a_sum = 0;             // for integral part
  double a_previous_error = cm; // for derivative part
  unsigned long a_prev_millis = millis();
  unsigned long b_prev_millis = millis();
  double b_sum = 0;             // for integral part
  double b_previous_error = cm; // for derivative part

  for (int i = 0; i < 200; i += 1)
  {
    left_motor.pwm(i);
    right_motor.pwm(i);
    delay(1);
  }

  while (true)
  {
    MyOled::write_oled_rotation_count(a_cm, b_cm, cm);

    a_cm = a_rotation_counter * click_to_cm_ratio;
    b_cm = b_rotation_counter * click_to_cm_ratio;

    int pwm_a = pid_control(cm, a_cm, propotion, integral, derivative, &a_sum, &a_previous_error, &a_prev_millis); //FIXME
    int pwm_b = pid_control(cm, b_cm, propotion, integral, derivative, &b_sum, &b_previous_error, &a_prev_millis); //FIXME
    if (pwm_a < 25 || pwm_b < 25)
    {
      break;
    }
    left_motor.pwm(pwm_a);
    right_motor.pwm(pwm_b);
  }
  MyMotors::stop_motors();
  MyOled::print_oled_welcome_prompt();
}

// PID
// Returns: PWM
int pid_control(double cm_total, double cm_driven, double propotion, double integral, double derivative, double *sum, double *previous_error, unsigned long *prev_mils)
{
  double error = cm_total - cm_driven; // cm to end
  double delta = *previous_error - error;
  int time_delta = millis() - *prev_mils;
  *previous_error = error;
  *sum += error;
  int pwm = (propotion * error) + (integral * (*sum)) + (derivative * delta);
  Serial.println("Error\t" + String(int(error)) + "\t Delta:\t" + String(delta) + " Prev err\t" + String(*previous_error) + " Sum\t" + String(*sum) + " PWM \t" + String(pwm));
  //int pwm = (100+propotion*error);
  if (pwm > 255)
  {
    pwm = 255;
  }
  *prev_mils = millis();

  return pwm;
}

void rotate(int value) //TODO
{
  a_rotation_counter = b_rotation_counter = 0;
  double desired = 320;
  int to_write_a = 0;
  int to_write_b = 0;
  int my_value = value;
  short max_pwm_value = 170;
  char offset = 50;
  if ((value > 0) && (value <= desired))
  {
    MyMotors::b_forward();
    MyMotors::a_backward();
    to_write_b = int((((double)(desired - value) / desired) * max_pwm_value)) + offset;
    analogWrite(enB, to_write_b);
    analogWrite(enA, 0);
  }
  else if ((value > desired) && (value <= desired * 2))
  {
    MyMotors::a_forward();
    MyMotors::b_backward();
    value = value - desired;
    to_write_a = int((((double)(value) / desired) * max_pwm_value)) + offset;
    analogWrite(enA, to_write_a);
    analogWrite(enB, 0);
  }
  else
  {
    MyMotors::a_fast_stop();
    MyMotors::b_fast_stop();
  }
  MyOled::print_oled_rotation_input(my_value, to_write_a, to_write_b);
}

const double p = -22;     // propocja 255 -> 100 = 155 to jest zjazd
const double i = -0.0149; // calka - stala wartosc przez ktora mnozy sie sume
const double d = -19;     // pochodona - stala wartosc przez ktora mnozy sie roznice
const short dt_increase_rate = 100;

void velocity(int value_pwm) {
  target_velocity = value_pwm;
}

void drive(int cm, bool direction)
{
  if(direction) {
    left_motor.forward();
    right_motor.forward();
  } else {
    left_motor.backward();
    right_motor.backward();
  }

  //TODO  resetownaie liczników pwmów
  left_motor.reset_encoder_counter();
  right_motor.reset_encoder_counter();

  //TODO kalibracja
  int start_value = 30;
  unsigned long sofstart_offset = millis();
  int x = 30;

  //SOFTstart loop
  while (x <= target_velocity)
  {
    if ((millis() - sofstart_offset) % 100 == 0)
    {
      left_motor.pwm(x);
      right_motor.pwm(x - start_value);
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
  double a_vel = (a_rotation_counter * (interval / dt_increase_rate)) * a_vel_calibration / (double)first_dt;
  double b_vel = 0;
  unsigned long offset = millis();
  unsigned int dt;
  unsigned int prev_dt = 0;
  unsigned long current_millis;

  // === zmienna przochowujaca wartosc wyliczona w PIDzie
  int pwn_b = 0;

  // === wyzerowanie wartosci licznikow rotacji enkoderow dzialajacych na przerwaniach
  a_rotation_counter = 0;
  b_rotation_counter = 0;
  boolean first_vel = true;

  // === stworzenie zmiennej pomocniczej w celu pomiarow inlosci bledow
  short infrared_counter = 0;
  short infrared_distance = 80;

  while (true)
  {
    dt = millis() - offset;
    // === obliczanie predkosci obydwu silnikow co okreslony interwal
    if (check_interval(dt, prev_dt, interval))
    {
      if (!first_vel)
      {
        prev_dt = dt;
        current_millis = millis();
        a_vel = measure_velocity(&a_previous_rotation, a_rotation_counter, current_millis, &a_prev_millis);
        b_vel = measure_velocity(&b_previous_rotation, b_rotation_counter, current_millis, &b_prev_millis);
        //Serial.println(pwn_b);
      }
      else
      {
        //Serial.println("[FIRST VEL CHECK OMMITTED]");
        first_vel = false;
      }
    }

    // === pierwsze wywolanie PIDa, ustawienie odpowiednich bledow
    if (first_delta_error)
    {
      a_previous_error = a_vel - b_vel;
      b_previous_error = b_vel - a_vel;
      first_delta_error = false;
    }

    // === ustawienie wartosci PWM silnika B na podstawie obliczen z PIDa
    pwn_b = pid_control_velocity(a_vel, &b_vel, p, i, d, &b_sum, &b_previous_error);
    analog_write_motors(enB, pwn_b);
    if(true);
  }

  // === zahamowanie dwoma silnikami
  left_motor.fstop();
  right_motor.fstop();
}
bool distance_reached(int cm) {
  return (cm>=calculate_distance());
}
float calculate_distance() {
  //TODO zwróć odległość na podstawie śrenidej enkoderów
  return 0;
}

bool check_interval(unsigned int dt, unsigned int prev_dt, int interval)
{
  return (dt % interval == 0) && (dt != prev_dt) && (dt > 0);
}

//  CHECKED - do obliczania błędu
double measure_velocity(double *previous_rotation, double current_rotation, unsigned long current_millis, unsigned long *previous_millis)
{
  double dt = (current_millis - *previous_millis) / dt_increase_rate;
  double velocity = (current_rotation - *previous_rotation) / dt;
  *previous_rotation = current_rotation;
  *previous_millis = current_millis;
  return velocity;
}
// INPUT : dane velocity
// ERROR
// OUTPUT: PWM
double pid_control_velocity(double other_velocity, double *my_vel,
                            double p, double i, double d, double *sum, double *previous_error)
{

  //  unsigned long mil = millis();
  double error = *my_vel - other_velocity; //Rożnica silnika na którym działa PID i drugiego silnika || rzedu max 1.2, ok. 0.8
  double delta = error - *previous_error;  // b male, jakies 0.1
  *sum += error;

  *previous_error = error;

  float pid_output_pwm = (p * error) + (i * *sum) + (d * delta);

  if (pid_output_pwm > 255)
  { //pwm z akceptowanym zakresie
    pid_output_pwm = 255;
  }
  else if (pid_output_pwm < 0)
  {
    pid_output_pwm = 0;
  }
  return int(pid_output_pwm);
}

//Handling interrupts
void encoder_a_interrupt_handler()
{
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_time_a > 1)
  {
    a_rotation_counter++;
  }
  last_time_a = interrupt_time;
}

void encoder_b_interrupt_handler()
{
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_time_b > 1)
  {
    b_rotation_counter++;
  }
  last_time_b = interrupt_time;
}

//TODO delete this useless fun
void analog_write_motors(int analog_pin, int analog_val)
{
  analogWrite(analog_pin, analog_val);
}


// ======= funkcje czytania z portu szeregowego
// ----- Dzialanie protokolu ------
// postac flagi: znak + trzy cyfry, no. f259, b000
// protokol umozliwia wysylanie dowolnej wartosci
// o maksymalnej liczbie cyfr rĂ„â€šÄąâ€šwnej 3 oraz znak
// oznaczajacy akcje wykonywana przez robota
// Przyklad: jezeli chcesz wyslac flage ze znakiem r o wartosci 90
// flaga wyslana przez port szeregowy powinna miec postac: r090
// ----- Zmienne -----
// bytes_read - tablica 4 bajtow do ktorej sa pakowane dane z portu szeregowego
// control - znak sterujacy robotem, odpowiednio: m,r,b,c,v,s,o
// read_value - integer, wartosc odczytana z portu szeregowego
void load_received_data()
{                                  // odczytuje 4 bajty i przypisuje je do zmiennych
  Serial.readBytes(bytes_read, 4); // zaladowanie bajtow do bytes_read

  Serial.flush();
  control = bytes_read[0]; // znak kierunku
  load_speed_value();      // zaladowanie do inta wartosci przyspieszenia
  //display_data(bytes_read[0],bytes_read[1],bytes_read[2],bytes_read[3]);
}

void load_speed_value()
{
  read_value_chars[0] = bytes_read[1];         // pierwsza cyfra liczby przyspieszenia
  read_value_chars[1] = bytes_read[2];         // druga cyfra liczby przyspieszenia
  read_value_chars[2] = bytes_read[3];         // trzecia cyfra liczby przyspieszenia
  read_value_chars[3] = '\0';                  // dodanie na koncu znaku konca ciagu zeby zrzutowac ladnie na inta
  sscanf(read_value_chars, "%d", &read_value); // czyta tablice znakow intow do zmiennej int korzystajac z adresu
}
