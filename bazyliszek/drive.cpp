#include "drive.h"
#include "motors.h"

//Motor A
//Silnik A
#define in1 7
#define in2 5
#define enA 6

//Motor B
//Silnik B
#define in3 9
#define in4 8
#define enB 11

const double p = -22;     // propocja 255 -> 100 = 155 to jest zjazd
const double i = -0.0149; // calka - stala wartosc przez ktora mnozy sie sume
const double d = -19;     // pochodona - stala wartosc przez ktora mnozy sie roznice
const short dt_increase_rate = 100;

void drive(int cm, bool direction)
{
  if(direction) {
    MyMotors::a_forward();
    MyMotors::b_forward();
  } else {
    MyMotors::a_backward();
    MyMotors::b_backward();
  }

  //TODO  resetownaie liczników pwmów
  a_rotation_counter = 0;
  b_rotation_counter = 0;

  //TODO kalibracja
  int start_value = 30;
  unsigned long sofstart_offset = millis();
  int x = 30;

  //SOFTstart loop
  while (x <= move_speed)
  {
    if ((millis() - sofstart_offset) % 100 == 0)
    {
      analogWrite(enA, x);
      analogWrite(enB, x - start_value);
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
  MyMotors::a_fast_stop();
  MyMotors::b_fast_stop();
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