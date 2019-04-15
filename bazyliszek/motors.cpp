#include "motors.h"
#include <Arduino.h>

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


void on(int pin)
{
  digitalWrite(pin, HIGH);
}

void off(int pin)
{
  digitalWrite(pin, LOW);
}

void a_forward()
{
  on(in1);
  off(in2);
}

void b_forward()
{
  on(in3);
  off(in4);
}
void a_backward()
{
  off(in1);
  on(in2);
}
void b_backward()
{
  off(in3);
  on(in4);
}
void a_free_stop()
{
  off(enA);
}
void b_free_stop()
{
  off(enB);
}
void a_fast_stop()
{
  on(enA);
  off(in1);
  off(in2);
}
void b_fast_stop()
{
  on(enB);
  off(in3);
  off(in4);
}
void stop_motors()
{
  a_fast_stop();
  b_fast_stop();
}