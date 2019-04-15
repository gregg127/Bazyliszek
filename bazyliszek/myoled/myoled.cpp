#include "myoled.h"
void display_char(char c, boolean listed, String extra_info)
{
  char ctp = c - 32;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Bazyliszek 0.1\nUser input:\n");
  display.print(ctp);
  if (!listed)
    display.print(" - unknown command");
  display.print(extra_info);
  display.display();
}

void write_oled_rotation_count(double a_rotation_counter, double b_rotation_counter, double rotation_quantity)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("to drive: ");
  display.print(rotation_quantity);
  display.print("\ndrivenA: ");
  display.print(a_rotation_counter);
  display.print("\ndrivenB: ");
  display.print(b_rotation_counter);
  display.print("\npwmA: ");
  display.print(enA_value);
  display.print("  pwmB: ");
  display.print(enB_value);
  display.display();
}