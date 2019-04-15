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
void setup_oled()
{
  String info = "Bazyliszek 0.1\n\nWaiting for\nuser input";
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)

  // Show image buffer on the display hardware. Since the buffer is intialized with
  // an Adafruit splashscreen internally, this will display the splashscreen.
  display.display();

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  print_oled_welcome_prompt();
}

void print_oled_welcome_prompt()
{
  String info = "Bazyliszek 0.1\n" + bluetooth_status + "\n\nWaiting for input";
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(info);
  display.display();
}

void print_oled_rotation_input(int input, int enA_speed, int enB_speed)
{
  char left = 24;
  char right = 25;
  char arrow;
  display.clearDisplay();
  display.setCursor(0, 0);
  String arrows = "";
  if (enB_speed > enA_speed)
  {
    arrow = 24; //left
  }
  else
  {
    arrow = 25; //right
  }
  int iterations = max(enA_speed, enB_speed);
  iterations = int(((double)iterations) / 11.25);
  for (int i = 0; i < iterations; i++)
  {
    arrows += arrow;
  }
  display.print("Bazyliszek szuka...\nX <0; 640>: " + String(input) + "\n" + arrows + "\nenA: " + String(enA_speed) + "     enB: " + String(enB_speed));
  display.display();
}