#include<string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
void display_char(char, bool, String);
void write_oled_rotation_count(double, double, double);
void print_oled_welcome_prompt();
void print_oled_rotation_input(int, int, int);
void setup_oled();