#include<string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
class MyOled {
    public:
        static void display_char(char, bool, String);
        static void write_oled_rotation_count(double, double, double);
        static void print_oled_welcome_prompt();
        static void print_oled_rotation_input(int, int, int);
        static void setup_oled();
};