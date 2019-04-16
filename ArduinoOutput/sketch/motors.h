#include <Arduino.h>
class MyMotors {
    public:
        static void on(int);
        static void off(int);
        static void a_forward();
        static void b_forward();
        static void a_backward();
        static void b_backward();
        static void a_free_stop();
        static void b_free_stop();
        static void a_fast_stop();
        static void b_fast_stop();
        static void stop_motors();

};

