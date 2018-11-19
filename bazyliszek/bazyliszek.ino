#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

//Sonar
#define sonar_pin_trigger A2
#define sonar_pin_echo A3

//Infrared sensor
//Czujnik odleglosci na podczerwien
#define infrared_input A0

//Servo
#define infrared_serwo A1

//Encoders
#define interruptA_pin 2
#define interruptB_pin 3

//Bluetooth
#define bluetooth_state A6
String bluetooth_status = "";
int prev_bluetooth_state = 3;

//OLED parameters
//Parametry OLEDa
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
#define OLED_RESET 4

int rotatorA = 1; // TODO
int rotatorB = 1; // TODO

//Servo object
//Obiekt servo
Servo infrared_servo;

//When interrupt was fired recently
//Kiedy ostatnio wystapilo przerwanie
static unsigned long last_time_a = 0;
static unsigned long last_time_b = 0;


//Rotation counters
//Liczniki obrotow osi
int a_rotation_counter = 0;
int b_rotation_counter = 0;

//Actual PWM value for motors
//Aktualne PWM silnikow
int enA_value = 0;
int enB_value = 0;

//Back sonar reading
//Odczyt ultradzwiekowego czujnika odleglosci
int back_sonar_reading = 1000;
unsigned long back_sonar_previousMillis = 0;

//Actual servo position
//Aktualna pozyja servo
int servo_position = 0;

Adafruit_SSD1306 display(OLED_RESET);

void setup() {
  Serial.begin(38400);

  set_pin_modes();

  initialize_servo();

  attach_interrupts();

  setup_oled();

}

void set_pin_modes() { //Setting I/O pins
  //Ustawienie pinow wejscia i wyjscia
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //pinMode(enA, OUTPUT);
  //pinMode(enB, OUTPUT);
  pinMode(interruptA_pin, INPUT);
  pinMode(interruptB_pin, INPUT);
  pinMode(bluetooth_state, INPUT);
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  pinMode(sonar_pin_trigger, OUTPUT);
  pinMode(sonar_pin_echo, INPUT);
}

void initialize_servo() {
  infrared_servo.attach(infrared_serwo);
  //infrared_servo.setMaximumPulse(2000);//FIXME
  //infrared_servo.setMinimumPulse(500);//FIXME
}

void attach_interrupts() { //Attaching functions to interrupts
  //Przypisanie funkcji do przerwan
  attachInterrupt(digitalPinToInterrupt(interruptA_pin), encoder_a_interrupt_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB_pin), encoder_b_interrupt_handler, CHANGE);
}

void setup_oled() { //Set up and prompt on OLED
  //Ustawienie i wiadomosc powitalna na OLEDzie
  String info = "Bazyliszek 0.1\n\nWaiting for\nuser input";
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  print_oled_welcome_prompt();
}
void print_oled_welcome_prompt() { //Printing prompt on OLED
  //Wyswietlenie wiadomosci powitalnej na ekranie
  String info = "Bazyliszek 0.1\n" + bluetooth_status + "\n\nWaiting for input";
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(info);
  display.display();
}

// == protokol kontrolowania przez port szeregowy == GG
char bytes_read[4]; // tablica 4 bajtow do odczytywania danych
char control; // znak sterowania - m,r,b,c,v,s,o
int read_value; // odczytana wartosc
char read_value_chars[4] = ""; // tymczasowa tablica przechowujaca odczytana wartosc + '\0' (znak konca linii)
// ========
int previous_state;
int current_state;

void loop() {
  check_bluetooth_state();
  //check_sonar_nonblocking();

  if (Serial.available() > 0) { // GG
    load_received_data(); // GG
    // Do sterowania nalezy korzystac ze zmiennych:
    // control - znak sterowania
    // read_value - wartosc odczytana, moze byc 0
    boolean listed = true;
    String extra_info = "";

    switch (control) {
      case 'm': // move
        move_robot(read_value); // GG
        break;
      case 'r': // rotate
        rotate(read_value); // GG
        break;
      case 'b':// back sonar
        extra_info = "\nSonar: ";
        extra_info += String(back_sonar());
        extra_info += "cm";
        break;
      case 'c':// IR infra
        extra_info = "\nIR: ";
        extra_info += String(infrared());
        extra_info += "cm";
        break;
      case 'v':// velocity
        velocity(read_value);
        break;
      case 's':// stop motors
        stop_motors();
        break;
      case 'o':// serwo mechanism
        extra_info = "\nServo: ";
        extra_info += String(read_value);
        extra_info += "'";
        serwo(read_value); // kat jako arg
        break;
      default:
        listed = false;
        break;
    }
    display_char(control, listed, extra_info);
  }
}

void check_bluetooth_state() {
  //  String bt_ok = "BT: connected";
  //  String bt_wait = "BT: no connection";
  //  int state = digitalRead(bluetooth_state);
  //  if (prev_bluetooth_state!=state) {
  //    prev_bluetooth_state=state;
  //   if(state==HIGH) {
  //       bluetooth_status=bt_ok;
  //    } else if(state==LOW) {
  //       bluetooth_status=bt_wait;
  //    }
  //    print_oled_welcome_prompt();
  //  }
}

void move_robot(int cm) { //Move robot for the delared distance, measured in encoder readings
  //Przesuniecie robota o zadana odleglosc liczona w odczytach enkoderow

  float click_to_cm_ratio = 0.6; // dystans w cm przejechany przy jednym obrocie kolka
  float a_cm = 0;
  float b_cm = 0;
  a_rotation_counter = 0; // wyzerowanie licznika obrotow dla silnika A
  b_rotation_counter = 0; // wyzerowanie licznika obrotow dla silnika B
  a_forward();
  b_forward();
  my_analog_write(enA, 255);
  my_analog_write(enB, 255);
  float propotion = 1.51; // propocja 255 -> 100 = 155 to jest zjazd
  float integral = 0.01; // calka - stala wartosc przez ktora mnozy sie sume
  float derivative = 1.51; // pochodona - stala wartosc przez ktora mnozy sie roznice


  float a_sum = 0; // for integral part
  float a_previous_error = cm; // for derivative part
  unsigned long a_prev_millis = millis();
  unsigned long b_prev_millis = millis();
  float b_sum = 0; // for integral part
  float b_previous_error = cm; // for derivative part

     for (int i = 0; i < 200; i += 1) {
      analogWrite(enA, i);
     analogWrite(enB, i);
     enA_value = enB_value = i;
     delay(1);
  }

  while (true) {
    write_oled_rotation_count(a_cm, b_cm, cm);

    a_cm = a_rotation_counter * click_to_cm_ratio;
    b_cm = b_rotation_counter * click_to_cm_ratio;

    int pwm_a = pid_control(cm, a_cm, propotion, integral, derivative, &a_sum, &a_previous_error, &a_prev_millis); //FIXME
    int pwm_b = pid_control(cm, b_cm, propotion, integral, derivative, &b_sum, &b_previous_error, &a_prev_millis); //FIXME
    if (pwm_a < 25 || pwm_b < 25) {
      break;
    }
    my_analog_write(enA, pwm_a);
    my_analog_write(enB, pwm_b);

  }
  stop_motors();
  print_oled_welcome_prompt();
}

// PID
// Returns: PWM
int pid_control(float cm_total, float cm_driven, float propotion, float integral, float derivative, float* sum, float* previous_error, unsigned long *prev_mils) {

  float error = cm_total - cm_driven; // cm to end
  float delta = *previous_error - error;
  int time_delta = millis() - *prev_mils;
  *previous_error = error;
  *sum += error;
  int pwm = (propotion * error) + (integral * (*sum)) + (derivative * delta);
  Serial.println("Error\t" + String(int(error)) + "\t Delta:\t" + String(delta) + " Prev err\t" + String(*previous_error) + " Sum\t" + String(*sum) + " PWM \t" + String(pwm));
  //int pwm = (100+propotion*error);
  if (pwm > 255) {
    pwm = 255;
  }
  *prev_mils = millis();

  return pwm;

}
// ===
void rotate(int value) {

}
int back_sonar() {
  int dist = measure_sonar_distance();
  Serial.println(dist);
  //Serial.println(dist);
  return (dist);
}

//Variables used in non-delay checking of sonar reading
//Zmienne pomocnicze obslugi sonara bez opoznien
int trigState = LOW;
int interval = 1; // interval in milliseconds at which trigPin turns on
int interval2 = 1000; //time in milliseconds at which the distance is printed in serial monitors
int printState = LOW;

void check_sonar_nonblocking() {
  unsigned long currentMillis = millis(); //time in milliseconds from which the code was started
  if (currentMillis - back_sonar_previousMillis >= interval) { //check "blink without delay" code
    back_sonar_previousMillis = currentMillis;
    if (trigState == LOW) {
      (trigState = HIGH);
    }
    else {
      (trigState = LOW);
    }
  }
  // printing if statement
  if (currentMillis - back_sonar_previousMillis >= interval2) { //check "blink without delay" code
    back_sonar_previousMillis = currentMillis;
    if (printState == LOW) {
      (printState = HIGH);
    }
    else {
      (printState = LOW);
    }
  }
  digitalWrite(sonar_pin_trigger, trigState);
  int duration, distance; //variables
  duration = pulseIn(sonar_pin_echo, HIGH);
  distance = (duration / 2) / 29.1;
  if (printState = HIGH) {
    if (distance != 0) {
      back_sonar_reading = distance;
      //Serial.print(distance);
      //Serial.println("cm");}
    }

  }
}
int infrared() { //Measure and calculate distance from infrared sensor
  //Zmierzenie i wyliczenie odleglosci z czujnika odleglosci na podczerwien
  float val = analogRead(infrared_input); // odczyt
  int cm = 10650.08 * pow(val, -0.935) - 10;
  if (cm > 150) {
    cm = -1;
  }
  Serial.println(cm);
  return (cm);
}
void velocity(int value) { //Set both motors to given PWM
  //Ustawienie PWMa obu silnikow do zadanej wartosci
  if (value > 1) { // go forward
    a_forward();
    b_forward();
    analogWrite(enA, 200);
    analogWrite(enB, 200);
    delayMicroseconds(100);
    for (int i = 70; i < value; i += 10) {
      analogWrite(enA, i);
      analogWrite(enB, i);
      enA_value = enB_value = i;
      update_oled();
      delay(50);
    }

  } else { // stop
    a_free_stop();
    b_free_stop();
  }
}
void stop_motors() {
  a_fast_stop();
  b_fast_stop();
}
void serwo(int angle) {
  infrared_servo.write(angle);
  servo_position = angle;
}

int measure_sonar_distance() {
  //[DEPRECATED]
  //[DEPRECATED]
  //[DEPRECATED]
  int maximumRange = 200; // Maximum range needed
  int minimumRange = 0; // Minimum range needed
  long duration, distance; // Duration used to calculate distance
  on(sonar_pin_trigger);
  delayMicroseconds(10);
  off(sonar_pin_trigger);
  duration = pulseIn(sonar_pin_echo, HIGH);
  distance = duration / 58.2;
  delayMicroseconds(100); // !!!!!!
  if (distance >= maximumRange || distance <= minimumRange)  // "out of range"
    return -1;
  else
    return distance;
}

//Handling interrupts
//Obsluga przerwan

void encoder_a_interrupt_handler() {
  ////Serial.println("A inter");
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_time_a > 1) {
    a_rotation_counter++;
    int foo = a_rotation_counter / 2;

  }
  last_time_a = interrupt_time;
}

void encoder_b_interrupt_handler() {
  //Serial.println("B inter");
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_time_b > 1) { // 1ms opoznienia
    b_rotation_counter++;
    //Serial.print("fun b");
    int foo = b_rotation_counter / 2;
  }
  last_time_b = interrupt_time;
}

void update_oled() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("enA:");
  display.setCursor(25, 0);
  display.print(enA_value);
  display.setCursor(60, 0);
  display.print("enB:");
  display.setCursor(85, 0);
  display.print(enB_value);
  display.setCursor(0, 8);
  display.print("l:");
  display.setCursor(15, 8);
  display.print(infrared ());
  display.setCursor(0, 16);
  display.print("sr:");
  display.setCursor(20, 16);
  display.print(servo_position);
  display.display();
}


void display_char(char c, boolean listed, String extra_info) {
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

void write_oled_rotation_count(float a_rotation_counter, float b_rotation_counter, float rotation_quantity) {
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

void my_analog_write(int analog_pin, int analog_val) {
  analogWrite(analog_pin, analog_val);
  if (analog_pin == enA) {
    enA_value = analog_val;
  } else if (analog_pin == enB) {
    enB_value = analog_val;
  }
}

void on(int pin) {
  digitalWrite(pin, HIGH);
}

void off(int pin) {
  digitalWrite(pin, LOW);
}

void a_forward() {
  on(in1);
  off(in2);
}

void b_forward() {
  on(in3);
  off(in4);
}
void a_backward() {
  off(in1);
  on(in2);
}
void b_backward() {
  off(in3);
  on(in4);
}
void a_free_stop() {
  off(enA);
}
void b_free_stop() {
  off(enB);
}
void a_fast_stop() {
  on(enA);
  off(in1);
  off(in2);
}
void b_fast_stop() {
  on(enB);
  off(in3);
  off(in4);
}

//======= funkcje czytania z portu szeregowego
// ----- Dzialanie protokolu ------
// postac flagi: znak + trzy cyfry, no. f259, b000
// protokol umozliwia wysylanie dowolnej wartosci
// o maksymalnej liczbie cyfr rĂłwnej 3 oraz znak
// oznaczajacy akcje wykonywana przez robota
// Przyklad: jezeli chcesz wyslac flage ze znakiem r o wartosci 90
// flaga wyslana przez port szeregowy powinna miec postac: r090
// ----- Zmienne -----
// bytes_read - tablica 4 bajtow do ktorej sa pakowane dane z portu szeregowego
// control - znak sterujacy robotem, odpowiednio: m,r,b,c,v,s,o
// read_value - integer, wartosc odczytana z portu szeregowego
void load_received_data() { // odczytuje 4 bajty i przypisuje je do zmiennych
  Serial.readBytes(bytes_read, 4); // zaladowanie bajtow do bytes_read
  Serial.flush();
  control = bytes_read[0]; // znak kierunku
  load_speed_value(); // zaladowanie do inta wartosci przyspieszenia
  //display_data(bytes_read[0],bytes_read[1],bytes_read[2],bytes_read[3]);
}

void load_speed_value() {
  read_value_chars[0] = bytes_read[1]; // pierwsza cyfra liczby przyspieszenia
  read_value_chars[1] = bytes_read[2]; // druga cyfra liczby przyspieszenia
  read_value_chars[2] = bytes_read[3]; // trzecia cyfra liczby przyspieszenia
  read_value_chars[3] = '\0'; // dodanie na koncu znaku konca ciagu zeby zrzutowac ladnie na inta
  sscanf(read_value_chars, "%d", &read_value); // czyta tablice znakow intow do zmiennej int korzystajac z adresu
}
