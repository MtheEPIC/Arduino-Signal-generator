#define LED_PIN 13
#define ZERO 0
#define PIN13_ENABLE B00100000
#define POT_FINE A0
#define POT_COARSE A1
#define TOLERANCE_FINE_UP 1
#define TOLERANCE_FINE_DOWN 1
#define ANALOG_READ_MIN 0
#define ANALOG_READ_MAX 1023
#define TIMER_CNT_MIN 100
#define TIMER_CNT_MAX 1000
bool toggle_timer = false;
int timer0_cnt = ZERO;
int read_fine_pot();
int read_coarse_pot();
void ISRLowFreq();
ISR(TIMER0_COMPA_vect);

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int fine_pot_val, prev_fine_pot_val = -1;
int coarse_pot_val, prev_coarse_pot_val = -1;

enum FreqRange {LOW_FREQ, MID_FREQ, HIGH_FREQ};
FreqRange freq_range = LOW_FREQ;
int freq_mult = 1;
///////////////////////////////////////////////////////////////////////
#include <Servo.h>
Servo servo;
int next_millis = 0;
int loop_index1 = 0;
bool servo_cw = false;
///////////////////////////////////////////////////////////////////////
#define square_pin 2
#define sine_pin 3
#define triangle_pin 4
#define sawtooth_pin 5
#define square_pin_shift 2
#define sine_pin_shift 3
#define triangle_pin_shift 4
#define sawtooth_pin_shift 5
#define square_pin_byte B00000100
#define sine_pin_byte B00001000
#define triangle_pin_byte B00010000
#define sawtooth_pin_byte B00100000
#define BINARY1 B00000001
#define out_pin 11 //uses timer2 //6uses timer0 //13
#define capacitor 7
bool square_pin_prev = LOW;
bool sine_pin_prev = LOW;
bool triangle_pin_prev = LOW;
bool sawtooth_pin_prev = LOW;
bool square_pin_curr;
bool sine_pin_curr;
bool triangle_pin_curr;
bool sawtooth_pin_curr;
bool running_square = false;
bool running_sine = false;
bool running_triangle = false;
bool running_sawtooth = false;
void runSquare();
void runSine();
void runTriangle();
void runSawtooth();
void runMenu();

int F = 2;
int Fs = 500;
const int FS_MAX = 500;
double t;
double sampling_interval;
double sampling_interval_total;
byte samples[FS_MAX];

void setup() {
  DDRB |= PIN13_ENABLE;
  pinMode(POT_FINE, INPUT_PULLUP);
  pinMode(POT_COARSE, INPUT_PULLUP);
  ISRLowFreq();
  ///////////////////////////////////
  servo.attach(out_pin);
  Serial.begin(9600);
  Serial.println("start");
  ////////////////////////////////////


  DDRD &= B11000011; // set digital 5-2 to inputs
  PORTD |= B00111100; // set digital 5-2 to HIGH (should make them pull-up)

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Choose Mode");
  lcd.setCursor(0, 1);
  lcd.print("Freq:");

  pinMode(square_pin, INPUT_PULLUP);
  pinMode(sine_pin, INPUT_PULLUP);
  pinMode(triangle_pin, INPUT_PULLUP);
  pinMode(sawtooth_pin, INPUT_PULLUP);
  pinMode(out_pin, OUTPUT);
  pinMode(capacitor, OUTPUT);


  for (int n = 0; n < Fs; n++) {
    t = (float) n / Fs;
    samples[n] = (byte) (127.0 * sin(2 * 3.14 * t) + 127.0);
  }
  //  sampling_interval = 1000000 / (F * Fs);

  ////////////////////////////////////
  cli();

  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();
}

void loop() {
  runMenu();

  freq_range = read_coarse_pot();
  fine_pot_val = read_fine_pot();
  coarse_pot_val = read_coarse_pot();

  if (prev_fine_pot_val > fine_pot_val + TOLERANCE_FINE_UP || prev_fine_pot_val < fine_pot_val - TOLERANCE_FINE_DOWN)
  {
    updateLcd();
    prev_fine_pot_val = fine_pot_val;
  }
  if (prev_coarse_pot_val != coarse_pot_val)
  {
    updateLcd();
    prev_coarse_pot_val = coarse_pot_val;
  }

  switch (coarse_pot_val) {
    case LOW_FREQ:
      ISRLowFreq();
      break;
    case 1:
      ISRMidFreq();
      break;
    case 2:
      ISRHighFreq();
      break;
  }

  sampling_interval_total = 1.0e6 / freq_mult;
  sampling_interval_total /= fine_pot_val;
  //  sampling_interval_total = 1.0 / (fine_pot_val);
  sampling_interval = sampling_interval_total / Fs;
  //  Serial.println(sampling_interval_total, 3);


  if (timer0_cnt * freq_mult >= fine_pot_val)
  {
    switch (toggle_timer) {
      case 0:
        PORTB |= PIN13_ENABLE;
        break;
      case 1:
        PORTB &= ~PIN13_ENABLE;
        break;
    }
    timer0_cnt = ZERO;
    toggle_timer = !toggle_timer;
  }
}
/////////////////////////////////////////

void runMenu() {
  lcd.setCursor(0, 0);

  //  square_pin_curr = digitalRead(square_pin);
  //  sine_pin_curr = digitalRead(sine_pin);
  square_pin_curr = (PIND >> square_pin_shift & BINARY1);
  sine_pin_curr = (PIND >> sine_pin_shift & BINARY1);
  triangle_pin_curr = (PIND >> triangle_pin_shift & BINARY1);
  sawtooth_pin_curr = (PIND >> sawtooth_pin_shift & BINARY1);

  if (square_pin_curr && !square_pin_prev)
  {
    lcd.print("square mode  ");
    running_square = true;
    running_sine = false;
    running_triangle = false;
    running_sawtooth = false;
  }
  else if (sine_pin_curr && !sine_pin_prev)
  {
    lcd.print("sine mode    ");
    running_square = false;
    running_sine = true;
    running_triangle = false;
    running_sawtooth = false;
  }
  else if (triangle_pin_curr && !triangle_pin_prev)
  {
    lcd.print("triangle mode");
    running_square = false;
    running_sine = false;
    running_triangle = true;
    running_sawtooth = false;
  }
  else if (sawtooth_pin_curr && !sawtooth_pin_prev)
  {
    lcd.print("sawtooth mode");
    running_square = false;
    running_sine = false;
    running_triangle = false;
    running_sawtooth = true;
  }

  square_pin_prev = square_pin_curr;
  sine_pin_prev = sine_pin_curr;
  triangle_pin_prev = triangle_pin_curr;
  sawtooth_pin_prev = sawtooth_pin_curr;

  if (running_square)
  {
    runSquare();
  }
  else if (running_sine)
  {
    runSine();
  }
  else if (running_triangle)
  {
    runTriangle();
  }
  else if (runSawtooth)
  {
    runSine();
  }

}

void runSquare() {
  digitalWrite(capacitor, HIGH);
}

void runSine() {
  digitalWrite(capacitor, LOW);

  /*
    //  //  OCR2A = 50 ;  // (0-255): dutycycle=f/(2*[1+ocr2a]) // assuming analogWrite() called already for pin 11
    for (int i = 0; i < Fs; i++) { // from 0-255
    //      analogWrite(out_pin, samples[i]); //OCR2A = samples[i]; PORTn |= out_pin_byte
    //      delayMicroseconds(sampling_interval); //milies() (maybe?)
    //    servo.writeMicroseconds(map(samples[i], 0, 255, 544, 2400));
    Serial.println(samples[i]);
    delayMicroseconds(sampling_interval);
    }
  */


  if (servo_cw)
  {
    if (loop_index1 < 0 || loop_index1 > 180) loop_index1 = 0;

    if (timer0_cnt >= next_millis || (timer0_cnt < 0 && next_millis > 0) || (next_millis - (2 * sampling_interval)) > timer0_cnt)
    {
      //      servo.writeMicroseconds(map(i, 0, 180, 544, 2400));
      servo.write(loop_index1);
      next_millis = timer0_cnt + sampling_interval;
      loop_index1++;
    }
  }
  else
  {
    if (loop_index1 < 0 || loop_index1 > 180) loop_index1 = 180;

    if (timer0_cnt >= next_millis || (timer0_cnt < 0 && next_millis > 0) || (next_millis - (2 * sampling_interval)) > timer0_cnt)
    {
      //      servo.writeMicroseconds(map(i, 0, 180, 544, 2400));
      servo.write(loop_index1);
      next_millis = timer0_cnt + sampling_interval;
      loop_index1--;
    }
  }


  if (loop_index1 < 0 || loop_index1 > 180) servo_cw = !servo_cw;


}

void runTriangle() {
  digitalWrite(capacitor, HIGH);
}

void runSawtooth() {
  digitalWrite(capacitor, HIGH);
}

/////////////////////////////////////////
void updateLcd() {
  lcd.setCursor(5, 1);
  lcd.print(fine_pot_val);
  switch (coarse_pot_val) {
    case LOW_FREQ:
      lcd.print("Hz  ");
      break;
    case MID_FREQ:
      lcd.print("KHz ");
      break;
    case HIGH_FREQ:
      lcd.print("KHz ");
      break;
  }
}


void ISRLowFreq() {
  freq_mult = 1;
}

void ISRMidFreq() {
  freq_mult = 1000;
}

void ISRHighFreq() {
  ISRMidFreq();
}

ISR(TIMER0_COMPA_vect) {//timer0 interrupt 1Hz toggles pin 13 (LED)
  timer0_cnt++;
}

int read_fine_pot() {
  int analog_read_val = analogRead(POT_FINE);
  return map(analog_read_val, ANALOG_READ_MIN, ANALOG_READ_MAX, TIMER_CNT_MIN, TIMER_CNT_MAX);
}

int read_coarse_pot() {
  int analog_read_val = analogRead(POT_COARSE);
  return map(analog_read_val, ANALOG_READ_MIN, ANALOG_READ_MAX, LOW_FREQ, HIGH_FREQ + 1);
}
