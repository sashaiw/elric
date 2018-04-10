#include <Servo.h>
#include <Filters.h>
#include <stdarg.h>

typedef enum state State;

// Tune-able parameters
const int POLL_RATE          = 1;        // Poll rate in ms
const int DEFAULT_MIN_INPUT  = 100;      // Minimum input value (0-1024
const int DEFAULT_MAX_INPUT  = 200;      // Maximum input value (0-1024)
const int RANGE_DELTA        = 25;       // Delta for calibrated range
const int CALIBRATE_LENGTH   = 50;       // Number of values to collect for calibration
const int CALIBRATE_TIME     = 5000;     // Duration (ms) to collect calibration data
const int CURVE_STEEPNESS    = 20;       // Steepness of output curve. DO NOT GO BELOW LIKE 4 OR SOMETHING.
const float FILTER_FREQUENCY = 1;        // Frequency in HZ of LPF

// Pins
const int MOTOR_PIN  = 23; // Servo
const int SENSOR_PIN = 26; // Myoelectric sensor
const int CALIB_BTN  = 22; // Button for calibration routine

// EEPROM addresses
const int MIN_VAL_ADDR = 0;
const int MAX_VAL_ADDR = 0 + sizeof(int);

Servo servo;
FilterOnePole lpf(LOWPASS, FILTER_FREQUENCY);

void setup() {
  Serial.begin(115200);
  servo.attach(MOTOR_PIN);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(CALIB_BTN, INPUT_PULLUP);
}

void loop() {
  float output_value;
  int input_value = analogRead(SENSOR_PIN);
  static int min_input = EEPROM_read_int(MIN_VAL_ADDR);
  static int max_input = EEPROM_read_int(MAX_VAL_ADDR);
  static State state = OPEN;
  static Mode mode = DEFAULT_MODE;

  // Calibration buttons
  if (digitalRead(MIN_BUTTON) == LOW) {
    calibration_sequence(&min_input, &max_input);
  }

  // Apply LPF
  lpf.input(input_value);
  output_value = lpf.output();

  // Scale to range
  output_value = scale_to_range(output_value, min_input, max_input, 0, 1);

  // Debug, plot two graphs
  debug_graph(5, // Number of arguments
              scale_to_range(input_value, 0, 1024, 0, 1024),
              scale_to_range(output_value, 0, 1, 0, 1024),
              min_input,
              max_input,
              BINARY_THRESHOLD);

  // Apply curve
  //output_value = curve(output_value, CURVE_STEEPNESS);

  // Write to servo
  servo.write(180 - scale_to_range(output_value, 0, 1, 0, 180));
  delay(POLL_RATE);
}

float scale_to_range(float input_value, float old_min, float old_max, float new_min, float new_max) {
  return clamp((input_value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min, new_min, new_max);
}

float curve(float x, int steepness) {
  return .5 + .5 * tanh(steepness * (x - .5));
}

void debug_graph(int nArgs, ...) {
    va_list valist;
    va_start(valist, nArgs);
    for (int i=0; i < nArgs - 1; i++) {
        Serial.print(va_arg(valist, i));
        Serial.print(",");
    }
    Serial.println(va_arg(valist, int));
    va_end(valist);
}

// TODO: Test
//float curve(float x, int k) {
//  w2 = 1 - x;
//  f1 = x;
//  f2 = 0.5 + 0.5 * tanh(s * (x-.5));
//  f = w1 * f1 + w2 * f2;
//}

float clamp(float x, float min_value, float max_value) {
  if (x < min_value) {
    x = min_value;
  }
  if (x > max_value) {
    x = max_value;
  }
  return x;
}

void twitch() {
  servo.write(180);
  delay(200);
  servo.write(0);
  delay(200);
}

void EEPROM_write_int(int ee, int value) {
  const byte* p = (const byte*)(const void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(ee++, *p++);
  }
}

int EEPROM_read_int(int ee) {
  int value;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(ee++);
  }
  return value;
}

void calibration_sequence(&min_value, &max_value) {
  while (calib_btn == LOW) {} // Wait for button press
  min_value = calibrate_values() + RANGE_DELTA;
  EEPROM_write_int(MIN_VAL_ADDR, min_value);
  twitch();                   // Signal that calibration is complete.
  while (calib_btn == LOW) {} // Wait for button press
  max_value = calibrate_values() - RANGE_DELTA;
  EEPROM_write_int(MAX_VAL_ADDR, max_val);
}

int calibrate_values() {
  Serial.println("CALIBRATING");
  int avg = 0;
  int readings[CALIBRATE_LENGTH];
  for (int i = 0; i < CALIBRATE_LENGTH; i++) {
    readings[i] = analogRead(SENSOR_PIN);
    delay(CALIBRATE_TIME / CALIBRATE_LENGTH);
  }
  for (int i = 0; i < CALIBRATE_LENGTH; i++) {
    avg += readings[i];
  }
  avg /= CALIBRATE_LENGTH;
  return avg;
}
