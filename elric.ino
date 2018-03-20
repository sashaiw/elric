#include <Servo.h>
#include <Filters.h>
#include <stdarg.h>

enum mode {VARIABLE, BINARY};
enum state {OPEN, CLOSED};
typedef enum mode Mode;
typedef enum state State;

// Tune-able parameters
const int POLL_RATE          = 1;        // Poll rate in ms
const int DEFAULT_MIN_INPUT  = 100;       // Minimum input value (0-1024
const int DEFAULT_MAX_INPUT  = 300;      // Maximum input value (0-1024)
const int RANGE_DELTA        = 25;       // Delta for calibrated range
const int CALIBRATE_LENGTH   = 50;       // Number of values to collect for calibration
const int CALIBRATE_TIME     = 5000;     // Duration (ms) to collect calibration data
const int CURVE_STEEPNESS    = 20;       // Steepness of output curve. DO NOT GO BELOW LIKE 4 OR SOMETHING.
const float FILTER_FREQUENCY = .1;       // Frequency in HZ of LPF
const Mode DEFAULT_MODE      = VARIABLE; // Mode to start in
const int BINARY_THRESHOLD   = 512;
const int BINARY_DELTA       = 200;

// Pins
const int MOTOR_PIN  = 23; // Servo
const int SENSOR_PIN = 26; // Myoelectric sensor
const int CAL_BUTTON = 22; // Button for calibration routine

Servo servo;
FilterOnePole lpf(LOWPASS, FILTER_FREQUENCY);

void setup() {
  Serial.begin(115200);
  servo.attach(MOTOR_PIN);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(MIN_BUTTON, INPUT_PULLUP);
  pinMode(MAX_BUTTON, INPUT_PULLUP);
}

void loop() {
  float output_value;
  int input_value = analogRead(SENSOR_PIN);
  static int min_input = DEFAULT_MIN_INPUT;
  static int max_input = DEFAULT_MAX_INPUT;
  static State state = OPEN;
  static Mode mode = DEFAULT_MODE;

  // Calibration buttons
  if (digitalRead(MIN_BUTTON) == LOW) {
    min_input = calibrate_values() + RANGE_DELTA;
    Serial.print("Min input calibrated to ");
    Serial.println(min_input);
  }
  if (digitalRead(MAX_BUTTON) == LOW) {
    max_input = calibrate_values() - RANGE_DELTA;
    Serial.print("Max input calibrated to ");
    Serial.println(max_input);
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

  if (mode == VARIABLE) {
    // Apply curve
    //output_value = curve(output_value, CURVE_STEEPNESS);

    // Write to servo
    servo.write(scale_to_range(output_value, 0, 1, 0, 180));

  } else if (mode == BINARY) {
    output_value = scale_to_range(output_value, 0, 1, 0, 1024);
    if (state == OPEN && output_value > BINARY_THRESHOLD) {
      delay(1000);
      servo.write(180);
      delay(1000);
      state = CLOSED;
    } else if (state == CLOSED && output_value < BINARY_THRESHOLD) {
      delay(1000);
      servo.write(0);
      delay(1000);
      state = OPEN;
    }
  }
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
        Serial.print(va_arg(valist, int));
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
