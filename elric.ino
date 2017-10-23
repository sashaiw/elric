#include <Servo.h>
#include <Filters.h>

// Tune-able parameters
const int POLL_RATE        = 10;  // Poll rate in ms
const int SIGMOID_GAIN     = 2;   // Gain for sigmoid function (e^-x)
const int MIN_INPUT        = 40;  // Minimum input value (0-1024
const int MAX_INPUT        = 200; // Maximum input value (0-1024)
const int FILTER_FREQUENCY = 100; // Frequency in HZ of LPF
const int SMOOTH_LENGTH    = 5;   // Length of average 

// Pins
const int MOTOR_PIN = 10; // Servo
const int SENSOR_PIN = 0; // Myoelectric sensor

Servo servo;
FilterOnePole lpf(LOWPASS, FILTER_FREQUENCY);

void setup() {
  Serial.begin(115200);
  servo.attach(MOTOR_PIN);
}

void loop() {
  int input_value;
  float output_value;
  
  // Get input value with LPF
  input_value = analogRead(SENSOR_PIN);

  // Scale to range
  output_value = scale_to_range(output_value, MIN_INPUT, MAX_INPUT, 0, 1);

  // Apply LPF
  output_value = lpf.input(input_value);
  
  // Average
  output_value = average(SMOOTH_LENGTH, input_value);
  
  // Apply sigmoid curve
  output_value = sigmoid(output_value, SIGMOID_GAIN);

  // Debug, plot two graphs
  Serial.print(input_value);
  Serial.print(",");
  Serial.println(output_value * 1024);

  // Write to servo
  servo.write(output_value * 180);
  delay(POLL_RATE);
}

float average(int smooth_length, float input_value) {
  static int input_readings[SMOOTH_LENGTH];
  static int input_read_index = 0;
  static float input_total = 0;
  static float input_average = 0;

  input_total -= input_readings[input_read_index];
  input_readings[input_read_index] = input_value;
  input_total += input_readings[input_read_index];
  input_read_index++;
  input_read_index = input_read_index >= SMOOTH_LENGTH ? 0 : input_read_index;

  return input_total / SMOOTH_LENGTH;
}

float scale_to_range(float input, int old_min, int old_max, int new_min, int new_max) {
  return (((input - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
}

float sigmoid(float input, float gain) {
  return 1 / (1 + pow(2.71828, (-gain) * (input - 0.5) ));
}

