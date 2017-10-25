#include <Servo.h>
#include <Filters.h>

// Tune-able parameters
const int POLL_RATE        = 10;    // Poll rate in ms
const int SIGMOID_GAIN     = 5;     // Gain for sigmoid function (e^-x)
const int MIN_INPUT        = 200;   // Minimum input value (0-1024
const int MAX_INPUT        = 250;   // Maximum input value (0-1024)
const int SMOOTH_LENGTH    = 100;   // Length of average 
const int CALIBRATE_LENGTH = 50;    // Number of values to collect for calibration
const int CALIBRATE_TIME   = 5000;  // Amount of time 
const float FILTER_FREQUENCY = .25; // Frequency in HZ of LPF

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
  float output_value;
  int input_value = analogRead(SENSOR_PIN);
  
  // Apply LPF
  lpf.input(input_value);
  output_value = lpf.output();

  // Scale to range
  output_value = scale_to_range(output_value, MIN_INPUT, MAX_INPUT, 0, 1);

  // Apply sigmoid curve
  output_value = sigmoid(output_value, SIGMOID_GAIN);

  // Debug, plot two graphs
  Serial.print(scale_to_range(input_value, 0, 1024, 0, 180));
  Serial.print(",");
  Serial.println(output_value * 180);

  // Write to servo
  servo.write(output_value * 180);
  delay(POLL_RATE);
}

float scale_to_range(float input_value, float old_min, float old_max, float new_min, float new_max) {
  return (((input_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
}

float sigmoid(float input, float gain) {
  return 1 / (1 + pow(2.71828, (gain - 2 * gain) * (input - 0.5) ));
}

int calibrate_values() {
  Serial.println("CALIBRATING");
  int avg = 0;
  int readings[CALIBRATE_LENGTH];
  for(int i = 0; i < CALIBRATE_LENGTH; i++) {
    readings[i]=analogRead(SENSOR_PIN);
    delay(CALIBRATE_TIME/CALIBRATE_LENGTH);
  }
  for(int i = 0; i < CALIBRATE_LENGTH; i++) {
    avg += readings[i];
  }
  avg /= CALIBRATE_LENGTH;
  return avg;
}
