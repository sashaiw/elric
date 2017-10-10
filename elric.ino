#include <Servo.h>

const int SIGMOID_GAIN = 2;
const int MIN_INPUT = 10;
const int MAX_INPUT = 707; 
const int SMOOTH_LENGTH = 10;

const int MOTOR_PIN = 10;
const int SENSOR_PIN = 0;

int input_readings[SMOOTH_LENGTH];
int input_read_index = 0;
float input_total = 0;
float input_average = 0;

Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(MOTOR_PIN);

  for (int i = 0; i < SMOOTH_LENGTH; i++) {
    input_readings[i] = 0;
  }
}

void loop() {
  int input = analogRead(SENSOR_PIN);
  Serial.print("Raw: ");
  Serial.println(input);

  // Smooth that shit
  input_total -= input_readings[input_read_index];
  input_readings[input_read_index] = input;
  input_total += input_readings[input_read_index];
  input_read_index++;
  if (input_read_index >= SMOOTH_LENGTH) { input_read_index = 0; }

  float motorval = input_total / SMOOTH_LENGTH;
  
  // BULLSHIT MATH
  // Scale between 0 and 1
  motorval = scale_to_range(motorval, MIN_INPUT, MAX_INPUT, 0, 1);
  // sigmoid curve
  motorval = sigmoid(motorval, SIGMOID_GAIN);

  Serial.println(motorval * 1024);
  servo.write(motorval * 180);
  delay(50);
}

float scale_to_range(float input, int old_min, int old_max, int new_min, int new_max) {
  return (((input - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
}

float sigmoid(float input, float gain) {
  return 1 / (1 + pow(2.71828, (gain - 2 * gain) * (input - 0.5) ));
}

