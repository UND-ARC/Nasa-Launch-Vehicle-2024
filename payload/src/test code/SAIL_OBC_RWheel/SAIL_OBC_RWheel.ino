#include <Adafruit_BNO055.h> // BNO055 Library

// Pin configuration for ESC
const int ESC_PIN = 23; // Connect the signal wire of ESC to digital pin 23

// BNO055 Sensor
Adafruit_BNO055 bno; // Create a BNO055 sensor object

// Desired roll rate (in radians per second)
const float DESIRED_ROLL_RATE = 0.0; // Adjust as needed

// PID control variables
float roll_rate_error, previous_roll_rate_error, roll_rate_integral;
float kp = 1.0; // Proportional gain (adjust as needed)
float ki = 0.01; // Integral gain (adjust as needed)
float kd = 0.1; // Derivative gain (adjust as needed)
float throttle;

void setup() {
  // Initialize the ESC control
  pinMode(ESC_PIN, OUTPUT);
  digitalWrite(ESC_PIN, LOW); // Start with the motor off

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Could not find a valid BNO055 sensor, check wiring!");
    while (1);
  }
  
  // Set BNO055 to read the desired data (e.g., roll rate)
  bno.setExtCrystalUse(true);

  // Initialize PID control variables
  roll_rate_integral = 0;
  previous_roll_rate_error = 0;
}

void loop() {
  // Read roll rate from the BNO055 sensor
  float currentRollRate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).x(); // Get roll rate data (adjust axis as needed)

  // Calculate roll rate error
  roll_rate_error = DESIRED_ROLL_RATE - currentRollRate;

  // Calculate control output (throttle) using PID control
  float p_term = kp * roll_rate_error;
  roll_rate_integral += ki * roll_rate_error;
  float d_term = kd * (roll_rate_error - previous_roll_rate_error);
  throttle = p_term + roll_rate_integral + d_term;

  // Map throttle to ESC PWM values (adjust as needed)
  int throttle_pwm = map(throttle, -1.0, 1.0, 1000, 2000); // Motor speed range depends on your ESC

  // Send throttle signal to ESC
  analogWrite(ESC_PIN, throttle_pwm);

  // Store the current roll rate error as previous for the next iteration
  previous_roll_rate_error = roll_rate_error;
}
