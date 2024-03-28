#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Define PID parameters
double Kp = 1.0;   // Proportional gain
double Ki = 0.01;   // Integral gain
double Kd = 0.1;   // Derivative gain

// Define PID variables
double setpoint = 0.0;
double input, output;
double error, lastError, sumError;

// Define PID limits
double outMin = 1000;  // Minimum output value for ESC (in microseconds)
double outMax = 1900;  // Maximum output value for ESC (in microseconds)

// Define PID sample time (in milliseconds)
unsigned long lastTime;
unsigned long sampleTime = 100; // 100 ms

// Define servo pin
int servoPin = 23; // Example pin, change to your setup
Servo esc;

// Define Gyro
Adafruit_BNO055 bno = Adafruit_BNO055();

// Finite State Machine states
enum State {
  NOT_TUMBLING,
  TUMBLING
};

State currentState = NOT_TUMBLING;

// Define rolling filter parameters
const int numReadings = 10; // Number of readings to average
double gyroReadings[numReadings]; // Array to store gyro readings
int currentIndex = 0; // Index to keep track of current position in the array
double gyroFilteredDeg = 0.0; // Filtered gyro value in degrees per second

double readAngularVelocity() {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return angVelocityData.gyro.z; // Corrected return statement
}

void setup() {
  Serial.begin(9600);

  // Initialize ESC
  esc.attach(servoPin);
  armESC();

  // Initialize BNO055
  if (!bno.begin())
  {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  lastTime = millis();
}

void loop() {
  // Calculate time difference since last loop iteration
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime >= sampleTime) {
    // Read gyro data
    sensors_event_t angVelocityData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); 
    double gyroRateRad = readAngularVelocity();
    double gyroRateDeg = gyroRateRad * (180.0 / PI); // Convert rad/s to deg/s

    // Apply rolling filter to gyro readings
    gyroReadings[currentIndex] = gyroRateDeg;
    gyroFilteredDeg = 0.0;
    for (int i = 0; i < numReadings; i++) {
      gyroFilteredDeg += gyroReadings[i];
    }
    gyroFilteredDeg /= numReadings;

    // Update index for next reading
    currentIndex = (currentIndex + 1) % numReadings;

    // Update PID setpoint based on FSM state
    if (gyroFilteredDeg > 360.0 && currentState != TUMBLING) {
      currentState = TUMBLING;
      setpoint = 0.0; // Setpoint for tumbling
      Serial.println("TUMBLING state");
    } else if (gyroFilteredDeg <= 45.0 && currentState != NOT_TUMBLING) {
      currentState = NOT_TUMBLING;
      setpoint = 0.0; // Setpoint for not tumbling
      Serial.println("NOT TUMBLING state");
    }

    // Compute PID output only if in the tumbling state
    if (currentState == TUMBLING) {
      // Compute PID output
      input = gyroFilteredDeg;
      computePID();
    } else {
      // If not tumbling, set the output to a default value (e.g., minimum throttle)
      output = outMin;
    }

    // Send output to ESC
    esc.writeMicroseconds((int)output);

    // Print debug information
    Serial.print("Gyro Rate (deg/s): ");
    Serial.println(gyroFilteredDeg);
    Serial.print("PID Output: ");
    Serial.println(output);
    Serial.print("Current PWM: ");
    Serial.println((int)output); // Print current PWM sent to ESC
    Serial.println(currentState);

    // Reset timing variables
    lastTime = currentTime;
    delay(10);
  }
}

void computePID() {
  // Calculate error
  error = setpoint - input;

  // Compute integral term (with anti-windup)
  sumError += (error * sampleTime) / 1000.0;
  sumError = constrain(sumError, outMin / Ki, outMax / Ki);

  // Compute derivative term
  double dError = (error - lastError) / (sampleTime / 1000.0);

  // Compute PID output
  output = Kp * error + Ki * sumError + Kd * dError;

  // Constrain output
  output = constrain(output, outMin, outMax);

  // Save error for next iteration
  lastError = error;
}

void armESC() {
  // Send a pulse to the ESC to arm it
  esc.writeMicroseconds(1000);
  delay(3000);
}
