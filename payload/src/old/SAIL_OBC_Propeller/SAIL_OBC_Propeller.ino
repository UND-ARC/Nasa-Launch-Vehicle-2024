#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// PID Parameters
double Setpoint, Input, Output;
double Kp=2.0, Ki=5.0, Kd=1.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize the BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("No BNO055 detected, check wiring!");
    while (1);
  } else {
    Serial.println("BNO055 detected successfully.");
  }

  analogWriteFrequency(3, 500);
  analogWriteResolution(8);

  Setpoint = 20; // Target descent velocity in ft/s (convert to your units if necessary)

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Adjust based on your ESC and motor capabilities

  Serial.println("Setup complete, entering main loop...");
}

void loop() {
  Input = getDescentVelocity();

  myPID.Compute();

  analogWrite(3, Output);

  // Serial debugging output
  Serial.print("Input (Velocity): ");
  Serial.print(Input);
  Serial.print(" ft/s, Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" ft/s, Output (PWM): ");
  Serial.println(Output);

  delay(100); // Adjust based on your control loop requirements
}

double getDescentVelocity() {
  // Placeholder for actual velocity measurement code
  // This example assumes euler.x() as the velocity, which needs proper implementation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double velocity = euler.x(); // Adjust according to your setup
  return velocity;
}
