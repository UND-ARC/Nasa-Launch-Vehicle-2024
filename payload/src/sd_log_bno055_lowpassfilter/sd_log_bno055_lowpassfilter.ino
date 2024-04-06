#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>

#define LOG_FILE "imu_data.csv"

Adafruit_BNO055 bno = Adafruit_BNO055(55);
File dataFile;

unsigned long lastTime;
double velocity[3] = {0, 0, 0}; // Initial velocity
double lastAcceleration[3] = {0, 0, 0};
double lastGyro[3] = {0, 0, 0};

const float alpha = 0.2; // Low-pass filter constant

void setup() {
  Serial.begin(9600);
  delay(1000);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055 sensor!");
    while (1);
  }

  delay(1000);

  lastTime = millis();

  if (!SD.begin(10)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  dataFile = SD.open(LOG_FILE, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening log file!");
    while (1);
  }
  
  dataFile.println("Time (ms), Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Velocity X (m/s), Velocity Y (m/s), Velocity Z (m/s), Gyro X (deg/s), Gyro Y (deg/s), Gyro Z (deg/s)");
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  unsigned long currentTime = millis();
  double deltaTime = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds
  lastTime = currentTime;

  // Apply low-pass filter to accelerometer data
  double acceleration[3] = {
    alpha * event.acceleration.x + (1 - alpha) * lastAcceleration[0],
    alpha * event.acceleration.y + (1 - alpha) * lastAcceleration[1],
    alpha * event.acceleration.z + (1 - alpha) * lastAcceleration[2]
  };

  // Apply low-pass filter to gyro data
  double gyro[3] = {
    alpha * event.gyro.x + (1 - alpha) * lastGyro[0],
    alpha * event.gyro.y + (1 - alpha) * lastGyro[1],
    alpha * event.gyro.z + (1 - alpha) * lastGyro[2]
  };

  for (int i = 0; i < 3; i++) {
    velocity[i] += 0.5 * (acceleration[i] + lastAcceleration[i]) * deltaTime;
    lastAcceleration[i] = acceleration[i];
    lastGyro[i] = gyro[i];
  }

  // Print data to Serial monitor
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(acceleration[0]);
  Serial.print(", ");
  Serial.print(acceleration[1]);
  Serial.print(", ");
  Serial.println(acceleration[2]);

  Serial.print("Velocity (m/s): ");
  Serial.print(velocity[0]);
  Serial.print(", ");
  Serial.print(velocity[1]);
  Serial.print(", ");
  Serial.println(velocity[2]);

  Serial.print("Gyro (deg/s): ");
  Serial.print(gyro[0]);
  Serial.print(", ");
  Serial.print(gyro[1]);
  Serial.print(", ");
  Serial.println(gyro[2]);

  // Log data to SD card
  dataFile.print(currentTime);
  dataFile.print(", ");
  dataFile.print(acceleration[0]);
  dataFile.print(", ");
  dataFile.print(acceleration[1]);
  dataFile.print(", ");
  dataFile.print(acceleration[2]);
  dataFile.print(", ");
  dataFile.print(velocity[0]);
  dataFile.print(", ");
  dataFile.print(velocity[1]);
  dataFile.print(", ");
  dataFile.print(velocity[2]);
  dataFile.print(", ");
  dataFile.print(gyro[0]);
  dataFile.print(", ");
  dataFile.print(gyro[1]);
  dataFile.print(", ");
  dataFile.println(gyro[2]);

  delay(100);

  dataFile.flush(); // Flush data to SD card
}
