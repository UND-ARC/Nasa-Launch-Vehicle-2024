#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>

#define LOG_FILE "imu_data.csv"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
File dataFile;

unsigned long lastTime;
double velocity[3] = {0, 0, 0}; // Initial velocity
double lastAcceleration[3] = {0, 0, 0};
double lastGyro[3] = {0, 0, 0};

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055 sensor!");
    while (1);
  }

  delay(1000);

  lastTime = millis();

  if (!SD.begin(5)) {
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

  double acceleration[3] = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  for (int i = 0; i < 3; i++) {
    velocity[i] += 0.5 * (acceleration[i] + lastAcceleration[i]) * deltaTime;
    lastAcceleration[i] = acceleration[i];
  }

  // Print data to Serial monitor
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(event.acceleration.x);
  Serial.print(", ");
  Serial.print(event.acceleration.y);
  Serial.print(", ");
  Serial.println(event.acceleration.z);

  Serial.print("Velocity (m/s): ");
  Serial.print(velocity[0]);
  Serial.print(", ");
  Serial.print(velocity[1]);
  Serial.print(", ");
  Serial.println(velocity[2]);

  Serial.print("Gyro (deg/s): ");
  Serial.print(event.gyro.x);
  Serial.print(", ");
  Serial.print(event.gyro.y);
  Serial.print(", ");
  Serial.println(event.gyro.z);

  // Log data to SD card
  dataFile.print(currentTime);
  dataFile.print(", ");
  dataFile.print(event.acceleration.x);
  dataFile.print(", ");
  dataFile.print(event.acceleration.y);
  dataFile.print(", ");
  dataFile.print(event.acceleration.z);
  dataFile.print(", ");
  dataFile.print(velocity[0]);
  dataFile.print(", ");
  dataFile.print(velocity[1]);
  dataFile.print(", ");
  dataFile.print(velocity[2]);
  dataFile.print(", ");
  dataFile.print(event.gyro.x);
  dataFile.print(", ");
  dataFile.print(event.gyro.y);
  dataFile.print(", ");
  dataFile.println(event.gyro.z);

  delay(100);

  dataFile.flush(); // Flush data to SD card
}
