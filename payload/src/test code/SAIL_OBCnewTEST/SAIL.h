
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "Adafruit_BMP3XX.h"
#include <RH_RF95.h>
#include <Adafruit_GPS.h>
#include <pitches.h>

// Constants
#define SEALEVELPRESSURE_INHG 30.13   // Sea level pressure in inches of mercury, adjust as per your location
#define GROUND_LEVEL_ELEVATION_FEET 830 // Ground level elevation in feet, adjust as per your location
#define SIGNAL_TIMEOUT 5000 // Radio timeout value in milliseconds
#define RF95_FREQ 915.0

#define GPSSerial Serial2 // pins 7 and 8 on Teensy 4.0
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
#define GPSECHO  false // Set to 'true' if you want to debug and listen to the raw GPS sentences
uint32_t timer = millis();

#define RFM95_CS 2
#define RFM95_RST 1
#define RFM95_INT 17

// Global variables
Servo esc1;
Servo esc2; 
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire); // by default address is 0x29 or 0x28
Sd2Card card;
SdVolume volume;
SdFile root;
File imuLogFile;
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//First, we'll set up the LEDs and buzzer
int R_LED = 9;
int G_LED = 3;
int B_LED = 6;
int Buzzer = 4;

#define BNO055_SAMPLERATE_DELAY_MS (100)

const int SDchipSelect = 5;

unsigned long lastLogTime = 0;
const unsigned long logInterval = 500; // Logging interval in milliseconds
unsigned long liftoffStartTime = 0;
unsigned long landingTime = 0;
bool liftoffDetected = false;
bool boostConfirmed = false;
bool landed = false;
const float liftoffAccelerationThreshold = 15; // Acceleration threshold for liftoff detection in m/s^2
const float boostAccelerationThreshold = 15;   // Acceleration threshold for boost confirmation in m/s^2
int loopCount = 0;

int PYRO = 21;

unsigned long lastAltitudeCheckTime = 0; // Variable to store the last time altitude was checked
unsigned long timeoutDuration = 5000; // Timeout duration in milliseconds
unsigned long startTime;

void goodTone() {
  tone(Buzzer, 988); delay(150); noTone(Buzzer); delay(150);
  tone(Buzzer, 1175); delay(150); noTone(Buzzer); delay(150);
  tone(Buzzer, 1319); delay(150); noTone(Buzzer); delay(300);
}

void badTone() {
  tone(Buzzer, 1319); delay(150); noTone(Buzzer); delay(150);
  tone(Buzzer, 1319); delay(150); noTone(Buzzer); delay(150);
  tone(Buzzer, 988); delay(200); noTone(Buzzer); delay(400);
}

void warningTone() {
  tone(Buzzer, 1000); delay(2000); noTone(Buzzer); delay(400);
}

void startupMelody() {
  int melody[] = {
    NOTE_C5, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, 0, NOTE_B4, NOTE_C5
  };
  int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4
  };

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(Buzzer, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(Buzzer);
  }
}

bool sendMessage(String message) {
  if (rf95.send((uint8_t *)message.c_str(), message.length())) {  // message.length()+1?
    rf95.waitPacketSent();
    Serial.println("Message sent successfully: " + message);
    return true; // Message sent successfully
  } else {
    Serial.println("Message failed: " + message);
    return false; // Message not sent due to activation state
  }
}

void fireLandingLegs() {   // Need to be able to receive force-open signal
  while (true) {
    if (millis() - lastAltitudeCheckTime >= 200) { // Check altitude every 200 milliseconds
      lastAltitudeCheckTime = millis(); // Update lastAltitudeCheckTime

      float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
      float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
      Serial.print("Altitude: ");
      Serial.print(altitudeAGL);
      Serial.println(" ft AGL");

      if (altitudeAGL < 350.0) {
        digitalWrite(PYRO, HIGH);
        Serial.println("Below 350ft, Releasing legs.");
        sendMessage("S: <350ft, releasing legs");
        delay(5000);
        digitalWrite(PYRO, LOW);
        Serial.println("Pyro wire deactivated.");
        break; // Exit the loop once pyro wire is deactivated
      }      
    }
  }
}

/**************************************************************************/
/*
    Displays some basic information on the BNO055 sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/////////////////////

    //Display some basic info about the BNO055 sensor status

/////////////////////////////////////
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display BNO055 sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}