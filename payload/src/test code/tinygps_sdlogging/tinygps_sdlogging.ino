#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <pitches.h>

// Define the serial connection pins
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8

// Define the hardware serial connection
#define GPS_SERIAL Serial2  // Assuming you're using Serial1 for GPS
#define GPS_BAUDRATE 38400

int Buzzer = 4;

// Create a SoftwareSerial object to communicate with the GPS module
//SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// Create a TinyGPS++ object to parse NMEA data
TinyGPSPlus gps;

// File to log data
File dataFile;

unsigned long currentTime = millis();

// Constants
#define SEALEVELPRESSURE_INHG 29.87   // Sea level pressure in inches of mercury, adjust as per your location
#define GROUND_LEVEL_ELEVATION_FEET 868 // Ground level elevation in feet, adjust as per your location

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp;

#define BNO055_SAMPLERATE_DELAY_MS (100)

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

void setup() {
  startupMelody();
  Serial.begin(115200); // Initialize serial communication for debugging
  //gpsSerial.begin(19200); // Initialize serial communication with GPS module
  //gpsSerial.println("$PMTK220,1000*2C<CR><LF>");
  //gpsSerial.println("$PMTK251,0*28<CR><LF>");

  GPS_SERIAL.begin(GPS_BAUDRATE); // Initialize serial communication with GPS module
  //GPS_SERIAL.println("$PMTK220,1000*2C<CR><LF>");
  //GPS_SERIAL.println("$PMTK251,0*28<CR><LF>");
  
  // Initialize SD card
  if (!SD.begin(5)) { // Pin 10 for CS
    Serial.println("SD card initialization failed!");
    badTone();
    return;
  }
  Serial.println("SD card initialized."); 
  goodTone();
  delay(1000);

  if (!bmp.begin_I2C()) {
  Serial.println("Could not find the BMP390 sensor :( Check your soldering and inspect for bad connections");
  } else {
    Serial.println("Found BMP390");
    goodTone();
    delay(1000);
  }

  if(bno.begin()) { 
    Serial.println("Found BNO055");
    goodTone();
    delay(1000);
  } else {
    Serial.println("Could not find BNO055");
  }

  // Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Create a new file on SD card
  dataFile = SD.open("gps_data.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening file!");
    return;
  }

    // Write header to the file
  dataFile.println("Latitude,Longitude,GPS Alt (ft AGL),GPS Speed (ft/s),Date,Time,BMP Alt (ft AGL),IMU Lin Accel Z (m/s),TempF, Ang Vel (rad/s)");
}

void loop() {

  float temperatureF = bmp.temperature * 1.8 + 32; // Convert Celsius to Fahrenheit
  float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); 
  float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET;

  sensors_event_t linearAccelData, orientationData, angVelocityData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Keep reading data from GPS module
  while (GPS_SERIAL.available() > 0) {
    if (gps.encode(GPS_SERIAL.read())) {
      // If new data is available, parse it
      if (gps.location.isValid()) {
        // Retrieve latitude and longitude
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        Serial.println("---------------------------------------");
        Serial.print("Latitude: ");
        Serial.println(latitude, 6); // Print latitude with 6 decimal places
        Serial.print("Longitude: ");
        Serial.println(longitude, 6); // Print longitude with 6 decimal places
      
        // Log data to SD card
        dataFile.print(latitude, 6);
        dataFile.print(",");
        dataFile.print(longitude, 6);
        dataFile.print(",");     
      }

      if (gps.altitude.isValid()) {
        // Retrieve GPS altitude
        float gpsAltitude = gps.altitude.meters();
        float gpsAltitudeFeet = 3.28084 * gpsAltitude; // Convert meters to feet
        float gpsAltitudeAGL = gpsAltitudeFeet - GROUND_LEVEL_ELEVATION_FEET;
        Serial.print("GPS Alt: ");
        Serial.print(gpsAltitudeFeet);
        Serial.println("ft MSL");
        Serial.print("GPS Alt: ");
        Serial.print(gpsAltitudeAGL);
        Serial.println("ft AGL");
        dataFile.print(gpsAltitudeAGL);
        dataFile.print(",");       
      }
      
      if (gps.speed.isValid()) {
        // Retrieve speed
        float speed_kmph = gps.speed.kmph();
        float speed_fts = speed_kmph * 0.911344; // Convert km/h to ft/s
        Serial.print("GPS Speed: ");
        Serial.print(speed_fts);
        Serial.println(" ft/s");

        // Log speed to SD card
        dataFile.print(speed_fts);
        dataFile.print(",");
      }
      
      if (gps.date.isValid() && gps.time.isValid()) {
        // Retrieve date and time
        int year = gps.date.year();
        int month = gps.date.month();
        int day = gps.date.day();
        int hour = gps.time.hour();
        int minute = gps.time.minute();
        int second = gps.time.second();
        
        // Print date and time
        Serial.print("Date: ");
        Serial.print(year);
        Serial.print("-");
        Serial.print(month);
        Serial.print("-");
        Serial.print(day);
        Serial.print(" Time: ");
        Serial.print(hour);
        Serial.print(":");
        if (minute < 10) Serial.print("0"); // Add leading zero if minute < 10
        Serial.print(minute);
        Serial.print(":");
        if (second < 10) Serial.print("0"); // Add leading zero if second < 10
        Serial.println(second);

        // Log date and time to SD card
        dataFile.print(year);
        dataFile.print("-");
        dataFile.print(month);
        dataFile.print("-");
        dataFile.print(day);
        dataFile.print(",");
        dataFile.print(hour);
        dataFile.print(":");
        if (minute < 10) dataFile.print("0"); // Add leading zero if minute < 10
        dataFile.print(minute);
        dataFile.print(":");
        if (second < 10) dataFile.print("0"); // Add leading zero if second < 10
        dataFile.print(second);
        dataFile.print(",");
      }
        Serial.print("BMP Altitude: ");
        Serial.print(altitudeAGL);
        Serial.println(" ft AGL");
        Serial.print("IMU Linear Accel Z: ");
        Serial.println(linearAccelData.acceleration.z);
        Serial.print("Ang Vel Z: ");
        Serial.print(angVelocityData.gyro.z);
        Serial.println(" rad/s");
        dataFile.print(altitudeAGL);
        dataFile.print(",");
        dataFile.print(linearAccelData.acceleration.z);
        dataFile.print(",");
        dataFile.print(temperatureF);
        dataFile.print(",");
        dataFile.print(angVelocityData.gyro.z);
        dataFile.println();

        dataFile.flush();
      }
    }
  }

