
// SAIL Flight Computer Startup Diagnostics
// Note1: Comment out "while(!Serial){} " in void setup before running on battery power!!!!
// Note2: Comment out pyro channel test before connecting anything to pyro channel
// Define altimeter rating for current conditions

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "Adafruit_BMP3XX.h"
#include <RH_RF95.h>
#include <Servo.h>

#define SEALEVELPRESSURE_INHG 30.03   // Sea level pressure in inches of mercury, adjust as per your location
#define GROUND_LEVEL_ELEVATION_FEET 889 // Ground level elevation in feet, adjust as per your location

Servo esc1; // Create a servo object to control the ESC
Servo esc2; // Create a servo object to control the ESC

//First, we'll set up the LEDs and buzzer
int R_LED = 9;
int G_LED = 3;
int B_LED = 6;
int Buzzer = 4;

//This is for the BMP390 barometer
Adafruit_BMP3XX bmp;
// double inHg = 29.92; // enter altimiter value
// double hPa = inHg * 33.8639;
//#define SEALEVELPRESSURE_HPA (hPa) // default: 1013.25

// For the BNO055 IMU, Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)

//This is for the SD card
Sd2Card card;
SdVolume volume;
SdFile root;
const int SDchipSelect = 5;

//This is for the pyro channel
int PYRO = 21;

#define RFM95_CS 2
#define RFM95_RST 1
#define RFM95_INT 17

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define SIGNAL_TIMEOUT 5000 // Timeout value in milliseconds (2 seconds) NEW

unsigned long lastAltitudeCheckTime = 0; // Variable to store the last time altitude was checked

void goodTone() { 
  tone(Buzzer, 988); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 988); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 2093); delay(100); noTone(Buzzer); delay(400);
}

void badTone() {
  tone(Buzzer, 2093); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 2093); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 988); delay(50); noTone(Buzzer); delay(400);
}

void warningTone() {
  tone(Buzzer, 1000); delay(2000); noTone(Buzzer); delay(400);
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
        sendMessage("<350ft, releasing legs");
        delay(10000);
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

/**************************************************************************/
/*
    Display some basic info about the BNO055 sensor status
*/
/**************************************************************************/
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




void setup() {
  delay(1000);

  esc1.attach(22); // Attach the ESC signal cable to pin 1
  esc2.attach(23);
  
  Serial.println();Serial.println();
  Serial.println("Hey! I'm the SAIL flight computer! Let's get started.");
  goodTone();
  delay(200);
  badTone();
  Serial.println();Serial.println();
  delay(1000);
  
  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

    /******* Now we'll test the radio ********/
  Serial.println("Now we'll check the radio module!");
  delay(500);
  Serial.println("Arduino LoRa TX Test!");
  delay(1000);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
/*
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  goodTone();
  delay(1000);
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  delay(1000);
  

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  
  rf95.setTxPower(23, false);

  */

  /******* END OF RADIO TEST *******/

  //Now Barometer
  Serial.println("First, let's see if the BMP390 Barometer is connected. Standby...");
  delay(1000);
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find the BMP390 sensor :( Check your soldering and inspect for bad connections");
    badTone();
    delay(2000);
    Serial.println();
    Serial.println("Proceeding with the rest of the startup process");
    Serial.println();
    delay(1000);
  }
  else{

// Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

     
    Serial.println("Found the BMP390 sensor! Here's some data...");
    goodTone();
    for (int i = 0; i <= 10; i++) 
    {
      Serial.println();
      float temperatureF = bmp.temperature * 1.8 + 32; // Convert Celsius to Fahrenheit
      Serial.print(F("Temperature = "));
      Serial.print(temperatureF);
      Serial.println(" °F");

      Serial.print("Current Altimeter Rating = ");
      Serial.print(SEALEVELPRESSURE_INHG);
      Serial.println(" inHg");

      float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
      Serial.print("Approx. Altitude (MSL) = ");
      Serial.print(altitudeMSL);
      Serial.println(" ft");

      float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
      Serial.print("Approx. Altitude (AGL) = ");
      Serial.print(altitudeAGL);
      Serial.println(" ft");
    delay(200);
  }
}

  Serial.println("Now we'll look for the the BNO055 IMU. Standby...");
  bno.begin();
  if(!bno.begin()) {
    badTone();
  }
  Serial.println(bno.begin() ? "Found it! BNO055 connection successful." : "BNO055 connection failed :(");
  goodTone();
  delay(1000);
  Serial.println();
  delay(1000);
  if(bno.begin())
  {
    Serial.println("Here's a little bit of IMU data!");
    /* Display some basic information on this sensor */
    displaySensorDetails();
     /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

     /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    
    for (int i = 0; i <= 50; i++) 
    {
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      
      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      /* Display calibration status for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      Serial.print("CALIBRATION: Sys=");
      Serial.print(system, DEC);
      Serial.print(" Gyro=");
      Serial.print(gyro, DEC);
      Serial.print(" Accel=");
      Serial.print(accel, DEC);
      Serial.print(" Mag=");
      Serial.println(mag, DEC);

      /* Wait the specified delay before requesting nex data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
         
      /* Optional: Display calibration status */
      //displayCalStatus();
    
      /* Optional: Display sensor status (debug only) */
      //displaySensorStatus();

      /* New line for the next sample */
    //  Serial.println("");
    
    }
  }

    /********** END OF IMU *************/
    
/*
  //Now the SD card
  delay(1000);
  Serial.print("Looking for an SD card. Standby...");
  if (!card.init(SPI_HALF_SPEED, SDchipSelect)) {
    Serial.println();
    Serial.println("Looks like there's no card here! Trying checking the following:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your soldering correct?");
    Serial.println("* is the chipselect pin correct?");
    badTone();
    Serial.println();
    delay(2000);
    Serial.println();
    Serial.println("We'll continue with the startup without the SD card now :(");
    Serial.println();
    delay(2000);
  } else {
    Serial.println("Found an SD card! Here's some information about it.");
    goodTone();
    delay(1000);
    Serial.println();
    Serial.print("Card type:         ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    }
    if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());
  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);
  Serial.println();
  Serial.println();
  delay(1000);
  }

  /********** END OF SD CARD *********/
  

  //************** TEST PYRO CHANNEL ****************/
  Serial.println();
  Serial.println("To finish up here, let's test the pyro channel");
  Serial.println();
  pinMode(PYRO, OUTPUT);
  digitalWrite(PYRO, LOW);
  delay(1000);
/*
          //The following code MUST BE REMOVED before you connect anything to the pyro channels
          Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
          delay(1000);
          digitalWrite(PYRO, HIGH);
          Serial.println("Pyro 1 is on!");
          warningTone();
          delay(2000);
          digitalWrite(PYRO, LOW);
          Serial.println("Pyro 1 is off");
          delay(2000);
          ///////////////////////////////////////////////////////////////////////////////////// */
  
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(1000);

  /***************** END OF PYRO TEST *******************/

  /****************** ESC ARMING SEQUENCE *********************/
  Serial.println("Time to arm the ESCs");
  Serial.println("Sending lowest throttle for arming...");  
  esc1.writeMicroseconds(1000); // Adjust this value if needed; 2000us usually indicates maximum throttle
  esc2.writeMicroseconds(1000); // Adjust this value if needed; 2000us usually indicates maximum throttle
  delay(2000); // Wait a bit longer for the ESC to recognize the arming sequence
  Serial.println("ESC should be armed now.");
  delay(1000);

  /****************** END OF ARMING SEQUENCE *********************/
  
  Serial.println("SAIL Diagnostics Complete!");
  sendMessage("SAIL rdy to go");
  delay(500);
  tone(Buzzer, 2000); delay(50); noTone(Buzzer); delay(75);
  tone(Buzzer, 2000); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 1000); delay(50); noTone(Buzzer); delay(75);
  tone(Buzzer, 1000); delay(50); noTone(Buzzer); delay(400);

  tone(Buzzer, 1319); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 1760); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2217); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2637); delay(100); noTone(Buzzer);
  delay(100);
  tone(Buzzer, 2217); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2637); delay(200); noTone(Buzzer);
  delay(400);
  Serial.println("Charge!");
  Serial.println();
  int slide = 5000;
  for (int i = 0; i <= 120; i++) {
    tone(Buzzer, slide); delay(10); noTone(Buzzer);
    slide = slide - 40;
  }
  noTone(Buzzer);
  
}

void loop() {

  unsigned long signalStartTime = millis(); // Store the start time of waiting for signal
  
  // Flush the receive buffer to discard any previous messages
  rf95.recv(nullptr, nullptr);  // NEW

  // Wait for a signal with timeout (NEW)
  while (!rf95.available()) {
    if (millis() - signalStartTime > SIGNAL_TIMEOUT) { // Timeout
      Serial.println("Timeout waiting for signal from Huntsville");
      return;
    }
    //Serial.println("Awaiting signal from Huntsville...");
  }
  
  // Signal received
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.recv(buf, &len)) {
  buf[len] = '\0'; // Null-terminate the received buffer
  Serial.print("SAIL received signal: ");
  Serial.println((char*)buf);
  Serial.print("RSSI: ");
  Serial.println(rf95.lastRssi(), DEC);
  Serial.print("Received message length: ");
  Serial.println(len);

  
      
  // Check the received signal
  if (strcmp((char*)buf, "Go") == 0) {
    Serial.println("Go signal received.");
    // Handle Go signal
    delay(1000);
    Serial.println("Firing landing legs at 350 ft AGL.");
    sendMessage("SAIL is GO, legs at 350 ft");
    fireLandingLegs(); // Check altitude for pyro trigger
    float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
    float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
          

      if (altitudeAGL < 350) {
      // Start reporting AGL altitude every 0.5 seconds
      unsigned long lastAltitudeReportTime = millis();
      while (true) {
        if (millis() - lastAltitudeReportTime >= 500) {
          lastAltitudeReportTime = millis();
          float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
          float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
          Serial.print("Altitude: ");
          Serial.print(altitudeAGL);
          Serial.println(" ft AGL");
          String message = "SAIL AGL: ";
          message += String(altitudeAGL, 2); // Convert altitude to string with 2 decimal points precision
          sendMessage(message);
        }
      }
      }

    } else if (strcmp((char*)buf, "Check") == 0) {        
    Serial.println("Check signal received.");
    // Handle Check signal
    delay(100);
    float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
    float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
    Serial.print("Altitude: ");
    Serial.print(altitudeAGL);
    Serial.println(" ft AGL");
    String message = "SAIL @ ";
    message += String(altitudeAGL, 2); // Convert altitude to string with 2 decimal points precision
    message += " ft AGL";
    sendMessage(message);  
      
    } else if (strcmp((char*)buf, "Legs open") == 0) {
      Serial.println("Open Legs signal received.");
      delay(1000);
      sendMessage("Open Legs recv");
      digitalWrite(PYRO, HIGH);
      Serial.println("Force Open, pyro firing.");
      delay(10000);
      digitalWrite(PYRO, LOW);
      Serial.println("End of Force Open, pyro off.");   
              
    } else if (strcmp((char*)buf, "Hello SAIL") == 0) {
      sendMessage("SAIL awaiting signal");
    }
  } else {
    Serial.println("Receive failed");
  }
}

