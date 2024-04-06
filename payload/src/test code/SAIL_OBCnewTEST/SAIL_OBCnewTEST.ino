// Notes
// MAY NEED TO DISABLE ARMING CODE FOR RC TO MANUALLY CONTROL
// COMMAND FOR GPS LOCATION ON COMMAND, LOGGING, AND LAT/LONG on Check
// Let's also log GPS altitude

// SAIL Flight Computer Startup Diagnostics
// Note1: Comment out pyro channel test before connecting anything to pyro channel
// Define altimeter rating for current conditions

#include "SAIL.h"

void setup() {
  delay(1000);

  GPS.begin(9600);

  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  esc1.attach(22);
  esc2.attach(23);
  
  Serial.println();Serial.println();
  Serial.println("SAIL OBC Startup!");
  startupMelody();
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

  pinMode(PYRO, OUTPUT);


  /* GPS TEST */

  Serial.println("GPS Parsing Test");
  unsigned long gpsTestStartTime = millis();
  // Run the loop for 5 seconds
  while (millis() - gpsTestStartTime < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      GPSSerial.write(c);
    }
    if (GPSSerial.available()) {
      char c = GPSSerial.read();
      Serial.write(c);
    }
  }

  /* Now we'll test the radio */
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

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    startTime = millis();
  }

  while (!rf95.init() && millis() - startTime < timeoutDuration) {
    // Do nothing, just wait
  }

  if (!rf95.init()) {
    Serial.println("Initialization failed after timeout");
    // Handle the failure here, e.g., retry initialization, reset the system, etc.
  }
  else {
    Serial.println("LoRa radio init OK!");
    goodTone();
  }

  delay(1000);
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    badTone();
  }
  
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  delay(1000);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  
  rf95.setTxPower(23, false);

  /* END OF RADIO TEST */

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

  for (int i = 0; i <= 10; i++) {
    Serial.println();
    float temperatureF = bmp.temperature * 1.8 + 32; // Convert Celsius to Fahrenheit
    Serial.print(F("Temperature = "));
    Serial.print(temperatureF);
    Serial.println(" °F");

    Serial.print("Current Altimeter Rating = ");
    Serial.print(SEALEVELPRESSURE_INHG);
    Serial.println(" inHg");

    float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); 
    Serial.print("Approx. Altitude (MSL) = ");
    Serial.print(altitudeMSL);
    Serial.println(" ft");

    float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET;
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
  Serial.println();
  delay(1000);
  if(bno.begin())
  {
    Serial.println("Here's a little bit of IMU data!");
    goodTone();
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
      
      // Display the floating point data 
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      // Display calibration status for each sensor.
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

  /* END OF IMU */

  /* SD CARD */
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


  // Create a file for logging
  imuLogFile = SD.open("IMU_Log.txt", FILE_WRITE);
  if (!imuLogFile) {
    Serial.println("Error opening IMU log file!");
   // return;
  }

    // Write headers to the log file
  imuLogFile.println("Timestamp,Z Accel (m/s^2),Rotation Rate (rad/s),Z Velocity (m/s)");

  /********** END OF SD CARD *********/
  

  //************** TEST PYRO CHANNEL ****************/
  Serial.println();
  Serial.println("To finish up here, let's test the pyro channel");
  Serial.println();
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
  delay(1000);

  /* END OF PYRO TEST */


  /* ESC ARMING SEQUENCE */ /*
  Serial.println("Time to arm the ESCs");
  Serial.println("Sending lowest throttle for arming...");  
  esc1.writeMicroseconds(1000); // Adjust this value if needed; 2000us usually indicates maximum throttle
  esc2.writeMicroseconds(1000); // Adjust this value if needed; 2000us usually indicates maximum throttle
  delay(2000); // Wait a bit longer for the ESC to recognize the arming sequence
  Serial.println("ESC should be armed now.");
  delay(1000);

  /* END OF ARMING SEQUENCE */
  
  Serial.println();
  Serial.println();
  Serial.println("SAIL Diagnostics Complete!");
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
  sendMessage("S: SAIL rdy to go");
}

void loop() {

  float latitude = GPS.latitudeDegrees;
  float longitude = GPS.longitudeDegrees;
  float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
  float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation 
 
 // Read IMU data
  float zAccel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z();
  float rotationRate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();
  float zVelocity = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL).z();

  // Get current timestamp
  unsigned long currentMillis = millis();
  unsigned long signalStartTime = millis(); // Store the start time of waiting for signal

  // Flush the receive buffer to discard any previous messages
  rf95.recv(nullptr, nullptr);

  // Wait for a signal with timeout
  while (!rf95.available()) {
    if (millis() - signalStartTime > SIGNAL_TIMEOUT) { 
      Serial.println("Timeout waiting for signal from Huntsville");
      return;
    }
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
    sendMessage("S: SAIL is GO, wait for 350 ft");
    fireLandingLegs(); // Check altitude for pyro trigger

    if (altitudeAGL < 350) {
    // Start reporting AGL altitude every 0.5 seconds
    unsigned long lastAltitudeReportTime = millis();
    while (true) {
      if (millis() - lastAltitudeReportTime >= 500) {
        lastAltitudeReportTime = millis();
        Serial.print("Altitude: ");
        Serial.print(altitudeAGL);
        Serial.println(" ft AGL");
        String message = "S: SAIL AGL: ";
        message += String(altitudeAGL, 2); // Convert altitude to string with 2 decimal points precision
        sendMessage(message);
      }
    }
  }

    } else if (strcmp((char*)buf, "Check") == 0) {        
    Serial.println("Check signal received.");
    // Handle Check signal
    delay(1000);
    float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
    float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
    Serial.print("Altitude: ");
    Serial.print(altitudeAGL);
    Serial.println(" ft AGL");
    String message = "S: SAIL @ ";
    message += String(altitudeAGL, 2); // Convert altitude to string with 2 decimal points precision
    message += " ft AGL";
    /*
    message += " | Lat: ";
    message += String(latitude, 6); // Convert latitude to string with 6 decimal points precision
    message += " | Long: ";
    message += String(longitude, 6); // Convert longitude to string with 6 decimal points precision
    */
    sendMessage(message);  
      
    } else if (strcmp((char*)buf, "Legs open") == 0) {
      Serial.println("Open Legs signal received.");
      delay(1000);
      sendMessage("S: Releasing legs");
      digitalWrite(PYRO, HIGH);
      Serial.println("Force Open, pyro firing.");
      delay(10000);
      digitalWrite(PYRO, LOW);
      Serial.println("End of Force Open, pyro off.");   
              
    } else if (strcmp((char*)buf, "Hello SAIL") == 0) {
      sendMessage("S: SAIL awaiting signal");
    }
  } else {
    Serial.println("Receive failed");
  }

   // Check for liftoff
  if (!liftoffDetected && zAccel >= liftoffAccelerationThreshold) {
    liftoffDetected = true;
    liftoffStartTime = currentMillis;
    Serial.println("Acceleration detected! Waiting for boost confirmation...");
  }

    // Check for boost confirmation
  if (!boostConfirmed && liftoffDetected && zAccel >= boostAccelerationThreshold && altitudeAGL >= 20) {
    boostConfirmed = true;
    Serial.println("Boost stage confirmed! Logging started.");
  }

   // Check if it's time to log data
  if (boostConfirmed && currentMillis - lastLogTime >= logInterval) {
    // Write timestamp and IMU data to the log file
    imuLogFile.print(currentMillis);
    imuLogFile.print(",");
    imuLogFile.print(zAccel);
    imuLogFile.print(",");
    imuLogFile.print(rotationRate);
    imuLogFile.print(",");
    imuLogFile.println(zVelocity);

    // Flush data to the SD card
    imuLogFile.flush();

    // Update last log time
    lastLogTime = currentMillis;
  }

  if(!landed && boostConfirmed && altitudeAGL < 15) {
    landed = true;
    landingTime = millis();
    sendMessage("S: SAIL has landed");
    Serial.println("SAIL has landed. Reporting landing and will start pinging location.");
  }

  if(landed && millis() - landingTime >= 15000) {
    String message = "S: Lat: ";
    message += String(latitude, 6); // Convert latitude to string with 6 decimal points precision
    message += " Long: ";
    message += String(longitude, 6); // Convert longitude to string with 6 decimal points precision
    sendMessage(message);

    // Reset timer for next ping
    landingTime = millis();
  }
}