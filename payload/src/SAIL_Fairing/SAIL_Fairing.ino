// Data logging not added (low priority)
// ADD FIREBELOW400 BEFORE SENDING FEEDBACK TO GUARANTEE?
// Longer messages not tested
// 4S LiPo connected to MOSFET
// Add SD logging?

#define SEALEVELPRESSURE_INHG 30.01   // Sea level pressure in inches of mercury, adjust as per your location
#define GROUND_LEVEL_ELEVATION_FEET 889 // Ground level elevation in feet, adjust as per your location

#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_BMP3XX.h"

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

#define RF95_FREQ 915.0

#define SIGNAL_TIMEOUT 5000 // Timeout value in milliseconds (2 seconds) NEW

unsigned long lastAltitudeCheckTime = 0; // Variable to store the last time altitude was checked

//This is for the pyro channel
const int PYRO = 10;

//Set up the LEDs
int R_LED = 5;
int G_LED = 6;
int B_LED = 9;

//This is for the BMP390 barometer
Adafruit_BMP3XX bmp;


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

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

void fireBelow400() {   // Need to be able to receive force-open signal
  Serial.println("Fairing Go: Waiting <400ft");
  while (true) {
    if (millis() - lastAltitudeCheckTime >= 200) { // Check altitude every 200 milliseconds
      lastAltitudeCheckTime = millis(); // Update lastAltitudeCheckTime

      float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
      float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
      Serial.print("Altitude: ");
      Serial.print(altitudeAGL);
      Serial.println(" ft AGL");

      if (altitudeAGL < 400.0) {
        digitalWrite(PYRO, HIGH);
        sendMessage("F: <400ft, open Fairing");
        Serial.println("Below 400ft, Activating pyro wire.");
        delay(5000);
        digitalWrite(PYRO, LOW);
        Serial.println("Pyro wire deactivated.");
        break; // Exit the loop once pyro wire is deactivated
      }
    }
  }
}

void setup() {

  Serial.begin(9600);
  delay(3000);
  
  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

  Serial.println("Fairing Controller Startup!");
  delay(2000);
  
  pinMode(PYRO, OUTPUT);
  
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(1000); 

 /********** END OF PYRO TEST *****************/

  // Barometer Check
  Serial.println("Let's see if the BMP390 Barometer is connected. Standby...");
  delay(1000);
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find the BMP390 sensor :( Check your soldering and inspect for bad connections");
    delay(2000);
    Serial.println();
    Serial.println("Proceeding with startup");
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

  Serial.println("Continuing with Fairing Startup!");
  delay(1000);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.println("Radio Check!");
  delay(1000);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  Serial.println("Fairing setup complete");
  sendMessage("F: Fairing rdy to go");

   rf95.setTxPower(23, false);
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
  }
  
  // Signal received
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.recv(buf, &len)) {
    buf[len] = '\0'; // Null-terminate the received buffer
    Serial.print("Fairing received signal: ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(rf95.lastRssi(), DEC);
    Serial.print("Received message length: ");
    Serial.println(len);
    
    // Check the received signal
    if (strcmp((char*)buf, "Go") == 0) {
      Serial.println("Go signal received.");
      delay(1000);
      sendMessage("F: Go recv. Firing <400ft AGL.");
      fireBelow400();
      
    } else if (strcmp((char*)buf, "Check") == 0) {        
      Serial.println("Check signal received.");
      delay(1000);
      float altitudeMSL = 3.28084 * (bmp.readAltitude(SEALEVELPRESSURE_INHG * 33.86389)); // Convert sea level pressure to hPa, then convert value in m to ft
      float altitudeAGL = (altitudeMSL) - GROUND_LEVEL_ELEVATION_FEET; // Convert altitude to feet and subtract ground level elevation
      Serial.print("Altitude: ");
      Serial.print(altitudeAGL);
      Serial.println(" ft AGL");
      String message = "F: Fairing @ ";
      message += String(altitudeAGL, 2); // Convert altitude to string with 2 decimal points precision
      message += " ft AGL";
      sendMessage(message);  
      
    }  else if (strcmp((char*)buf, "Force Open") == 0) {
      Serial.println("Force Open signal received.");
      Serial.println("Force Open, pyro firing for 5s.");
      delay(500);
      sendMessage("F: Force Open recieved.");
      digitalWrite(PYRO, HIGH);
      delay(5000);
      digitalWrite(PYRO, LOW);
      Serial.println("Pyro wire deactivated.");
      
      delay(5000);
      Serial.println("End of Force Open, pyro off.");     
      
    } else if (strcmp((char*)buf, "Hello Fairing.") == 0) {
      delay(500);
      sendMessage("F: Fairing awaiting signal.");
    }
  }
}
