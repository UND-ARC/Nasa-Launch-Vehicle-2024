// Data logging not added (low priority)
// Require message from Ground Station to proceed? Currently starts regardless
// Longer messages not tested
// 4S LiPo connected to MOSFET. 9V doesn't discharged enough

#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_BMP3XX.h"

// ENTER CURRENT METAR ALTIMETER
double inHg = 29.92; 

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

#define RF95_FREQ 915.0

#define SIGNAL_TIMEOUT 5000 // Timeout value in milliseconds (2 seconds) NEW

unsigned long lastAltitudeCheckTime = 0; // Variable to store the last time altitude was checked

//This is for the pyro channel
int PYRO = 10;

//Set up the LEDs
int R_LED = 5;
int G_LED = 6;
int B_LED = 9;

//This is for the BMP390 barometer
Adafruit_BMP3XX bmp;
double hPa = inHg * 33.8639;
#define SEALEVELPRESSURE_HPA (hPa) // default: 1013.25

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

unsigned long previousMillis = 0;
unsigned long currentMillis;

void nonBlockingDelay(unsigned long interval) {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // If the specified interval has passed, update previousMillis
    previousMillis = currentMillis;
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

void awaitSignal(int flashDuration) {
  unsigned long signalStartTime = millis(); // Store the start time of waiting for signal
  
  // Flush the receive buffer to discard any previous messages
  rf95.recv(nullptr, nullptr);  // NEW

  // Wait for a signal with timeout (NEW)
  while (!rf95.available()) {
    if (millis() - signalStartTime > SIGNAL_TIMEOUT) { // Timeout
      Serial.println("Timeout waiting for signal");
      return;
    }
    Serial.println("Awaiting signal from Huntsville...");
  }
  
  // Signal received
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.recv(buf, &len)) {
    Serial.print("Received signal: ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(rf95.lastRssi(), DEC);
    Serial.print("Received message length: ");
    Serial.println(len);
    
    // Check the received signal
    if (strcmp((char*)buf, "Go") == 0) {
      Serial.println("Go signal received.");
      // Handle Go signal
      sendMessage("Go signal received, Fairing firing at 400ft AGL.");
      FireBelow400(); // Check altitude for pyro trigger
      
    } else if (strcmp((char*)buf, "Check") == 0) {        
      Serial.println("Check signal received.");
      // Handle Check signal
      delay(1000); // delay to not interfere with 
      sendMessage("Fairing checking in.");
      
    } else if (strcmp((char*)buf, "Force Open") == 0) {
      Serial.println("Force Open signal received.");
      // Handle Force Open signal
      sendMessage("Force Open received, Fairing releasing payload.");
      digitalWrite(13, HIGH);
      Serial.println("Force Open, pyro firing for 5s.");
      delay(5000);
      digitalWrite(13, LOW);
      Serial.println("End of Force Open, pyro off.");     
      
    } else if (strcmp((char*)buf, "Hello Fairing") == 0) {
      // Reply back "And hello to you, Huntsville"
      sendMessage("And hello to you, Huntsville. This is Fairing awaiting your signal");
    }
  }
}

void FireBelow400() {   // Need to be able to receive force-open signal
  Serial.println("Go signal received: Waiting for altitude to drop below 400ft...");
  while (true) {
    if (millis() - lastAltitudeCheckTime >= 200) { // Check altitude every 200 milliseconds
      lastAltitudeCheckTime = millis(); // Update lastAltitudeCheckTime
      float altitudeMeters = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Read altitude from BMP390 sensor in meters
      float altitudeFeet = altitudeMeters * 3.281; // Convert altitude to feet

      Serial.print("Altitude: ");
      Serial.print(altitudeFeet);
      Serial.println(" ft");

      if (altitudeFeet < 400.0) {
        // Altitude is below 400ft
        // Activate pyro charge
        digitalWrite(13, HIGH);
        Serial.println("Below 400ft, Activating pyro wire.");
        sendMessage("Fairing below 400ft, releasing payload.");
        delay(5000);
        digitalWrite(13, LOW);
        Serial.println("Pyro wire deactivated.");
        break; // Exit the loop once pyro wire is deactivated
      }
    }
  }
}

void setup() {

  Serial.begin(9600);
  delay(5000);
  
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
  
  /********** PYRO TEST *****************/
  Serial.println();
  Serial.println("Let's test the pyro channel if it's not commented out");
  Serial.println();
  pinMode(PYRO, OUTPUT);
  digitalWrite(PYRO, LOW);
  delay(2000);
  
 //////////////////////////////////////////////////////////////////////////////////////////
  //The following code MUST BE REMOVED before you connect anything to the pyro channels
  Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
  delay(2000);
  digitalWrite(PYRO, HIGH);
  Serial.println("PYRO is on!");

  digitalWrite(PYRO, LOW); // Set pyro pin low
  Serial.println("Pyro is off");
  delay(5000); // Wait for 5 seconds
  ////////////////////////////////////////////////////////////////////////////////////////

          
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(1000);

 /********** END OF PYRO TEST *****************/

  // Barometer Check
  Serial.println("Let's see if the BMP390 Barometer is connected. Standby...");
  delay(3000);
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find the BMP390 sensor :( Check your soldering and inspect for bad connections");
    delay(3000);
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
    Serial.print(F("Temperature = "));
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m"); Serial.println();
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
  sendMessage("This is Fairing");
}

void loop() {

  awaitSignal(2000);
  delay(200); // need???

}
