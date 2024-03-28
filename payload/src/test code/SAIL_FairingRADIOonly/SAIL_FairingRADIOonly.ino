// Data logging not added (low priority)
// Require message from Ground Station to proceed? Currently starts regardless
// Longer messages not tested
// 4S LiPo connected to MOSFET. 9V doesn't discharge enough

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

#define RF95_FREQ 915.0

#define SIGNAL_TIMEOUT 5000 // Timeout value in milliseconds (2 seconds) NEW

unsigned long lastAltitudeCheckTime = 0; // Variable to store the last time altitude was checked

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

void setup() {

  Serial.begin(9600);
  delay(5000);

  Serial.println("Fairing Controller Startup!");
  delay(2000);
  
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
    Serial.print("Fairing received signal: ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(rf95.lastRssi(), DEC);
    Serial.print("Received message length: ");
    Serial.println(len);
    
    // Check the received signal
    if (strcmp((char*)buf, "Go") == 0) {
      Serial.println("Go signal received.");
      // Handle Go signal
      delay(500);
      sendMessage("Go signal received.");
      
    } else if (strcmp((char*)buf, "Check") == 0) {        
      Serial.println("Check signal received.");
      // Handle Check signal
      delay(500);
      sendMessage("Fairing checking in.");
      
    } else if (strcmp((char*)buf, "Force Open") == 0) {
      Serial.println("Force Open signal received.");
      // Handle Force Open signal
      delay(500);
      sendMessage("Force Open received, releasing Fairing.");
      Serial.println("Force Open, pyro firing for 5s.");
      delay(5000);
      Serial.println("End of Force Open, pyro off.");     
      
    } else if (strcmp((char*)buf, "Hello Fairing.") == 0) {
      // Reply back "And hello to you, Huntsville"
      delay(500);
      sendMessage("And hello to you, Huntsville. This is Fairing awaiting your signal.");
    }
  }


}
