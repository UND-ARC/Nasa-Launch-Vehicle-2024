#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

const int BUTTON_GO_PIN          = 10;  // Pin for "Go" button
const int BUTTON_FORCE_OPEN_PIN = 11;  // Pin for "Force Fairing Open" button
const int BUTTON_BEGIN_DESCENT_PIN = 9;  // Pin for "Begin Controlled Descent" button
const int ARM_BUTTON_PIN    = 6;   // Pin for the button to activate states
const int CHECK_PIN         = 12;
const int ABORT_PIN

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool activationState = false; // Variable to store the activation state

// Function to send a message via LoRa
bool sendMessage(String message) {
  unsigned long currentMillis = millis();

  if (rf95.send((uint8_t *)message.c_str(), message.length())) {
    rf95.waitPacketSent();
    Serial.println("Message sent successfully: " + message);
    return true; // Message sent successfully 
  } else {
    Serial.println("Message failed: " + message);
    return false; // Message not sent due to activation state
  }
}

void setup() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(BUTTON_GO_PIN, INPUT);
  pinMode(BUTTON_FORCE_OPEN_PIN, INPUT);
  pinMode(BUTTON_BEGIN_DESCENT_PIN, INPUT);
  pinMode(ARM_BUTTON_PIN, INPUT);
  pinMode(CHECK_PIN, INPUT);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);
  
  Serial.println("Feather LoRa Ground Station Startup!");
  delay(1000);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  delay(1000);

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  delay(1000);

  rf95.setTxPower(23, false);

  int16_t packetnum = 0;

  sendMessage("Hello SAIL.");
  delay(500);

  Serial.println("Waiting for SAIL to reply...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is SAIL around?");
  }

  sendMessage("Hello Fairing.");
  delay(500);
  
  Serial.println("Waiting for reply from Fairing...");
  if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
          Serial.print("Received reply from Fairing: ");
          Serial.println((char*)buf);
      } else {
          Serial.println("Receive from Fairing failed");
      }
  } else {
      Serial.println("No reply received from Fairing");
  }
  delay(500);

  Serial.println("Setup Complete");
}

void loop() {
  
  int armButtonState = digitalRead(ARM_BUTTON_PIN);
  int goButtonState = digitalRead(BUTTON_GO_PIN);
  int forceOpenButtonState = digitalRead(BUTTON_FORCE_OPEN_PIN);
  int beginDescentButtonState = digitalRead(BUTTON_BEGIN_DESCENT_PIN);
  int checkButtonState = digitalRead(CHECK_PIN);

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
  }

  rf95.recv(nullptr, nullptr);

  if (armButtonState == LOW) {
    activationState = true;
  } else {
    activationState = false;
  }

  delay(100);

  if (activationState) {
    if (armButtonState == LOW) {
      Serial.println("ARM button pressed!");
      if (goButtonState == LOW) {
        sendMessage("Go");
      } else if (forceOpenButtonState == LOW) {
        sendMessage("Force Open");
      } else if (beginDescentButtonState == LOW) {
        sendMessage("Begin Controlled Descent");
      } else if (checkButtonState == LOW) {
        sendMessage("Check");
      }
    } else {
      Serial.println("Activation button not pressed!");
    }
  } else {
    if (goButtonState == LOW || forceOpenButtonState == LOW || beginDescentButtonState == LOW || checkButtonState == LOW) {
      Serial.println("Activation button not pressed!");
      delay(200);
    }
  }
}
