// Feather m0 RFM95
// Comments:
// Need to add LED feedback for idle after setup and receiving from Fairing and SAIL
// Add initial message to Fairing, wait for Fairing and Sail to respond before continuing?

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

const int BUTTON_GO_PIN          = 10;  // Pin for "Go" button
const int BUTTON_NOGO_PIN        = 11;  // Pin for "No-Go" button
const int BUTTON_FORCE_OPEN_PIN = 12;  // Pin for "Force Fairing Open" button
const int BUTTON_BEGIN_DESCENT_PIN = 9;  // Pin for "Begin Controlled Descent" button
const int ARM_BUTTON_PIN    = 6;   // Pin for the button to activate states

//define system led pins
int R_LED = 9;
int G_LED = 3;
int B_LED = 6;
int Buzzer = 4;

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool activationState = false; // Variable to store the activation state
unsigned long lastDebounceTime = 0;  // the last time the button input pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

bool sendMessage(String message) {
  if (rf95.send((uint8_t *)message.c_str(), message.length())) {
    rf95.waitPacketSent();
    Serial.println("Message sent: " + message);
    return true; // Message sent successfully 
  } else {
    Serial.println("Message failed: " + message);
    return false; // Message not sent due to activation state
  }
}

// Function to blink the RGB LED with a specified color and duration
void blinkRGB(int red, int green, int blue, int duration) {
  for (int i = 0; i < 5; i++) { // Blink 5 times
    digitalWrite(R_LED, HIGH);
    digitalWrite(G_LED, HIGH);
    digitalWrite(B_LED, HIGH);
    delay(duration / 2);
    digitalWrite(R_LED, red == 255 ? LOW : HIGH); // If the color is 255, turn off the LED
    digitalWrite(G_LED, green == 255 ? LOW : HIGH);
    digitalWrite(B_LED, blue == 255 ? LOW : HIGH);
    delay(duration / 2);
  }
}

void setup() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(BUTTON_GO_PIN, INPUT);
  pinMode(BUTTON_NOGO_PIN, INPUT);
  pinMode(BUTTON_FORCE_OPEN_PIN, INPUT);
  pinMode(BUTTON_BEGIN_DESCENT_PIN, INPUT);
  pinMode(ARM_BUTTON_PIN, INPUT);

  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

  Serial.begin(9600); // Adafruit example has 115200
  while (!Serial) delay(1);
  delay(100);
  
  Serial.println("Feather LoRa Ground Station Startup!");
  delay(1000);

  // manual reset
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

  int16_t packetnum = 0;  // packet counter, we increment per xmission

  // Send a message to SAIL
  sendMessage("Hello SAIL");
  delay(500);

  // Send a message to Fairing
  sendMessage("Hello Fairing");
  delay(500);
  
  // After sending "Hello SAIL" message
  Serial.println("Waiting for SAIL to reply...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is SAIL around?");  // NTS: Do not want this to loop
  }
  
  // After sending "Hello Fairing" message
  Serial.println("Waiting for reply from Fairing...");
  if (rf95.waitAvailableTimeout(1000)) {
      // If a reply is received from Fairing
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
  
  // Read the state of each button
  int armButtonState = digitalRead(ARM_BUTTON_PIN);
  int goButtonState = digitalRead(BUTTON_GO_PIN);
  int noGoButtonState = digitalRead(BUTTON_NOGO_PIN);
  int forceOpenButtonState = digitalRead(BUTTON_FORCE_OPEN_PIN);
  int beginDescentButtonState = digitalRead(BUTTON_BEGIN_DESCENT_PIN);

  // Check if ARM button is pressed and update activationState accordingly
  if (armButtonState == LOW) {
    activationState = true;  // ARM button is pressed, set activationState to true
  } else {
    activationState = false; // ARM button is not pressed, set activationState to false
  }

  delay(200); // Wait 1 second between transmits, could also 'sleep' here!
  
  // Check if activation state is true (arm button is pressed)
  if (activationState) {
    // If activation button is pressed, send the corresponding command
    if (armButtonState == LOW) {
      Serial.println("ARM button pressed!");
      if (goButtonState == LOW) {
        sendMessage("Go");
        digitalWrite(G_LED, LOW);
        delay(2000);
        digitalWrite(G_LED, HIGH);
        delay(1000); // Debounce delay
      } else if (noGoButtonState == LOW) {
        sendMessage("No-Go");
        digitalWrite(R_LED, LOW);
        delay(2000);
        digitalWrite(R_LED, HIGH);
        delay(1000); // Debounce delay
      } else if (forceOpenButtonState == LOW) {
        sendMessage("Force Fairing Open");
        digitalWrite(B_LED, LOW);
        delay(500);
        digitalWrite(B_LED, HIGH);
        delay(500);
        digitalWrite(B_LED, LOW);
        delay(500);
        digitalWrite(B_LED, HIGH);
        delay(1000); // Debounce delay
      } else if (beginDescentButtonState == LOW) {
        sendMessage("Begin Controlled Descent");
        digitalWrite(G_LED, LOW);
        delay(500);
        digitalWrite(G_LED, HIGH);
        delay(500);
        digitalWrite(G_LED, LOW);
        delay(500);
        digitalWrite(G_LED, HIGH);
        delay(1000); // Debounce delay
      }
  } else {
    // If activation button is not pressed, print the message
    Serial.println("Activation button not pressed!");
  }
  } else {
  // If arm button is not pressed, print the message for any command button press
  if (goButtonState == LOW || noGoButtonState == LOW || forceOpenButtonState == LOW || beginDescentButtonState == LOW) {
    Serial.println("Activation button not pressed!");
  }
}
}
