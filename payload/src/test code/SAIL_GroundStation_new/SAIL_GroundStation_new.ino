// Feather m0 RFM95
// Comments:
// Need to add LED feedback for idle after setup and receiving from Fairing and SAIL
// Add initial message to Fairing, wait for Fairing and Sail to respond before continuing?

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_SSD1306.h> // Include the SSD1306 library
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialize the OLED display object

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

const int BUTTON_GO_PIN          = 10;  // Pin for "Go" button
const int BUTTON_FORCE_OPEN_PIN = 11;  // Pin for "Force Fairing Open" button
const int BUTTON_BEGIN_DESCENT_PIN = 9;  // Pin for "Begin Controlled Descent" button
const int ARM_BUTTON_PIN    = 6;   // Pin for the button to activate states
const int CHECK_PIN         = 12;
const int ABORT_PIN         = 13;  // NEEDS TO BE CHANGED

//define system led pins
int R_LED = 9;
int G_LED = 3;
int B_LED = 6;
//int Buzzer = 4;

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool activationState = false; // Variable to store the activation state
unsigned long lastDebounceTime = 0;  // the last time the button input pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

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
  if (rf95.send((uint8_t *)message.c_str(), message.length())) {
    rf95.waitPacketSent();
    Serial.println("Message sent successfully: " + message);
    return true; // Message sent successfully 
  } else {
    Serial.println("Message failed: " + message);
    return false; // Message not sent due to activation state
  }
}

// Function to display a message on the OLED display
void displayMessage(const char* message) {
  display.clearDisplay(); // Clear the display
  display.setTextSize(1); // Set text size
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setCursor(0, 0); // Set cursor position
  display.println(message); // Print message
  display.display(); // Display the message on the OLED
}

// Define constants for row positions
#define SENT_MESSAGE_ROW 10
#define RECEIVED_MESSAGE_ROW 30

void setup() {
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(BUTTON_GO_PIN, INPUT);
  pinMode(BUTTON_FORCE_OPEN_PIN, INPUT);
  pinMode(BUTTON_BEGIN_DESCENT_PIN, INPUT);
  pinMode(ARM_BUTTON_PIN, INPUT);
  pinMode(CHECK_PIN, INPUT);

  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

  Serial.begin(115200); // Adafruit example has 115200
  while (!Serial) delay(1);
  delay(100);
  
  Serial.println("Feather LoRa Ground Station Startup!");
  displayMessage("Ground Station Startup!");
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
  displayMessage("LoRa radio init OK!");
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
  displayMessage("Sent: Hello SAIL");
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
      displayMessage((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is SAIL around?");  // NTS: Do not want this to loop
  }

  // Send a message to Fairing
  sendMessage("Hello Fairing");
  displayMessage("Sent: Hello Fairing");
  delay(500);
  

  // After sending "Hello Fairing" message
  Serial.println("Waiting for reply from Fairing...");
  if (rf95.waitAvailableTimeout(1000)) {
      // If a reply is received from Fairing
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
          Serial.print("Received reply from Fairing: ");
          Serial.println((char*)buf);
          displayMessage((char*)buf);
      } else {
          Serial.println("Receive from Fairing failed");
      }
  } else {
      Serial.println("No reply received from Fairing");
  }
  delay(500);

  Serial.println("Setup Complete");
  displayMessage("Setup Complete");
  digitalWrite(G_LED, LOW);
  nonBlockingDelay(2000);
  digitalWrite(G_LED, HIGH);
  }
}

void loop() {
  
  // Read the state of each button
  int armButtonState = digitalRead(ARM_BUTTON_PIN);
  int goButtonState = digitalRead(BUTTON_GO_PIN);
  int forceOpenButtonState = digitalRead(BUTTON_FORCE_OPEN_PIN);
  int beginDescentButtonState = digitalRead(BUTTON_BEGIN_DESCENT_PIN);
  int checkButtonState = digitalRead(CHECK_PIN);
  int abortButtonState = digitalRead (ABORT_PIN);

  // Check if ARM button is pressed and update activationState accordingly
  if (armButtonState == LOW) {
    activationState = true;  // ARM button is pressed, set activationState to true
  } else {
    activationState = false; // ARM button is not pressed, set activationState to false
  }

  delay(100); // Wait 1 second between transmits, could also 'sleep' here!
  
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      displayMessage((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
  }
  // Check if activation state is true (arm button is pressed)
  if (activationState) {
    // If activation button is pressed, send the corresponding command
      Serial.println("ARM button pressed!");
      displayMessage("ARM button pressed!");
      if (goButtonState == LOW) {
        sendMessage("Go");
        displayMessage("Sent: Go");
        delay(1000); // Debounce delay
      } else if (forceOpenButtonState == LOW) {
        sendMessage("Force Fairing Open");
        displayMessage("Sent: Force Fairing Open");
        delay(1000); // Debounce delay
      } else if (beginDescentButtonState == LOW) {
        sendMessage("Begin Controlled Descent");
        displayMessage("Sent: Begin Controlled Descent");
        delay(1000); // Debounce delay
      } else if (checkButtonState == LOW) {
        sendMessage("Check");
        displayMessage("Sent: Check");
        delay(1000);
      }
  }
  else {
    // If arm button is not pressed, print the message for any command button press
    if (goButtonState == LOW || forceOpenButtonState == LOW || beginDescentButtonState == LOW || checkButtonState == LOW || abortButtonState == LOW) {
      Serial.println("Activation button not pressed!");
      displayMessage("Activation button not pressed!");
    }
  }
}
