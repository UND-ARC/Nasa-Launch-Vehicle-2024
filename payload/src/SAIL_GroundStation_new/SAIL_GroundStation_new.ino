// ADD sd card logging?

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

#define VBATPIN A7

const int ARM_BUTTON_PIN    = 6;   // Pin for the button to activate states
const int BUTTON_GO_PIN          = 10;  // Pin for "Go" button
const int BUTTON_FORCE_OPEN_PIN = 11;  // Pin for "Force Fairing Open" button
const int BUTTON_BEGIN_DESCENT_PIN = 9;  // Pin for "Begin Controlled Descent" button
const int CHECK_PIN         = 12;
const int LEG_PIN = 5;

unsigned long startTime = 0; // Variable to store the start time

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
    displayMessage(8, "Signal Sent");
    return true; // Message sent successfully 
  } else {
    Serial.println("Message failed: " + message);
    return false; // Message not sent due to activation state
  }
}

// Function to display a message on the OLED display
void displayMessage(int row, const char* message) {
  int16_t textWidth = display.getCursorX(); // Get the width of the text
  
  // If text width exceeds the screen width, enable scrolling
  if (textWidth > SCREEN_WIDTH) {
    for (int16_t i = 0; i < textWidth - SCREEN_WIDTH; i++) {
      display.clearDisplay(); // Clear the display
      display.setCursor(-i, row * 8); // Set cursor position with offset for scrolling
      display.println(message); // Print message
      display.display(); // Display the message on the OLED
      delay(100); // Adjust the delay to control scrolling speed
    }
  } else {
    // Clear only the specified row
    display.fillRect(0, row * 8, SCREEN_WIDTH, 8, SSD1306_BLACK);
    display.setTextSize(1); // Set text size
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0, row * 8); // Set cursor position
    display.println(message); // Print message
    display.display(); // Display the message on the OLED
  }
}

void floatToString(float number, char* buffer, int bufferSize, int decimalDigits) {
  // Convert the integer part to string
  int intPart = (int)number;
  itoa(intPart, buffer, 10);

  // Find the start of the decimal part
  int decIndex = strlen(buffer);

  // Add the decimal point
  buffer[decIndex++] = '.';

  // Extract digits from the float one by one
  for (int i = 0; i < decimalDigits; i++) {
    number -= intPart;                  // Subtract the integer part
    number *= 10;                       // Shift by one decimal place
    intPart = (int)number;              // Extract the integer part
    buffer[decIndex++] = '0' + intPart; // Convert to character and add to buffer
  }

  // Null-terminate the string
  buffer[decIndex] = '\0';
}

void setup() {

  startTime = millis(); // Record the start time

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64
  display.clearDisplay(); // Clear the display

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(BUTTON_GO_PIN, INPUT);
  pinMode(BUTTON_FORCE_OPEN_PIN, INPUT);
  pinMode(BUTTON_BEGIN_DESCENT_PIN, INPUT);
  pinMode(ARM_BUTTON_PIN, INPUT);
  pinMode(CHECK_PIN, INPUT);
  pinMode(LEG_PIN, INPUT);  

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  char voltageString[10]; // Adjust the size as needed
  floatToString(measuredvbat, voltageString, 10, 2); // Convert float to string with 2 decimal places

  Serial.begin(9600);
  delay(1000);
  
  Serial.println("Feather LoRa Ground Station Startup!");
  displayMessage(1, "Ground Station Setup!");
  delay(1000);
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  displayMessage(1, voltageString);

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
      displayMessage(1, "Got reply:");
      displayMessage(1, (char*)buf);
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
          displayMessage(1, "Received reply");
          displayMessage(2, (char*)buf);
          Serial.println((char*)buf);
      } else {
          Serial.println("Receive from Fairing failed");
      }
  } else {
      Serial.println("No reply received from Fairing");
  }
  delay(500);

  Serial.println("Setup Complete");
  displayMessage(1, "Setup Complete!");
}

void loop() {
  // Calculate elapsed time
  unsigned long elapsedTime = millis() - startTime;

  // Convert elapsed time to hours, minutes, and seconds
  unsigned long hours = elapsedTime / 3600000;
  unsigned long minutes = (elapsedTime % 3600000) / 60000;
  unsigned long seconds = (elapsedTime % 60000) / 1000;

  // Display elapsed time on the first row
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  // Print hours
  if (hours < 10) {
    display.print("0");
  }
  display.print(hours);
  display.print(":");
  // Print minutes
  if (minutes < 10) {
    display.print("0");
  }
  display.print(minutes);
  display.print(":");
  // Print seconds
  if (seconds < 10) {
    display.print("0");
  }
  display.print(seconds);
  display.display();

  displayMessage(1, "Ground----");
  displayMessage(3, "SAIL----");
  displayMessage(5, "FAIRING----");
  displayMessage(7, "Status---");
  
  int armButtonState = digitalRead(ARM_BUTTON_PIN);
  int goButtonState = digitalRead(BUTTON_GO_PIN);
  int forceOpenButtonState = digitalRead(BUTTON_FORCE_OPEN_PIN);
  int beginDescentButtonState = digitalRead(BUTTON_BEGIN_DESCENT_PIN);
  int checkButtonState = digitalRead(CHECK_PIN);
  int legButtonState = digitalRead(LEG_PIN);

  // Signal received
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.recv(buf, &len)) {
    buf[len] = '\0'; // Null-terminate the received buffer
    Serial.print("Received signal: ");
    Serial.println((char*)buf);
    displayMessage(8, "Signal received");
    //displayMessage(2, (char*)buf);
  
    Serial.print("RSSI: ");
    Serial.println(rf95.lastRssi(), DEC);
    Serial.print("Received message length: ");
    Serial.println(len);

    // Check if the message is from the Fairing
    if (strncmp((char*)buf, "F: ", 3) == 0) {
        // Display the Fairing message in row 7
        displayMessage(7, (char*)(buf + 3)); // Skip the "F: " prefix
    }
    // Check if the message is from SAIL
    else if (strncmp((char*)buf, "S: ", 3) == 0) {
        // Display the SAIL message in row 4
        displayMessage(4, (char*)(buf + 3)); // Skip the "S: " prefix
    } 
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
      displayMessage(8, "ARMED!");
      if (goButtonState == LOW) {
        sendMessage("Go");
        displayMessage(3, "Sent: GO!");

      } else if (forceOpenButtonState == LOW) {
        sendMessage("Force Open");
        displayMessage(3, "Sent: Fairing Force Open");
        
      } else if (beginDescentButtonState == LOW) {
        sendMessage("Begin Controlled Descent");
        displayMessage(3, "Sent: Cont. Descent");
        
      } else if (checkButtonState == LOW) {
        sendMessage("Check");
        displayMessage(3, "Sent: Range Check");
        
      } else if (legButtonState == LOW) {
        sendMessage("Legs open");
        displayMessage(3, "Sent: Legs open");
        
      }
    } else {
      Serial.println("ARM button not pressed!");
      displayMessage(8,"ARM button not pressed!");
      delay(200);
    }
  } else {
    if (goButtonState == LOW || forceOpenButtonState == LOW || beginDescentButtonState == LOW || checkButtonState == LOW || legButtonState == LOW) {
      Serial.println("ARM not pressed!");
      displayMessage(8,"ARM not pressed!");
      //displayMessage(2,"Message not sent");
      delay(200);
    }
  }
}
