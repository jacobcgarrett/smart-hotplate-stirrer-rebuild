/*
  esp32_ui.ino
  ESP32S3 firmware for smart hot plate & stirrer
  - OLED (SSD1306) display via I2C
  - Bluetooth Low-Energy Interface
  - Potentially WiFi at some future point

  Author: Jacob Garrett
  Date: May 2025
*/



#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Hardware Serial for communication with Teensy (RX=11, TX=10)
HardwareSerial TeensySerial(1); // Use Serial1, pins 11 (RX), 10 (TX)

// SSD1306 Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BLE UART Service UUIDs (Nordic UART Service)
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String receivedData = "";
String lastStatus = ""; // Store the last status message
String tempMessage = ""; // Store temporary non-status message
unsigned long tempMessageStartTime = 0; // Time when temp message was displayed
const unsigned long TEMP_MESSAGE_DURATION = 5000; // 5 seconds

// Callback for BLE connection/disconnection
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();  // Use Arduino String type
      if (rxValue.length() > 0) {
        // Forward BLE data to Teensy
        for (int i = 0; i < rxValue.length(); i++) {
          TeensySerial.write(rxValue[i]);
        }
        // Optionally echo to Serial for debugging
        Serial.write(rxValue.c_str(), rxValue.length());
      }
    }
};

void setup() {
  // Start Serial for debugging
  Serial.begin(9600);

  // Start Hardware Serial for Teensy communication (RX=11, TX=10)
  TeensySerial.begin(9600, SERIAL_8N1, 11, 10);

  // Initialize I2C with custom pins (SDA=9, SCL=8)
  Wire.begin(9, 8); // SDA, SCL
  Wire.setClock(50000); // Set I2C clock to 50 kHz

  // Initialize SSD1306 Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C
    Serial.println("SSD1306 allocation failed");
    while (1);
  }

  // Display Initial Message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Waiting for");
  display.println("Teensy...");
  display.display();

  // Initialize BLE
  BLEDevice::init("HotplateESP");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  
  // Set a custom passkey (6-digit numeric)
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setStaticPIN(857142); // Custom passkey for pairing

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  // TX Characteristic (for sending data to phone)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX Characteristic (for receiving data from phone)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Functions that help with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE started, waiting for connections...");
}

void loop() {
  // Handle BLE connection state
  if (deviceConnected && !oldDeviceConnected) {
    // Device just connected
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE device connected");
  }
  if (!deviceConnected && oldDeviceConnected) {
    // Device just disconnected
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE device disconnected");
    BLEDevice::startAdvertising(); // Restart advertising
  }

  // Read data from Teensy
  while (TeensySerial.available()) {
    char incomingChar = TeensySerial.read();
    receivedData += incomingChar;

    if (incomingChar == '\n') {
      // Forward Teensy messages to BLE if connected
      if (deviceConnected) {
        //receivedData += '\r';
        pTxCharacteristic->setValue(receivedData.c_str());
        pTxCharacteristic->notify();
      }
      receivedData.trim();
      if (receivedData.startsWith("H:")) {
        lastStatus = receivedData; // Store status message
        tempMessage = ""; // Clear any temporary message
      } else {
        tempMessage = receivedData; // Store non-status message
        tempMessageStartTime = millis(); // Start timer
      }

      updateDisplay();
      receivedData = "";
    }
  }

  // Check if temporary message should be cleared
  if (tempMessage != "" && (millis() - tempMessageStartTime >= TEMP_MESSAGE_DURATION)) {
    tempMessage = ""; // Clear temporary message after 5 seconds
    updateDisplay();
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (lastStatus != "") {
    // Parse status message
    String mode = parseStringValue(lastStatus, "M:");
    float heaterTemp = parseValue(lastStatus, "H:");
    float probeTemp = parseValue(lastStatus, "P:");
    float targetTemp = parseValue(lastStatus, "T:");
    float heaterPower = parseValue(lastStatus, "W:");
    float stirPower = parseValue(lastStatus, "S:");

    // Handle error values and constrain display values
    String heaterStr = (heaterTemp == -999.0) ? "ERR" : String((int)heaterTemp);
    String probeStr = (probeTemp == -999.0) ? "ERR" : String((int)probeTemp);
    heaterPower = constrain(heaterPower, 0, 99); // Limit to 0-99 for display
    stirPower = constrain(stirPower, 0, 99); // Limit to 0-99 for display

    // Format temperature strings with leading spaces for alignment (3 digits)
    if (heaterStr == "ERR") {
      heaterStr = "ERR"; // Already 3 chars
    } else if (heaterTemp < 10) {
      heaterStr = "00" + heaterStr; // e.g., "005"
    } else if (heaterTemp < 100) {
      heaterStr = "0" + heaterStr; // e.g., "025"
    }

    if (probeStr == "ERR") {
      probeStr = "ERR"; // Already 3 chars
    } else if (probeTemp < 10) {
      probeStr = "00" + probeStr; // e.g., "009"
    } else if (probeTemp < 100) {
      probeStr = "0" + probeStr; // e.g., "029"
    }

    // Format first line based on mode (8 characters: "H 25>P 29", ">H 25 P 29")
    String firstLine;
    if (mode == "P") {
      firstLine = "H" + heaterStr + ">P" + probeStr; // "H 25>P 29"
    } else if (mode == "H") {
      firstLine = ">H" + heaterStr + " P" + probeStr; // ">H 25 P 29"
    } else {
      firstLine = "H" + heaterStr + " P" + probeStr; // "H 25 P 29"
    }

    // Format second line (9 characters: "T100W80S50")
    String secondLine;
    if (tempMessage != "") {
      secondLine = tempMessage.substring(0, 9); // Limit to 9 chars
    } else if (mode == "O") {
      secondLine = "Off S";
      if (stirPower < 10) {
        secondLine += "0" + String((int)stirPower); // e.g., "Off S05"
      } else {
        secondLine += String((int)stirPower); // e.g., "Off S50"
      }
    } else {
      String targetStr = String((int)targetTemp);
      if (targetTemp < 10) {
        targetStr = "00" + targetStr; // e.g., "005"
      } else if (targetTemp < 100) {
        targetStr = "0" + targetStr; // e.g., "025"
      }
      String heaterPowerStr = String((int)heaterPower);
      if (heaterPower < 10) {
        heaterPowerStr = "0" + heaterPowerStr; // e.g., "08"
      }
      String stirPowerStr = String((int)stirPower);
      if (stirPower < 10) {
        stirPowerStr = " " + stirPowerStr; // e.g., "05"
      }
      secondLine = "T" + targetStr + "W" + heaterPowerStr + "S" + stirPowerStr; // e.g., "T100W80S50"
    }

    // Display with text size 2
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(firstLine);
    display.setCursor(0, 16);
    display.print(secondLine);
  } else {
    // Show waiting message with text size 1
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Waiting for");
    display.println("Teensy...");
  }
  display.display();
}

float parseValue(String data, String prefix) {
  int startIndex = data.indexOf(prefix) + prefix.length();
  int endIndex = data.indexOf(",", startIndex);
  if (endIndex == -1) endIndex = data.length();
  String valueStr = data.substring(startIndex, endIndex);
  return valueStr.toFloat();
}

String parseStringValue(String data, String prefix) {
  int startIndex = data.indexOf(prefix) + prefix.length();
  int endIndex = data.indexOf(",", startIndex);
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex);
}
