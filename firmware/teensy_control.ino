/*
  teensy_control.ino
  Teensy 4.1 firmware for smart hot plate & stirrer
  - Phase-angle control (PWM output to Krida modules, heater & stirrer)
  - PID loop for temperature regulation
  - Reading temperature from MAX6675s
  - Rotary encoder and button interface
  - SD logging and USB passthrough
  Communicates with ESP32-S3 over UART.

  Author: Jacob Garrett
  Date: 2025
*/


#include <SD.h>
#include <MTP_Teensy.h>  // KurtE's library
#include <Encoder.h>     // For EC11 encoders
#include <Bounce.h>      // For debouncing encoder switches

// Pins
#define STIRRER_PIN 4   // PWM output for stirrer
#define HEATER_PIN 7    // PWM output for heater
#define TRANSITION_RATE 20 // % change per second

#define PRO_MICRO_SERIAL Serial8 // Serial8 for ESP32-S3 Mini (TX=35, RX=34)

// MAX6675 SPI Pins
#define MAX6675_CS1 10  // Chip Select for probe thermocouple
#define MAX6675_CS2 0   // Chip Select for hot top thermocouple
#define MAX6675_SCK 13  // Shared Serial Clock
#define MAX6675_SO1 12  // Serial Data Output for probe
#define MAX6675_SO2 1   // Serial Data Output for hot top

// SD Card (built-in on Teensy 4.1)
#define SD_CS BUILTIN_SDCARD  // Use built-in SD card slot

// Encoder Pins (EC11)
Encoder encStirrer(14, 15);  // Stirrer: A=14, B=15
const int switchStirrer = 16; // Stirrer switch
Encoder encHeater(37, 38);   // Heater: A=37, B=38
const int switchHeater = 36; // Heater switch

// Debounce objects for encoder switches (10 ms debounce interval)
Bounce stirrerSwitch = Bounce(switchStirrer, 10);
Bounce heaterSwitch = Bounce(switchHeater, 10);

// PID Control Variables
float targetTemperature = 0.0;
float heaterTemperature = 0.0;
float probeTemperature = 0.0;
float limitPower = 90;
float maxTemp = 500;
String controlMode = "O"; // "O" = off, "P" = probe control, "H" = hot top control
int modeIndex = 0;        // 0: off, 1: probe, 2: hot top

float heaterTarget = 0;

float tuneRange = 0.05;
float trackRange = 0.15;

float KpTrack = 2;
float KpTune = 0.6;
float Kp = KpTune;  
float Ki = 0.2;  
float Kd = 1.;  
float probeKp = 5;
float probeKi = 0.005;
float probeKd = 10;

float dt = 0.2;   // 5 Hz sampling
float deltaMax = 10; // max change in heating power per PID update

float autoStatusInterval = 1; // seconds for status display

float integral = 0.0;
float lastError = 0.0;
float probeIntegral = 0.0, probeLastError = 0.0;
bool freezeIntegral = false; // freeze integral updates in probe control when hitting output safety limits

unsigned long lastPIDTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastLogTime = 0;

float pidHeaterPower = 0.0;
float lastPidHeaterPower = 0.0;
float rampedHeaterPower = 0.0;
float oldHeaterPower = 0.0;
float newHeaterPower = 0.0;
unsigned long rampStartTime = 0;
bool ramping = false;
const unsigned long RAMP_DURATION_MS = 195;

String receivedData = "";
int targetStirPower = 0;
float currentStirPower = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastRampTime = 0;

// Remember last target value when stirrer and heater are toggled off/on
int switchOnStirPower = 40;
int switchOnHotTopTargetTemperature = 80;
int switchOnProbeTargetTemperature = 80;

String logFilename = "";
bool loggingEnabled = false;
unsigned long logInterval = 60000; // 1 minute in milliseconds
bool logMute = false;

void setup() {
  Serial.begin(9600);    
  PRO_MICRO_SERIAL.begin(9600); // Serial8 for ESP32-S3 Mini

  pinMode(STIRRER_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);

  analogWriteFrequency(STIRRER_PIN, 1000);
  analogWriteFrequency(HEATER_PIN, 1000);
  
  analogWrite(STIRRER_PIN, 0);
  analogWrite(HEATER_PIN, 0);

  pinMode(MAX6675_CS1, OUTPUT);
  digitalWrite(MAX6675_CS1, HIGH);
  pinMode(MAX6675_CS2, OUTPUT);
  digitalWrite(MAX6675_CS2, HIGH);
  pinMode(MAX6675_SCK, OUTPUT);

  pinMode(switchStirrer, INPUT_PULLUP);
  pinMode(switchHeater, INPUT_PULLUP);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    PRO_MICRO_SERIAL.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");
  PRO_MICRO_SERIAL.println("SD card initialized.");

  // MTP Setup using KurtE's API
  MTP.begin();  // Initialize MTP
  MTP.addFilesystem(SD, "Teensy SD");  // Add SD card as "Teensy SD"
  Serial.println("MTP initialized.");

  String startupMsg = "JC hotplate-stirrer system ready. Commands: 'S50','T100','L50','M400','Ffilename.csv','FCLEAR','FLIST','FMUTE','F60','N60','STATUS','SETTINGS','HELP','FPRINT','FPRINTfileName.csv'.";
  Serial.println(startupMsg);
  PRO_MICRO_SERIAL.println(startupMsg);
}

void loop() {
  bool statusRequestedSerial = readSerialCommand(Serial);    
  bool statusRequestedBTserial = readSerialCommand(PRO_MICRO_SERIAL);
  unsigned long now = millis();

  // Encoder Readings for Stirrer and Heater
  long stirPos = encStirrer.read() / 4; // 20 pulses/rev, 4 counts/pulse
  if (stirPos != 0) {
    targetStirPower = constrain(int(targetStirPower) + stirPos % 100, 0, 100); // Map to 0-100%
    encStirrer.write(0);
  }
  if (stirrerSwitch.update()) {
    if (stirrerSwitch.fallingEdge()) { // Detect press (LOW state, INPUT_PULLUP)
      if (targetStirPower == 0) {
        targetStirPower = switchOnStirPower; // Set to last value
      } else {
        switchOnStirPower = targetStirPower; // Remember last value
        targetStirPower = 0;
      }
    }
  }

  long heatPos = encHeater.read() / 4;
  if (heatPos != 0) {
    targetTemperature = constrain(int(targetTemperature) + heatPos % int(maxTemp), 0, maxTemp); // Map to 0-maxTemp
    if (heatPos < 0) {
      integral -= integral * 0.02; // make the response a bit faster if turning temp down
    } else if (heatPos > 0) {
      if (integral<0) {integral = 0;}
      integral += integral * 0.005; // a little more conservative when turning temp up
    }
    encHeater.write(0);
  }
  if (heaterSwitch.update()) {
    if (heaterSwitch.fallingEdge()) { // Detect press (LOW state, INPUT_PULLUP)
      modeIndex = (modeIndex + 1) % 3;
      if (modeIndex == 0) {
        switchOnHotTopTargetTemperature = targetTemperature;
        controlMode = "O";
        targetTemperature = 0;
      } else if (modeIndex == 1) {
        controlMode = "P";
        if (targetTemperature == 0) targetTemperature = switchOnProbeTargetTemperature;
      } else if (modeIndex == 2) {
        switchOnProbeTargetTemperature = targetTemperature;
        controlMode = "H";
        if (targetTemperature == 0) targetTemperature = switchOnHotTopTargetTemperature;
      }
    }
    integral = 0; // Reset integral if mode changes
  }

  if (statusRequestedSerial || statusRequestedBTserial || (now - lastStatusTime >= autoStatusInterval * 1000)) {
    reportStatus();
    lastStatusTime = now;
  }

  if (loggingEnabled && (now - lastLogTime >= logInterval)) {
    logData(now);
    lastLogTime = now;
  }

  if (now - lastPIDTime >= (unsigned long)(dt * 1000)) {
    readMAX6675();
    if (heaterTemperature == -999.0 || probeTemperature == -999.0) {
      Serial.println("[" + String(now) + "] ⚠️ ERROR: Thermocouple disconnected! Shutting off heater.");
      PRO_MICRO_SERIAL.println("[" + String(now) + "] ⚠️ ERROR: Thermocouple disconnected! Shutting off heater.");
      pidHeaterPower = 0.0;
    } else {
      if (controlMode == "P") {
        heaterTarget = computeProbePID(targetTemperature, probeTemperature);
        float rawPID = computePID(heaterTarget, heaterTemperature);
        pidHeaterPower = min(rawPID, limitPower);
      } else if (controlMode == "H") {
        float rawPID = computePID(targetTemperature, heaterTemperature);
        pidHeaterPower = min(rawPID, limitPower);
      } else if (controlMode == "O") {
        pidHeaterPower = 0.0;
      }
      beginHeaterRamp(pidHeaterPower);
      lastPidHeaterPower = pidHeaterPower;
    }
    lastPIDTime = now;
  }

  if (now - lastUpdateTime >= 50) {
    float stepSize = TRANSITION_RATE * (50.0 / 1000.0);
    if (currentStirPower < targetStirPower) {
      currentStirPower += stepSize;
      if (currentStirPower > targetStirPower) currentStirPower = targetStirPower;
    } else if (currentStirPower > targetStirPower) {
      currentStirPower -= stepSize;
      if (currentStirPower < targetStirPower) currentStirPower = targetStirPower;
    }
    analogWrite(STIRRER_PIN, map(currentStirPower, 0, 100, 0, 255));
    lastUpdateTime = now;
  }
  if (now - lastRampTime >= (unsigned long)((dt/5) * 1000)) {
    updateHeaterRamp();
    lastRampTime = now;
  }

  // Handle MTP events
  MTP.loop();
}

void reportStatus() {
  String outputString = "";
  if (controlMode == "P") {
    outputString = String(heaterTarget/10,0); // temporary solution to monitor the probe PID loop as it needs tuning
  } else {
    outputString = String(pidHeaterPower,0);
  }
  // Verbose status for Serial
  String verboseStatusMsg = "H:" + String(heaterTemperature,0) + "°C, P:" + String(probeTemperature,0) + "°C, T:" + String(targetTemperature,0) + "°C, W:" + outputString + "%, S:" + String(targetStirPower) + "%";
  Serial.println(verboseStatusMsg);

  // Compact status for PRO_MICRO_SERIAL (ESP32-S3 Mini display)
  String compactStatusMsg = "H:" + String(heaterTemperature,0) + ",P:" + String(probeTemperature,0) + ",T:" + String(targetTemperature,0) + ",W:" + outputString + ",S:" + String(targetStirPower) + ",M:" + controlMode + "\n";
  PRO_MICRO_SERIAL.print(compactStatusMsg);
}

void beginHeaterRamp(float newPower) {
  oldHeaterPower = rampedHeaterPower;  
  newHeaterPower = newPower;           
  rampStartTime = millis();
  ramping = true;
}

void updateHeaterRamp() {
  static float lastRampedPower = -1.0;
  unsigned long now = millis();
  int pwmValue;

  if (!ramping) {
    pwmValue = map(rampedHeaterPower, 0, 100, 0, 255);
    analogWrite(HEATER_PIN, pwmValue);
    if (abs(rampedHeaterPower - lastRampedPower) >= 1.0) {
      lastRampedPower = rampedHeaterPower;
    }
    return;
  }

  float elapsed = (float)(now - rampStartTime);
  float ratio = elapsed / (float)RAMP_DURATION_MS;
  if (ratio >= 1.0) {
    rampedHeaterPower = newHeaterPower;
    ramping = false;
  } else {
    rampedHeaterPower = oldHeaterPower + (newHeaterPower - oldHeaterPower) * ratio;
  }
  pwmValue = map(rampedHeaterPower, 0, 100, 0, 255);
  analogWrite(HEATER_PIN, pwmValue);
  if (abs(rampedHeaterPower - lastRampedPower) >= 1.0) {
    lastRampedPower = rampedHeaterPower;
  }
}

bool readSerialCommand(Stream &serialInterface) {
  bool requestStatus = false;

  while (serialInterface.available()) {
    char incomingChar = serialInterface.read();
    receivedData += incomingChar;

    if (incomingChar == '\n') {
      receivedData.trim();

      if (receivedData.length() == 0 || receivedData.equalsIgnoreCase("STATUS")) {
        requestStatus = true;
      } else if (receivedData.equalsIgnoreCase("FCLEAR")) {
        purgeLogs();
      } else if (receivedData.equalsIgnoreCase("FLIST")) {
        listLogs();
      } else if (receivedData.equalsIgnoreCase("SETTINGS") || receivedData.equalsIgnoreCase("SETTING")) {
        printSettings(serialInterface);
      } else if (receivedData.equalsIgnoreCase("HELP")) {
        printHelp(serialInterface);
      } else if (receivedData.equalsIgnoreCase("FPRINT")) {
        printLog(serialInterface, logFilename);
      } else if (receivedData.startsWith("FPRINT")) {
        String fileToPrint = receivedData.substring(6);
        printLog(serialInterface, fileToPrint);
      } else {
        String commandStr = receivedData.substring(0, 1).toUpperCase();
        if (commandStr.length() > 0) {
          char command = commandStr[0];
          String param = receivedData.substring(1);

          if (command == 'F') {
            if (param.length() == 0) {
              loggingEnabled = false;
              serialInterface.println("Logging stopped");
            } else if (param.equalsIgnoreCase("MUTE")) {
              logMute = !logMute;
              serialInterface.print("Logging mute: ");
              serialInterface.println(logMute ? "ON" : "OFF");
            } else if (param.toInt() > 0) {
              int val = param.toInt();
              logInterval = val * 1000;
              serialInterface.print("Logging interval set to: ");
              serialInterface.print(val);
              serialInterface.println(" seconds");
            } else {
              logFilename = param;
              loggingEnabled = true;
              serialInterface.print("Logging to: ");
              serialInterface.println(logFilename);
            }
          } else if (command == 'S') {
            int val = param.toInt();
            if (val >= 0 && val <= 100) {
              targetStirPower = val;
              serialInterface.print("Stirrer set to: ");
              serialInterface.print(targetStirPower);
              serialInterface.println("%");
            } else {
              serialInterface.println("Invalid Stirrer value");
            }
          } else if (command == 'T') {
            float val = param.toFloat();
            if (val >= 0 && val <= maxTemp) {
              targetTemperature = val;
              serialInterface.print("Target temperature set to: ");
              serialInterface.print(targetTemperature);
              serialInterface.println("°C");
              if (integral < 0) integral = 0;
            } else {
              serialInterface.println("Invalid Target temperature");
            }
          } else if (command == 'L') {
            float val = param.toFloat();
            if (val >= 0 && val <= 100) {
              limitPower = val;
              serialInterface.print("Max heater power set to: ");
              serialInterface.print(limitPower);
              serialInterface.println("%");
              integral = 0;
            } else {
              serialInterface.println("Invalid Max heater power");
            }
          } else if (command == 'M') {
            float val = param.toFloat();
            if (val >= 0 && val <= 500) {
              maxTemp = val;
              serialInterface.print("Max temperature setting: ");
              serialInterface.print(maxTemp);
              serialInterface.println("°C");
            } else {
              serialInterface.println("Invalid Max temperature");
            }
          } else if (command == 'N') {
            int val = param.toInt();
            if (val > 0) {
              logInterval = val * 1000;
              serialInterface.print("Logging interval set to: ");
              serialInterface.print(val);
              serialInterface.println(" seconds");
            } else {
              serialInterface.println("Invalid logging interval");
            }
          } else if (command == 'P') {
            controlMode = "P";
            modeIndex = 1;
            integral = 0;
            float val = param.toFloat();
            if (val >= 0 && val <= maxTemp) {
              targetTemperature = val;
              serialInterface.print("Probe target temperature set to: ");
              serialInterface.print(targetTemperature);
              serialInterface.println("°C");
            } else {
              serialInterface.println("Invalid Target temperature");
            }
          } else if (command == 'H') {
            controlMode = "G";
            modeIndex = 2;
            integral = 0;
            float val = param.toFloat();
            if (val >= 0 && val <= maxTemp) {
              targetTemperature = val;
              serialInterface.print("Hot-top target temperature set to: ");
              serialInterface.print(targetTemperature);
              serialInterface.println("°C");
            } else {
              serialInterface.println("Invalid Target temperature");
            }
          } else if (command == 'O') {
            controlMode = "O";
            modeIndex = 0;
            integral = 0;
            targetTemperature = 0;
            serialInterface.println("Heating off");
          } else {
            serialInterface.println("Unknown command");
          }
        }
      }
      receivedData = "";
    }
  }
  return requestStatus;
}

float computePID(float setpoint, float measured) {
  float error = setpoint - measured;
  float relError = setpoint != 0 ? error / setpoint : error;
  float derivative = (error - lastError) / dt;

  if ((error > 0) && (relError > tuneRange)) {
    Kp = KpTrack;
    if ((error > 0) && (relError > trackRange)) {
      integral = 0;
    } else {
      integral += error * (1 - atan((relError - tuneRange) / trackRange)) * dt;
    }
  } else {
    integral += error * dt;
    Kp = KpTune;
  }

  float output = (Kp * error) + (Kd * derivative) + (Ki * integral);
  if (measured >= maxTemp + 25) {
    Serial.println("[" + String(millis()) + "] ⚠️ SAFETY ALERT: Temperature exceeded max temp + 25°C! Shutting off heater.");
    PRO_MICRO_SERIAL.println("[" + String(millis()) + "] ⚠️ ERROR: Temperature exceeded max temp + 25°C! Shutting off heater.");
    output = 0.0;
    pinMode(HEATER_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW);
    rampedHeaterPower = 0.0;
  }

  if (output > pidHeaterPower + deltaMax) {
    output = pidHeaterPower + deltaMax;
  } else if (output < pidHeaterPower - deltaMax) {
    output = pidHeaterPower - deltaMax;
  }
  if (output < 0) {
    output = 0;
  }
  lastError = error;
  if (setpoint == 0) {
    integral = 0;
    output = 0;
  }
  return output;
}

float computeProbePID(float setpoint, float measured) {
  float error = setpoint - measured;
  float derivative = (error - probeLastError) / dt;
  if (!freezeIntegral) {probeIntegral += error * dt;}
  float output = (probeKp * (error+50)) + (probeKi * probeIntegral) + (probeKd * derivative);
  probeLastError = error;
  if ((output - measured) >= 250) { 
    freezeIntegral = true;
    output = measured + 250;
  } else {
    freezeIntegral = false;
  }
  if (output >= maxTemp) {
    output = maxTemp;
    freezeIntegral = true;
  } else
  {
    freezeIntegral = false;
  }
  if (output < 0) { output = 0; }
  if (setpoint == 0) {
    probeIntegral = 0;
    output = 0;
  }
  return output;
}

void readMAX6675() {
  uint16_t rawData1 = 0;
  uint16_t rawData2 = 0;

  digitalWrite(MAX6675_CS1, LOW);
  digitalWrite(MAX6675_CS2, LOW);
  delayMicroseconds(300);
  pinMode(MAX6675_SO1, INPUT);
  pinMode(MAX6675_SO2, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);

  for (int i = 15; i >= 0; i--) {
    digitalWrite(MAX6675_SCK, LOW);
    delayMicroseconds(10);
    digitalWrite(MAX6675_SCK, HIGH);
    delayMicroseconds(10);
    rawData1 |= (digitalRead(MAX6675_SO1) << i);
    rawData2 |= (digitalRead(MAX6675_SO2) << i);
  }

  digitalWrite(MAX6675_CS1, HIGH);
  digitalWrite(MAX6675_CS2, HIGH);

  if (rawData1 & 0x04) {
    heaterTemperature = -999;
  } else {
    rawData1 >>= 3;
    probeTemperature = rawData1*0.25 - 5;
  }

  if (rawData2 & 0x04) {
    probeTemperature = -999;
  } else {
    rawData2 >>= 3;
    heaterTemperature = rawData2*0.25;
  }
  
  digitalWrite(MAX6675_SCK, LOW);

}

void logData(unsigned long timestamp) {
  char fileNameChar[logFilename.length() + 1];
  logFilename.toCharArray(fileNameChar, sizeof(fileNameChar));
  File logFile = SD.open(fileNameChar, FILE_WRITE);
  if (logFile) {
    if (logFile.size() == 0) {
      logFile.println("Time (ms),Heater Temp (°C),Probe Temp (°C),Target Temp (°C),Heater Power (%),Stirrer Speed (%)");
    }
    String logMsg = String(timestamp) + "," +
                    (heaterTemperature == -999.0 ? "ERROR" : String(heaterTemperature)) + "," +
                    (probeTemperature == -999.0 ? "ERROR" : String(probeTemperature)) + "," +
                    String(targetTemperature) + "," +
                    String(pidHeaterPower) + "," +
                    String(targetStirPower);
    logFile.println(logMsg);
    logFile.close();
    if (!logMute) {
      Serial.println("Logged: " + logMsg);
      PRO_MICRO_SERIAL.println("Logged: " + logMsg);
    }
  } else {
    Serial.println("Error opening log file: " + logFilename);
    PRO_MICRO_SERIAL.println("Error opening log file: " + logFilename);
  }
}

void purgeLogs() {
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    String name = entry.name();
    if (name.toUpperCase().endsWith(".CSV")) {
      char fileNameChar[name.length() + 1];
      name.toCharArray(fileNameChar, sizeof(fileNameChar));
      SD.remove(fileNameChar);
    }
    entry.close();
  }
  root.close();
  Serial.println("All logs cleared");
  PRO_MICRO_SERIAL.println("All logs cleared");
}

void listLogs() {
  File root = SD.open("/");
  bool found = false;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    String name = entry.name();
    if (name.toUpperCase().endsWith(".CSV")) {
      Serial.println(name);
      PRO_MICRO_SERIAL.println(name);
      found = true;
    }
    entry.close();
  }
  root.close();
  if (!found) {
    Serial.println("No logs found");
    PRO_MICRO_SERIAL.println("No logs found");
  }
}

void printSettings(Stream &serialInterface) {
  serialInterface.print("Current Settings: ");
  serialInterface.print("Heating power limit: ");
  serialInterface.print(limitPower);
  serialInterface.print("%");
  serialInterface.print(" Max temperature setting: ");
  serialInterface.print(maxTemp);
  serialInterface.print("°C ");
  if (loggingEnabled) {
    serialInterface.print("Current log file: ");
    serialInterface.print(logFilename);
    serialInterface.print(" Log rate: ");
    serialInterface.print(logInterval / 1000);
    serialInterface.print(" seconds");
    int entryCount = countLogEntries(logFilename);
    serialInterface.print(" Number of log entries: ");
    serialInterface.println(entryCount);
  } else {
    serialInterface.println("Logging is disabled");
  }
}

int countLogEntries(String filename) {
  char fileNameChar[filename.length() + 1];
  filename.toCharArray(fileNameChar, sizeof(fileNameChar));
  File logFile = SD.open(fileNameChar);
  int count = 0;
  if (logFile) {
    while (logFile.available()) {
      String line = logFile.readStringUntil('\n');
      if (line.length() > 0) count++;
    }
    logFile.close();
    if (count > 0) count--; // Subtract header line
  }
  return count;
}

void printHelp(Stream &serialInterface) {
  String helpMsg = "Available commands:\n"
                   "  S### - Set stirrer speed (0-100%)\n"
                   "  T### - Set target temperature (0-500°C)\n"
                   "  P### - Set probe target temperature (0-500°C)\n"
                   "  H### - Set hot-top target temperature (0-500°C)\n"
                   "  O - Turn off heating\n"
                   "  L### - Set max heater power (0-100%)\n"
                   "  M### - Set max temperature (0-500°C)\n"
                   "  Ffilename.csv - Start logging to filename.csv\n"
                   "  F - Stop logging\n"
                   "  FMUTE - Toggle logging mute\n"
                   "  F### - Set logging interval in seconds\n"
                   "  N### - Set logging interval in seconds\n"
                   "  FCLEAR - Clear all logs\n"
                   "  FLIST - List all logs\n"
                   "  STATUS - Print current status\n"
                   "  SETTINGS - Print current settings\n"
                   "  HELP - Print this help message\n"
                   "  FPRINT - Print last 200 lines of current log\n"
                   "  FPRINTfileName.csv - Print last 200 lines of specified log";
  serialInterface.println(helpMsg);
}

void printLog(Stream &serialInterface, String filename) {
  char fileNameChar[filename.length() + 1];
  filename.toCharArray(fileNameChar, sizeof(fileNameChar));
  File logFile = SD.open(fileNameChar);
  if (logFile) {
    String lines[200];
    int lineCount = 0;
    while (logFile.available()) {
      String line = logFile.readStringUntil('\n');
      if (line.length() > 0) {
        lines[lineCount % 200] = line;
        lineCount++;
      }
    }
    logFile.close();
    int start = (lineCount > 200) ? (lineCount - 200) : 0;
    for (int i = start; i < lineCount; i++) {
      serialInterface.println(lines[i % 200]);
    }
  } else {
    serialInterface.println("Error opening log file: " + filename);
  }
}
