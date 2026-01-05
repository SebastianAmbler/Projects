/*
====================================================================================
FlySky FS-iA6B iBUS Test Code
====================================================================================
This code tests your FlySky receiver connection and displays all channel values
in real-time. Use this to verify your receiver is working before flying.

Wiring:
  FS-iA6B iBUS SERVO (Signal) → ESP32 GPIO 16 (RX2)
  FS-iA6B +5V                 → 5V BEC (from ESC)
  FS-iA6B GND                 → ESP32 GND

Expected Values:
  - Sticks centered: ~1500
  - Throttle down: ~1000
  - Full stick deflection: 1000-2000
  - Switches: 1000 (off) or 2000 (on)
====================================================================================
*/

#include <Arduino.h>
#include <IBusBM.h>

// iBUS Configuration
IBusBM ibus;
const int IBUS_RX_PIN = 16;  // GPIO 16 for Serial2 RX
const int IBUS_TX_PIN = 17;  // GPIO 17 for Serial2 TX (optional)

// Timing
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 100; // Print every 100ms

// Signal monitoring
unsigned long lastSignalTime = 0;
bool signalDetected = false;
int failsafeCount = 0;

// Channel names for easy reading
const char* channelNames[] = {
  "Roll    (Ch1)",
  "Pitch   (Ch2)", 
  "Throttle(Ch3)",
  "Yaw     (Ch4)",
  "SwA     (Ch5)",
  "SwB     (Ch6)",
  "Channel 7   ",
  "Channel 8   ",
  "Channel 9   ",
  "Channel 10  "
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║   FlySky FS-iA6B iBUS Receiver Test           ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize iBUS on Serial2
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  ibus.begin(Serial2);
  
  Serial.println("✓ iBUS initialized on GPIO 16 (RX2)");
  Serial.println("✓ Waiting for receiver signal...");
  Serial.println();
  Serial.println("Expected wiring:");
  Serial.println("  FS-iA6B iBUS Signal → GPIO 16");
  Serial.println("  FS-iA6B +5V         → 5V BEC");
  Serial.println("  FS-iA6B GND         → ESP32 GND");
  Serial.println();
  Serial.println("═══════════════════════════════════════════════");
  Serial.println();
  
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read all 10 channels
  int channels[10];
  bool validData = false;
  
  for (int i = 0; i < 10; i++) {
    channels[i] = ibus.readChannel(i);
    
    // Check if we have valid data (iBUS returns 1000-2000 range)
    if (channels[i] >= 1000 && channels[i] <= 2000) {
      validData = true;
      lastSignalTime = currentTime;
    }
  }
  
  // Update signal status
  if (validData) {
    if (!signalDetected) {
      Serial.println("\n✓✓✓ SIGNAL DETECTED! ✓✓✓\n");
      signalDetected = true;
    }
    failsafeCount = 0;
  } else {
    if (currentTime - lastSignalTime > 1000) { // 1 second timeout
      failsafeCount++;
      if (signalDetected && failsafeCount == 1) {
        Serial.println("\n⚠️  SIGNAL LOST! ⚠️\n");
        signalDetected = false;
      }
    }
  }
  
  // Print channel values periodically
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    
    if (signalDetected) {
      printChannelData(channels);
    } else {
      printNoSignal();
    }
  }
  
  delay(10); // Small delay to prevent overwhelming the serial buffer
}

void printChannelData(int channels[]) {
  // Clear previous lines (ANSI escape codes for terminal)
  Serial.print("\033[15A"); // Move cursor up 15 lines
  Serial.print("\033[J");   // Clear from cursor to end of screen
  
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║            CHANNEL VALUES (iBUS)               ║");
  Serial.println("╠════════════════════════════════════════════════╣");
  
  for (int i = 0; i < 6; i++) { // Show first 6 channels (main controls)
    Serial.print("║ ");
    Serial.print(channelNames[i]);
    Serial.print(": ");
    
    // Print value with bar graph
    printValueWithBar(channels[i]);
    
    Serial.println(" ║");
  }
  
  Serial.println("╠════════════════════════════════════════════════╣");
  
  // Show additional channels if they have non-default values
  bool hasExtraChannels = false;
  for (int i = 6; i < 10; i++) {
    if (channels[i] > 0 && channels[i] != 1500) {
      hasExtraChannels = true;
      break;
    }
  }
  
  if (hasExtraChannels) {
    Serial.println("║ Extra Channels:                                ║");
    for (int i = 6; i < 10; i++) {
      if (channels[i] > 0) {
        Serial.print("║   Ch");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(channels[i]);
        Serial.println("                                      ║");
      }
    }
    Serial.println("╠════════════════════════════════════════════════╣");
  }
  
  // Status indicators
  Serial.print("║ Signal: ✓ GOOD   ");
  
  // Check throttle position (CRITICAL for safety)
  if (channels[2] < 1100) {
    Serial.print("| Throttle: ✓ SAFE (LOW)");
  } else if (channels[2] > 1100 && channels[2] < 1200) {
    Serial.print("| Throttle: ⚠ IDLE      ");
  } else {
    Serial.print("| Throttle: ⚠️ HIGH!     ");
  }
  
  Serial.println("  ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
}

void printValueWithBar(int value) {
  // Print numeric value
  if (value < 1000) {
    Serial.print("    ");
    Serial.print(value);
  } else {
    Serial.print(value);
  }
  
  Serial.print(" ");
  
  // Print bar graph
  int barLength = map(value, 1000, 2000, 0, 20);
  barLength = constrain(barLength, 0, 20);
  
  Serial.print("[");
  for (int i = 0; i < 20; i++) {
    if (i < barLength) {
      Serial.print("█");
    } else if (i == barLength) {
      Serial.print("▌");
    } else {
      Serial.print(" ");
    }
  }
  Serial.print("]");
}

void printNoSignal() {
  static int dotCount = 0;
  
  Serial.print("\r"); // Carriage return to overwrite line
  Serial.print("⚠️  NO SIGNAL DETECTED - Check connections");
  
  // Animated dots
  for (int i = 0; i < (dotCount % 4); i++) {
    Serial.print(".");
  }
  Serial.print("   ");
  
  dotCount++;
  
  if (dotCount % 20 == 0) {
    Serial.println();
    Serial.println("\nTroubleshooting:");
    Serial.println("  1. Is receiver powered? (Check LED)");
    Serial.println("  2. Is transmitter ON and bound?");
    Serial.println("  3. Is iBUS wire connected to GPIO 16?");
    Serial.println("  4. Is FS-iA6B set to iBUS mode?");
    Serial.println();
  }
}