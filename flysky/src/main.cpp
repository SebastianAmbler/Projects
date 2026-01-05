/*
====================================================================================
FlySky FS-iA6B PPM Test Code
====================================================================================
This code tests your FlySky receiver PPM connection and displays all channel values
in real-time. Use this to verify your receiver is working before flying.

Wiring:
  FS-iA6B PPM/CH1 (Signal) → ESP32 GPIO 4 (any interrupt-capable pin)
  FS-iA6B +5V              → 5V BEC (from ESC)
  FS-iA6B GND              → ESP32 GND

Expected Values:
  - Sticks centered: ~1500
  - Throttle down: ~1000
  - Full stick deflection: 1000-2000
  - Switches: 1000 (off) or 2000 (on)

Note: Make sure FS-iA6B is set to PPM output mode!
====================================================================================
*/

#include <PPMReader.h>

// PPM Configuration
byte ppmInterruptPin = 16;      // GPIO 4 - can be any interrupt-capable pin
byte channelAmount = 6;        // FS-iA6B has 6 channels in PPM mode
PPMReader* ppm = nullptr;

// Timing
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 100; // Print every 100ms

// Signal monitoring
unsigned long lastSignalTime = 0;
bool signalDetected = false;
int lastChannelValues[6] = {0, 0, 0, 0, 0, 0};
const int changeThreshold = 10;

// Channel names for easy reading
const char* channelNames[] = {
  "Roll    (Ch1)",
  "Pitch   (Ch2)", 
  "Throttle(Ch3)",
  "Yaw     (Ch4)",
  "SwA     (Ch5)",
  "SwB     (Ch6)"
};

// Function declarations
void printChannelData(int channels[]);
void printValueWithBar(int value);
void printNoSignal();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║   FlySky FS-iA6B PPM Receiver Test            ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
  
  Serial.println("✓ PPM initialized on GPIO 4 (Interrupt)");
  Serial.println("✓ Waiting for receiver signal...");
  Serial.println();
  Serial.println("Expected wiring:");
  Serial.println("  FS-iA6B PPM/CH1 → GPIO 4");
  Serial.println("  FS-iA6B +5V     → 5V BEC");
  Serial.println("  FS-iA6B GND     → ESP32 GND");
  Serial.println();
  Serial.println("IMPORTANT: Receiver must be in PPM mode!");
  Serial.println("  (Not PWM or iBUS mode)");
  Serial.println();
  Serial.println("═══════════════════════════════════════════════");
  Serial.println();
  
  delay(2000);
  lastSignalTime = millis();

  // Initialize PPM reader after system is ready
  ppm = new PPMReader(ppmInterruptPin, channelAmount);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read all channels
  int channels[6];
  bool validData = false;
  bool valuesChanged = false;
  
  for (int i = 0; i < channelAmount; i++) {
    channels[i] = ppm->latestValidChannelValue(i + 1, 0); // Channels start from 1
    
    // Check if we have valid data (PPM returns 1000-2000 range)
    if (channels[i] >= 800 && channels[i] <= 2200) {
      validData = true;
      
      // Check if values have changed significantly
      if (abs(channels[i] - lastChannelValues[i]) > changeThreshold) {
        valuesChanged = true;
        lastSignalTime = currentTime;
      }
      
      lastChannelValues[i] = channels[i];
    }
  }
  
  // Update signal status
  if (validData && valuesChanged) {
    if (!signalDetected) {
      Serial.println("\n✓✓✓ PPM SIGNAL DETECTED! ✓✓✓\n");
      signalDetected = true;
    }
  } else if (currentTime - lastSignalTime > 2000) { // 2 second timeout
    if (signalDetected) {
      Serial.println("\n⚠️  SIGNAL LOST OR FROZEN! ⚠️\n");
      signalDetected = false;
    }
  }
  
  // Print channel values periodically
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    
    if (signalDetected || validData) {
      printChannelData(channels);
    } else {
      printNoSignal();
    }
  }
  
  delay(10); // Small delay
}

void printChannelData(int channels[]) {
  // Clear previous lines (ANSI escape codes for terminal)
  Serial.print("\033[12A"); // Move cursor up 12 lines
  Serial.print("\033[J");   // Clear from cursor to end of screen
  
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║            CHANNEL VALUES (PPM)                ║");
  Serial.println("╠════════════════════════════════════════════════╣");
  
  for (int i = 0; i < channelAmount; i++) {
    Serial.print("║ ");
    Serial.print(channelNames[i]);
    Serial.print(": ");
    
    // Print value with bar graph
    printValueWithBar(channels[i]);
    
    Serial.println(" ║");
  }
  
  Serial.println("╠════════════════════════════════════════════════╣");
  
  // Status indicators
  Serial.print("║ Signal: ");
  if (signalDetected) {
    Serial.print("✓ GOOD   ");
  } else {
    Serial.print("⚠ WEAK   ");
  }
  
  // Check throttle position (CRITICAL for safety)
  if (channels[2] < 1100) {
    Serial.print("| Throttle: ✓ SAFE (LOW)");
  } else if (channels[2] >= 1100 && channels[2] < 1200) {
    Serial.print("| Throttle: ⚠ IDLE      ");
  } else {
    Serial.print("| Throttle: ⚠️ HIGH!     ");
  }
  
  Serial.println("  ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
}

void printValueWithBar(int value) {
  // Handle invalid values
  if (value < 800 || value > 2200) {
    Serial.print("INVALID");
    Serial.print("                           ");
    return;
  }
  
  // Print numeric value
  if (value < 1000) {
    Serial.print(" ");
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
  Serial.print("⚠️  NO PPM SIGNAL DETECTED - Check connections");
  
  // Animated dots
  for (int i = 0; i < (dotCount % 4); i++) {
    Serial.print(".");
  }
  Serial.print("   ");
  
  dotCount++;
  
  if (dotCount % 20 == 0) {
    Serial.println("\n");
    Serial.println("Troubleshooting:");
    Serial.println("  1. Is receiver powered? (Check LED)");
    Serial.println("  2. Is transmitter ON and bound?");
    Serial.println("  3. Is PPM wire connected to GPIO 4?");
    Serial.println("  4. Is FS-iA6B in PPM OUTPUT mode?");
    Serial.println("     - Check receiver jumper/settings");
    Serial.println("     - PPM should come from CH1/PPM port");
    Serial.println("  5. Try a different channel port (CH1-CH6)");
    Serial.println();
  }
}