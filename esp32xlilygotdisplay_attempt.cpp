#include <HardwareSerial.h>
#include <SoftwareSerial.h>

// UART2 for first UWB module (HardwareSerial - more reliable)
HardwareSerial SerialUWB1(2);  // Uses UART2 (RX=16, TX=17)

// SoftwareSerial for second UWB module (RX=4, TX=5)
SoftwareSerial SerialUWB2(4, 5);  // RX=4, TX=5 (customizable)

void setup() {
  // Start USB Serial for debugging
  Serial.begin(115200);

  // Start HardwareSerial (UART2) for UWB1
  SerialUWB1.begin(115200, SERIAL_8N1, 16, 17);

  // Start SoftwareSerial for UWB2
  SerialUWB2.begin(115200);

  Serial.println("Dual UWB Module Test (SoftwareSerial)");
}

void loop() {
  // Read from UWB1 (HardwareSerial - UART2)
  if (SerialUWB1.available()) {
    String data1 = SerialUWB1.readStringUntil('\n');
    Serial.print("[UWB1] ");
    Serial.println(data1);
  }

  // Read from UWB2 (SoftwareSerial)
  if (SerialUWB2.available()) {
    String data2 = SerialUWB2.readStringUntil('\n');
    Serial.print("[UWB2] ");
    Serial.println(data2);
  }

  // Optional: Send commands to UWB modules
  // SerialUWB1.println("PING");
  // SerialUWB2.println("PING");
}
