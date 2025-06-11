#include <HardwareSerial.h>

HardwareSerial SerialUWB1(PE7, PE8);  // UART7 (PE7=RX, PE8=TX)
HardwareSerial SerialUWB2(PC7, PC6);  // USART6 (PC7=RX, PC6=TX)

bool commandSent = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
 
  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);
  Serial.println("Nucleo-F767ZI Dual UWB Test");
}

void loop() {
  // Send "les" + Enter only once at startup
  if (!commandSent) {
    SerialUWB1.write("les\n");  // Explicitly send "les" + newline (Enter)
    SerialUWB2.write("les\n");
    Serial.println("Sent 'les + Enter' to both UWBs");
    commandSent = true;
  }

  // Read responses
  if (SerialUWB1.available()) {
    Serial.print("[UWB1] ");
    Serial.println(SerialUWB1.readStringUntil('\n'));
  }
  if (SerialUWB2.available()) {
    Serial.print("[UWB2] ");
    Serial.println(SerialUWB2.readStringUntil('\n'));
  }

  // Optional: Allow sending new commands from Serial monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    SerialUWB1.println(input);  // Automatically adds \n
    SerialUWB2.println(input);
    Serial.println("Sent command to both UWBs: " + input);
  }
}
