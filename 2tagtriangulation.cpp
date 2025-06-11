#include <HardwareSerial.h>
#include <math.h>

// UART Definitions
HardwareSerial SerialUWB1(PE7, PE8);  // COM8
HardwareSerial SerialUWB2(PC7, PC6);  // COM9

// Triangle structure
struct Triangle {
  float d1 = 0; // COM8 to COM11
  float d2 = 0; // COM9 to COM11
  float d3 = 0.3; // COM8 to COM9 (fixed base length)
  float angle_top = 0, angle_left = 0, angle_right = 0;
  float x = 0;  // COM11 X coordinate (calculated)
  float y = 0;  // COM11 Y coordinate (calculated)
};

Triangle uwb_triangle;
bool d1_updated = false;
bool d2_updated = false;

unsigned long lastRequestTime = 0;
const unsigned long requestInterval = 1000; // ms

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);

  Serial.println("=== UWB TRIANGULATION INITIALIZED ===");
}

void loop() {
  // Send periodic requests
  if (millis() - lastRequestTime >= requestInterval) {
    requestDistances();
    lastRequestTime = millis();
  }

  processIncomingData();

  // Only calculate and display if both distances are updated
  if (d1_updated && d2_updated) {
    calculateTriangle();
    calculateXYPosition();
    displayResults();
    d1_updated = false;
    d2_updated = false;
  }
}

void requestDistances() {
  SerialUWB1.write("les\n");
  SerialUWB2.write("les\n");
}

void processIncomingData() {
  static String buffer1 = "", buffer2 = "";

  // COM8 -> COM11
  while (SerialUWB1.available()) {
    char c = SerialUWB1.read();
    if (c == '\n') {
      if (buffer1.length() > 0) {
        float parsed = parseDistance(buffer1);
        if (parsed > 0.0) {
          uwb_triangle.d1 = parsed;
          d1_updated = true;
        }
        buffer1 = "";
      }
    } else {
      buffer1 += c;
    }
  }

  // COM9 -> COM11
  while (SerialUWB2.available()) {
    char c = SerialUWB2.read();
    if (c == '\n') {
      if (buffer2.length() > 0) {
        float parsed = parseDistance(buffer2);
        if (parsed > 0.0) {
          uwb_triangle.d2 = parsed;
          d2_updated = true;
        }
        buffer2 = "";
      }
    } else {
      buffer2 += c;
    }
  }
}

float parseDistance(String &data) {
  int equalsIndex = data.indexOf('=');
  if (equalsIndex != -1 && equalsIndex < data.length() - 1) {
    String distStr = data.substring(equalsIndex + 1);
    return distStr.toFloat();
  }
  return 0;
}

void calculateTriangle() {
  float a = uwb_triangle.d1;
  float b = uwb_triangle.d2;
  float c = uwb_triangle.d3;

  if (a + b > c && a + c > b && b + c > a) {
    uwb_triangle.angle_top = acos((a * a + b * b - c * c) / (2 * a * b)) * 180.0 / PI;
    uwb_triangle.angle_left = acos((a * a + c * c - b * b) / (2 * a * c)) * 180.0 / PI;
    uwb_triangle.angle_right = 180.0 - uwb_triangle.angle_top - uwb_triangle.angle_left;
  } else {
    uwb_triangle.angle_top = 0;
    uwb_triangle.angle_left = 0;
    uwb_triangle.angle_right = 0;
    Serial.println("Invalid triangle configuration.");
  }
}

void calculateXYPosition() {
  // Tag positions
  float x1 = 0.0, y1 = 0.0;   // COM8
  float x2 = 0.3, y2 = 0.0;   // COM9

  float r1 = uwb_triangle.d1;  // COM11 to COM8
  float r2 = uwb_triangle.d2;  // COM11 to COM9
  float d = uwb_triangle.d3;   // COM8 to COM9

  // Compute unit vector from COM8 to COM9
  float exx = (x2 - x1) / d;
  float exy = (y2 - y1) / d;

  // Compute scalar projection
  float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);

  // Height from base to COM11
  float h_squared = r1 * r1 - a * a;

  if (h_squared < 0) {
    uwb_triangle.x = 0;
    uwb_triangle.y = 0;
    return; // Invalid geometry
  }

  float h = sqrt(h_squared);

  // Perpendicular vector
  float ppx = -exy;
  float ppy = exx;

  // Solution 1: Upper side of the base
  float x_top = x1 + a * exx + h * ppx;
  float y_top = y1 + a * exy + h * ppy;

  // Solution 2: Lower side of the base
  float x_bot = x1 + a * exx - h * ppx;
  float y_bot = y1 + a * exy - h * ppy;

  // ✅ Choose the solution that keeps continuity or allows negative coordinates
  // Example logic: choose the one with positive y if anchor is in front of vehicle
  // You can also use `y_bot` if expecting below

  // To allow negative Y when COM11 goes below the base
  // let's assume y_top is above and y_bot is below
  uwb_triangle.x = x_top;
  uwb_triangle.y = y_top;

  // OPTIONAL: uncomment below to use the one below base instead
  // uwb_triangle.x = x_bot;
  // uwb_triangle.y = y_bot;

  // OPTIONAL: automatically pick lower y if y_bot < 0
  // uwb_triangle.x = (y_bot < y_top) ? x_bot : x_top;
  // uwb_triangle.y = (y_bot < y_top) ? y_bot : y_top;
}


void displayResults() {
  Serial.write(27); Serial.print("[2J");
  Serial.write(27); Serial.print("[H");

  Serial.println("=== UWB TRIANGULATION SYSTEM ===");
  Serial.println("         Real-Time View");
  Serial.println("--------------------------------");

  // Distances
  Serial.println("DISTANCES (in meters):");
  Serial.print("  COM8 <-> COM11:  ");
  Serial.println(uwb_triangle.d1, 3);
  Serial.print("  COM9 <-> COM11:  ");
  Serial.println(uwb_triangle.d2, 3);
  Serial.print("  COM8 <-> COM9  :  ");
  Serial.println(uwb_triangle.d3, 3);

  Serial.println();

  // Angles
  Serial.println("ANGLES (in degrees):");
  Serial.print("  At COM11 (Top):     ");
  Serial.println(uwb_triangle.angle_top, 2);
  Serial.print("  At COM8 (Left):     ");
  Serial.println(uwb_triangle.angle_left, 2);
  Serial.print("  At COM9 (Right):    ");
  Serial.println(uwb_triangle.angle_right, 2);

  Serial.println();

  // Position
  Serial.println("COM11 ESTIMATED POSITION:");
  Serial.print("  X: "); Serial.println(uwb_triangle.x, 3);
  Serial.print("  Y: "); Serial.println(uwb_triangle.y, 3);

  Serial.println();

  // Triangle classification
  Serial.println("TRIANGLE TYPE:");
  if (uwb_triangle.angle_top > 90 || uwb_triangle.angle_left > 90 || uwb_triangle.angle_right > 90) {
    Serial.println("  Type: Obtuse");
  } else if (fabs(uwb_triangle.angle_top - 90) < 0.1 ||
             fabs(uwb_triangle.angle_left - 90) < 0.1 ||
             fabs(uwb_triangle.angle_right - 90) < 0.1) {
    Serial.println("  Type: Right");
  } else {
    Serial.println("  Type: Acute");
  }

  Serial.println("--------------------------------");
  Serial.println("Waiting for next measurement...");
}
