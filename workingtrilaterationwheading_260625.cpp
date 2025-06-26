#include <HardwareSerial.h> // support multiple serial ports
#include <math.h> // coords, angle and distance calculation

// UART Definitions
HardwareSerial SerialUWB1(PE7, PE8);  // COM8
HardwareSerial SerialUWB2(PC7, PC6);  // COM9
HardwareSerial SerialUWB3(PD6, PD5);  // COM11

// Triangle structure
struct Triangle {
  float d1 = 0; // COM8 to COM18
  float d2 = 0; // COM9 to COM18
  float d3 = 0; // COM11 to COM18
  float x = 0;  // COM18 X coordinate (calculated)
  float y = 0;  // COM18 Y coordinate (calculated)
  // all this is just initial setting for the float value to exist
};

Triangle uwb_triangle; // setting flags for indication of new data
bool d1_updated = false;
bool d2_updated = false;
bool d3_updated = false; // new flag

unsigned long lastRequestTime = 0; // tracks the time that has passed since last request - later connects to millis
const unsigned long requestInterval = 1000; // how long between each data send

// FollowMe Implementation
bool isFollowMode = false;     // For RC vs UWB

void setup() {
  Serial.begin(115200);
  while (!Serial); // while serial is not yet ready, do nothing (empty block)

  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);
  SerialUWB3.begin(115200);

  Serial.println("=== UWB TRIANGULATION INITIALIZED ===");

  pinMode(PB12, INPUT_PULLUP); // Physical toggle button to switch to FollowMe Mode
  pinMode(PA15, OUTPUT);       // LED indicator (Green/Blue) for visual cue on which mode we're in
}

void loop() {
  // Send periodic requests
  if (!d1_updated && !d2_updated && !d3_updated && millis() - lastRequestTime >= requestInterval) { // if LHS goes above the defined interval, LHS >= RHS, it will send a request, and then update last request time
    requestDistances();
    lastRequestTime = millis();
  }

  processIncomingData(); // function that read and parse data from uwb tags com8 com9

  // Only calculate and display if all distances are updated i.e. all are true
  if (d1_updated && d2_updated && d3_updated) {
    calculateXYPosition(); // use 3-point trilateration
    displayResults();
    d1_updated = false;
    d2_updated = false;
    d3_updated = false; // reset all boolean values to false
  }

    // Handle button toggle between RC and Follow Me
  if (digitalRead(PB12) == LOW) {  // Button pressed
    isFollowMode = !isFollowMode;  // Toggle
    delay(300);                    // Debounce
    digitalWrite(PA15, isFollowMode ? HIGH : LOW); // Blue if Follow mode
    if (isFollowMode) Serial.println("Switched to UWB Follow Mode");
    else Serial.println("Switched to RC Mode");
  }

  if (isFollowMode) {
  generateFollowCommand();
  } 
}

void requestDistances() {
  SerialUWB1.write("les\n");
  SerialUWB2.write("les\n");
  SerialUWB3.write("les\n");
} // \n marks the end of command, apparently most serial-based UWB modules require and expect it

void processIncomingData() {
  static String buffer1 = "", buffer2 = "", buffer3 = ""; // static will allow buffer1, 2 and 3 to retain their values, and not reset back to an empty string every function run (accumulate serial data over time)

  // COM8 -> COM18
  while (SerialUWB1.available()) {
    char c = SerialUWB1.read();
    if (c == '\n') { // only when it is at the end (newline)
      Serial.print("Full line: ");
      Serial.println(buffer1);

      if (buffer1.length() > 0) { // if the extracted text is longer than 0 characters
        float parsed = parseDistance(buffer1); // extracts the numerical value of the uwb output
        if (parsed > 0.0) { // if the value the uwb passes to us is more than 0.0, consider updated
          Serial.print("Parsed from buffer1: ");
          Serial.println(parsed);
          uwb_triangle.d1 = parsed; // updates distance 1 as 'parsed'
          d1_updated = true;
          Serial.println("d1_updated = true");
        }
        buffer1 = "";
      }
    } else {
      buffer1 += c; // this reads every character one by one, until it reaches \n which signals the end of the line, so the full string in buffer 1 could be 'dist=1.234'
    } // it then gets passed to parseDistance to extract only the numerical value which gets stored in uwb_triangle.d1, setting update to true
  }// buffer1 then gets reset to an empty string at the end

  // COM9 -> COM18
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

  // COM11 -> COM18
  while (SerialUWB3.available()) {
    char c = SerialUWB3.read();
    if (c == '\n') {
      if (buffer3.length() > 0) {
        float parsed = parseDistance(buffer3);
        if (parsed > 0.0) {
          uwb_triangle.d3 = parsed;
          d3_updated = true;
        }
        buffer3 = "";
      }
    } else {
      buffer3 += c;
    }
  }
}

float parseDistance(String &data) {
  data.trim();
  int equalsIndex = data.indexOf('='); // searches for the character position of = in the string
  if (equalsIndex != -1 && equalsIndex < data.length() - 1) { // if '=' not found, dataindexOf() will return -1, so this checks that = has been found. the second condition prevents trigger on empty string, because if it is empty, then data.length() - 1 < equalsIndex
    String distStr = data.substring(equalsIndex + 1);// extracts  the substring after the '='
    distStr.trim();
    float val = distStr.toFloat();
    if (val > 0.0 && val < 100.0) return val;  // simple sanity check
  }
  return 0; // if no float can be sent / no '=' was found, then it returns a 0
}

void calculateXYPosition() {
  float x1 = -0.20, y1 = 0.0;   // COM8 (updated)
  float x2 =  0.20, y2 = 0.0;   // COM9 (updated)
  float x3 =  0.0,  y3 = -0.45; // COM11 (updated)

  float r1 = uwb_triangle.d1;
  float r2 = uwb_triangle.d2;
  float r3 = uwb_triangle.d3;

// for circle equation.  (x - x₁)² + (y - y₁)² = r₁²  for each circle centered around a tag
// (x - x₂)² - (x - x₁)² + (y - y₂)² - (y - y₁)² = r₂² - r₁² - subtract one eqn from another
// simplifies to 2x(x₁ - x₂) + 2y(y₁ - y₂) = (r₁² - r₂² + x₂² - x₁² + y₂² - y₁²), Which can be expressed in this form - A·x + B·y = C
// If we observe the coefficients of coordinates, coefficient of x = 2(x1-x2)
// Let float A = 2 * (x₂ - x₁)
// this is for COM8 AND COM9 Simultaneous eqn
  float A = 2 * (x2 - x1); 
  float B = 2 * (y2 - y1);
  float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;

// this is for COM9 AND COM11 Simultaneous eqn
  float D = 2 * (x3 - x2);
  float E = 2 * (y3 - y2);
  float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3;

// use determinant of 2x2 matrix ad-bc
// A·x + B·y = C       --- equation (4)
// D·x + E·y = F       --- equation (5) --> can frm into matrix eqn
  float denominator = A * E - B * D; // this is just determinant of 2x2 matrix
  if (fabs(denominator) < 1e-6) {
    uwb_triangle.x = 0;
    uwb_triangle.y = 0;
    Serial.println("Trilateration failed (denominator too small)");
    return; // if magnitude of denominator is too small, the lines are parallel and there will be no unique solution
  }
// Cramer's rule, which solves the x and y in a 2x2 linear system
  uwb_triangle.x = (C * E - F * B) / denominator;
  uwb_triangle.y = (A * F - D * C) / denominator;
}

//heading / bearing calculation
float calculateHeading() {
  float dx = uwb_triangle.x - 0.0;
  float dy = uwb_triangle.y - 0.0;
  float heading_rad = atan2(dx, dy); // Note: dx and dy reversed for 0° = forward
  float heading_deg = heading_rad * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;
  return heading_deg;
}

// printing of all results
void displayResults() {
  Serial.write(27); Serial.print("[2J");
  Serial.write(27); Serial.print("[H");

  Serial.println("=== UWB TRIANGULATION SYSTEM ===");
  Serial.println("--------------------------------");

  // Distances to check for uwb functionality (i'm just going to take out the non-crucial ones)
  Serial.println("DISTANCES (in meters):");
  Serial.print("  COM8 <-> COM18:  ");
  Serial.println(uwb_triangle.d1, 3);
  Serial.print("  COM9 <-> COM18:  ");
  Serial.println(uwb_triangle.d2, 3);
  Serial.print("  COM11 <-> COM18: ");
  Serial.println(uwb_triangle.d3, 3); // actually if we're using distance from baseline to user instead, we won't need this
  Serial.print("  COM8 <-> COM9  :  "); // to confirm set coordinates of COM8 & COM9
  Serial.println(0.4, 3); // set this later

  Serial.println();

  // Position
  Serial.println("COM18 ESTIMATED POSITION:");
  Serial.print("  X: "); Serial.println(uwb_triangle.x, 3); // prints to 3dp
  Serial.print("  Y: "); Serial.println(uwb_triangle.y, 3);

  // Show current required action from calculated heading & distance
  float heading = calculateHeading();
  float distance = sqrt(uwb_triangle.x * uwb_triangle.x + uwb_triangle.y * uwb_triangle.y);
  float x_cmd = (distance > 2.0) ? 1.0 : 0.0;
  float z_cmd = 0.0;
  if (!(heading >= 340 || heading <= 20)) {
    z_cmd = (heading > 20 && heading <= 180) ? 0.8 : -0.9;
  }
  float y_cmd = 0.0;

  Serial.print("  Required Action: x=");
  Serial.print(x_cmd, 2);
  Serial.print(", y=");
  Serial.print(y_cmd, 2);
  Serial.print(", z=");
  Serial.println(z_cmd, 2);
}

void generateFollowCommand() {
  float heading = calculateHeading();
  float distance = sqrt(uwb_triangle.x * uwb_triangle.x + uwb_triangle.y * uwb_triangle.y);

  float x = 0.0; // forward/backward
  float y = 0.0; // strafe (unused for now)
  float z = 0.0; // rotation

  // Heading-based rotation
  if ((heading >= 340 || heading <= 20)) {
    z = 0.0;
  } else if (heading > 20 && heading <= 180) {
    z = 0.8; // turn left
  } else {
    z = -0.9; // turn right
  }

  // Distance-based forward motion
  if (distance > 2.0) {
    x = 1.0; // move forward
  } else {
    x = 0.0;
  }

  Serial.print("Heading: ");
  Serial.print(heading, 1);
  Serial.print("°, Distance: ");
  Serial.println(distance, 2);

  sendToJetson_Action(x, y, z);
}

void sendToJetson_Action(float x, float y, float z) {
  Serial.print("required action: x=");
  Serial.print(x, 2);
  Serial.print(", y=");
  Serial.print(y, 2);
  Serial.print(", z=");
  Serial.println(z, 2);
}


