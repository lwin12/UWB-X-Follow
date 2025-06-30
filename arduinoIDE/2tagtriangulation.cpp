#include <HardwareSerial.h> // support multiple serial pots
#include <math.h> // coords, angle and distance calculation

// UART Definitions
HardwareSerial SerialUWB1(PE7, PE8);  // COM8
HardwareSerial SerialUWB2(PC7, PC6);  // COM9
// these are just the uart pins i am utilising, check pinout for more options

// Triangle structure
struct Triangle {
  float d1 = 0; // COM8 to COM11
  float d2 = 0; // COM9 to COM11
  float d3 = 0.3; // COM8 to COM9 (fixed base length)
  float angle_top = 0, angle_left = 0, angle_right = 0;
  float x = 0;  // COM11 X coordinate (calculated)
  float y = 0;  // COM11 Y coordinate (calculated)
  // all this is just initial setting for the float value to exist
};

Triangle uwb_triangle; // setting flags for indication of new data
bool d1_updated = false;
bool d2_updated = false;

unsigned long lastRequestTime = 0; // tracks the time that has passed since last request - later connects to millis
const unsigned long requestInterval = 1000; // how long between each data send

void setup() {
  Serial.begin(115200);
  while (!Serial); // while serial is not yet ready, do nothing (empty block)

  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);

  Serial.println("=== UWB TRIANGULATION INITIALIZED ===");
}

void loop() {
  // Send periodic requests
  if (millis() - lastRequestTime >= requestInterval) { // if LHS goes above the defined interval, LHS >= RHS, it will send a request, and then update last request time
    requestDistances();
    lastRequestTime = millis();
  }

  processIncomingData(); // function that read and parse data from uwb tags com8 com9

  // Only calculate and display if both distances are updated i.e. both are true
  if (d1_updated && d2_updated) {
    calculateTriangle();
    calculateXYPosition();
    displayResults();
    // triggers functions that are defined below
    d1_updated = false;
    d2_updated = false; // reset both boolean values to false
  }
}

void requestDistances() {
  SerialUWB1.write("les\n");
  SerialUWB2.write("les\n");
} // \n marks the end of command, apparently most serial-based UWB modules require and expect it

void processIncomingData() {
  static String buffer1 = "", buffer2 = ""; // static will allow buffer1 and 2 to retain their values, and not reset back to an empty string every function run (accummulate serial data over time)

  // COM8 -> COM11
  while (SerialUWB1.available()) { // this section is to accommodate serial data not arriving all at once
    char c = SerialUWB1.read();
    if (c == '\n') { // only when it is at the end (newline)
      if (buffer1.length() > 0) { // if the extracted text is longer than 0 characters
        float parsed = parseDistance(buffer1); // extracts the numerical value of the uwb output
        if (parsed > 0.0) { // if the value the uwb passes to us is more than 0.0, consider updated
          uwb_triangle.d1 = parsed; // updates distance 1 as 'parsed'
          d1_updated = true;
        }
        buffer1 = "";
      }
    } else {
      buffer1 += c; // this reads every character one by one, until it reaches \n which signals the end of the line, so the full string in buffer 1 could be 'dist=1.234'
    } // it then gets passed to parseDistance to extract only the numerical value which gets stored in uwb_triangle.d1, setting update to true
  }// buffer1 then gets reset to an empty string at the end

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
} // same as above

float parseDistance(String &data) {
  int equalsIndex = data.indexOf('='); // searches for the character position of = in the string
  if (equalsIndex != -1 && equalsIndex < data.length() - 1) { // if '=' not found, dataindexOf() will return -1, so this checks that = has been found. the second condition prevents trigger on empty string, because if it is empty, then data.length() - 1 < equalsIndex
    String distStr = data.substring(equalsIndex + 1); // extracts  the substring after the '='
    return distStr.toFloat(); // converts to float
  }
  return 0; // if no float can be sent / no '=' was found, then it returns a 0
}

void calculateTriangle() {
  float a = uwb_triangle.d1;
  float b = uwb_triangle.d2;
  float c = uwb_triangle.d3; // so far I've set d3=0.3 (30cm)

  if (a + b > c && a + c > b && b + c > a) { // checks for a valid triangle (sum of 2 sides must always be greater than the last side)
    uwb_triangle.angle_top = acos((a * a + b * b - c * c) / (2 * a * b)) * 180.0 / PI;
    uwb_triangle.angle_left = acos((a * a + c * c - b * b) / (2 * a * c)) * 180.0 / PI;
    uwb_triangle.angle_right = 180.0 - uwb_triangle.angle_top - uwb_triangle.angle_left; // cos rule calcs
  } else {
    uwb_triangle.angle_top = 0;
    uwb_triangle.angle_left = 0;
    uwb_triangle.angle_right = 0;
    Serial.println("Invalid triangle configuration."); // in case serial data results in an invalid triangle
  }
}

void calculateXYPosition() {
  // Tag positions
  float x1 = 0.0, y1 = 0.0;   // COM8 (left)
  float x2 = 0.3, y2 = 0.0;   // COM9 (right)

  float r1 = uwb_triangle.d1;  // COM11 to COM8
  float r2 = uwb_triangle.d2;  // COM11 to COM9
  float d = uwb_triangle.d3;   // COM8 to COM9

  // Compute unit vector from COM8 to COM9
  float exx = (x2 - x1) / d;
  float exy = (y2 - y1) / d;

  // Compute scalar projection
  float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d); // intersection of 2 circles centered at com8 and com9 with radius r1 and r2, solving for x gives x coord of com11, or projection of r1 on baseline

  // Height from base to COM11
  float h_squared = r1 * r1 - a * a; // pythagoras' theorem

  if (h_squared < 0) { // check for negative height, but may have to change if negative coordinates are implemented
    uwb_triangle.x = 0;
    uwb_triangle.y = 0;
    return; // Invalid geometry
  }

  float h = sqrt(h_squared);

  // Perpendicular vector , rotating the unit vector 90 degrees ACW
  float ppx = -exy;
  float ppy = exx;

  // Solution 1: Upper side of the base
  float x_top = x1 + a * exx + h * ppx; // starts from COM8's x1, + projection of r1 on baseline * unit vector, + baseline deviation IF the baseline isnt perfectly horizontal, but in this case it is, so the last part is always = 0
  float y_top = y1 + a * exy + h * ppy; // same thing, the last term to us will alwyas be 0 unless we define the baseline between COM8 and COM9 to be non-horizontal

  // Solution 2: Lower side of the base
  float x_bot = x1 + a * exx - h * ppx;
  float y_bot = y1 + a * exy - h * ppy;

    // === DYNAMIC SELECTION ===
  // Option A: Use whichever point is farther from the baseline (more negative or more positive Y)
  // This lets COM11 float naturally above or below the baseline
  if (abs(y_bot) > abs(y_top)) {

    uwb_triangle.x = x_bot;
    uwb_triangle.y = y_bot;
  } else {
    uwb_triangle.x = x_top;
    uwb_triangle.y = y_top;
  }

  // Option B: If you prefer a "flip" only when the point is below baseline (y < 0), use this instead:
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
