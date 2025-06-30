#include <HardwareSerial.h> // support multiple serial ports
#include <math.h> // coords, angle and distance calculation
#include <std_msgs.h> // Assuming we're using ROS Serial

// UART Definitions
HardwareSerial SerialUWB1(PE7, PE8);  // COM8
HardwareSerial SerialUWB2(PC7, PC6);  // COM9
HardwareSerial SerialUWB3(PD6, PD5);  // COM10 // these are just the uart pins i am utilising, check pinout for more options

// Triangle structure
struct Triangle {
  float d1 = 0; // COM8 to COM11
  float d2 = 0; // COM9 to COM11
  float d3 = 0; // COM10 to COM11
  float angle_top = 0, angle_left = 0, angle_right = 0;
  float x = 0;  // COM11 X coordinate (calculated)
  float y = 0;  // COM11 Y coordinate (calculated)
  // all this is just initial setting for the float value to exist
};

Triangle uwb_triangle; // setting flags for indication of new data
bool d1_updated = false;
bool d2_updated = false;
bool d3_updated = false; // new flag

unsigned long lastRequestTime = 0; // tracks the time that has passed since last request - later connects to millis
const unsigned long requestInterval = 1000; // how long between each data send

// FollowMe Implementation
float user_distance_from_baseline = 0.0;
bool isFollowMode = false;     // For RC vs UWB
bool facingDone = false;       // Whether we've already rotated to face user
bool isoscelesAchieved = false;

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
  if (millis() - lastRequestTime >= requestInterval) { // if LHS goes above the defined interval, LHS >= RHS, it will send a request, and then update last request time
    requestDistances();
    lastRequestTime = millis();
  }

  processIncomingData(); // function that read and parse data from uwb tags com8 com9

  // Only calculate and display if all distances are updated i.e. all are true
  if (d1_updated && d2_updated && d3_updated) {
    calculateTriangle(); // for baseline angle comparison
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
    runFollowLogic();
  } else {
    sendToJetson_RC();  // Optional: Reset or idle command to Jetson, maybe print something?
  }
}

void requestDistances() {
  SerialUWB1.write("les\n");
  SerialUWB2.write("les\n");
  SerialUWB3.write("les\n");
} // \n marks the end of command, apparently most serial-based UWB modules require and expect it

void processIncomingData() {
  static String buffer1 = "", buffer2 = "", buffer3 = ""; // static will allow buffer1, 2 and 3 to retain their values, and not reset back to an empty string every function run (accumulate serial data over time)

  // COM8 -> COM11
  while (SerialUWB1.available()) {
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

  // COM10 -> COM11
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
  int equalsIndex = data.indexOf('='); // searches for the character position of = in the string
  if (equalsIndex != -1 && equalsIndex < data.length() - 1) { // if '=' not found, dataindexOf() will return -1, so this checks that = has been found. the second condition prevents trigger on empty string, because if it is empty, then data.length() - 1 < equalsIndex
    String distStr = data.substring(equalsIndex + 1);// extracts  the substring after the '='
    return distStr.toFloat();// converts to float
  }
  return 0; // if no float can be sent / no '=' was found, then it returns a 0
}

void calculateTriangle() { // this uses the triangle of COM8, COM9 and COM11, I'm keeping this around for shoulder angle isosceles triangle for robot facing, turning and strafing
  float a = uwb_triangle.d1;
  float b = uwb_triangle.d2;
  float c = 0.18; // distance between COM8 and COM9 baseline

  if (a + b > c && a + c > b && b + c > a) {
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
  float x1 = -0.09, y1 = 0.0;   // COM8
  float x2 = 0.09, y2 = 0.0;    // COM9
  float x3 = 0.0,  y3 = 0.45;   // COM10

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

// this is for COM9 AND COM10 Simultaneous eqn
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

/*
// Distance from COM10 to COM11 (user to robot) - this is just to calculate the distance from COM10 to COM11 based on the calculated coordinates of COM11, right now i wont use this
float dx = uwb_triangle.x - 0.0;   // COM10 x = 0
float dy = uwb_triangle.y - 0.45;  // COM10 y = 0.45
float user_to_robot = sqrt(dx * dx + dy * dy);
*/

// printing of all results
void displayResults() {
  Serial.write(27); Serial.print("[2J");
  Serial.write(27); Serial.print("[H");

  Serial.println("=== UWB TRIANGULATION SYSTEM ===");
  Serial.println("         Real-Time View");
  Serial.println("--------------------------------");

  // Distances (i'm just going to take out the non-crucial ones)
  //Serial.println("DISTANCES (in meters):");
  //Serial.print("  COM8 <-> COM11:  ");
 // Serial.println(uwb_triangle.d1, 3);
  //Serial.print("  COM9 <-> COM11:  ");
  //Serial.println(uwb_triangle.d2, 3);
 // Serial.print("  COM10 <-> COM11: ");
  //Serial.println(uwb_triangle.d3, 3); // actually if we're using distance from baseline to user instead, we won't need this
  //Serial.print("  COM8 <-> COM9  :  ");
 // Serial.println(0.18, 3);

  Serial.println();

  // Angles for 8/9/11 triangle (only require baseline angles)
  Serial.println("ANGLES (in degrees):");
  //Serial.print("  At COM11 (Top):     ");
  //Serial.println(uwb_triangle.angle_top, 2);
  Serial.print("  At COM8 (Left):     ");
  Serial.println(uwb_triangle.angle_left, 2);
  Serial.print("  At COM9 (Right):    ");
  Serial.println(uwb_triangle.angle_right, 2);

  Serial.println();

  // Position
  Serial.println("COM11 ESTIMATED POSITION:");
  Serial.print("  X: "); Serial.println(uwb_triangle.x, 3); // prints to 3dp
  Serial.print("  Y: "); Serial.println(uwb_triangle.y, 3);

  Serial.println();

 /*
  // Triangle classification (honestly not necessary, may need for debugging)
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
 */

  Serial.println("--------------------------------");
  Serial.println("Waiting for next measurement...");
}

// FollowMe Logic
void runFollowLogic() {
  float mid_x = 0.0; // midpoint of COM8 and COM9
  float mid_y = 0.0;

  float dx = uwb_triangle.x - mid_x;
  float dy = uwb_triangle.y - mid_y;
  user_distance_from_baseline = sqrt(dx * dx + dy * dy); // so we're using the distance from the mdpt. of baseline to the user

  // 1. Check if user is behind (y < 0)
  if (!facingDone && uwb_triangle.y < 0) {
    sendToJetson_Twist(0.0, 0.0, -0.6); // Turn right
    return;
  } else {
    facingDone = true;
  }

  // 2. Rotate to achieve isosceles (angles ~ 86-89)
  if (!isoscelesAchieved) {
    if (uwb_triangle.angle_left > 89 && uwb_triangle.angle_right < 86) { // basically if user is standing on the left of robot
      sendToJetson_Twist(0.0, 0.0, 0.6); // Turn left
      return;
    } else if (uwb_triangle.angle_left < 86 && uwb_triangle.angle_right > 89) {
      sendToJetson_Twist(0.0, 0.0, -0.6); // Turn right
      return;
    } else {
      isoscelesAchieved = true;
    }
  }

  // 3. Forward movement if distance > 3m
  if (user_distance_from_baseline > 3.0) {
    sendToJetson_Twist(0.6, 0.0, 0.0); // Move forward
  } else {
    sendToJetson_Twist(0.0, 0.0, 0.0); // Stop
  }
}

void sendToJetson_RC() {
  // Used when switching to manual RC control; robot should not follow UWB commands
  sendToJetson_Twist(0.0, 0.0, 0.0); // this just ensures that in RC mode, velocity command is always stop
}


// --- MOCK ROS MESSAGE PUBLISHING FUNCTION for debugging ---
void sendToJetson_Twist(float x, float y, float z) {
  Serial.print("ROS2 CMD: linear(x="); Serial.print(x);
  Serial.print(", y="); Serial.print(y);
  Serial.print(") angular(z="); Serial.print(z);
  Serial.println(")");

  // Replace below with actual ROS Serial publishing logic if integrated
  // geometry_msgs::Twist msg;
  // msg.linear.x = x;
  // msg.linear.y = y;
  // msg.angular.z = z;
  // ros_pub.publish(&msg);
}
