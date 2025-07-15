#include <HardwareSerial.h> // support multiple serial ports
#include <math.h> // coords, angle and distance calculation

// UART Definitions
HardwareSerial SerialUWB1(PE7, PE8);  // COM8
HardwareSerial SerialUWB2(PC7, PC6);  // COM9
HardwareSerial SerialUWB3(PD6, PD5);  // COM11

// Anchor Tag Positions (in meters)
const float COM8_X = -0.25;
const float COM8_Y =  0.00;
const float COM9_X =  0.25;
const float COM9_Y =  0.00;
const float COM11_X =  0.00;
const float COM11_Y = -0.55;

bool d1_updated = false;
bool d2_updated = false;
bool d3_updated = false; // new flag

unsigned long lastRequestTime = 0; // tracks the time that has passed since last request - later connects to millis
static unsigned long lastDataTime = 0; // Send periodic requests
const unsigned long requestInterval = 2000; // how long between each data send

// FollowMe Implementation
bool isFollowMode = false;     // For RC vs UWB

struct AnchorTag {
  float distance;
};

struct Position {
  float x;
  float y;
};

AnchorTag COM8, COM9, COM11;
Position user_position_COM18;

//button and led system
#define BUTTON_PIN PB13
bool lastButtonState = HIGH;
#define LED_PIN PB12

// buzzer system
#define BUZZER_PIN PA0 
bool buzzerOverridden = false; 

enum BuzzerTone {
  NONE,
  UWB_FAILURE,
  OUT_OF_RANGE,
  OBSTACLE,
  HEARTBEAT
};

BuzzerTone currentTone = NONE;
unsigned long buzzerTimer = 0;
bool buzzerState = false;
int buzzerStep = 0;

// obstacle detection
bool isObstacleDetected = false;


void setup() {
  Serial.begin(115200);
  while (!Serial); // while serial is not yet ready, do nothing (empty block)

  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);
  SerialUWB3.begin(115200);

  // Serial.println("=== UWB TRIANGULATION INITIALIZED ===");
  delay(1000); 

  // Wake sequence for COM8, 9, 11 - activate shell mode
  for (int i = 0; i < 2; i++) {
    SerialUWB1.write(0x0d ); delay(100);

    SerialUWB2.write(0x0d); delay (100);

    SerialUWB3.write(0x0d); delay(100);
  }

  delay(1000);

  SerialUWB1.write("les\n");
  delay(500);
  SerialUWB2.write("les\n");
   delay(500);
  SerialUWB3.write("les\n");
  delay(500);

  // Serial.println("Started continuous distance streaming via 'les'");

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // <-- Add this line

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // off by default

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

unsigned long lastHeartbeatTime = 0;

void loop() {
  checkForJetsonCommand();  // This may activate tone 4

  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    isFollowMode = !isFollowMode;
    digitalWrite(LED_PIN, isFollowMode ? HIGH : LOW);
    resetAndInitializeUWB();
    delay(300);  // debounce
  }
  lastButtonState = currentButtonState;

  buzzerOverridden = false;  // Reset tone flag

  if (isFollowMode) {
    processIncomingData();

    // Highest priority: UWB failure
    if ((d1_updated || d2_updated || d3_updated) && millis() - lastDataTime > 2000) {
      currentTone = UWB_FAILURE;
      buzzerOverridden = true;
      resetAndInitializeUWB();
      d1_updated = d2_updated = d3_updated = false;
      return;
    }

    // Priority 2: Obstacle
    if (isObstacleDetected) {
      currentTone = OBSTACLE;
      buzzerOverridden = true;
    }

    // Priority 3: Out-of-range
    if (!buzzerOverridden && d1_updated && d2_updated && d3_updated) {
      calculateXYPosition();
      float dist = sqrt(user_position_COM18.x * user_position_COM18.x + user_position_COM18.y * user_position_COM18.y);
      if (dist >= 10.0) {
        currentTone = OUT_OF_RANGE;
        buzzerOverridden = true;
      }
      displayResults();
      generateFollowCommand();
      d1_updated = d2_updated = d3_updated = false;
    }

    if (!buzzerOverridden && currentTone == NONE && millis() - lastHeartbeatTime > 1000) {
      currentTone = HEARTBEAT;
      lastHeartbeatTime = millis();
    }
  }

  static BuzzerTone lastTone = NONE;
  if (currentTone != lastTone) {
    buzzerStep = 0;
    buzzerState = false;
    buzzerTimer = millis();
    lastTone = currentTone;
  }
  
  handleBuzzer();
}

void processIncomingData() {
  static String buffer1 = "", buffer2 = "", buffer3 = ""; // static will allow buffer1, 2 and 3 to retain their values, and not reset back to an empty string every function run (accumulate serial data over time)

  // COM8 -> COM18
  while (SerialUWB1.available()) {
    char c = SerialUWB1.read();
    if (c == '\n') { // only when it is at the end (newline)

      
      // Serial.print("Full line: ");
      // Serial.println(buffer1);
      

      if (buffer1.length() > 0) { // if the extracted text is longer than 0 characters
        float parsed = parseDistance(buffer1); // extracts the numerical value of the uwb output
        if (parsed > 0.0) { // if the value the uwb passes to us is more than 0.0, consider updated

          // Serial.print("Parsed from buffer1: ");
          // Serial.println(parsed);         

          COM8.distance = parsed; // updates distance 1 as 'parsed'
          d1_updated = true;

          // Serial.println("d1_updated = true");
        }
        lastDataTime = millis(); // refresh last successful data time
        buffer1 = "";
      }
    } else {
      buffer1 += c; // this reads every character one by one, until it reaches \n which signals the end of the line, so the full string in buffer 1 could be 'dist=1.234'
    } // it then gets passed to parseDistance to extract only the numerical value which gets stored in COM8.distance, setting update to true
  }// buffer1 then gets reset to an empty string at the end

  // COM9 -> COM18
  while (SerialUWB2.available()) {
    char c = SerialUWB2.read();
    if (c == '\n') {

      // Serial.print("Full line: ");
      // Serial.println(buffer2);

      if (buffer2.length() > 0) {
        float parsed = parseDistance(buffer2);
        if (parsed > 0.0) {

          // Serial.print("Parsed from buffer2: ");
          // Serial.println(parsed);

          COM9.distance = parsed;
          d2_updated = true;

          // Serial.println("d2_updated = true");
        }
        lastDataTime = millis(); // refresh last successful data time
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

      // Serial.print("Full line: ");
      // Serial.println(buffer3);

      if (buffer3.length() > 0) {
        float parsed = parseDistance(buffer3);
        if (parsed > 0.0) {

          // Serial.print("Parsed from buffer3: ");
          // Serial.println(parsed);
          COM11.distance = parsed;
          d3_updated = true;

          // Serial.println("d3_updated = true");
        }
        lastDataTime = millis(); // refresh last successful data time
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
  float x1 = COM8_X,  y1 = COM8_Y;
  float x2 = COM9_X,  y2 = COM9_Y;
  float x3 = COM11_X, y3 = COM11_Y; // update accordingly

  float r1 = COM8.distance;
  float r2 = COM9.distance;
  float r3 = COM11.distance;

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
    user_position_COM18.x = 0;
    user_position_COM18.y = 0;
    // Serial.println("Trilateration failed (denominator too small - parallel lines detected)");
    return; // if magnitude of denominator is too small, the lines are parallel and there will be no unique solution
  }
// Cramer's rule, which solves the x and y in a 2x2 linear system
  user_position_COM18.x = (C * E - F * B) / denominator;
  user_position_COM18.y = (A * F - D * C) / denominator;
}

//heading / bearing calculation
float calculateHeading() {
  float dx = user_position_COM18.x - 0.0;
  float dy = user_position_COM18.y - 0.0;
  float heading_rad = atan2(dx, dy); // Note: dx and dy reversed for 0° = forward
  float heading_deg = heading_rad * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;
  return heading_deg;
}

// printing of all results
void displayResults() {
  /*
  Serial.write(27); Serial.print("[2J");
  Serial.write(27); Serial.print("[H");
  */ //this section is the ANSI code to clear the terminal and set the cursor to (0,0) for a stable data update fashion, but not all terminals can accept ANSI code

  String action; 

/*
  Serial.println("\n-----------------------------\n");
  Serial.println("=== UWB TRIANGULATION SYSTEM ===");
  Serial.println("--------------------------------");

  // Distances to check for uwb functionality (i'm just going to take out the non-crucial ones)
  Serial.println("DISTANCES (in meters):");
  Serial.print("  COM8 <-> COM18 (User):  ");
  Serial.println(COM8.distance, 3);
  Serial.print("  COM9 <-> COM18 (User):  ");
  Serial.println(COM9.distance, 3);
  Serial.print("  COM11 <-> COM18 (User): ");
  Serial.println(COM11.distance, 3); // actually if we're using distance from baseline to user instead, we won't need this, just keep in for debug


  Serial.println();

  // Position
  Serial.println("COM18 (User) Estimated Position:");
  Serial.print("  X: "); Serial.println(user_position_COM18.x, 3); // prints to 3dp
  Serial.print("  Y: "); Serial.println(user_position_COM18.y, 3);
  */


  // Show current required action from calculated heading & distance
  float heading = calculateHeading();
  float distance = sqrt(user_position_COM18.x * user_position_COM18.x + user_position_COM18.y * user_position_COM18.y);
  /*
  float x_cmd = (distance > 2.0) ? 1.0 : 0.0;
  float z_cmd = 0.0;
  if (!(heading >= 340 || heading <= 20)) {
    z_cmd = (heading > 20 && heading <= 180) ? 0.8 : -0.9;
  }
  float y_cmd = 0.0;
  */

  /*
  Serial.print("Heading: ");
  Serial.print(heading, 1);
  Serial.println("°");
  */


  action = getActionFromHeadingAndDistance(heading, distance);

  Serial.print("  Required Action: ");
  Serial.println(action);
}

void generateFollowCommand() {
  if (!(d1_updated && d2_updated && d3_updated)) {
    sendToJetson_Action("hold");  // Send hold if incomplete data
    return;
  }
  float heading = calculateHeading();
  float distance = sqrt(user_position_COM18.x * user_position_COM18.x + user_position_COM18.y * user_position_COM18.y);

  String action;

  action = getActionFromHeadingAndDistance(heading, distance);

  /*
  // Debug info
  Serial.print("Heading: ");
  Serial.print(heading, 1);
  Serial.print("°, Distance: ");
  Serial.println(distance, 2);
  */

  sendToJetson_Action(action);  // Call the updated string-based function
}


void sendToJetson_Action(String action) {
  // Serial.print("Required action: ");
  Serial.println(action);
}

String getActionFromHeadingAndDistance(float heading, float distance) {
  if (heading > 20 && heading <= 180) return "turn_right";
  if (heading > 180 && heading < 340) return "turn_left";
  if (distance > 2.5) return "forward";
  //if (distance < 1.0) return "backward";
  return "hold";
}

void resetAndInitializeUWB() {
  for (int i = 0; i < 2; i++) {
    SerialUWB1.write(0x0d); delay(100);
    SerialUWB2.write(0x0d); delay(100);
    SerialUWB3.write(0x0d); delay(100);
  }

  SerialUWB1.write("reset\n"); delay(500);
  SerialUWB2.write("reset\n"); delay(500);
  SerialUWB3.write("reset\n"); delay(500);

  for (int i = 0; i < 2; i++) {
    SerialUWB1.write(0x0d); delay(100);
    SerialUWB2.write(0x0d); delay(100);
    SerialUWB3.write(0x0d); delay(100);
  }

  SerialUWB1.write("les\n"); delay(500);
  SerialUWB2.write("les\n"); delay(500);
  SerialUWB3.write("les\n"); delay(500);
}

void handleBuzzer() {
  unsigned long now = millis();

  switch (currentTone) {

    case UWB_FAILURE:
      if (now - buzzerTimer >= (buzzerState ? 100 : 200)) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        buzzerTimer = now;
        if (!buzzerState) {
          buzzerStep++;
          if (buzzerStep >= 6) { // 3 beeps (ON/OFF pairs)
            currentTone = NONE;
            buzzerStep = 0;
          }
        }
      }
      break;

    case OUT_OF_RANGE:
      if (buzzerStep < 3) {
        if (now - buzzerTimer >= (buzzerState ? 100 : 200)) {
          buzzerState = !buzzerState;
          digitalWrite(BUZZER_PIN, buzzerState);
          buzzerTimer = now;
          if (!buzzerState) buzzerStep++;
        }
      } else if (buzzerStep == 3) {
        if (now - buzzerTimer >= 600) {
          digitalWrite(BUZZER_PIN, HIGH);
          buzzerTimer = now;
          buzzerStep++;
        }
      } else if (buzzerStep == 4) {
        if (now - buzzerTimer >= 600) {
          digitalWrite(BUZZER_PIN, LOW);
          buzzerTimer = now;
          buzzerStep++;
        }
      } else if (buzzerStep < 9) {
        if (now - buzzerTimer >= (buzzerState ? 100 : 200)) {
          buzzerState = !buzzerState;
          digitalWrite(BUZZER_PIN, buzzerState);
          buzzerTimer = now;
          if (!buzzerState) buzzerStep++;
        }
      } else {
        currentTone = NONE;
        buzzerStep = 0;
      }
      break;

    case OBSTACLE:
      digitalWrite(BUZZER_PIN, HIGH); // Constant tone
      break;

    case HEARTBEAT:
      if (now - buzzerTimer >= (buzzerState ? 80 : 400)) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        buzzerTimer = now;
      }
      break;

    case NONE:
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }
}

void checkForJetsonCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "estop_activated") {
      isFollowMode = false;
      digitalWrite(LED_PIN, LOW);
      resetAndInitializeUWB();
    } else if (cmd == "obstacle_detected") {
      isObstacleDetected = true;
    } else {
      isObstacleDetected = false;
    }
  }
}
