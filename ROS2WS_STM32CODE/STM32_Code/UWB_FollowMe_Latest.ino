#include <HardwareSerial.h> // support multiple serial ports
#include <math.h> // coords, angle and distance calculation

#define BUTTON_INPUT A4
#define LED_OUTPUT   A5
#define BUZZER_OUTPUT A0

// System States
enum SystemState { IDLE, UWB_ACTIVE };
SystemState currentState = IDLE;

bool Button_Signal = false;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

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
bool d3_updated = false;

// Non-blocking timing variables
unsigned long lastUWBRequest = 0;
unsigned long lastDataTime = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastLEDUpdate = 0;
unsigned long initializationStartTime = 0;
unsigned long lastInitStep = 0;

enum ResetState { RESET_IDLE, RESET_STEP1, RESET_STEP2, RESET_STEP3, RESET_COMPLETE };
ResetState resetState = RESET_IDLE;
unsigned long resetLastStep = 0;
const unsigned long RESET_STEP_DELAY = 200; // 200ms delay between steps
bool resetInProgress = false;
bool stopRequested = false;


// Timing intervals
const unsigned long UWB_REQUEST_INTERVAL = 2000;
const unsigned long DATA_TIMEOUT = 3000;
const unsigned long BUTTON_CHECK_INTERVAL = 50;
const unsigned long LED_UPDATE_INTERVAL = 100;
const unsigned long INIT_STEP_DELAY = 100;

// Non-blocking buzzer variables
unsigned long buzzerStartTime = 0;
unsigned long buzzerLastToggle = 0;
int buzzerCurrentBeep = 0;
int buzzerTargetBeeps = 0;
int buzzerFrequency = 100;
int buzzerDowntime = 0;
bool buzzerActive = false;
bool buzzerState = false; // false = off, true = on
bool buzzerSpecialMode = false; // for the 5-beep LED flash mode

// Initialization state machine
enum InitState { INIT_NOT_STARTED, INIT_WAKE_1, INIT_WAKE_2, INIT_COMPLETE };
InitState initState = INIT_NOT_STARTED;
int initWakeStep = 0;

struct AnchorTag {
  float distance;
};

struct Position {
  float x;
  float y;
};

AnchorTag COM8, COM9, COM11;
Position user_position_COM18;

void setup() {
  pinMode(BUTTON_INPUT, INPUT_PULLDOWN);
  pinMode(LED_OUTPUT, OUTPUT);
  pinMode(BUZZER_OUTPUT, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial); // Wait for serial to be ready

  Serial.println("Hello World!");
  
  SerialUWB1.begin(115200);
  SerialUWB2.begin(115200);
  SerialUWB3.begin(115200);
  
  initializationStartTime = millis();
  initState = INIT_WAKE_1;
}

void loop() {
  // Always run these non-blocking functions
  updateButton();
  updateBuzzer();
  updateLED();
  updateReset();  // ADD THIS LINE
  processJetsonCommands();
  
  // Handle initialization if not complete
  if (initState != INIT_COMPLETE) {
    handleInitialization();
    return; // Don't process UWB until init is done
  }
  
  // State machine for main operation
  switch (currentState) {
    case IDLE:
      if (Button_Signal) {
        currentState = UWB_ACTIVE;
        lastUWBRequest = millis();
      }
      break;
      
    case UWB_ACTIVE:
      if (!Button_Signal) {
        currentState = IDLE;
      } else {
        processUWB();
      }
      break;
  }
}

void handleInitialization() {
  unsigned long currentTime = millis();
  
  switch (initState) {
    case INIT_WAKE_1:
      if (currentTime - lastInitStep >= INIT_STEP_DELAY) {
        switch (initWakeStep) {
          case 0:
            SerialUWB1.write(0x0d);
            break;
          case 1:
            SerialUWB2.write(0x0d);
            break;
          case 2:
            SerialUWB3.write(0x0d);
            break;
        }
        initWakeStep++;
        lastInitStep = currentTime;
        
        if (initWakeStep >= 3) {
          initWakeStep = 0;
          initState = INIT_WAKE_2;
        }
      }
      break;
      
    case INIT_WAKE_2:
      if (currentTime - lastInitStep >= INIT_STEP_DELAY) {
        switch (initWakeStep) {
          case 0:
            SerialUWB1.write(0x0d);
            break;
          case 1:
            SerialUWB2.write(0x0d);
            break;
          case 2:
            SerialUWB3.write(0x0d);
            break;
        }
        initWakeStep++;
        lastInitStep = currentTime;
        
        if (initWakeStep >= 3) {
          initState = INIT_COMPLETE;
          // Serial.println("UWB initialization complete");
        }
      }
      break;
  }
}

void updateButton() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastButtonCheck >= BUTTON_CHECK_INTERVAL) {
    static bool lastButtonState = false;
    bool currentButtonState = digitalRead(BUTTON_INPUT) == HIGH;
    
    if (currentButtonState && !lastButtonState) {
      // Button just pressed
      if (!Button_Signal) {
        // If OFF -> turn ON immediately
        Button_Signal = true;
        sendToJetson_Action("start");   // <--- send "start" once
      } else {
        // If ON -> request stop (but only stop after reset is done)
        stopRequested = true;
        reset();   // start sending "les" sequence
      }
    }
    
    lastButtonState = currentButtonState;
    lastButtonCheck = currentTime;
  }
}


void updateLED() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
    if (Button_Signal) {
      digitalWrite(LED_OUTPUT, HIGH);
    } else {
      digitalWrite(LED_OUTPUT, LOW);
    }
    lastLEDUpdate = currentTime;
  }
}

void startBuzzer(int nbeeps, int frequency, int downtime) {
  buzzerTargetBeeps = nbeeps;
  buzzerFrequency = frequency;
  buzzerDowntime = downtime;
  buzzerCurrentBeep = 0;
  buzzerActive = true;
  buzzerState = false;
  buzzerStartTime = millis();
  buzzerLastToggle = millis();
  
  // Check for special 5-beep mode
  buzzerSpecialMode = (nbeeps == 5 && frequency == 100 && downtime == 0);
}

void updateBuzzer() {
  if (!buzzerActive) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - buzzerLastToggle;
  
  if (!buzzerState) {
    // Currently off, check if time to turn on
    if (elapsed >= buzzerFrequency) {
      digitalWrite(BUZZER_OUTPUT, HIGH);
      if (buzzerSpecialMode) {
        digitalWrite(LED_OUTPUT, HIGH);
      }
      buzzerState = true;
      buzzerLastToggle = currentTime;
    }
  } else {
    // Currently on, check if time to turn off
    if (elapsed >= buzzerFrequency) {
      digitalWrite(BUZZER_OUTPUT, LOW);
      if (buzzerSpecialMode) {
        digitalWrite(LED_OUTPUT, LOW);
      }
      buzzerState = false;
      buzzerCurrentBeep++;
      buzzerLastToggle = currentTime;
      
      // Check if we've completed all beeps
      if (buzzerCurrentBeep >= buzzerTargetBeeps) {
        buzzerActive = false;
        // Apply downtime if it's the last beep (though your original code only applied downtime differently)
      }
    }
  }
}

void processUWB() {
  unsigned long currentTime = millis();
  
  // Check for data timeout (non-blocking)
  if ((d1_updated || d2_updated || d3_updated) && 
      currentTime - lastDataTime > DATA_TIMEOUT) {
    Serial.println("[Warning] Timeout: No complete set of distance data received. Resetting flags.");
    d1_updated = d2_updated = d3_updated = false;
  }
  
  // Process any available incoming data
  processIncomingData();
  
  // Check for extended timeout and reset if needed
  if (currentTime - lastUWBRequest > (DATA_TIMEOUT + 1000) && 
      (!d1_updated || !d2_updated || !d3_updated)) {
    Serial.println("[Warning] Still waiting for full UWB update set after extended time.");
    startBuzzer(10, 50, 0);
    reset();
    lastUWBRequest = currentTime; // Reset the timer
  }
  
  // Calculate position if all data is ready
  if (d1_updated && d2_updated && d3_updated) {
    calculateXYPosition();
    displayResults();
    generateFollowCommand();
    
    // Reset flags and timer
    d1_updated = d2_updated = d3_updated = false;
    lastUWBRequest = currentTime;
  }
  
  // Request new data periodically if needed
  if (currentTime - lastUWBRequest > UWB_REQUEST_INTERVAL && 
      (!d1_updated || !d2_updated || !d3_updated)) {
    // Could add periodic data requests here if needed
    // For now, the UWB modules should be streaming continuously
  }
}

void processJetsonCommands() {
  String jetsonCmd = GetJetsonData();
  
  if (jetsonCmd.length() > 0) {
    if (jetsonCmd == "estop") {
      Button_Signal = false;
      startBuzzer(5, 100, 0);
    } else if (jetsonCmd == "stop") {
      Button_Signal = false;
      startBuzzer(2, 100, 0);
    }
    else if(jetsonCmd == "reset")
    {
      NVIC_SystemReset();
    }
    // For "continue" or other commands, just keep running
  }
}

void reset() {
  if (!resetInProgress) {
    // Start the reset sequence
    resetState = RESET_STEP1;
    resetInProgress = true;
    resetLastStep = millis();
  }
}

void updateReset() {
  if (!resetInProgress) {
    // If reset finished and stop was requested, now turn off
    if (stopRequested) {
      Button_Signal = false;   // finally turn off
      stopRequested = false;
    }
    return;
  }
  
  unsigned long currentTime = millis();
  
  switch (resetState) {
    case RESET_STEP1:
      if (currentTime - resetLastStep >= 0) {
        SerialUWB1.write("les\n");
        resetState = RESET_STEP2;
        resetLastStep = currentTime;
      }
      break;
      
    case RESET_STEP2:
      if (currentTime - resetLastStep >= RESET_STEP_DELAY) {
        SerialUWB2.write("les\n");
        resetState = RESET_STEP3;
        resetLastStep = currentTime;
      }
      break;
      
    case RESET_STEP3:
      if (currentTime - resetLastStep >= RESET_STEP_DELAY) {
        SerialUWB3.write("les\n");
        resetState = RESET_COMPLETE;
        resetLastStep = currentTime;
      }
      break;
      
    case RESET_COMPLETE:
      resetInProgress = false;
      resetState = RESET_IDLE;
      break;
  }
}

void processIncomingData() {
  static String buffer1 = "", buffer2 = "", buffer3 = "";
  
  // COM8 -> COM18
  while (SerialUWB1.available()) {
    char c = SerialUWB1.read();
    if (c == '\n') {
      if (buffer1.length() > 0) {
        float parsed = parseDistance(buffer1);
        if (parsed > 0.0) {
          COM8.distance = parsed;
          d1_updated = true;
        }
        lastDataTime = millis();
        buffer1 = "";
      }
    } else {
      buffer1 += c;
    }
  }
  
  // COM9 -> COM18
  while (SerialUWB2.available()) {
    char c = SerialUWB2.read();
    if (c == '\n') {
      if (buffer2.length() > 0) {
        float parsed = parseDistance(buffer2);
        if (parsed > 0.0) {
          COM9.distance = parsed;
          d2_updated = true;
        }
        lastDataTime = millis();
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
          COM11.distance = parsed;
          d3_updated = true;
        }
        lastDataTime = millis();
        buffer3 = "";
      }
    } else {
      buffer3 += c;
    }
  }
}

float parseDistance(String &data) {
  data.trim();
  int equalsIndex = data.indexOf('=');
  if (equalsIndex != -1 && equalsIndex < data.length() - 1) {
    String distStr = data.substring(equalsIndex + 1);
    distStr.trim();
    float val = distStr.toFloat();
    if (val > 0.0 && val < 100.0) return val;
  }
  return 0;
}

void calculateXYPosition() {
  float x1 = COM8_X,  y1 = COM8_Y;
  float x2 = COM9_X,  y2 = COM9_Y;
  float x3 = COM11_X, y3 = COM11_Y;
  
  float r1 = COM8.distance;
  float r2 = COM9.distance;
  float r3 = COM11.distance;
  
  float A = 2 * (x2 - x1);
  float B = 2 * (y2 - y1);
  float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
  
  float D = 2 * (x3 - x2);
  float E = 2 * (y3 - y2);
  float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3;
  
  float denominator = A * E - B * D;
  if (fabs(denominator) < 1e-6) {
    user_position_COM18.x = 0;
    user_position_COM18.y = 0;
    return;
  }
  
  user_position_COM18.x = (C * E - F * B) / denominator;
  user_position_COM18.y = (A * F - D * C) / denominator;
}

float calculateHeading() {
  float dx = user_position_COM18.x - 0.0;
  float dy = user_position_COM18.y - 0.0;
  float heading_rad = atan2(dx, dy);
  float heading_deg = heading_rad * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;
  return heading_deg;
}

void displayResults() {
  String action;
  float heading = calculateHeading();
  float distance = sqrt(user_position_COM18.x * user_position_COM18.x + user_position_COM18.y * user_position_COM18.y);
  
  action = getActionFromHeadingAndDistance(heading, distance);
  CheckUserDistance(distance);
  
  Serial.print("  Required Action: ");
  Serial.println(action);
}

void CheckUserDistance(float getDistance) {
  Serial.println(getDistance);
  if (getDistance >= 10.0) {
    Serial.println("Warning, user is more than or equals to 10m.");
    // Could trigger buzzer here if needed
    // startBuzzer(3, 200, 100);
  }
}

void generateFollowCommand() {
  if (!(d1_updated && d2_updated && d3_updated)) {
    sendToJetson_Action("hold");
    return;
  }
  
  float heading = calculateHeading();
  float distance = sqrt(user_position_COM18.x * user_position_COM18.x + user_position_COM18.y * user_position_COM18.y);
  
  String action = getActionFromHeadingAndDistance(heading, distance);
  sendToJetson_Action(action);
}

void sendToJetson_Action(String action) {
  Serial.println(action);
}

String getActionFromHeadingAndDistance(float heading, float distance) {
  if (heading > 20 && heading <= 180) return "turn_right";
  if (heading > 180 && heading < 340) return "turn_left";
  if (distance > 2.5) return "forward";
  return "hold";
}

bool Button_Function() {
  // This function is now just for compatibility
  // The actual button processing happens in updateButton()
  return Button_Signal;
}

// Legacy buzzer function - now redirects to non-blocking version
void Buzzer(int nbeeps, int frequency, int downtime) {
  startBuzzer(nbeeps, frequency, downtime);
}

String GetJetsonData() {
  static String current = "";
  String latest = "";
  
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    
    if (c == '\n') {
      current.trim();
      if (current.length() > 0) {
        latest = current;
      }
      current = "";
    } else {
      current += c;
    }
  }
  
  return latest;
}