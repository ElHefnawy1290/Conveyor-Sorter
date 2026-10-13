#include <Servo.h>

// ==========================================
// 1. HARDWARE CONNECTIONS (Where wires go)
// ==========================================
const uint8_t PIN_IR     = 2;  // IR Sensor: Detects if a box has arrived
const uint8_t PIN_IND    = 3;  // Inductive Sensor: Detects if the box is made of metal
const uint8_t PIN_SERVO  = 9;  // Servo Motor: The robotic arm that pushes metal boxes
const uint8_t PIN_EN1    = 5;  // L293D Enable 1: Controls the speed of the conveyor belt
const uint8_t PIN_IN1    = 7;  // L293D Input 1: Controls motor direction (forward)
const uint8_t PIN_IN2    = 8;  // L293D Input 2: Controls motor direction (backward)
const uint8_t PIN_FB     = A0; // Feedback Pin (Not used in this simulation)

// ==========================================
// 2. SYSTEM SETTINGS & RULES
// ==========================================
#define IR_ACTIVE_LEVEL   LOW  // The IR sensor sends 0 Volts (LOW) when it sees a box
#define IND_ACTIVE_LEVEL  LOW  // The Metal sensor sends 0 Volts (LOW) when it sees metal
#define USE_FEEDBACK      0    // We are simulating motor physics, not using a real speed sensor

const int SPEED_RUN_PWM     = 200; // Normal belt speed (0 is stopped, 255 is max speed)
const int SERVO_DEFAULT_DEG = 90;   // Arm position for cardboard/plastic (let it pass)
const int SERVO_DIVERT_DEG  = 150;  // Arm position to push metal boxes off the belt

// Time limits (in milliseconds) for different actions
const unsigned long DEBOUNCE_MS     = 30;   // Wait 30ms to confirm the IR sensor isn't glitching
const unsigned long STOP_TIMEOUT_MS = 1500; // Max time to wait for the belt to completely stop
const unsigned long SETTLE_MS       = 500;  // Let the box sit still for half a second before scanning
const unsigned long INSPECT_MS      = 300;  // Spend 0.3 seconds scanning for metal
const unsigned long DIVERT_RUN_MS   = 2000; // Run belt for 2 seconds to push metal box away
const unsigned long CLEAR_MS        = 700;  // Run belt for 0.7 seconds to let normal boxes pass
const unsigned long PID_DT_MS       = 20;   // Update the motor speed every 20ms

// PID Math Variables (To make the motor start and stop smoothly, not violently)
float Kp = 1.2f;
float Ki = 0.8f;
float Kd = 0.05f;

// ==========================================
// 3. THE STATE MACHINE (The brain of the system)
// ==========================================
// This list represents the 5 phases of our machine's job.
enum State { RUNNING, STOPPING, INSPECTING, SORTING, CLEARING };
State state = RUNNING; // The machine starts in the RUNNING phase

Servo sorter; // Create the software object to control the servo arm

// Variables for calculating smooth motor speed
float setpoint   = 0;  // Target speed
float speedEst   = 0;  // Current estimated speed
float integral   = 0;
float prevError  = 0;
float pidOut     = 0;
unsigned long lastPid = 0;

// Variables to keep track of time and sensor readings
unsigned long stateStart = 0; // Remembers when the current phase started
bool metalFound = false;      // Final decision: Is it metal?
int metalVotes = 0, metalSamples = 0; // Used to take multiple sensor readings for accuracy

// ==========================================
// 4. HELPER FUNCTIONS (Mini-tasks)
// ==========================================

// Sets the motor to spin forward or backward
void motorDirection(bool forward) {
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW : HIGH);
}

// Sends the actual power (PWM) to the motor driver
void motorWrite(int pwm) {
  pwm = constrain(pwm, 0, 255); // Never allow power to go below 0 or above 255
  analogWrite(PIN_EN1, pwm);
}

// Simulates the physical momentum of a heavy conveyor belt slowing down or speeding up
void updateSpeed(float dt) {
#if USE_FEEDBACK
  speedEst = analogRead(PIN_FB) / 1023.0f * 255.0f;
#else
  const float tau = 0.3f;
  speedEst += (pidOut - speedEst) * (dt / tau);
#endif
}

// The math that calculates exactly how much power the motor needs right now
void runPID() {
  unsigned long now = millis();
  if (now - lastPid < PID_DT_MS) return; // Only do math every 20 milliseconds
  float dt = (now - lastPid) / 1000.0f;
  lastPid = now;

  updateSpeed(dt);

  float error = setpoint - speedEst; // How far are we from our target speed?
  integral += error * dt;
  integral = constrain(integral, -100, 100);
  float derivative = (error - prevError) / dt;
  prevError = error;

  pidOut = Kp * error + Ki * integral + Kd * derivative;
  pidOut = constrain(pidOut, 0, 255);

  // If we want to stop entirely, cut the power to 0
  if (setpoint == 0 && speedEst < 5) {
    pidOut = 0;
    integral = 0;
  }
  motorWrite((int)pidOut); // Send the calculated power to the motor
}

// Reads the IR sensor. Returns true if a box is there.
bool irDetected() {
  return digitalRead(PIN_IR) == IR_ACTIVE_LEVEL;
}

// Reads the Inductive sensor. Returns true if metal is there.
bool metalDetected() {
  return digitalRead(PIN_IND) == IND_ACTIVE_LEVEL;
}

// Double-checks the IR sensor to make sure it wasn't a random glitch
bool irConfirmed() {
  if (!irDetected()) return false;
  delay(DEBOUNCE_MS);
  return irDetected();
}

// Changes the machine's current phase and logs the time it happened
void changeState(State s) {
  state = s;
  stateStart = millis(); // Record the exact millisecond we entered this new phase
  const char* names[] = {"RUNNING", "STOPPING", "INSPECTING", "SORTING", "CLEARING"};
  Serial.print(F("State -> "));
  Serial.println(names[s]); // Print the new phase to the serial monitor
}

// ==========================================
// 5. SETUP (The true starting point)
// ==========================================
void setup() {
  Serial.begin(9600); // Open the communication channel to talk to the PC

  // Configure sensor pins. INPUT_PULLUP keeps them stable until a sensor triggers them.
  pinMode(PIN_IR, INPUT_PULLUP);
  pinMode(PIN_IND, INPUT_PULLUP);
  
  // Configure motor driver pins to send signals OUT
  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  motorDirection(true); // Tell the belt to move forward
  motorWrite(0);        // But keep the speed at 0 for now

  sorter.attach(PIN_SERVO);        // Connect software servo to physical pin 9
  sorter.write(SERVO_DEFAULT_DEG); // Move arm to 0 degrees (straight/open)

  setpoint = SPEED_RUN_PWM; // Set our target speed to 200 (start the belt!)
  lastPid = millis();
  Serial.println(F("Conveyor sorter ready"));
  changeState(RUNNING); // officially enter the RUNNING phase
}

// ==========================================
// 6. LOOP (The "Main" program that runs forever)
// ==========================================
void loop() {
  runPID(); // 1. Always keep updating the motor speed smoothly in the background

  // 2. Decide what to do based on the machine's current phase
  switch (state) {
    
    // PHASE 1: Belt is moving, waiting for a box to arrive
    case RUNNING:
      setpoint = SPEED_RUN_PWM; // Keep target speed at 200
      if (irConfirmed()) {      // IF the IR sensor sees a box...
        Serial.println(F("Product detected (IR)"));
        setpoint = 0;           // Change target speed to 0 (hit the brakes!)
        changeState(STOPPING);  // Move to the STOPPING phase
      }
      break;

    // PHASE 2: Waiting for the heavy belt to physically stop moving
    case STOPPING:
      // IF the belt has slowed down to near 0 AND has settled... OR if it took too long...
      if ((speedEst < 5 && millis() - stateStart > SETTLE_MS) ||
          millis() - stateStart > STOP_TIMEOUT_MS) {
        
        metalVotes = 0;   // Reset our metal detectors
        metalSamples = 0;
        changeState(INSPECTING); // The box is perfectly still. Move to INSPECTING phase.
      }
      break;

    // PHASE 3: Scanning the box to see what it's made of
    case INSPECTING: {
      static unsigned long lastSample = 0;
      
      // Take a reading every 10 milliseconds
      if (millis() - lastSample >= 10) {
        lastSample = millis();
        metalSamples++;                       // Count total readings taken
        if (metalDetected()) metalVotes++;    // If metal is seen, add a vote
      }
      
      // After 300 milliseconds of scanning are finished...
      if (millis() - stateStart >= INSPECT_MS) {
        // If more than half the readings saw metal, we confirm it IS metal
        metalFound = (metalVotes * 2 > metalSamples); 
        Serial.println(metalFound ? F("Result: METAL") : F("Result: NON-METAL"));
        changeState(SORTING); // Decision made. Move to SORTING phase.
      }
      break;
    }

    // PHASE 4: Moving the robotic arm
    case SORTING:
      if (metalFound) {
        sorter.write(SERVO_DIVERT_DEG); // It's metal! Swing arm to 60 degrees to block it.
        delay(400);                     // Wait 0.4 seconds for the mechanical arm to finish moving.
      } else {
        sorter.write(SERVO_DEFAULT_DEG); // Not metal. Keep arm at 0 degrees.
      }
      setpoint = SPEED_RUN_PWM; // Turn the conveyor belt back on (speed = 200)
      changeState(CLEARING);    // Move to CLEARING phase to push the box away.
      break;

    // PHASE 5: Letting the box leave the inspection area
    case CLEARING: {
      // If it's metal, wait 2 seconds. If it's normal, wait 0.7 seconds.
      unsigned long minTime = metalFound ? DIVERT_RUN_MS : CLEAR_MS;
      
      // IF the time has passed AND the IR sensor is completely clear...
      if (millis() - stateStart >= minTime && !irDetected()) {
        sorter.write(SERVO_DEFAULT_DEG); // Return the arm to normal 0 degree position
        metalFound = false;              // Forget about this box
        changeState(RUNNING);            // Go back to Phase 1 and wait for the next box!
      }
      break;
    }
  }
}