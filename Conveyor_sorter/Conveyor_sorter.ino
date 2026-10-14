#include <Servo.h>

// =====================================================
// 1. PINS
// =====================================================
const byte ENCODER_PIN = 2;
const byte PIN_IR = 3;
const byte PIN_IND = 6;

const byte PIN_EN1 = 5;
const byte PIN_IN1 = 7;
const byte PIN_IN2 = 8;

const byte PIN_SERVO = 9;

// =====================================================
// 2. SENSOR LOGIC
// =====================================================
#define IR_ACTIVE_LEVEL LOW
#define IND_ACTIVE_LEVEL HIGH

// =====================================================
// 3. RPM / ENCODER
// =====================================================
const float PULSES_PER_REV = 24.0;

const unsigned long RPM_SAMPLE_MS = 500;

const float SPEED_RUN_RPM  = 60.0;
const float SPEED_SLOW_RPM = 30.0;

volatile unsigned long encoderPulses = 0;
unsigned long lastRpmSample = 0;

float measuredRPM = 0.0;
float targetRPM = SPEED_RUN_RPM;

// =====================================================
// 4. PID
// =====================================================
const float KP = 1.0;
const float KI = 1.2;
const float KD = 0.02;

const float BASE_PWM = 170.0;
const float INTEGRAL_LIMIT = 100.0;

float integral = 0.0;
float previousError = 0.0;
float pidOut = 0.0;

int motorPWM = 0;

// =====================================================
// 5. SERVO
// =====================================================
Servo sorter;

const int SERVO_DEFAULT_US = 1500;
const int SERVO_METAL_US = 1333;
const int SERVO_NONMETAL_US = 1667;

const unsigned long SERVO_MOVE_MS = 400;
const unsigned long SENSOR_TO_SERVO_MS = 500;
const unsigned long SERVO_RETURN_MS = 400;

const unsigned long SENSOR_CLEAR_DEBOUNCE_MS = 100;

// =====================================================
// 6. STATES
// =====================================================
enum State {
  RUNNING,
  STOPPING,
  INSPECTING,
  SORTING,
  CLEARING
};

State state = RUNNING;

unsigned long stateStart = 0;

bool previousProductDetected = false;
bool metalFound = false;

bool sensorClearTimerRunning = false;
unsigned long sensorClearSince = 0;

// =====================================================
// 7. ENCODER INTERRUPT
// =====================================================
void encoderISR() {
  encoderPulses++;
}

// =====================================================
// 8. MOTOR
// =====================================================
void motorDirection(bool forward) {
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW : HIGH);
}

void motorWrite(int pwm) {
  motorPWM = constrain(pwm, 0, 255);
  analogWrite(PIN_EN1, motorPWM);
}

// =====================================================
// 9. PID SPEED CONTROL
// =====================================================
void updateSpeed() {
  unsigned long now = millis();

  if (now - lastRpmSample < RPM_SAMPLE_MS) {
    return;
  }

  unsigned long elapsed = now - lastRpmSample;
  lastRpmSample = now;

  unsigned long pulses;

  noInterrupts();
  pulses = encoderPulses;
  encoderPulses = 0;
  interrupts();

  measuredRPM =
    (pulses * 60000.0) / (PULSES_PER_REV * elapsed);

  float dt = elapsed / 1000.0;

  // PID error
  float error = targetRPM - measuredRPM;

  // Proportional
  float P = KP * error;

  // Integral with anti-windup limit
  integral += error * dt;
  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT);

  float I = KI * integral;

  // Derivative
  float derivative = (error - previousError) / dt;
  float D = KD * derivative;

  // Required PWM
  pidOut = BASE_PWM + P + I + D;

  pidOut = constrain(pidOut, 0, 255);

  // When target speed is zero, stop the motor.
  if (targetRPM <= 0) {
    pidOut = 0;
    integral = 0;
  }

  motorWrite((int)pidOut);

  previousError = error;

  Serial.print(F("RPM: "));
  Serial.print(measuredRPM, 1);

  Serial.print(F(" | Target: "));
  Serial.print(targetRPM, 1);

  Serial.print(F(" | PWM: "));
  Serial.println(motorPWM);
}

// =====================================================
// 10. SENSOR READING
// =====================================================
bool irDetected() {
  return digitalRead(PIN_IR) == IR_ACTIVE_LEVEL;
}

bool metalDetected() {
  return digitalRead(PIN_IND) == IND_ACTIVE_LEVEL;
}

// =====================================================
// 11. CHANGE STATE
// =====================================================
void changeState(State newState) {
  state = newState;
  stateStart = millis();

  const char* names[] = {
    "RUNNING",
    "STOPPING",
    "INSPECTING",
    "SORTING",
    "CLEARING"
  };

  Serial.print(F("State -> "));
  Serial.println(names[newState]);
}

// =====================================================
// 12. PRODUCT DETECTION
// =====================================================
void updateProductDetection() {
  bool detected = irDetected();

  // A new product has just reached the IR sensor.
  if (detected && !previousProductDetected && state == RUNNING) {

    Serial.println(F("Product detected"));

    // Slow down the conveyor.
    targetRPM = SPEED_SLOW_RPM;

    // Classify the product using the inductive sensor.
    metalFound = metalDetected();

    Serial.print(F("Classification: "));
    Serial.println(
      metalFound ? F("METAL") : F("NON-METAL"));

    // Move the servo to the selected sorting position.
    if (metalFound) {
      sorter.writeMicroseconds(SERVO_METAL_US);
    } else {
      sorter.writeMicroseconds(SERVO_NONMETAL_US);
    }

    changeState(STOPPING);
  }

  // Detect when the product has cleared the IR sensor.
  if (!detected) {
    if (!sensorClearTimerRunning) {
      sensorClearSince = millis();
      sensorClearTimerRunning = true;
    }
  } else {
    sensorClearTimerRunning = false;
  }

  previousProductDetected = detected;
}

// =====================================================
// 13. MAIN SORTING STATE MACHINE
// =====================================================
void updateSorting() {
  unsigned long now = millis();

  switch (state) {

    case RUNNING:
      // Normal conveyor speed.
      targetRPM = SPEED_RUN_RPM;
      break;

    case STOPPING:
      // The conveyor slows down; it does not stop completely.
      // Wait for the product to clear the IR sensor.
      if (sensorClearTimerRunning && now - sensorClearSince >= SENSOR_CLEAR_DEBOUNCE_MS && now - stateStart >= SERVO_MOVE_MS) {

        // Resume normal speed while the product travels
        // from the IR sensor toward the sorting servo.
        targetRPM = SPEED_RUN_RPM;

        changeState(INSPECTING);

        Serial.println(
          F("IR cleared. Waiting for product travel time."));
      }
      break;

    case INSPECTING:
      // Keep the sorting position while the product travels.
      if (now - stateStart >= SENSOR_TO_SERVO_MS) {

        // Return the servo to its default position.
        sorter.writeMicroseconds(SERVO_DEFAULT_US);

        changeState(SORTING);

        Serial.println(F("Sorting position completed."));
      }
      break;

    case SORTING:
      // Allow the servo to finish returning.
      if (now - stateStart >= SERVO_RETURN_MS) {
        changeState(CLEARING);
      }
      break;

    case CLEARING:
      // Prepare for the next product.
      metalFound = false;
      targetRPM = SPEED_RUN_RPM;

      changeState(RUNNING);

      Serial.println(F("Ready for next product."));
      break;
  }
}

void testMotorMaxSpeed() {
  motorDirection(true);
  analogWrite(PIN_EN1, 255);

  Serial.println(F("Testing maximum motor speed..."));

  delay(5000);

  noInterrupts();
  unsigned long pulses = encoderPulses;
  encoderPulses = 0;
  interrupts();

  Serial.print(F("Pulses in 5 seconds = "));
  Serial.println(pulses);

  float rpm = (pulses * 60000.0) / (PULSES_PER_REV * 5000.0);
  analogWrite(PIN_EN1, 0);

  Serial.print(F("Measured RPM = "));
  Serial.println(rpm, 1);
}

// =====================================================
// 14. SETUP
// =====================================================
void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_IND, INPUT);

  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_PIN),
    encoderISR,
    FALLING);

  motorDirection(true);
  motorWrite(0);

  sorter.attach(PIN_SERVO);
  sorter.writeMicroseconds(SERVO_DEFAULT_US);

  targetRPM = SPEED_RUN_RPM;
  lastRpmSample = millis();

  Serial.println(F("Smart Conveyor Sorting System"));
  Serial.println(F("Encoder=D2 | IR=D3 | Inductive=D6"));
  Serial.println(F("Motor: EN=D5, IN1=D7, IN2=D8"));
  Serial.println(F("Servo=D9"));

  changeState(RUNNING);
  testMotorMaxSpeed();
}

// =====================================================
// 15. MAIN LOOP
// =====================================================
void loop() {
  updateProductDetection();
  updateSorting();
  updateSpeed();
}
