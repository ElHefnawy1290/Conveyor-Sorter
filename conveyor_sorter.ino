/*
  نظام فرز ذكي على سير ناقل صناعي (Smart Industrial Conveyor Belt Sorting System)
  المتحكم: Arduino Uno

  التوصيلات:
    D2 : حساس IR (كشف وصول المنتج)
    D3 : حساس حثي Inductive (كشف المعدن)
    D9 : إشارة السيرفو (ذراع الفرز)
    D5 : EN1 (PWM) لدرايفر L293D
    D7 : IN1
    D8 : IN2

  منطق التشغيل (State Machine):
    RUNNING -> STOPPING -> INSPECTING -> SORTING -> CLEARING -> RUNNING

  ملاحظة PID:
    يتم التحكم في سرعة المحرك بمتحكم PID (بدء/إيقاف سلس للسير).
    - إذا لم يوجد حساس سرعة: يُستخدم نموذج برمجي للمحرك (USE_FEEDBACK = 0)
      وهو الوضع المناسب لمحاكاة Proteus بالتوصيلات المطلوبة فقط.
    - إذا أضفت تاكوميتر/انكودر يعطي جهدًا تناظريًا على A0: اجعل USE_FEEDBACK = 1.
*/

#include <Servo.h>

// ---------------- الأطراف ----------------
const uint8_t PIN_IR     = 2;
const uint8_t PIN_IND    = 3;
const uint8_t PIN_SERVO  = 9;
const uint8_t PIN_EN1    = 5;   // PWM
const uint8_t PIN_IN1    = 7;
const uint8_t PIN_IN2    = 8;
const uint8_t PIN_FB     = A0;  // اختياري: تغذية خلفية للسرعة

// ---------------- الإعدادات ----------------
// مستوى الإشارة عند "الكشف" (معظم الحساسات الصناعية NPN تعطي LOW عند الكشف)
// في Proteus مع LogicState غيّر القيمة حسب مفتاحك.
#define IR_ACTIVE_LEVEL    LOW
#define IND_ACTIVE_LEVEL   LOW

#define USE_FEEDBACK       0      // 0 = نموذج برمجي، 1 = قراءة A0

const int   SPEED_RUN_PWM      = 200;   // سرعة التشغيل (0-255)
const int   SERVO_DEFAULT_DEG  = 0;     // الوضع الافتراضي (يمر غير المعدني)
const int   SERVO_DIVERT_DEG   = 60;    // وضع تحويل المعدن
const unsigned long DEBOUNCE_MS     = 30;
const unsigned long STOP_TIMEOUT_MS = 1500;
const unsigned long SETTLE_MS       = 500;   // استقرار المنتج قبل الفحص
const unsigned long INSPECT_MS      = 300;   // مدة أخذ عينات الحساس الحثي
const unsigned long DIVERT_RUN_MS   = 2000;  // مدة تشغيل السير لدفع المعدن للمسار الثانوي
const unsigned long CLEAR_MS        = 700;   // مدة تأكد خروج المنتج قبل الإرجاع
const unsigned long PID_DT_MS       = 20;

// معاملات PID (قابلة للضبط)
float Kp = 1.2f;
float Ki = 0.8f;
float Kd = 0.05f;

// ---------------- المتغيرات ----------------
enum State { RUNNING, STOPPING, INSPECTING, SORTING, CLEARING };
State state = RUNNING;

Servo sorter;

float setpoint   = 0;      // السرعة المطلوبة (0-255)
float speedEst   = 0;      // السرعة المقاسة/المقدّرة
float integral   = 0;
float prevError  = 0;
float pidOut     = 0;
unsigned long lastPid = 0;

unsigned long stateStart = 0;
bool metalFound = false;
int  metalVotes = 0, metalSamples = 0;

// ---------------- دوال المحرك ----------------
void motorDirection(bool forward) {
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW : HIGH);
}

void motorWrite(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(PIN_EN1, pwm);
}

// ---------------- PID لسرعة السير ----------------
void updateSpeed(float dt) {
#if USE_FEEDBACK
  speedEst = analogRead(PIN_FB) / 1023.0f * 255.0f;
#else
  // نموذج مبسط للمحرك: استجابة من الدرجة الأولى (tau ~ 0.3 s)
  const float tau = 0.3f;
  speedEst += (pidOut - speedEst) * (dt / tau);
#endif
}

void runPID() {
  unsigned long now = millis();
  if (now - lastPid < PID_DT_MS) return;
  float dt = (now - lastPid) / 1000.0f;
  lastPid = now;

  updateSpeed(dt);

  float error = setpoint - speedEst;
  integral += error * dt;
  integral = constrain(integral, -100, 100);          // anti-windup
  float derivative = (error - prevError) / dt;
  prevError = error;

  pidOut = Kp * error + Ki * integral + Kd * derivative;
  pidOut = constrain(pidOut, 0, 255);

  // مع setpoint = 0 نقطع المحرك تمامًا عند توقف السرعة تقريبًا
  if (setpoint == 0 && speedEst < 5) {
    pidOut = 0;
    integral = 0;
  }
  motorWrite((int)pidOut);
}

// ---------------- قراءة الحساسات ----------------
bool irDetected() {
  return digitalRead(PIN_IR) == IR_ACTIVE_LEVEL;
}
bool metalDetected() {
  return digitalRead(PIN_IND) == IND_ACTIVE_LEVEL;
}

// تأكيد IR لفترة قصيرة (debounce)
bool irConfirmed() {
  if (!irDetected()) return false;
  delay(DEBOUNCE_MS);
  return irDetected();
}

void changeState(State s) {
  state = s;
  stateStart = millis();
  const char* names[] = {"RUNNING", "STOPPING", "INSPECTING", "SORTING", "CLEARING"};
  Serial.print(F("State -> "));
  Serial.println(names[s]);
}

// ---------------- الإعداد ----------------
void setup() {
  Serial.begin(9600);

  pinMode(PIN_IR, INPUT_PULLUP);
  pinMode(PIN_IND, INPUT_PULLUP);
  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  motorDirection(true);
  motorWrite(0);

  sorter.attach(PIN_SERVO);
  sorter.write(SERVO_DEFAULT_DEG);

  setpoint = SPEED_RUN_PWM;
  lastPid = millis();
  Serial.println(F("Conveyor sorter ready"));
  changeState(RUNNING);
}

// ---------------- الحلقة الرئيسية ----------------
void loop() {
  runPID();   // يعمل دائمًا للحفاظ على السرعة / البدء والإيقاف السلس

  switch (state) {

    case RUNNING:
      setpoint = SPEED_RUN_PWM;
      if (irConfirmed()) {
        Serial.println(F("Product detected (IR)"));
        setpoint = 0;                 // أوقف السير
        changeState(STOPPING);
      }
      break;

    case STOPPING:
      // انتظر حتى يتوقف السير فعليًا (أو انتهاء المهلة) ثم فترة استقرار
      if ((speedEst < 5 && millis() - stateStart > SETTLE_MS) ||
          millis() - stateStart > STOP_TIMEOUT_MS) {
        metalVotes = 0;
        metalSamples = 0;
        changeState(INSPECTING);
      }
      break;

    case INSPECTING: {
      // أخذ عدة عينات من الحساس الحثي (تصويت أغلبية لتقليل الضوضاء)
      static unsigned long lastSample = 0;
      if (millis() - lastSample >= 10) {
        lastSample = millis();
        metalSamples++;
        if (metalDetected()) metalVotes++;
      }
      if (millis() - stateStart >= INSPECT_MS) {
        metalFound = (metalVotes * 2 > metalSamples);
        Serial.println(metalFound ? F("Result: METAL") : F("Result: NON-METAL"));
        changeState(SORTING);
      }
      break;
    }

    case SORTING:
      if (metalFound) {
        sorter.write(SERVO_DIVERT_DEG);   // حوّل المنتج المعدني للمسار الثانوي
        delay(400);                        // وقت وصول الذراع
      } else {
        sorter.write(SERVO_DEFAULT_DEG);  // يبقى الذراع في مكانه
      }
      setpoint = SPEED_RUN_PWM;           // استئناف السير في الحالتين
      changeState(CLEARING);
      break;

    case CLEARING: {
      // انتظر خروج المنتج من نقطة الفحص
      unsigned long minTime = metalFound ? DIVERT_RUN_MS : CLEAR_MS;
      if (millis() - stateStart >= minTime && !irDetected()) {
        sorter.write(SERVO_DEFAULT_DEG);  // ارجع الذراع للوضع الافتراضي
        metalFound = false;
        changeState(RUNNING);
      }
      break;
    }
  }
}
