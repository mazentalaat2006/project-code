// ================== PIN DEFINITIONS (unchanged) ==================
#define INR1 4
#define INR2 5
#define INL1 12
#define INL2 7
#define ENABLERIGHT 9
#define ENABLELEFT 10
#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1
#define SENSOR_MIDDLE A3
#define SENSOR_FARLEFT A2
#define SENSOR_FAR_RIGHT A4

// ================== TUNABLE PARAMETERS ==================
// Sensor value above this = "sees the (dark) line".
// Re-check this on your hardware with the Serial Monitor.
int lineThreshold = 400;

// Speed range used by the autonomous PID controller.
int low_speed = 90;     // minimum forward speed (used in sharp turns / recovery floor)
int high_speed = 120;   // maximum forward speed (straight-line speed, raised from 200)

// PID gains -- start here, then tune on the real robot.
// Increase Kp first for faster response, add Kd to stop oscillation,
// leave Ki at 0 unless you see a steady-state drift to one side.
float Kp = 24.0;
float Ki = 0.05;
float Kd = 12.0;

// Manual-mode speed (unchanged behavior)
int speed = 140;

bool autonomous = false, manual = true;

// ================== BLUETOOTH SAFETY ==================
// HC-05/HC-06 modules can spit out random noise on their TX pin
// (which goes into Arduino RX) for a moment right after power-on,
// before they've booted or paired with anything. That noise can
// accidentally match a valid command byte ('A', 'F', etc.) and
// make the robot move on its own with nothing connected.
//
// Fix #1 (always active): ignore all serial input for STARTUP_GRACE_MS
// after power-on, and keep flushing the buffer during that time.
const unsigned long STARTUP_GRACE_MS = 2000;
bool bootFlushDone = false;
unsigned long bootStartTime = 0;

// Fix #2 (optional, stronger): if your HC-05 has a STATE pin, wire it
// to an Arduino digital pin and set USE_BT_STATE_PIN to true below.
// STATE goes HIGH only when a phone/app is actually connected, so any
// noise received while nothing is connected gets ignored completely.
#define USE_BT_STATE_PIN false
#define BT_STATE_PIN 8

// PID state
float lastError = 0;
float integral = 0;
int lastDirection = 0; // -1 = last seen line was to the left, 1 = to the right

unsigned long lastPrintTime = 0;

// Forward declarations (manual-mode helpers, unchanged)
void forward(int speed);
void left(int speed);
void right(int speed);
void backward(int speed);
void stop();
void forwardleft(int speed);
void forwardright(int speed);
void backwardleft(int speed);
void backwardright(int speed);

// New: differential drive helper used by the autonomous PID controller
void driveMotors(int leftSpeed, int rightSpeed);

void setup() {
  Serial.begin(9600);
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);
  pinMode(ENABLERIGHT, OUTPUT);
  pinMode(ENABLELEFT, OUTPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_MIDDLE, INPUT);
  pinMode(SENSOR_FARLEFT, INPUT);
  pinMode(SENSOR_FAR_RIGHT, INPUT);

  #if USE_BT_STATE_PIN
  pinMode(BT_STATE_PIN, INPUT);
  #endif

  // Force everything off immediately, ignoring any noise on the pins
  // caused by them floating for a few microseconds before pinMode().
  autonomous = false;
  manual = true;
  stop();

  bootStartTime = millis();
}

void loop() {
  // ---- Boot safety window: swallow any noise for the first couple seconds ----
  if (!bootFlushDone) {
    while (Serial.available() > 0) Serial.read(); // discard whatever came in
    if (millis() - bootStartTime >= STARTUP_GRACE_MS) {
      bootFlushDone = true;
      while (Serial.available() > 0) Serial.read(); // final flush right before trusting input
    }
  }

  // ---- Optional: only trust commands while a phone/app is actually connected ----
  bool serialTrusted = bootFlushDone;
  #if USE_BT_STATE_PIN
  serialTrusted = serialTrusted && (digitalRead(BT_STATE_PIN) == HIGH);
  if (digitalRead(BT_STATE_PIN) == LOW) {
    while (Serial.available() > 0) Serial.read(); // nothing is really connected, ignore/flush
  }
  #endif

  if (serialTrusted && Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'A') {
      autonomous = true;
      manual = false;
      integral = 0;
      lastError = 0;
      stop();
    } else if (command == 'M') {
      autonomous = false;
      manual = true;
      stop();
    }

    if (manual) {
      if (command == 'F') forward(speed);
      else if (command == 'L') left(speed);
      else if (command == 'R') right(speed);
      else if (command == 'B') backward(speed);
      else if (command == 'S') stop();
      else if (command == 'a') forwardleft(speed);
      else if (command == 'b') forwardright(speed);
      else if (command == 'c') backwardleft(speed);
      else if (command == 'd') backwardright(speed);
      else if (command == '+') {
        speed += 10;
        if (speed > 255) speed = 255;
      } else if (command == '-') {
        speed -= 10;
        if (speed < 0) speed = 0;
      }
    }
  }

  if (autonomous) {
    int farleft  = analogRead(SENSOR_FARLEFT);
    int leftV    = analogRead(SENSOR_LEFT);
    int middle   = analogRead(SENSOR_MIDDLE);
    int rightV   = analogRead(SENSOR_RIGHT);
    int farright = analogRead(SENSOR_FAR_RIGHT);

    bool sFarLeft  = farleft  > lineThreshold;
    bool sLeft     = leftV    > lineThreshold;
    bool sMiddle   = middle   > lineThreshold;
    bool sRight    = rightV   > lineThreshold;
    bool sFarRight = farright > lineThreshold;

    int activeCount = sFarLeft + sLeft + sMiddle + sRight + sFarRight;

    if (millis() - lastPrintTime >= 250) {
      lastPrintTime = millis();
      Serial.print("FL:"); Serial.print(farleft);
      Serial.print(" L:"); Serial.print(leftV);
      Serial.print(" M:"); Serial.print(middle);
      Serial.print(" R:"); Serial.print(rightV);
      Serial.print(" FR:"); Serial.print(farright);
      Serial.print(" | err:"); Serial.println(lastError);
    }

    // All five sensors dark: usually a perpendicular/cross line. Stop for safety.
    // (Change this to "continue straight briefly" if your track has intersections
    // you want the robot to drive through.)
    if (activeCount == 5) {
      stop();
      return;
    }

    if (activeCount > 0) {
      // Weighted position of the line under the sensor array.
      // Negative = line is to the left, positive = line is to the right.
      float weightedSum = (sFarLeft ? -2.0 : 0) + (sLeft ? -1.0 : 0) +
                           (sMiddle ?  0.0 : 0) + (sRight ?  1.0 : 0) +
                           (sFarRight ? 2.0 : 0);
      float error = weightedSum / activeCount;

      if (error != 0) lastDirection = (error > 0) ? 1 : -1;

      // Sharp edge case: only a far sensor sees the line -> pivot fast instead
      // of a slow proportional correction, so we don't run off the track.
      if (error <= -2.0) {
        driveMotors(-high_speed, high_speed);
        integral = 0;
        lastError = error;
        return;
      }
      if (error >= 2.0) {
        driveMotors(high_speed, -high_speed);
        integral = 0;
        lastError = error;
        return;
      }

      // ---- PID ----
      integral += error;
      integral = constrain(integral, -50, 50); // anti-windup clamp
      float derivative = error - lastError;
      float correction = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;

      // Slow down on turns, speed up on straights: more accurate AND faster overall.
      int baseSpeed = high_speed - (int)(abs(error) * (high_speed - low_speed) / 2.0);
      baseSpeed = constrain(baseSpeed, low_speed, high_speed);

      int leftSpeed  = constrain(baseSpeed + (int)correction, -255, 255);
      int rightSpeed = constrain(baseSpeed - (int)correction, -255, 255);

      driveMotors(leftSpeed, rightSpeed);

    } else {
      // Line lost entirely: pivot toward the last known side to reacquire it
      // quickly, instead of reversing first (faster recovery).
      integral = 0;
      if (lastDirection == 1) {
        driveMotors(high_speed, -high_speed);
      } else if (lastDirection == -1) {
        driveMotors(-high_speed, high_speed);
      } else {
        stop();
      }
    }
  }
}

// ================== DRIVE HELPERS ==================

// Signed differential drive for the PID controller.
// Positive = forward, negative = reverse, magnitude = PWM speed (0-255).
void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(INL1, 1);
    digitalWrite(INL2, 0);
  } else {
    digitalWrite(INL1, 0);
    digitalWrite(INL2, 1);
  }
  if (rightSpeed >= 0) {
    digitalWrite(INR1, 1);
    digitalWrite(INR2, 0);
  } else {
    digitalWrite(INR1, 0);
    digitalWrite(INR2, 1);
  }
  analogWrite(ENABLELEFT, constrain(abs(leftSpeed), 0, 255));
  analogWrite(ENABLERIGHT, constrain(abs(rightSpeed), 0, 255));
}

// ---- Manual-mode helpers (unchanged from the original) ----

void forward(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void backward(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void right(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void left(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed);
}

void stop() {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, 0);
  analogWrite(ENABLELEFT, 0);
}

void forwardright(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed / 4);
  analogWrite(ENABLELEFT, speed);
}

void forwardleft(int speed) {
  digitalWrite(INR1, 1);
  digitalWrite(INR2, 0);
  digitalWrite(INL1, 1);
  digitalWrite(INL2, 0);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed / 4);
}

void backwardright(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed / 4);
  analogWrite(ENABLELEFT, speed);
}

void backwardleft(int speed) {
  digitalWrite(INR1, 0);
  digitalWrite(INR2, 1);
  digitalWrite(INL1, 0);
  digitalWrite(INL2, 1);
  analogWrite(ENABLERIGHT, speed);
  analogWrite(ENABLELEFT, speed / 4);
}
