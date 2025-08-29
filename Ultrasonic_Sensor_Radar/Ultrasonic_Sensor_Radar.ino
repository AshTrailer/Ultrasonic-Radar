/*
 * Ultrasonic Radar with Stepper Motor Scanning
 * --------------------------------------------
 * This Arduino sketch controls a stepper motor to perform radar-style scanning,
 * while an ultrasonic sensor (HC-SR04 or similar) measures distances.
 * The scan angle, speed, and acceleration are configurable, and measured data
 * (time, angle, distance) are sent via Serial for visualization in MATLAB/Python.
 *
 * Author: Ash_Trailer
 * Repository: https://github.com/AshTrailer/Ultrasonic-Radar
 */

#include <AccelStepper.h>

// ================== Stepper Motor Parameters ==================
#define STEPPER_PIN1 8
#define STEPPER_PIN2 9
#define STEPPER_PIN3 10
#define STEPPER_PIN4 11
#define STEPS_PER_REV 2048             // Total steps per full revolution (typical 28BYJ-48 motor)

// ================== Scanning Parameters ==================
// Note: speed/acceleration are expressed in degrees/s, internally converted to steps
#define SCAN_ANGLE 360                 // Maximum scanning angle (degrees)
#define ANGULAR_SPEED 10               // Stepper angular speed (°/s). Typical range: 30–170 °/s
#define ANGULAR_ACCELERATION 10        // Stepper angular acceleration (°/s²). Typical range: 10–60 °/s²
#define CLOCKWISE 1
#define ANTI_CLOCKWISE -1
int currentDirection = CLOCKWISE;      // Initial scan direction (CW)

// Create stepper object (4-wire full-step sequence)
AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);


// ================== Ultrasonic Sensor Parameters ==================
#define TRIG_PIN 12
#define ECHO_PIN 13
#define ULTRASONIC_ACTIVE_TIME 10      // Trigger HIGH pulse width (µs)
#define ULTRASONIC_PERIOD 150          // Minimum sampling period (ms)
#define ULTRASONIC_MAX_DURATION 40000  // Maximum echo time (µs) ≈ 6.5 m
#define ULTRASONIC_MIN_DURATION 100    // Minimum echo time (µs) to filter out noise
#define OUT_OF_RANGE -1                // Distance flag for "no object detected"

// Ultrasonic state variables
bool ultrasonic_enable = true;
unsigned long echo_start = 0;
unsigned long echo_end = 0;
bool wait_echo_high = false;
bool wait_echo_low = false;
float ultrasonic_distance = 0.0;
unsigned long last_ultrasonic_time = 0;

// Data output buffer
char output_data[40]; 
unsigned long last_print_time = 0;


// ================== Setup Function ==================
void setup() {
  // Serial port initialization (used for PC visualization)
  Serial.begin(115200);

  // Configure ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configure stepper motor max speed and acceleration
  stepper.setMaxSpeed((ANGULAR_SPEED * STEPS_PER_REV) / 360.0);
  stepper.setAcceleration((ANGULAR_ACCELERATION * STEPS_PER_REV) / 360.0);

  // Initial move: one full scan in currentDirection
  stepper.moveTo((SCAN_ANGLE / 360.0) * STEPS_PER_REV * currentDirection);
  delay(1000); // Small delay before starting scan
}


// ================== Main Loop ==================
void loop() {
  Stepper_Mission();      // Control stepper scanning
  Ultrasonic_Mission();   // Handle ultrasonic measurement
}


// ================== Stepper Control ==================
void Stepper_Mission() {
  // Run motor (non-blocking, managed by AccelStepper)
  stepper.run();

  // When scan limit reached, reverse direction
  if (stepper.distanceToGo() == 0) {
    currentDirection = -currentDirection;
    stepper.setCurrentPosition(0);
    stepper.moveTo((SCAN_ANGLE / 360.0) * STEPS_PER_REV * currentDirection);
  }
}


// ================== Ultrasonic Control ==================
void Ultrasonic_Mission() {
  unsigned long now = millis();

  // Trigger a new measurement periodically
  if (ultrasonic_enable && !wait_echo_high && !wait_echo_low && (now - last_ultrasonic_time >= ULTRASONIC_PERIOD)) {
    triggerUltrasonic();
    last_ultrasonic_time = now;
  }

  // Wait for echo rising edge (start of signal)
  if (wait_echo_high) {
    checkEchoRising(now);
  }

  // Wait for echo falling edge (end of signal)
  if (wait_echo_low) {
    checkEchoFalling(now);
  }
}


// ================== Ultrasonic Measurement Functions ==================
void triggerUltrasonic() {
  // Generate trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(ULTRASONIC_ACTIVE_TIME);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo rising edge
  wait_echo_high = true;
}

void checkEchoRising(unsigned long now) {
  // Detect rising edge (ECHO goes HIGH)
  if (digitalRead(ECHO_PIN) == HIGH) {
    echo_start = micros();
    wait_echo_high = false;
    wait_echo_low = true;
  }
  // Timeout (no echo detected)
  else if (now - last_ultrasonic_time > 50) {
    wait_echo_high = false;
    ultrasonic_distance = OUT_OF_RANGE;
  }
}

void checkEchoFalling(unsigned long now) {
  unsigned long duration = micros() - echo_start;

  // Either echo ended or timeout exceeded
  if (duration > ULTRASONIC_MAX_DURATION || digitalRead(ECHO_PIN) == LOW) {
    wait_echo_low = false;

    // Validate measurement
    if (duration >= ULTRASONIC_MIN_DURATION && duration <= ULTRASONIC_MAX_DURATION) {
      ultrasonic_distance = duration / 58.0; // Convert µs to cm
    }
    else {
      ultrasonic_distance = OUT_OF_RANGE;
    }

    // Print result
    printUltrasonicData(now);
  }
}


// ================== Serial Data Output ==================
void printUltrasonicData(unsigned long now) {
  // Format current time (HH:MM:SS)
  unsigned long total_seconds = now / 1000;
  unsigned int hh = (total_seconds / 3600) % 24;
  unsigned int mm = (total_seconds / 60) % 60;
  unsigned int ss = total_seconds % 60;

  // Convert stepper position into angle
  float angle = stepper.currentPosition() * 360.0 / STEPS_PER_REV;
  float distance = ultrasonic_distance;

  // Format strings
  char angle_str[12];
  char dist_str[13];
  dtostrf(angle, 6, 2, angle_str);

  if (distance == OUT_OF_RANGE) {
    strcpy(dist_str, "OUT OF RANGE");
  } else {
    dtostrf(distance, 6, 2, dist_str);
  }

  // Example output: [12:34:56,  45.00,  123.45]
  sprintf(output_data, "[%02u:%02u:%02u,%.9s,%.13s]", hh, mm, ss, angle_str, dist_str);
  Serial.println(output_data);
}
