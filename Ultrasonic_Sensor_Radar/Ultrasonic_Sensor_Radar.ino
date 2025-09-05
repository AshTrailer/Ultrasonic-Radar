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
#define STEPS_PER_REV 2048             // Steps per full revolution (28BYJ-48 typical value)

// ================== Scanning Parameters ==================
// Speed and acceleration are expressed in degrees/s and degrees/s², 
// internally converted into steps.
#define SCAN_ANGLE 360                 // Maximum scanning angle (degrees)
#define ANGULAR_SPEED 20               // Angular speed (°/s)
#define ANGULAR_ACCELERATION 10        // Angular acceleration (°/s²)
#define CLOCKWISE 1
#define ANTI_CLOCKWISE -1
int currentDirection = CLOCKWISE;      // Initial scan direction
unsigned long now = millis();
long savedPosition = 0;                // Reference "home" position in steps
bool scanningEnabled = true;           // Stepper scanning task enabled/disabled
bool ultrasonicEnabled = true;         // Ultrasonic measurement enabled/disabled
bool returningHome = false;            // True if returning to zero position
long pausedTarget = 0;                 // Saved target when paused
bool paused = false;                   // Pause state
bool scanningForward = true;           // Current scan direction

// Stepper motor object (4-wire full-step mode)
AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

// ================== Ultrasonic Sensor Parameters ==================
#define TRIG_PIN 12 
#define ECHO_PIN 13
#define ULTRASONIC_ACTIVE_TIME 10      // Trigger HIGH pulse width (µs)
#define ULTRASONIC_PERIOD 500          // Minimum sampling period (ms)
#define ULTRASONIC_MAX_DURATION 40000  // Max echo time (µs) ≈ 6.5 m
#define ULTRASONIC_MIN_DURATION 100    // Min echo time (µs) to reject noise
#define OUT_OF_RANGE -1                // Distance flag for "no object detected"

// Ultrasonic state variables
bool ultrasonic_enable = true;
unsigned long echo_start = 0;
unsigned long echo_end = 0;
bool wait_echo_high = false;
bool wait_echo_low = false;
float ultrasonic_distance = 0.0;
unsigned long last_ultrasonic_time = 0;

// Serial output buffer
char output_data[40]; 
unsigned long last_print_time = 0;

// ================== LCD Parameters ==================
#include <LiquidCrystal.h>
// Example wiring for 16x2 LCD
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);
unsigned long total_row_last_Update = 0;
const unsigned long fresh_rate = 1000;
bool readErrorShown = false;

// ================== Button Class ==================
enum ButtonMode {
  BUTTON_TOGGLE,   // Toggle on press
  BUTTON_HOLD,     // Active while held down
  BUTTON_PULSE,    // Single pulse on press
  BUTTON_REPEAT    // Repeated pulses while held
};

class Button {
  public:
    Button(int pin, ButtonMode mode, unsigned long debounce = 20, unsigned long repeatInterval = 500)
      : _pin(pin), _mode(mode), _debounce(debounce), _repeatInterval(repeatInterval),
        _lastState(HIGH), _buttonState(HIGH), _lastDebounceTime(0),
        _output(false), _lastOutputTime(0) {
      pinMode(pin, INPUT_PULLUP);
    }

    bool update() {
      bool reading = digitalRead(_pin);

      // Debounce handling
      if (reading != _lastState) {
        _lastDebounceTime = millis();
      }

      if ((millis() - _lastDebounceTime) > _debounce) {
        bool lastLogic = _buttonState;
        _buttonState = reading;

        switch (_mode) {
          case BUTTON_TOGGLE:
            if (lastLogic == HIGH && _buttonState == LOW) {
              _output = true;
            }
            else {
              _output = false;
            }
            break;

          case BUTTON_HOLD:
            _output = (_buttonState == LOW);
            break;

          case BUTTON_PULSE:
            if (lastLogic == HIGH && _buttonState == LOW) {
              _output = true;
            } else {
              _output = false;
            }
            break;

          case BUTTON_REPEAT:
            if (_buttonState == LOW) {
              if (millis() - _lastOutputTime >= _repeatInterval) {
                _output = true;
                _lastOutputTime = millis();
              } else {
                _output = false;
              }
            } else {
              _output = false;
            }
            break;
        }
      }

      _lastState = reading;
      return _output;
    }

  private:
    int _pin;
    ButtonMode _mode;
    unsigned long _debounce;
    unsigned long _repeatInterval;

    bool _lastState;
    bool _buttonState;
    unsigned long _lastDebounceTime;

    bool _output;
    unsigned long _lastOutputTime;
};

// Button pins
const int button1_Pin = 2;
const int button2_Pin = 3;
const int button3_Pin = 4;

// Button objects
Button btn1(button1_Pin, BUTTON_TOGGLE); // Pause/resume scanning
Button btn2(button2_Pin, BUTTON_TOGGLE); // Return to home
Button btn3(button3_Pin, BUTTON_TOGGLE); // Enable/disable ultrasonic

// ================== Setup Function ==================
void setup() {
  Serial.begin(115200);

  lcd.begin(16,2);

  // Configure ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configure stepper motor speed and acceleration
  stepper.setMaxSpeed((ANGULAR_SPEED * STEPS_PER_REV) / 360.0);
  stepper.setAcceleration((ANGULAR_ACCELERATION * STEPS_PER_REV) / 360.0);

  stepper.setCurrentPosition(0);
  savedPosition = 0;
  Serial.println("Startup: position set to 0");

  scanningEnabled = false;   // Start in paused state
  ultrasonicEnabled = false; // Ultrasonic disabled at startup

  delay(1000); // Small delay before system starts
}

// ================== Main Loop ==================
void loop() {
  now = millis();

  // -------- Return to home logic --------
  if (returningHome) {
    stepper.run(); 
    if (stepper.distanceToGo() == 0) {
      returningHome = false;
      scanningEnabled = false;
      paused = false;
      scanningForward = true;  // Reset to forward scanning
      Serial.println("Returned to home and stopped");
    }
  } 
  // -------- Normal scanning logic --------
  else if (scanningEnabled && !paused) {
    stepper.run();

    if (stepper.distanceToGo() == 0) {
      scanningForward = !scanningForward;
      long target = scanningForward ? (SCAN_ANGLE / 360.0) * STEPS_PER_REV : 0;
      stepper.moveTo(target);
    }
  }

  // -------- Ultrasonic measurement --------
  if (ultrasonicEnabled) {
    Ultrasonic_Mission(now);
  }

  // -------- Serial output --------
  if (now - last_print_time >= ULTRASONIC_PERIOD) {
    printUltrasonicData(now);
    last_print_time = now;
  }

  // -------- Button handling --------
  if (!returningHome && btn1.update()) { // Pause/resume
    if (!paused) {
      pausedTarget = stepper.targetPosition();
      stepper.stop();  
      paused = true;
      scanningEnabled = false;
      Serial.println("Stepper paused");
    } 
    else {
      paused = false;
      scanningEnabled = true;

      stepper.setMaxSpeed(20);
      stepper.setAcceleration(10);

      stepper.moveTo(pausedTarget);
      Serial.println("Stepper resumed");
    }
  }

  if (btn2.update() && !returningHome) { // Return to home
    ultrasonicEnabled = false;
    returningHome = true;
    scanningEnabled = false;
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
    stepper.setMaxSpeed(30);
    stepper.setAcceleration(10);
    stepper.moveTo(0);
    Serial.println("Returning to saved position...");
  }

  if (btn3.update()) { // Ultrasonic on/off
    ultrasonicEnabled = !ultrasonicEnabled;
    Serial.println(ultrasonicEnabled ? "Ultrasonic enabled" : "Ultrasonic disabled");
  }
}

// ================== Stepper Control ==================
void Stepper_Mission() {
    if (!returningHome && scanningEnabled && !paused) {
        stepper.run();

        if (stepper.distanceToGo() == 0) {
            scanningForward = !scanningForward;
            long target = scanningForward ? (SCAN_ANGLE / 360.0) * STEPS_PER_REV : 0;
            stepper.moveTo(target);
        }
    } else {
        stepper.run(); // Run while homing or paused
    }
}

// ================== Ultrasonic Control ==================
void Ultrasonic_Mission(unsigned long now) {
  // Trigger measurement periodically
  if (ultrasonic_enable && !wait_echo_high && !wait_echo_low && (now - last_ultrasonic_time >= ULTRASONIC_PERIOD)) {
    triggerUltrasonic();
    last_ultrasonic_time = now;
  }

  // Rising edge detection
  if (wait_echo_high) {
    checkEchoRising(now);
  }

  // Falling edge detection
  if (wait_echo_low) {
    checkEchoFalling(now);
  }
}

// ================== Ultrasonic Measurement Functions ==================
void triggerUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(ULTRASONIC_ACTIVE_TIME);
  digitalWrite(TRIG_PIN, LOW);

  wait_echo_high = true;
}

void checkEchoRising(unsigned long now) {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echo_start = micros();
    wait_echo_high = false;
    wait_echo_low = true;
  }
  else if (now - last_ultrasonic_time > 50) {
    wait_echo_high = false;
    ultrasonic_distance = OUT_OF_RANGE;
  }
}

void checkEchoFalling(unsigned long now) {
  unsigned long duration = micros() - echo_start;

  if (duration > ULTRASONIC_MAX_DURATION || digitalRead(ECHO_PIN) == LOW) {
    wait_echo_low = false;

    if (duration >= ULTRASONIC_MIN_DURATION && duration <= ULTRASONIC_MAX_DURATION) {
      ultrasonic_distance = duration / 58.0; // Convert µs to cm
    }
    else {
      ultrasonic_distance = OUT_OF_RANGE;
    }

    printUltrasonicData(now);
  }
}

// ================== Serial & LCD Output ==================
void printUltrasonicData(unsigned long now) {
  unsigned long total_seconds = now / 1000;
  unsigned int hh = (total_seconds / 3600) % 24;
  unsigned int mm = (total_seconds / 60) % 60;
  unsigned int ss = total_seconds % 60;

  float angle = stepper.currentPosition() * 360.0 / STEPS_PER_REV;
  float distance = ultrasonic_distance;

  char angle_str[12];
  char dist_str[13];
  dtostrf(angle, 6, 2, angle_str);

  if (distance == OUT_OF_RANGE) {
    strcpy(dist_str, "OUT OF RANGE");
  } else {
    dtostrf(distance, 6, 2, dist_str);
  }

  sprintf(output_data, "[%02u:%02u:%02u,%.9s,%.13s]", hh, mm, ss, angle_str, dist_str);
  Serial.println(output_data);

  unsigned long total_row_time = millis();
  if (total_row_time - total_row_last_Update >= fresh_rate * 0.5) {
    total_row_last_Update = total_row_time;
    
    if (scanningEnabled && ultrasonicEnabled) {
      lcd.setCursor(0, 0);
      lcd.print("Angle: ");
      lcd.print(angle, 2);
      lcd.print("         ");

      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distance, 2);
      lcd.print("         ");
    }
    else if (scanningEnabled && !ultrasonicEnabled) {
      lcd.setCursor(0, 0);
      lcd.print("Angle: ");
      lcd.print(angle, 2);
      lcd.print("         ");

      lcd.setCursor(0, 1);
      lcd.print("Dist:  STOP     ");
    }
    else if (!scanningEnabled && ultrasonicEnabled) {
      lcd.setCursor(0, 0);
      lcd.print("Angle: STOP     ");

      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distance, 2);
      lcd.print("         ");
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("Angle: STOP     ");
      lcd.setCursor(0, 1);
      lcd.print("Dist:  STOP     ");
    }
  } 
}
