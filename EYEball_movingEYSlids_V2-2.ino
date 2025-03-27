#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // I2C address 0x40

// Servo positions (tuned for your setup)
#define SERVO_MIN 120   // 0 degrees
#define SERVO_CENTER 340 // Neutral position (~90°)
#define SERVO_MAX 560   // 180 degrees

// Servo mapping based on your setup
#define LEFT_LOWER_EYELID 0
#define LEFT_UPPER_EYELID 1
#define EYES_UP_DOWN 2
#define EYES_LEFT_RIGHT 3
#define RIGHT_UPPER_EYELID 4
#define RIGHT_LOWER_EYELID 5

#define STEP_SIZE 5   // Smoother motion
#define DELAY_TIME 10 // Milliseconds per step

// Move servo smoothly to a target position
void moveServoSmoothly(uint8_t servo, int target) {
  int currentPos = SERVO_CENTER; // Assume starting at center
  if (target > currentPos) {
    for (int pos = currentPos; pos <= target; pos += STEP_SIZE) {
      pwm.setPWM(servo, 0, pos);
      delay(DELAY_TIME);
    }
  } else {
    for (int pos = currentPos; pos >= target; pos -= STEP_SIZE) {
      pwm.setPWM(servo, 0, pos);
      delay(DELAY_TIME);
    }
  }
}

// Blink animation (eyelids close & open)
void blink() {
  moveServoSmoothly(LEFT_LOWER_EYELID, SERVO_MIN);
  moveServoSmoothly(LEFT_UPPER_EYELID, SERVO_MIN);
  moveServoSmoothly(RIGHT_LOWER_EYELID, SERVO_MAX);
  moveServoSmoothly(RIGHT_UPPER_EYELID, SERVO_MIN);
  delay(200); // Keep eyes closed for a moment
  moveServoSmoothly(LEFT_LOWER_EYELID, SERVO_CENTER);
  moveServoSmoothly(LEFT_UPPER_EYELID, SERVO_CENTER);
  moveServoSmoothly(RIGHT_LOWER_EYELID, SERVO_CENTER);
  moveServoSmoothly(RIGHT_UPPER_EYELID, SERVO_CENTER);
}

// Move eyeballs left and right
void lookLeftRight() {
  moveServoSmoothly(EYES_LEFT_RIGHT, SERVO_MIN);
  delay(500);
  moveServoSmoothly(EYES_LEFT_RIGHT, SERVO_MAX);
  delay(500);
  moveServoSmoothly(EYES_LEFT_RIGHT, SERVO_CENTER);
}

// Move eyeballs up and down
void lookUpDown() {
  moveServoSmoothly(EYES_UP_DOWN, SERVO_MIN);
  delay(500);
  moveServoSmoothly(EYES_UP_DOWN, SERVO_MAX);
  delay(500);
  moveServoSmoothly(EYES_UP_DOWN, SERVO_CENTER);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Smooth Eye Movement Test");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50Hz for servos
  delay(10); // Allow PCA9685 to stabilize

  // Start with neutral positions
  for (int i = 0; i <= 5; i++) {
    pwm.setPWM(i, 0, SERVO_CENTER);
  }
  delay(1000);
}

void loop() {
  Serial.println("Blinking...");
  blink();
  delay(1000);

  Serial.println("Looking left and right...");
  lookLeftRight();
  delay(1000);

  Serial.println("Looking up and down...");
  lookUpDown();
  delay(1000);
}
