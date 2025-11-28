#include <Servo.h>      // Include Servo library

// Ultrasonic sensor pins
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

// Servo pin
const int SERVO_PIN = 10;

// L298N motor driver pins
const int IN1 = 2;      // Left motor direction 1
const int IN2 = 3;      // Left motor direction 2
const int IN3 = 4;      // Right motor direction 1
const int IN4 = 7;      // Right motor direction 2

const int ENA = 5;      // PWM - speed control for left motors
const int ENB = 6;      // PWM - speed control for right motors

// Movement / distance related constants
const int SAFE_DISTANCE = 20;       // in cm, threshold for obstacle
const int BASE_SPEED = 180;         // 0–255 PWM (you can tune this)

// Servo angle definitions
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 150;
const int SERVO_RIGHT = 30;

Servo scanServo;   // Servo object

// Measure distance using ultrasonic sensor
long getDistanceCM() {
  long duration;
  long distance;

  // Send 10us pulse on TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time
  duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms

  // If nothing received, return a large distance
  if (duration == 0) {
    return 400; // assume no obstacle
  }

  // Convert time to distance (cm)
  distance = (duration * 0.0343) / 2;

  return distance;
}

// Take multiple distance samples and average them
int getAverageDistance(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += getDistanceCM();
    delay(30);
  }
  return sum / samples;
}

// Motor control functions
void moveForward() {
  // Left motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Right motors forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void moveBackward() {
  // Left motors backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Right motors backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void turnLeft() {
  // Left motors backward, right motors forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // left backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // right forward

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void turnRight() {
  // Left motors forward, right motors backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // left forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // right backward

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Servo helper functions
void lookCenter() {
  scanServo.write(SERVO_CENTER);
  delay(300);
}

int lookLeftDistance() {
  scanServo.write(SERVO_LEFT);
  delay(400);
  return getAverageDistance(3);
}

int lookRightDistance() {
  scanServo.write(SERVO_RIGHT);
  delay(400);
  return getAverageDistance(3);
}

void setup() {
  // Serial for debugging (optional)
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach servo
  scanServo.attach(SERVO_PIN);
  lookCenter();

  // Initially stop
  stopCar();
  delay(500);
}

void loop() {
  // Look forward
  lookCenter();
  int distance = getAverageDistance(3);

  Serial.print("Forward distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > SAFE_DISTANCE) {
    // Path is clear → move forward
    moveForward();
  } else {
    // Obstacle detected
    stopCar();
    delay(300);

    // Check left & right
    int leftDist = lookLeftDistance();
    Serial.print("Left distance: ");
    Serial.println(leftDist);

    int rightDist = lookRightDistance();
    Serial.print("Right distance: ");
    Serial.println(rightDist);

    // Return servo to center
    lookCenter();

    if (leftDist > rightDist && leftDist > SAFE_DISTANCE) {
      // Turn left
      turnLeft();
      delay(400);   // tune this for your robot
    } else if (rightDist >= leftDist && rightDist > SAFE_DISTANCE) {
      // Turn right
      turnRight();
      delay(400);
    } else {
      // If both sides blocked, go backward a bit
      moveBackward();
      delay(400);
    }

    stopCar();
    delay(200);
  }

  delay(50); // small delay before next cycle
}
