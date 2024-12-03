#include <LiquidCrystal.h>

// Define Motor Pins (for OUT3 and OUT4 on L298N)
#define IN3 9
#define IN4 10
#define ENABLE 11 // PWM pin for motor speed control

// Define Encoder Pins
#define ENCODER_A 2
#define ENCODER_B 3

// Define Joystick Pin
#define JOYSTICK_PIN A0

// Encoder resolution (pulses per revolution)
#define PPR 410 // Adjust this value based on your encoder's specification

// Variables for Encoder
volatile long encoderCount = 0;
float motorTurns = 0.0;
float motorAngle = 0.0;
int motorSpeed = 0;

// Initialize the LCD (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 13, 4, 5, 6, 7);

// Interrupt Service Routine for Encoder
void encoderISR() {
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);
  if (stateA == stateB) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);

  // LCD setup
  lcd.begin(16, 2); // 16 columns and 2 rows
  lcd.print("Initializing...");

  // Initialize motor pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Initialize joystick pin
  pinMode(JOYSTICK_PIN, INPUT);
}

void loop() {
  // Read joystick value and map it to motor speed
  int joystickValue = analogRead(JOYSTICK_PIN);
  motorSpeed = map(joystickValue, 0, 1023, -255, 255);

  // Set motor direction and speed
  if (motorSpeed > 0) { // Forward direction
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (motorSpeed < 0) { // Backward direction
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else { // Stop the motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENABLE, abs(motorSpeed)); // Set motor speed using PWM

  // Calculate the number of motor turns and angle
  motorTurns = float(encoderCount) / PPR; // Divide encoder counts by PPR to get turns
  motorAngle = (float(encoderCount) / PPR) * 360.0; // Calculate angle in degrees

  // Wrap the angle within 0-360 degrees
  motorAngle = fmod(motorAngle + 360.0, 360.0);

  // Print encoder data to Serial Monitor
  Serial.print("Encoder Count: ");
  Serial.print(encoderCount);
  Serial.print(" Turns: ");
  Serial.print(motorTurns);
  Serial.print(" Angle: ");
  Serial.println(motorAngle);

  // Display data on the LCD
  lcd.setCursor(0, 0); // Move to the first line
  lcd.print("Cnt: ");
  lcd.print(encoderCount);
  lcd.print(" T:");
  lcd.print(motorTurns, 1); // Display turns with 1 decimal place

  lcd.setCursor(0, 1); // Move to the second line
  lcd.print("Ang: ");
  lcd.print(motorAngle, 1); // Display angle with 1 decimal place

  delay(100);
}
