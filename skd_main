#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define ENC_A 2
#define ENC_B 3
#define IN1 5
#define IN2 6
#define pwm 10

LiquidCrystal_I2C lcd(0x27, 20, 4); // set LCD address to 0x27 for a 20x4 display

char tempBuffer[10];
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
unsigned long lastTime; // Untuk waktu PID
int pos = 0;

void Compute();
void SetTunings(double Kp, double Ki, double Kd);
void readEncoder();
void setMotor(int dir, int pwmVal, int PWM, int in1, int in2);

void setup() {
  Serial.begin(9600);
  
  // Encoder pins
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, RISING);

  // Motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(pwm, OUTPUT);
  
  // Initialize variables
  lastTime = millis();
  errSum = 0;
  lastErr = 0;

  lcd.init();
  lcd.backlight();
}

void loop() {
  // Update setpoint and PID tunings from potentiometers
  Setpoint = analogRead(A3) * 0.25; // Scale to 0-255
  double Kp = analogRead(A0) * 0.1; // Scale to appropriate range
  double Ki = analogRead(A1) * 0.01; // Scale to appropriate range
  double Kd = analogRead(A2) * 0.1; // Scale to appropriate range
  SetTunings(Kp, Ki, Kd);

  // Update PID input with RPM
  Input = pos;

  // Compute PID
  Compute();

  // Motor power and direction
  float pwr = fabs(Output);
  if (pwr > 255) {
    pwr = 255;
  }

  int dir = (Output < 0) ? -1 : 1;

  // Signal the motor
  setMotor(dir, pwr, pwm, IN1, IN2);
  
  // Debugging output
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(", Pos: ");
  Serial.print(pos);
  Serial.print(", Output: ");
  Serial.println(pwr);

  // Print to LCD with one decimal place
  lcd.setCursor(0, 0);
  lcd.print("Sp:");
  lcd.setCursor(3, 0);
  lcd.print(dtostrf(Setpoint, 5, 1, tempBuffer));
  lcd.setCursor(8, 0);
  lcd.print("Kp:");
  lcd.setCursor(11, 0);
  lcd.print(dtostrf(Kp, 5, 1, tempBuffer));

  lcd.setCursor(0, 1);
  lcd.print("Ki:");
  lcd.setCursor(3, 1);
  lcd.print(dtostrf(Ki, 5, 1, tempBuffer));
  lcd.setCursor(8, 1);
  lcd.print("Kd:");
  lcd.setCursor(11, 1);
  lcd.print(dtostrf(Kd, 5, 1, tempBuffer));
  
  delay(100); // Small delay for stability
}
