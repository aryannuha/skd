#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define ENCA 2 
#define ENCB 3 
#define PWM 10
#define IN2 5
#define IN1 6

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
char tempBuffer[10];

LiquidCrystal_I2C lcd(0x27, 20, 4); // set LCD address to 0x27 for a 20x4 display

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  // Motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  Serial.println("target pos");

  lcd.init();
  lcd.backlight();
}

void loop() {
// set target position
  int target = analogRead(A3) * 0.25;
  // target = 250 * sin(prevT / 1e6);

  // PID constants
  float kp = analogRead(A0) * 0.1;
  float ki = analogRead(A1) * 0.1;
  float kd = analogRead(A2) * 0.01;

  // calculate control signal using PID
  float u = calculatePID(target, pos, kp, kd, ki);

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

  // Print to LCD with one decimal place
  lcd.setCursor(0, 0);
  lcd.print("Sp:");
  lcd.setCursor(3, 0);
  lcd.print(dtostrf(target, 5, 1, tempBuffer));
  lcd.setCursor(8, 0);
  lcd.print("Kp:");
  lcd.setCursor(11, 0);
  lcd.print(dtostrf(kp, 5, 1, tempBuffer));

  lcd.setCursor(0, 1);
  lcd.print("Ki:");
  lcd.setCursor(3, 1);
  lcd.print(dtostrf(ki, 5, 1, tempBuffer));
  lcd.setCursor(8, 1);
  lcd.print("Kd:");
  lcd.setCursor(11, 1);
  lcd.print(dtostrf(kd, 5, 1, tempBuffer));
}



