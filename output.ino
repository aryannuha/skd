void setMotor(int dir, int pwmVal, int PWM, int in1, int in2) {
  pwmVal = constrain(pwmVal, 0, 255); // Batas nilai PWM 0-255
  analogWrite(PWM, pwmVal);
  
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
