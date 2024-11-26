void Compute() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime) / 1000.0; // Waktu dalam detik
  if (timeChange == 0) return; // Hindari pembagian nol

  double error = Setpoint - Input;
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;

  Output = kp * error + ki * errSum + kd * dErr;

  lastErr = error;
  lastTime = now;
}

void SetTunings(double Kp, double Ki, double Kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
}
