// Fungsi untuk menghitung sinyal kontrol menggunakan PID
float calculatePID(int target, int pos, float kp, float kd, float ki) {
  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / deltaT;

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // store previous error
  eprev = e;

  return u;
}
