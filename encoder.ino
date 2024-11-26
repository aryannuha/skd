void readEncoder() {
  int b = digitalRead(ENC_B);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}
