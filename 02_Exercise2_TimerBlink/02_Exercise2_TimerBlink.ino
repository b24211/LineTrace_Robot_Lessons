// 4.3 演習2：タイマを用いたLED点滅（Timer1 CTCモード 1ms周期割込み）
volatile unsigned int msCount = 0;
volatile bool toggle = false;

void setup() {
  pinMode(13, OUTPUT);

  noInterrupts();
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12) | (1 << CS11); // CTCモード設定 & 1/8分周（1カウント=0.5us）
  OCR1A = 1999;                        // 2000カウント（0〜1999）で1ms経過
  TIMSK1 |= (1 << OCIE1A);             // 1msごとの比較一致割込みを許可
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  msCount++;
  if (msCount >= 500) {                // 1ms × 500回 = 0.5秒経過したら
    msCount = 0;
    toggle = !toggle;                  // 点灯・消灯状態を反転
    digitalWrite(13, toggle ? HIGH : LOW);
  }
}

void loop() {
  // ハードウェアタイマ割込みで自動点滅するため、loop内は空で動作する
}
