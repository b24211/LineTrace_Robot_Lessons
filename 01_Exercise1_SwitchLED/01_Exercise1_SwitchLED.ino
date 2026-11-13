// 3.3 演習1：タクトスイッチによるLED点灯制御
const int BUTTON_PIN = A0;
const int LED_PIN = A1;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); // スイッチ（押していない時HIGH、押した時LOW）
  pinMode(LED_PIN, OUTPUT);          // LED出力用
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    digitalWrite(LED_PIN, HIGH); // スイッチ押下時：LED点灯
  } else {
    digitalWrite(LED_PIN, LOW);  // スイッチ開放時：LED消灯
  }
}
