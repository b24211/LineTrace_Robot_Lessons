// 5.2 演習3：PWM制御による基本ライントレース
#define speedPinR 5
#define speedPinL 6
#define RightMotorDirPin1 7
#define RightMotorDirPin2 8
#define LeftMotorDirPin1 9
#define LeftMotorDirPin2 10

#define LFSensor_0 A0 // 左端
#define LFSensor_1 A1 // 左
#define LFSensor_2 A2 // 中央
#define LFSensor_3 A3 // 右
#define LFSensor_4 A4 // 右端

void setup() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(LFSensor_0, INPUT);
  pinMode(LFSensor_1, INPUT);
  pinMode(LFSensor_2, INPUT);
  pinMode(LFSensor_3, INPUT);
  pinMode(LFSensor_4, INPUT);

  stop_Stop();
}

void loop() {
  int s0 = digitalRead(LFSensor_0);
  int s1 = digitalRead(LFSensor_1);
  int s2 = digitalRead(LFSensor_2);
  int s3 = digitalRead(LFSensor_3);
  int s4 = digitalRead(LFSensor_4);

  if (s2 == HIGH)                   go_Advance(); // 中央が黒線：前進
  else if (s0 == HIGH || s1 == HIGH) go_Left();    // 左側が黒線：左旋回
  else if (s3 == HIGH || s4 == HIGH) go_Right();   // 右側が黒線：右旋回
  else                              stop_Stop();  // 黒線なし：停止
}

void go_Advance() {
  analogWrite(speedPinL, 80);
  analogWrite(speedPinR, 80);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void go_Left() {
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 80);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void go_Right() {
  analogWrite(speedPinL, 80);
  analogWrite(speedPinR, 0);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void stop_Stop() {
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
}
