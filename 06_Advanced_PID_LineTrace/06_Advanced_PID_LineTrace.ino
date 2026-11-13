// 6.2 発展：動的ベース速度補正・コース復帰を備えたPID制御ライントレース
#define speedPinR 5
#define speedPinL 6
#define RightMotorDirPin1 7
#define RightMotorDirPin2 8
#define LeftMotorDirPin1 9
#define LeftMotorDirPin2 10

#define LFSensor_0 A0 
#define LFSensor_1 A1 
#define LFSensor_2 A2 
#define LFSensor_3 A3 
#define LFSensor_4 A4 

// 【左右差の補正値（ms）】
int offsetL = -1; 
int offsetR = 0; 

int advanceCount = 0;

// 【PID制御パラメータ】
float Kp = 1.5; // 比例ゲイン（ズレに対する反応の強さ）
float Kd = 2.5; // 微分ゲイン（振れ・曲がりすぎを抑えるカウンターブレーキ）
float Ki = 0.0; // 積分ゲイン（基本は0でよい）

// 【制御用状態変数】
int previousError = 0;
int integral = 0;
int lastKnownError = 0;
unsigned long lostLineTime = 0; 
bool isLost = false;

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

  // 全て黒の場合は交差点とみなして停止
  if (s0 == HIGH && s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH) {
    advanceCount = 0;
    previousError = 0;
    integral = 0;
    stop_Stop();
    return;
  }

  // ① 5chセンサの加重平均による現在位置（エラー値：-20〜+20）の計算
  int activeSensors = 0;
  int sensorSum = 0;
  
  if (s0 == HIGH) { sensorSum -= 20; activeSensors++; }
  if (s1 == HIGH) { sensorSum -= 10; activeSensors++; }
  if (s2 == HIGH) { sensorSum += 0;  activeSensors++; }
  if (s3 == HIGH) { sensorSum += 10; activeSensors++; }
  if (s4 == HIGH) { sensorSum += 20; activeSensors++; }

  int error = 0;

  // ② ライン検出状況の判定
  if (activeSensors > 0) {
    error = sensorSum / activeSensors; // -20(左) 〜 +20(右)
    lastKnownError = error;
    isLost = false;
  } else {
    // 完全にラインを見失った場合（全て白）
    if (!isLost) {
      isLost = true;
      lostLineTime = millis();
    }
    // 2秒間は直前の記憶を元に最大旋回（±25）で復帰を試みる
    if (millis() - lostLineTime < 2000) {
      error = (lastKnownError > 0) ? 25 : -25;
    } else {
      advanceCount = 0;
      stop_Stop();
      return;
    }
  }

  // ③ 直進カウントの更新（エラーが±5以内なら直進とみなす）
  if (abs(error) <= 5 && !isLost) {
    advanceCount++;
  } else {
    advanceCount = 0;
  }

  // ④ 動的ベース速度の計算（直進は加速、カーブは大幅に減速）
  int baseSpeed = 12;
  if (advanceCount > 10) baseSpeed = 14;
  if (advanceCount > 20) baseSpeed = 16;
  if (advanceCount > 30) baseSpeed = 18;
  if (advanceCount > 40) baseSpeed = 20;

  // ズレた瞬間に直進スピードを強力に削る
  int curvePenalty = abs(error); 
  baseSpeed = baseSpeed - curvePenalty;
  if (baseSpeed < 5) baseSpeed = 5; // 最低ベース速度

  // ⑤ PID値の計算
  int P = error;
  integral += error;
  int I = integral;
  int D = error - previousError;
  previousError = error;

  // 補正値の決定
  int correction = (Kp * P) + (Ki * I) + (Kd * D);

  // ⑥ モーター出力の決定
  int onL = baseSpeed + correction;
  int onR = baseSpeed - correction;

  // 異常な値（0未満や上限45ms以上）にならないよう制限
  if (onL < 0) onL = 0;
  if (onR < 0) onR = 0;
  if (onL > 45) onL = 45;
  if (onR > 45) onR = 45;

  set_Forward_Dir();
  drive_Motors(onL + offsetL, onR + offsetR);
}

void drive_Motors(int onL, int onR) {
  digitalWrite(speedPinL, HIGH);
  digitalWrite(speedPinR, HIGH);

  if (onL < onR) {
    delay(onL);
    digitalWrite(speedPinL, LOW);
    delay(onR - onL);
    digitalWrite(speedPinR, LOW);
    delay(50 - onR);
  } else {
    delay(onR);
    digitalWrite(speedPinR, LOW);
    delay(onL - onR);
    digitalWrite(speedPinL, LOW);
    delay(50 - onL);
  }
}

void set_Forward_Dir() {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void stop_Stop() {
  digitalWrite(speedPinL, HIGH);
  digitalWrite(speedPinR, HIGH);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, HIGH);
}
