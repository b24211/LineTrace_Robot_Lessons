// 5.4 Step 4（応用演習）：直進判定の緩和とコースアウト自動復帰（2秒間）
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

// 【コースアウト復帰用変数】
int lastLineState = 0; // 0:直進/不明, 1:左寄り, 2:右寄り
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

  // 直前の反応方向を記憶する
  if (s0 == HIGH || s1 == HIGH) {
    lastLineState = 1; // 左にラインがあった
    isLost = false;
  } else if (s4 == HIGH || s3 == HIGH) {
    lastLineState = 2; // 右にラインがあった
    isLost = false;
  } else if (s2 == HIGH) {
    isLost = false;    // 真ん中で踏んでいる場合はロスト解除
  }

  // 直進時のスピード計算
  int currentOnTime = 10;
  if (advanceCount > 10) currentOnTime = 11;
  if (advanceCount > 20) currentOnTime = 12;
  if (advanceCount > 30) currentOnTime = 13;
  if (advanceCount > 40) currentOnTime = 14;
  if (advanceCount > 50) currentOnTime = 15;
  if (advanceCount > 60) currentOnTime = 16; 
  if (advanceCount > 70) currentOnTime = 17;
  if (advanceCount > 80) currentOnTime = 18;
  if (advanceCount > 90) currentOnTime = 19;
  if (advanceCount > 100) currentOnTime = 20;
  if (advanceCount > 110) currentOnTime = 22; 

  // ① 完全にラインを見失った場合（すべて白）の復帰処理
  if (s0 == LOW && s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW) {
    if (!isLost) {
      isLost = true;
      lostLineTime = millis(); // 見失った瞬間の時刻を記録
    }
    
    // 2秒間(2000ms)は直前の記憶を頼りに強めに旋回してコース復帰を試みる
    if (millis() - lostLineTime < 2000) {
      if (lastLineState == 1) {
        go_Left_Sharp(12);  // 左へ復帰（少し強めに回す）
      } else if (lastLineState == 2) {
        go_Right_Sharp(12); // 右へ復帰（少し強めに回す）
      } else {
        stop_Stop();
      }
    } else {
      // 2秒経過してもラインが見つからなければ停止
      advanceCount = 0;
      stop_Stop();
    }
  }
  // すべて黒の場合は交差点とみなして停止
  else if (s0 == HIGH && s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH) {
    advanceCount = 0;
    stop_Stop();
  }
  // ② 限界まで直進（真ん中のs2が黒線を踏んでいれば無条件で直進・加速）
  else if (s2 == HIGH) {
    advanceCount++;
    go_Advance(currentOnTime);
  } 
  // ③ 急カーブ
  else if (s0 == HIGH) {
    advanceCount = 0; 
    go_Left_Sharp(10);
  }
  else if (s4 == HIGH) {
    advanceCount = 0; 
    go_Right_Sharp(10);
  }
  // ④ 緩やかなカーブ
  else if (s1 == HIGH) {
    advanceCount = 0; 
    go_Left_Slight(10);
  }
  else if (s3 == HIGH) {
    advanceCount = 0; 
    go_Right_Slight(10);
  }
  // ⑤ フェールセーフ
  else {
    advanceCount = 0;
    stop_Stop();
  }
}

void set_Forward_Dir() {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

void go_Advance(int baseOnT) {
  set_Forward_Dir();
  drive_Motors(baseOnT + offsetL, baseOnT + offsetR);
}

void go_Left_Sharp(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 4 / 10; 
  drive_Motors(innerOnT + offsetL, baseOnT + offsetR);
}

void go_Right_Sharp(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 4 / 10;
  drive_Motors(baseOnT + offsetL, innerOnT + offsetR);
}

void go_Left_Slight(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 7 / 10;
  drive_Motors(innerOnT + offsetL, baseOnT + offsetR);
}

void go_Right_Slight(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 7 / 10;
  drive_Motors(baseOnT + offsetL, innerOnT + offsetR);
}

void drive_Motors(int onL, int onR) {
  if (onL < 0) onL = 0;
  if (onR < 0) onR = 0;

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

void stop_Stop() {
  digitalWrite(speedPinL, HIGH);
  digitalWrite(speedPinR, HIGH);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, HIGH);
}
