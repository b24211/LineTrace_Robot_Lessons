// 5.3 演習4：周期50ms間欠駆動と加減速・左右差補正の導入
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

  if ((s0 == LOW && s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW) ||
      (s0 == HIGH && s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH)) {
    advanceCount = 0;
    stop_Stop();
  }
  else if (s2 == HIGH && s0 == LOW && s4 == LOW) {
    advanceCount++;
    go_Advance(currentOnTime);
  } 
  else if ((s0 == HIGH && s1 == HIGH) || s0 == HIGH) {
    advanceCount = 0; 
    go_Left_Sharp(10);
  }
  else if ((s3 == HIGH && s4 == HIGH) || s4 == HIGH) {
    advanceCount = 0; 
    go_Right_Sharp(10);
  }
  else if ((s1 == HIGH && s2 == HIGH) || s1 == HIGH) {
    advanceCount = 0; 
    go_Left_Slight(10);
  }
  else if ((s2 == HIGH && s3 == HIGH) || s3 == HIGH) {
    advanceCount = 0; 
    go_Right_Slight(10);
  }
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
