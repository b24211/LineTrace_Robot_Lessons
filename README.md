# 機械工作・制御実習

**作成日**：令和8年〇月〇日  
**作成者**：B24211  

---

## 目次

1. [目的](#1-目的)
2. [開発環境の設定](#2-開発環境の設定)
3. [IO操作とタクトスイッチの基礎](#3-io操作とタクトスイッチの基礎)
   - [3.1 IO操作とタクトスイッチの仕組み](#31-io操作とタクトスイッチの仕組み)
   - [3.2 プルアップ,プルダウン接続](#32-プルアッププルダウン接続)
   - [3.3 演習1(タクトスイッチによるLED点灯制御)](#33-演習1タクトスイッチによるled点灯制御)
4. [タイマ制御とLED点滅](#4-タイマ制御とled点滅)
   - [4.1 ソフトウェアタイマとハードウェアタイマの違い](#41-ソフトウェアタイマとハードウェアタイマの違い)
   - [4.2 分周とタイマ割込みの仕組み](#42-分周とタイマ割込みの仕組み)
   - [4.3 演習2(タイマを用いたLED点滅)](#43-演習2タイマを用いたled点滅)
5. [ライントレースプログラムの段階的構築](#5-ライントレースプログラムの段階的構築)
   - [5.1 Step 1：ハードウェア構成とピン定義](#51-step-1ハードウェア構成とピン定義)
   - [5.2 Step 2：PWM制御による基本ライントレース（演習3）](#52-step-2pwm制御による基本ライントレース演習3)
   - [5.3 Step 3：間欠駆動と加減速・左右差補正の導入（演習4）](#53-step-3間欠駆動と加減速左右差補正の導入演習4)
   - [5.4 Step 4：直進判定の緩和とコースアウト自動復帰（応用演習）](#54-step-4直進判定の緩和とコースアウト自動復帰応用演習)
6. [発展：PID制御による高度なライントレース](#6-発展pid制御による高度なライントレース)
   - [6.1 条件分岐制御の限界とPID制御の概要](#61-条件分岐制御の限界とpid制御の概要)
   - [6.2 5chセンサによる偏差の算出と動的ベース速度補正付きPID演算の実装](#62-5chセンサによる偏差の算出と動的ベース速度補正付きpid演算の実装)
   - [6.3 PIDゲインの調整手順](#63-pidゲインの調整手順)

---

## 1. 目的

本実習では、Arduino Unoを用いた開発環境の設定方法を習得し、基本的なデジタル入出力（IO）操作やハードウェアタイマによる割込み制御を理解した上で、それらを段階的に組み合わせてライントレースロボットの制御プログラムを構築できるようになることを目的とする。

---

## 2. 開発環境の設定

**使用ツール**：Arduino IDE（プログラムの作成とArduino Unoへの書き込みに使用する）

* Arduino IDEを起動し、Arduino UnoをPCに接続する
* ツールメニューのボード設定から「Arduino Uno」を選択する
  <!-- 画像を表示する場合はリポジトリに画像をアップロードし、パスを指定してください -->
  <!-- ![ボード設定](images/board_setting.png) -->
* ツールメニューのシリアルポートから対応するCOMポートを選択する
  <!-- ![シリアルポート設定](images/port_setting.png) -->
* 画面左上のチェックマークボタン（検証）をクリックし、プログラムに文法上の誤りがないか確認する
* 検証が完了したら、右矢印ボタン（マイコンボードに書き込む）をクリックし、Arduino Uno本体へプログラムを転送する

### サンプルコードの公開先（GitHub）

本テキスト内では、各章の学習項目で重要となるプログラムの抜粋を掲載している。各演習の完成版サンプルコード一式は、本リポジトリ内の各フォルダにて公開している。

* **GitHubリポジトリURL**： `https://github.com/b24211/LineTrace_Robot_Lessons`

### GitHubリポジトリのファイル階層

```text
LineTrace_Robot_Lessons/
├── README.md                              (本テキスト全文・各演習の解説)
├── 01_Exercise1_SwitchLED/
│   └── 01_Exercise1_SwitchLED.ino         (3.3 演習1：タクトスイッチとLED点灯制御)
├── 02_Exercise2_TimerBlink/
│   └── 02_Exercise2_TimerBlink.ino        (4.3 演習2：Timer1割込みによるLED点滅)
├── 03_Exercise3_BasicPWMTrace/
│   └── 03_Exercise3_BasicPWMTrace.ino     (5.2 演習3：analogWriteによる基本トレース)
├── 04_Exercise4_SteppedPulseTrace/
│   └── 04_Exercise4_SteppedPulseTrace.ino (5.3 演習4：周期50ms間欠駆動・緩加速・2段階旋回)
├── 05_Applied_CourseRecovery/
│   └── 05_Applied_CourseRecovery.ino      (5.4 応用：直進判定緩和・2秒間コースアウト自動復帰)
└── 06_Advanced_PID_LineTrace/
    └── 06_Advanced_PID_LineTrace.ino      (6.2 発展：動的ベース速度減速＆復帰付きPD/PID制御)
```

---

## 3. IO操作とタクトスイッチの基礎

### 3.1 IO操作とタクトスイッチの仕組み

IO操作とは、Input/Output（入出力）の略称であり、スイッチやセンサから情報を読み取る入力処理（Input）と、計算結果に基づいてLEDやモータなどを動かす出力処理（Output）の総称である。マイコン制御のプログラムは、初期化を行う `setup()` 関数と、入力の読み取り・条件判定・出力を繰り返す `loop()` 関数で構成される。

本演習では、入力部品としてタクトスイッチを使用する。タクトスイッチは、ボタンを押している間だけ内部の接点がつながって電気を通し（ON）、指を離すとバネの力で接点が離れて電気が遮断される（OFF）部品である。

#### IO操作で使用する3つの基本関数
* `pinMode(ピン番号, モード)` : ピンの役割を設定する。モードには `INPUT`（入力）、`OUTPUT`（出力）、`INPUT_PULLUP`（内部プルアップ有効入力）を指定する。
* `digitalRead(ピン番号)` : 指定したピンの電圧状態を読み取り、5Vなら `HIGH`、0Vなら `LOW` を返す。
* `digitalWrite(ピン番号, 状態)` : 指定したピンから電圧を出力する。`HIGH` で5V出力、`LOW` で0V出力となる。

### 3.2 プルアップ,プルダウン接続

タクトスイッチがOFF（開放状態）のとき、マイコンの入力ピンが電源（5V）にもGND（0V）にもつながっていない状態（浮いた状態）になると、周囲のノイズを拾って勝手にON/OFFが切り替わる誤作動が発生する。これを防ぐため、抵抗器を使って普段の電圧を5V（`HIGH`）または0V（`LOW`）のどちらかに固定する必要がある。

| 接続方式 | スイッチOFF時（通常） | スイッチON時（押下） | 特徴・用途 |
| :--- | :--- | :--- | :--- |
| **プルアップ接続** | `HIGH` (5V) | `LOW` (0V) | 抵抗を介して電源側に引き上げる方式。Arduino内蔵の抵抗を利用可能。 |
| **プルダウン接続** | `LOW` (0V) | `HIGH` (5V) | 抵抗を介してGND側に引き下げる方式。直感的な正論理で扱える。 |

#### プルアップ回路とプルダウン回路の接続図

```text
【プルアップ回路】                  【プルダウン回路】
  VCC (5V)                            VCC (5V)
   │                                   │
  [抵抗]                              [スイッチ]
   │                                   │
   ├────── マイコン入力ピン             ├────── マイコン入力ピン
   │                                   │
 [スイッチ]                           [抵抗]
   │                                   │
  GND (0V)                            GND (0V)
```

Arduino Unoのマイコン内部には、あらかじめプルアップ用の抵抗が組み込まれている。プログラム上で `INPUT_PULLUP` を指定して内部プルアップ機能を有効にすれば、外付けの抵抗器や5Vピンへの配線を省略し、スイッチを入力ピンとGNDピンにつなぐだけで安定したプルアップ回路を構成できる。

### 3.3 演習1(タクトスイッチによるLED点灯制御)

* 完成コード：[01_Exercise1_SwitchLED.ino](./01_Exercise1_SwitchLED/01_Exercise1_SwitchLED.ino)

#### 制作要件と配線
* タクトスイッチをA0ピン（内部プルアップ有効）、LEDをA1ピン（抵抗経由）とGNDに接続すること。
* タクトスイッチを押している間（`LOW` のとき）だけLEDが点灯し、離すと消灯するプログラムを作成すること。

#### 重要コード抜粋：内部プルアップ入力と条件分岐

```cpp
// 1. setup()内でのピン設定（内部プルアップを有効化）
pinMode(A0, INPUT_PULLUP); // スイッチ（押していない時HIGH、押した時LOW）
pinMode(A1, OUTPUT);       // LED出力用

// 2. loop()内での入力読み取りと条件分岐（重要部分）
int buttonState = digitalRead(A0);
if (buttonState == LOW) {
  digitalWrite(A1, HIGH);  // スイッチ押下時：LED点灯
} else {
  digitalWrite(A1, LOW);   // スイッチ開放時：LED消灯
}
```

ここで学んだ「`digitalRead()` でセンサやスイッチの状態を読み取り、`if-else` 文で出力を切り替える」という構造が、第5章で作成するライントレースプログラムの骨格となる。

---

## 4. タイマ制御とLED点滅

### 4.1 ソフトウェアタイマとハードウェアタイマの違い

プログラムを一時停止させて時間を待つ簡単な方法として、`delay()` という命令（ソフトウェアタイマ）がある。しかし、これを使うとマイコン自身が1秒、2秒とじっと時計を見て時間を数える状態になり、この間、マイコンは他の作業（スイッチの確認やセンサの読み取りなど）が一切できなくなる。

これに対しハードウェアタイマは、マイコン内部にあるキッチンタイマーのような専用の装置である。時間をセットしておけば、マイコンは別の作業をしながら待つことができる。設定した時間が来るとタイマがマイコンに合図（割込み）を送り、あらかじめ決めておいた処理を自動で行う仕組みである。

### 4.2 分周とタイマ割込みの仕組み

Arduino Unoの内部時計（クロック）は1秒間に1600万回（16MHz、1カウント62.5ナノ秒）という非常に速いペースで進んでいる。そのまま数えるとすぐに上限に達してしまうため、歯車で時計の進みを遅くするように、分周という設定を行ってカウント速度を遅くする。例えばクロックを8回に1回だけ数える設定（1/8分周）にすると、1カウントは0.5マイクロ秒となり、2000回カウントするとちょうど1ミリ秒（0.001秒）になる。この1ミリ秒ごとに割込み合図を出す設定（CTCモード）を行い、割込み回数を500回数えることで0.5秒を正確に計測できる。

### 4.3 演習2(タイマを用いたLED点滅)

* 完成コード：[02_Exercise2_TimerBlink.ino](./02_Exercise2_TimerBlink/02_Exercise2_TimerBlink.ino)

#### 制作要件
* Arduino Unoの13番ピン（基板内蔵LED、外部配線不要）を0.5秒間隔で点滅させること。
* `delay()` 関数を使用せず、ハードウェアタイマ（Timer1）の1ミリ秒周期割込みを用いて時間を計測すること。

#### 重要コード抜粋：タイマレジスタ設定と割込みルーチン

```cpp
// 1. setup()内でのTimer1初期設定（1/8分周・1ms周期のCTCモード）
TCCR1A = 0x00;
TCCR1B = (1 << WGM12) | (1 << CS11); // CTCモード設定 & 1/8分周（1カウント=0.5us）
OCR1A = 1999;                        // 2000カウント（0〜1999）で1ms経過
TIMSK1 |= (1 << OCIE1A);             // 1msごとの比較一致割込みを許可

// 2. 1msごとに自動実行される割込み処理（重要部分）
ISR(TIMER1_COMPA_vect) {
  msCount++;
  if (msCount >= 500) {              // 1ms * 500回 = 0.5秒経過したら
    msCount = 0;
    toggle = !toggle;                // 点灯・消灯状態を反転
    digitalWrite(13, toggle ? HIGH : LOW);
  }
}
```

---

## 5. ライントレースプログラムの段階的構築

本章では、これまでの入出力操作と時間制御の知識を組み合わせ、ステップバイステップでライントレースロボットのプログラムを組み立てていく。Step 1から順番にコードのパーツを作成・追加していくことで、最終的に安定したライントレースプログラムが完成する構成となっている。

### 5.1 Step 1：ハードウェア構成とピン定義

ライントレースロボットは、車体底面の5チャンネル赤外線ラインセンサで床面の黒線を検知し、モータドライバを介して左右2つのDCモータを制御する。黒線の上ではセンサ出力が `HIGH`（1）、白い床面では `LOW`（0）となる。各ピンの役割を以下に示す。

| 機能分類 | 定数名 | ピン番号 | 役割・動作説明 |
| :--- | :--- | :--- | :--- |
| **右モータ制御** | `speedPinR` | D5 | 右モータの駆動出力（PWMまたはON/OFFパルス） |
| **右モータ制御** | `RightMotorDirPin1 / 2` | D7 / D8 | 右モータの回転方向（`D7=HIGH`, `D8=LOW` で前進） |
| **左モータ制御** | `speedPinL` | D6 | 左モータの駆動出力（PWMまたはON/OFFパルス） |
| **左モータ制御** | `LeftMotorDirPin1 / 2` | D9 / D10 | 左モータの回転方向（`D9=HIGH`, `D10=LOW` で前進） |
| **ラインセンサ** | `LFSensor_0 〜 4` | A0 〜 A4 | 赤外線センサ（A0:左端, A1:左, A2:中央, A3:右, A4:右端） |

#### Step 1 コード：ピン番号のマクロ定義

```cpp
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

// setup()内ではモータ制御ピン(5〜10)をOUTPUT、センサピン(A0〜A4)をINPUTに設定する
```

### 5.2 Step 2：PWM制御による基本ライントレース（演習3）

* 完成コード：[03_Exercise3_BasicPWMTrace.ino](./03_Exercise3_BasicPWMTrace/03_Exercise3_BasicPWMTrace.ino)

まずは最も単純な構成として、`analogWrite(ピン番号, 0〜255)` 関数によるPWM制御でモータ速度を `80` に固定し、センサの反応位置に応じて「前進・左旋回・右旋回・停止」の4パターンに切り替える基本プログラムを組み立てる。

#### 演習3 制作要件
* Step 1のピン定義に続き、以下の基本動作関数と `loop()` 内の条件分岐を実装して速度80でライントレースさせること。
* 中央センサ（A2）が `HIGH` なら前進、左（A0, A1）が `HIGH` なら左旋回、右（A3, A4）が `HIGH` なら右旋回、すべて `LOW` なら停止とすること。

#### Step 2 コード：PWM駆動関数とセンサ条件分岐

```cpp
// 1. 基本の前進関数（左右旋回・停止も同様にDirPinのHIGH/LOWを組み合わせて作成する）
void go_Advance() {
  analogWrite(speedPinL, 80);
  analogWrite(speedPinR, 80);
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
}

// 2. loop()内でのセンサ読み取りと基本分岐
int s0 = digitalRead(LFSensor_0);
int s1 = digitalRead(LFSensor_1);
int s2 = digitalRead(LFSensor_2);
int s3 = digitalRead(LFSensor_3);
int s4 = digitalRead(LFSensor_4);

if (s2 == HIGH)                   go_Advance(); // 中央が黒線：前進
else if (s0 == HIGH || s1 == HIGH) go_Left();    // 左側が黒線：左旋回
else if (s3 == HIGH || s4 == HIGH) go_Right();   // 右側が黒線：右旋回
else                              stop_Stop();  // 黒線なし：停止
```

### 5.3 Step 3：間欠駆動と加減速・左右差補正の導入（演習4）

* 完成コード：[04_Exercise4_SteppedPulseTrace.ino](./04_Exercise4_SteppedPulseTrace/04_Exercise4_SteppedPulseTrace.ino)

Step 2のプログラムを実機で動かすと、`analogWrite()` の値が80では平均電圧が低すぎてタイヤの摩擦に負け、モータが回らないことがある。このトルク不足を解消し、コースを滑らかに安定周回させるため、Step 2のプログラムに以下の3つの改良パーツを組み込む。

#### 改良①：周期50msの間欠駆動と左右差補正（drive_Motors関数）
`analogWrite()` の代わりに `digitalWrite()` を用い、1周期50ミリ秒のうち「最初の10ミリ秒だけ `HIGH`（100%フルパワー）で回して強い起動トルクを得て、残りの40ミリ秒は `LOW` にして惰性で進ませる」間欠駆動を行う。さらに、左右モータの個体差による直進時の曲がり癖を直すため、補正値（`offsetL`, `offsetR`）で左右のON時間を1〜2ミリ秒微調整できるようにする。また、停止時（`stop_Stop` 関数）にはすべての制御ピンを `HIGH` にしてショートブレーキをかけ、惰性によるオーバーランを防ぐ。

#### Step 3 コード①：左右差補正付き間欠駆動パルス生成

```cpp
int offsetL = -1; // 左モータが強くて右へ寄る場合、左のON時間を1ms減らす
int offsetR = 0;

// 左右のON時間（onL, onR）を受け取り、合計50ms周期のパルスを生成する関数
void drive_Motors(int onL, int onR) {
  if (onL < 0) onL = 0;
  if (onR < 0) onR = 0;
  digitalWrite(speedPinL, HIGH);
  digitalWrite(speedPinR, HIGH);

  if (onL < onR) {
    delay(onL);        digitalWrite(speedPinL, LOW); // 短い左を先にOFF
    delay(onR - onL);  digitalWrite(speedPinR, LOW); // 差分の後に右をOFF
    delay(50 - onR);                                 // 残りのOFF時間待機
  } else {
    delay(onR);        digitalWrite(speedPinR, LOW);
    delay(onL - onR);  digitalWrite(speedPinL, LOW);
    delay(50 - onL);
  }
}
```

#### 改良②：内輪減速による2段階カーブ旋回と減速比の微調整
旋回時に片輪を逆回転させると車体が大きく振られて脱線しやすいため、両輪とも前進方向（`set_Forward_Dir`）に固定したまま、内側のセンサ（A1またはA3）が反応した「緩やかなカーブ」では内輪のON時間を外輪の70%に落とし、外側のセンサ（A0またはA4）が反応した「急カーブ」では内輪のON時間を40%に落とすことで滑らかに曲がらせる。なお、実機の摩擦特性によって左右の振れ幅が大きい場合は減速比をマイルド（急カーブ50%・緩カーブ80%など）に引き上げ、逆に内輪がトルク不足で止まり内側に巻き込みすぎてコースアウトする（オーバーステア）場合は、内輪の最低ON時間や外輪のパルス幅（10msから12ms等）を微調整する。

#### Step 3 コード②：内輪減速による緩・急カーブ関数

```cpp
// 急カーブ：内輪のON時間を外輪(baseOnT)の40%に落とす（右急カーブも同様に作成）
void go_Left_Sharp(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 4 / 10; 
  drive_Motors(innerOnT + offsetL, baseOnT + offsetR);
}

// 緩やかなカーブ：内輪のON時間を外輪(baseOnT)の70%に落とす（右緩カーブも同様に作成）
void go_Left_Slight(int baseOnT) {
  set_Forward_Dir();
  int innerOnT = baseOnT * 7 / 10;
  drive_Motors(innerOnT + offsetL, baseOnT + offsetR);
}
```

#### 改良③：直線での緩やかな加速とカーブでの即時減速
直進（中央センサA2のみ反応）が連続している間は `advanceCount` を1ずつ増やし、10ループ（約0.5秒）ごとに基準ON時間（10ms）を1msずつ増やして最大22msまで加速させる。カーブを検知した瞬間、またはすべて白（コースアウト）・すべて黒（交差点）を検知したときは `advanceCount` を0にリセットし、安全で安定した10msに即座に戻す。

#### Step 3 コード③：直線緩加速と5段階センサ判定ロジック

```cpp
// 1. 直進連続回数に応じたON時間の段階的引き上げ
int currentOnTime = 10;
if (advanceCount > 10)  currentOnTime = 11;
if (advanceCount > 20)  currentOnTime = 12;
// ・・・（10カウントごとに+1msずつ増加）・・・
if (advanceCount > 110) currentOnTime = 22; // 最高速度の制限

// 2. センサ状態に応じた5段階の判定（loop内）
if ((s0==LOW && s1==LOW && s2==LOW && s3==LOW && s4==LOW) ||
    (s0==HIGH && s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH)) {
  advanceCount = 0; stop_Stop();               // 全白または全黒：停止
} else if (s2 == HIGH && s0 == LOW && s4 == LOW) {
  advanceCount++;   go_Advance(currentOnTime); // 直進：カウントを増やし加速
} else if ((s0 == HIGH && s1 == HIGH) || s0 == HIGH) {
  advanceCount = 0; go_Left_Sharp(10);         // 左急カーブ：10msに減速して旋回
} else if ((s1 == HIGH && s2 == HIGH) || s1 == HIGH) {
  advanceCount = 0; go_Left_Slight(10);        // 左緩カーブ：10msに減速して旋回
} // （※右急カーブ・右緩カーブも同様に記述）
```

#### 演習4 制作要件
* 上記の改良パーツ①〜③を組み合わせ、周期50msの間欠駆動、左右差オフセット調整、直線緩加速、および2段階カーブ旋回を備えたライントレースプログラムを完成させること。

### 5.4 Step 4：直進判定の緩和とコースアウト自動復帰（応用演習）

* 完成コード：[05_Applied_CourseRecovery.ino](./05_Applied_CourseRecovery/05_Applied_CourseRecovery.ino)

Step 3のプログラムをさらに実戦的に発展させるため、コースの黒線幅（センサ3〜4個分の太さ）に合わせた「直進判定の緩和」と、高速進入でカーブの外へ膨らんだ際の「コースアウト自動復帰」の2つの機能を組み込む。

#### 改良④：黒線の太さに合わせた直進判定の緩和
黒線の幅が広い（センサ3〜4個分ある）コースにおいて、端のセンサが反応するたびに毎回減速・旋回していると、直線でも小刻みに減速が入り最高速が維持できない。そこで、中央のセンサ（`s2`）が黒線を捉えている限りは、左右の端のセンサが反応していても無条件で「直進（`s2 == HIGH`）」とみなすように条件を緩和する。これにより、カーブの入口ギリギリまで最高速を維持し、中央センサがラインから外れた瞬間にのみ旋回を開始する高速走行が可能になる。

#### 改良⑤：状態記憶によるコースアウトからの自動復帰（2秒間）
ギリギリまで直進して高速でカーブに進入すると、曲がりきれずにすべてのセンサが白い床面にはみ出る（全白になる）ことがある。このとき即座に停止するのではなく、「見失う直前に左右どちらのセンサが反応していたか（`lastLineState`）」と「見失った瞬間の時刻（`lostLineTime`）」を記憶しておき、全白になってから2秒間（2000ms）は記憶した方向へ強めの旋回（外輪12ms）を継続させることで、粘り強くコースへ自動復帰させる。

#### Step 4 コード：直進判定緩和と2秒間コースアウト復帰ロジック

```cpp
// 【コースアウト復帰用変数】
int lastLineState = 0; // 0:直進/不明, 1:左寄り, 2:右寄り
unsigned long lostLineTime = 0; 
bool isLost = false;

// loop()内：直前の反応方向を記憶
if (s0 == HIGH || s1 == HIGH)      { lastLineState = 1; isLost = false; }
else if (s4 == HIGH || s3 == HIGH) { lastLineState = 2; isLost = false; }
else if (s2 == HIGH)               { isLost = false; }

// ① 完全にラインを見失った場合（すべて白）の2秒間復帰処理
if (s0 == LOW && s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW) {
  if (!isLost) {
    isLost = true;
    lostLineTime = millis(); // 見失った瞬間の時刻を記録
  }
  if (millis() - lostLineTime < 2000) { // 2秒以内なら直前の方向へ強く旋回して復帰
    if (lastLineState == 1)      go_Left_Sharp(12);
    else if (lastLineState == 2) go_Right_Sharp(12);
    else                         stop_Stop();
  } else {
    advanceCount = 0; stop_Stop();      // 2秒経過しても見つからなければ停止
  }
}
// すべて黒の場合は交差点とみなして停止
else if (s0 == HIGH && s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH) {
  advanceCount = 0; stop_Stop();
}
// ② 限界まで直進（真ん中のs2が黒線を踏んでいれば無条件で直進・加速）
else if (s2 == HIGH) {
  advanceCount++;
  go_Advance(currentOnTime);
}
// ③ 中央が外れた瞬間に旋回開始
else if (s0 == HIGH) { advanceCount = 0; go_Left_Sharp(10); }
else if (s4 == HIGH) { advanceCount = 0; go_Right_Sharp(10); }
else if (s1 == HIGH) { advanceCount = 0; go_Left_Slight(10); }
else if (s3 == HIGH) { advanceCount = 0; go_Right_Slight(10); }
```

#### 応用演習 制作要件
* Step 3のプログラムに「直進判定の緩和（`s2 == HIGH`）」および「2秒間のコースアウト自動復帰」を組み込み、高速かつ粘り強くコースを周回するプログラムを完成させること。

---

## 6. 発展：PID制御による高度なライントレース

* 完成コード：[06_Advanced_PID_LineTrace.ino](./06_Advanced_PID_LineTrace/06_Advanced_PID_LineTrace.ino)

### 6.1 条件分岐制御の限界とPID制御の概要

第5章で作成したプログラムは、「どのセンサが反応したか」によってあらかじめ決めた旋回パターン（内輪70%や40%）に切り替える方式（ON/OFF制御・多段制御）である。この方式は仕組みが単純で理解しやすい反面、高速走行させるとラインを飛び越えて左右に激しく揺れる現象（ハンチング・蛇行）や、旋回時に内側に巻き込みすぎて停止する現象（オーバーステア）が起きやすいという課題がある。

より高速かつ滑らかにラインへ追従させるための発展的な制御手法が、PID制御（Proportional-Integral-Derivative Control）である。PID制御では、目標位置（車体中央のセンサA2）と現在の黒線位置とのズレ量（偏差 `e`）を求め、以下の3つの要素を計算して左右のモータ出力を連続的（無段階）に調整する。

| 制御要素 | 名称と意味 | ライントレースにおける役割 | 強すぎた場合の悪影響 |
| :--- | :--- | :--- | :--- |
| **P制御** | Proportional（比例）<br>現在のズレに比例した操作 | ラインから外れた距離に比例して強くステアリングを切る主力制御。 | 反応が過敏になり、ラインを中心に左右へ激しく蛇行（振動）する。 |
| **I制御** | Integral（積分）<br>過去のズレの累積に応じた操作 | モータの個体差や路面の傾きなど、P制御だけでは消えない定常的なズレを補正する。 | 過去のズレを引きずりすぎて反応が遅れ、カーブで大きく膨らむ（ワインドアップ）。 |
| **D制御** | Derivative（微分）<br>ズレの変化速度に応じた操作 | ラインへ急激に戻る動きを検知して逆向きのブレーキをかけ、行き過ぎや蛇行を抑える。 | センサの細かな変化に過剰反応し、車体が小刻みに震える。 |

### 6.2 5chセンサによる偏差の算出と動的ベース速度補正付きPID演算の実装

5チャンネルのデジタルラインセンサ（`s0`〜`s4`）でPID制御を行うには、まず各センサの位置に数値を割り当てる（重み付け）。整数演算でも細かい精度が出せるよう、中央（`s2`）を `0` とし、左へ行くほどマイナス（`s1 = -10`, `s0 = -20`）、右へ行くほどプラス（`s3 = +10`, `s4 = +20`）の重みを設定する。複数のセンサが同時に黒線を検知した際（例：`s0` と `s1` が `HIGH`）は、それらの平均値（`(-20 + -10) / 2 = -15`）をとることで、ラインのズレ量（偏差 `error`）を `-20` 〜 `+20` の範囲で連続的に数値化できる。

さらに、実機で「直進の慣性が強すぎてカーブを曲がりきれずに直進してしまう」現象や「曲がりすぎて止まる」現象を防ぐため、①ズレ量（`abs(error)`）に応じて直進のベース速度を即座に削り落とす「動的ベース速度減速（`curvePenalty`）」、②内輪を瞬時に0msまで落としつつ微分項（`Kd`）で強力なカウンターブレーキを当てる「高ゲインPD制御」、③全白（コースアウト）時に2秒間だけ仮想最大エラー（`+25` または `-25`）を与えて復帰させる「PIDコース復帰処理」の3つを統合する。

#### 発展コード抜粋：重み付け偏差・動的減速・コース復帰を備えたPID制御ループ

```cpp
// 【PID制御パラメータ】※効きを強化し、動的減速と組み合わせた設定
float Kp = 1.5; // 比例ゲイン（ズレに対する反応の強さ）
float Kd = 2.5; // 微分ゲイン（振れ・曲がりすぎを抑えるカウンターブレーキ）
float Ki = 0.0; // 積分ゲイン（基本は0でよい）
int previousError = 0, integral = 0, lastKnownError = 0;
unsigned long lostLineTime = 0; 
bool isLost = false;

// 1. 5chセンサの加重平均による現在位置（エラー値：-20〜+20）の計算
int activeSensors = 0, sensorSum = 0;
if (s0 == HIGH) { sensorSum -= 20; activeSensors++; }
if (s1 == HIGH) { sensorSum -= 10; activeSensors++; }
if (s2 == HIGH) { sensorSum += 0;  activeSensors++; }
if (s3 == HIGH) { sensorSum += 10; activeSensors++; }
if (s4 == HIGH) { sensorSum += 20; activeSensors++; }

int error = 0;
if (activeSensors > 0) {
  error = sensorSum / activeSensors; // ライン上のズレ量（-20 〜 +20）
  lastKnownError = error;            // 見失った時用に記憶
  isLost = false;
} else {
  // 完全にラインを見失った場合（全て白）：2秒間は最大旋回(±25)で復帰を試みる
  if (!isLost) { isLost = true; lostLineTime = millis(); }
  if (millis() - lostLineTime < 2000) {
    error = (lastKnownError > 0) ? 25 : -25;
  } else {
    advanceCount = 0; stop_Stop(); return;
  }
}

// 2. 直進カウントの更新と動的ベース速度の計算（直進は加速、カーブは自動急減速）
if (abs(error) <= 5 && !isLost) advanceCount++;
else                            advanceCount = 0;

int baseSpeed = 12;
if (advanceCount > 10) baseSpeed = 14;
if (advanceCount > 20) baseSpeed = 16;
if (advanceCount > 30) baseSpeed = 18;
if (advanceCount > 40) baseSpeed = 20;

int curvePenalty = abs(error); // ズレた瞬間に直進スピードを強力に殺す
baseSpeed = baseSpeed - curvePenalty;
if (baseSpeed < 5) baseSpeed = 5; // 最低ベース速度

// 3. PID演算による補正値（correction）と左右モータ出力の決定
int P = error;
integral += error;
int I = integral;
int D = error - previousError;
previousError = error;

int correction = (Kp * P) + (Ki * I) + (Kd * D);
int onL = baseSpeed + correction;
int onR = baseSpeed - correction;

// 異常な値（0未満や上限45ms以上）にならないよう制限して駆動
if (onL < 0) onL = 0; if (onL > 45) onL = 45;
if (onR < 0) onR = 0; if (onR > 45) onR = 45;
set_Forward_Dir();
drive_Motors(onL + offsetL, onR + offsetR);
```

### 6.3 PIDゲインの調整手順

PID制御で高い走行性能を引き出すには、車体の重量やモータの摩擦特性に合わせてベース速度の減速量および3つのゲイン（`Kp`, `Ki`, `Kd`）を以下の順序で調整する。

1. **ベース速度とカーブ減速量（`curvePenalty`）の確認**：まず直進時の最高速度（`baseSpeed`）を決める。カーブで曲がりきれずに直進して飛び出す場合は、比例ゲインを上げる前に「カーブ進入時にベース速度が十分に落ちているか（`curvePenalty` が効いているか）」を確認する。直進の慣性を削ることで、無理のないステアリング量で滑らかに曲がれるようになる。
2. **`Kp`（比例ゲイン）の決定**：`Ki = 0`, `Kd = 0` に設定し、`Kp` だけを小さい値（0.5程度）から少しずつ上げていく。カーブで旋回力が足りず外へ膨らむ場合は `Kp` を増やし（1.0〜1.5程度）、直線で左右に激しく首を振る（発振する）ようになったら少し下げる。
3. **`Kd`（微分ゲイン）の追加**：`Kp` を決めた状態で、カーブでの内側への巻き込み（曲がりすぎて止まる現象）や出口での蛇行を抑えるために `Kd` を少しずつ増やしていく（1.5〜2.5程度）。ライントレースロボットでは、このP制御とD制御を組み合わせた「PD制御（`Ki = 0`）」だけでも非常に滑らかな無段階走行が可能となる。
4. **`Ki`（積分ゲイン）の微調整**：PD制御および `offsetL / offsetR` で走行させてもなお、左右モータの特性差などで常にラインの片側へ偏る場合のみ、ごく小さな値（0.01〜0.05程度）の `Ki` を追加する。
