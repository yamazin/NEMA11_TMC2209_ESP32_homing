#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

// =============================================
// ピン定義
// =============================================
#define DIR_PIN     14   // D14
#define STEP_PIN    12   // D12
#define EN_PIN      13   // D13
#define DIAG_PIN    15   // D15  ← TMC2209 DIAG (StallGuard出力)
#define UART_RX     16   // D16  ← TMC2209 PDN_UART (直結)
#define UART_TX     17   // D17  → TMC2209 PDN_UART (1kΩ経由)

// =============================================
// TMC2209 設定
// =============================================
#define DRIVER_ADDRESS  0b00    // MS1=LOW, MS2=LOW → アドレス0
#define R_SENSE         0.11f   // センス抵抗値 [Ω]（モジュールにより異なる）
#define RMS_CURRENT     900     // モーター電流 [mA]
#define MICROSTEPS      2       // ホーミング時マイクロステッピング

// =============================================
// モーション設定
// =============================================
#define HOMING_SPEED    1000    // ホーミング速度 [steps/sec]
// 起動時に既に壁際にいる場合、加速中の「SG_RESULT監視前」の区間で壁に押し込まれ続けデシンクする問題がある。
// 段階的な実測(500,750:問題なし / 1000:外力でトルク不足)により、余裕を持って750を採用。
#define HOMING_ACCEL    750     // ホーミング加速度 [steps/sec²]
// watchdog開始位置が比率通りに移動したことから、watchdog切替直後のログ連続出力によるブロッキングが
// 原因と判明・修正済み(lastDebugTimeをここでも更新)。巡航速度付近の95%に戻す。
#define HOMING_WATCHDOG_SPEED_RATIO 0.95f
#define HOMING_STALL_TIMEOUT_MS 2000       // 監視開始後この時間内にストール確定しなければ異常停止
#define HOMING_BACKOFF  10      // ストール後に引き戻すステップ数
#define MARGIN_STEPS    50      // 端からの安全マージン [steps]
#define PERIOD_MS       4000    // 往復周期 [ms]（片道2秒）※速度計算用に残す
#define NUM_SECTIONS    6       // 動作範囲の区画数
#define MOVE_INTERVAL   1000    // ランダム移動間隔 [ms]
#define SINE_PERIOD_MS  4500    // 正弦波往復モードの周期 [ms]（円運動の投影）

// StallGuard感度 (0–255): 値が大きいほど敏感
// フリー走行時SG>100、壁ストール時SG<100 となる値
#define STALL_THRESHOLD 50

// =============================================
// インスタンス
// =============================================
HardwareSerial TMCSerial(2);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// =============================================
// グローバル変数
// =============================================
long rightEnd = 0;              // 右端ステップ位置（ホーミングで決定）
bool homingDone = false;

// =============================================
// ユーティリティ
// =============================================
// 通信/ストール検知の異常時にモーターを止めて停止する
void haltOnError(const char* msg) {
    digitalWrite(EN_PIN, HIGH);
    Serial.println(msg);
    while (true) { delay(1000); }
}

// モーターを指定方向に動かし、ストールを検出したら止める
// direction: +1=右, -1=左
void moveUntilStall(int direction) {
    stepper.moveTo(direction * 1000000L);

    unsigned long lastDebugTime = 0;
    unsigned long lastStallCheck = 0;
    unsigned long watchdogStartTime = 0;
    bool watchdog = false;
    int stallCount = 0;
    uint16_t lastSG = 0;    // 表示用。SG_RESULTのUART読み取りをストール判定側と共有し、二重読み取りを避ける
    const int STALL_COUNT_THRESHOLD = 3;   // 連続N回（×5ms）で確定。ノイズ余裕のため2→3に
    const unsigned long STALL_CHECK_MS  = 5;
    // 実測では本物の壁接触はSG_RESULTが0〜10程度まで落ちる(ノイズ由来の誤検知は70〜96程度)。
    // 3回連続確認(15ms)を待つ間に脱調が進み信号が回復し確定できないことがあったため、
    // 極端に低い値は1発で即確定する高速パスを設ける。
    const uint16_t STALL_IMMEDIATE_SG = 20;

    while (true) {
        stepper.run();

        unsigned long now = millis();

        if (!watchdog && abs(stepper.speed()) >= HOMING_SPEED * HOMING_WATCHDOG_SPEED_RATIO) {
            watchdog = true;
            watchdogStartTime = now;
            lastStallCheck = now;   // watchdog開始直後の1発ノイズで即確定しないよう、ここから5ms計測を始める
            // この直後に旧lastDebugTimeのままデバッグ表示printも即座に発生すると、
            // println連続でシリアル送信がブロックしstepper.run()が滞り、実際に一瞬脱調する。
            // そのためここでlastDebugTimeも更新し、次の表示は通常間隔まで待たせる。
            lastDebugTime = now;
            Serial.println(">> ストール監視 開始");
        }

        if (watchdog && now - watchdogStartTime >= HOMING_STALL_TIMEOUT_MS) {
            haltOnError(">>> エラー: 監視開始からタイムアウトしてもストールを検知できませんでした。異常停止します。");
        }

        if (watchdog && now - lastStallCheck >= STALL_CHECK_MS) {
            lastStallCheck = now;
            lastSG = driver.SG_RESULT();
            if (lastSG < STALL_IMMEDIATE_SG) {
                stepper.setCurrentPosition(stepper.currentPosition());
                Serial.printf(">> ストール確定! (即時, SG=%u)\n", lastSG);
                break;
            } else if (lastSG < STALL_THRESHOLD * 2) {
                stallCount++;
                if (stallCount >= STALL_COUNT_THRESHOLD) {
                    stepper.setCurrentPosition(stepper.currentPosition());
                    Serial.printf(">> ストール確定! (連続%d回 × %dms)\n",
                                  stallCount, STALL_CHECK_MS);
                    break;
                }
            } else {
                stallCount = 0;
            }
        }

        unsigned long debugInterval = watchdog ? 20 : 100;
        if (now - lastDebugTime >= debugInterval) {
            lastDebugTime = now;
            if (!watchdog) lastSG = driver.SG_RESULT();   // watchdog前はここでのみ読み取る
            bool stall = (lastSG < STALL_THRESHOLD * 2);
            Serial.printf("[SG_RESULT: %3u | STALL: %d | count: %d | pos: %ld | watchdog: %d]\n",
                          lastSG, stall, stallCount, stepper.currentPosition(), watchdog);
        }
    }
}

// 指定ステップだけ引き戻す（ブロックから離脱）
void backOff(int steps) {
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(100);
}

// =============================================
// 診断用: 加速度耐性テスト
// StealthChopでの無負荷ログにより「巡航速度に達するまでSG_RESULTは信頼できない」ことが判明したため、
// watchdog開始比率を95%に変更(HOMING_WATCHDOG_SPEED_RATIO)。診断モードを解除しホーミングへ復帰する。
// =============================================
#define DIAG_SPREADCYCLE_TEST 0
#define DIAG_TEST_SPEED  HOMING_SPEED
#define DIAG_TEST_ACCEL  HOMING_ACCEL

void spreadCycleDiagTest() {
    Serial.println("=== StealthChop 無負荷加速テスト ===");
    driver.en_spreadCycle(false);
    driver.TPWMTHRS(0);
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(DIAG_TEST_SPEED);
    stepper.setAcceleration(DIAG_TEST_ACCEL);

    Serial.printf("[diag] microsteps  : %u (期待値: %d)\n", driver.microsteps(), MICROSTEPS);
    Serial.printf("[diag] rms_current : %u mA (期待値: %d)\n", driver.rms_current(), RMS_CURRENT);
    Serial.printf("[diag] cs_actual   : %u\n", driver.cs_actual());
    Serial.printf("[diag] テスト速度  : %d steps/sec / 加速度: %d steps/sec²\n", DIAG_TEST_SPEED, DIAG_TEST_ACCEL);

    unsigned long lastLog = 0;

    while (true) {
        stepper.moveTo(2000);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            unsigned long now = millis();
            if (now - lastLog >= 20) {
                lastLog = now;
                Serial.printf("[SG_RESULT: %3u | pos: %ld | speed: %.0f]\n",
                              driver.SG_RESULT(), stepper.currentPosition(), stepper.speed());
            }
        }
        Serial.printf("→ +2000 到達 (pos: %ld)\n", stepper.currentPosition());
        delay(500);

        stepper.moveTo(0);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            unsigned long now = millis();
            if (now - lastLog >= 20) {
                lastLog = now;
                Serial.printf("[SG_RESULT: %3u | pos: %ld | speed: %.0f]\n",
                              driver.SG_RESULT(), stepper.currentPosition(), stepper.speed());
            }
        }
        Serial.printf("→ 0 に帰還 (pos: %ld)\n", stepper.currentPosition());
        delay(500);
    }
}

// =============================================
// ホーミング
// =============================================
void homing() {
    Serial.println("--- ホーミング開始 ---");

    // StealthChop2のまま維持（StallGuard4に必須）
    driver.en_spreadCycle(false);
    driver.SGTHRS(STALL_THRESHOLD);
    delay(200);

    Serial.printf("[homing] en_spreadCycle : %d (期待値: 0=StealthChop)\n", driver.en_spreadCycle());
    Serial.printf("[homing] TPWMTHRS       : 0x%08X (期待値: 0)\n", driver.TPWMTHRS());
    Serial.printf("[homing] SGTHRS         : %d\n", STALL_THRESHOLD);

    stepper.setMaxSpeed(HOMING_SPEED);
    stepper.setAcceleration(HOMING_ACCEL);

    // === Step1: 左端へ ===
    Serial.println("左端を探索中...");
    moveUntilStall(-1);
    backOff(HOMING_BACKOFF);
    stepper.setCurrentPosition(0);
    Serial.println("左端検知 → 原点設定");
    delay(300);

    // === Step2: 右端へ ===
    Serial.println("右端を探索中...");
    moveUntilStall(+1);
    backOff(-HOMING_BACKOFF);
    rightEnd = stepper.currentPosition();
    Serial.printf("右端検知: %ld steps (可動域)\n", rightEnd);
    delay(300);

    // === Step3: 中央へ移動 ===
    long center = rightEnd / 2;
    stepper.setMaxSpeed(rightEnd / 2.0f);
    stepper.setAcceleration(rightEnd / 2.0f);
    stepper.moveTo(center);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    Serial.println("中央へ移動完了");
    delay(500);

    homingDone = true;
}

// =============================================
// 動作モード（ホーミング完了後に1回だけ抽選する）
// =============================================
enum MotionMode { MODE_RANDOM, MODE_SINE };
MotionMode motionMode = MODE_RANDOM;

float sineCenter = 0;
float sineAmplitude = 0;
unsigned long sineStartTime = 0;

// 正弦波往復（円運動の投影）の初期化。速度・加速度上限は正弦波の理論最大値に余裕を持たせて設定する
void startSineMotion() {
    long travelSteps = rightEnd - 2 * MARGIN_STEPS;
    sineAmplitude = travelSteps / 2.0f;
    sineCenter = MARGIN_STEPS + sineAmplitude;
    sineStartTime = millis();

    float omega = 2.0f * PI / (SINE_PERIOD_MS / 1000.0f);
    float sineMaxSpeed = sineAmplitude * omega * 1.3f;         // 理論最大速度に30%余裕
    float sineMaxAccel = sineAmplitude * omega * omega * 1.3f; // 理論最大加速度に30%余裕
    stepper.setMaxSpeed(sineMaxSpeed);
    stepper.setAcceleration(sineMaxAccel);

    Serial.printf("動作モード: 正弦波往復 / 周期%dms / 中心%.0f / 振幅%.0f / 速度上限%.0f / 加速度上限%.0f\n",
                  SINE_PERIOD_MS, sineCenter, sineAmplitude, sineMaxSpeed, sineMaxAccel);
}

// 毎ループ呼び出し、現在時刻に応じた正弦波上の目標位置を追従させる
void sineMotionUpdate() {
    float t = (millis() - sineStartTime) / 1000.0f;
    float omega = 2.0f * PI / (SINE_PERIOD_MS / 1000.0f);
    long target = sineCenter + (long)(sineAmplitude * sinf(omega * t));
    stepper.moveTo(target);
}

// 前方宣言
void initSections();

// =============================================
// Setup
// =============================================
void setup() {
    Serial.begin(115200);
    Serial.println("=== スライダー テスト ===");

    pinMode(EN_PIN, OUTPUT);
    pinMode(DIAG_PIN, INPUT);
    digitalWrite(EN_PIN, HIGH);

    TMCSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
    driver.begin();
    driver.toff(4);
    driver.rms_current(RMS_CURRENT);
    driver.microsteps(MICROSTEPS);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.TPWMTHRS(0);

    digitalWrite(EN_PIN, LOW);
    delay(200);

    uint8_t conn = driver.test_connection();
    Serial.printf("UART接続テスト : %s (code=%d)\n", conn == 0 ? "OK" : "NG", conn);
    Serial.printf("ドライババージョン: 0x%02X (期待値: 0x21)\n", driver.version());

    if (conn != 0) {
        haltOnError("!!! UART通信エラー。ホーミングは実行しません !!!");
    }

#if DIAG_SPREADCYCLE_TEST
    // 診断モード: ホーミングをスキップしてSpreadCycleで無負荷往復を繰り返すのみ
    spreadCycleDiagTest();
    return;
#endif

    // ホーミング実行
    homing();

    // =============================================
    // 通常動作設定（ホーミング完了後に適用）
    // =============================================
    // 高速往復はSpreadCycleの方がトルクが安定する（StealthChopは低速・静音向け）
    driver.en_spreadCycle(true);
    driver.TPWMTHRS(0);        // SpreadCycle固定（自動切り替えなし）
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    const int RUN_MICROSTEPS = 16;
    driver.microsteps(RUN_MICROSTEPS);
    long scaleFactor = RUN_MICROSTEPS / MICROSTEPS;  // 8
    stepper.setCurrentPosition(stepper.currentPosition() * scaleFactor);
    rightEnd = rightEnd * scaleFactor;
    long travelSteps = rightEnd - 2 * MARGIN_STEPS;
    // 速度は片道をPERIOD_MS/2で走り切る程度に設定
    float runSpeed = travelSteps / (PERIOD_MS / 2000.0f) * 3.6f;  // 1.2f × 3倍
    // 加速度を速度の3倍に設定してキビキビした動きに（脱調する場合は倍率を下げる）
    float runAccel = runSpeed * 3.0f;
    stepper.setMaxSpeed(runSpeed);
    stepper.setAcceleration(runAccel);
    Serial.printf("通常動作設定: 1/%d マイクロステップ / 可動域: 0〜%ld steps / 速度: %.0f steps/sec / 加速度: %.0f steps/sec²\n",
                  RUN_MICROSTEPS, rightEnd, runSpeed, runAccel);

    // ホーミング完了後、動作モードを1回だけ抽選する
    randomSeed(esp_random());
    motionMode = (random(2) == 0) ? MODE_RANDOM : MODE_SINE;
    if (motionMode == MODE_SINE) {
        startSineMotion();
    } else {
        Serial.println("動作モード: ランダム区画移動");
        initSections();
    }
}

// =============================================
// Loop（ランダム移動動作）
// =============================================
long sectionPositions[NUM_SECTIONS];
int currentSection = -1;
unsigned long lastMoveTime = 0;

void initSections() {
    long travelSteps = rightEnd - 2 * MARGIN_STEPS;
    for (int i = 0; i < NUM_SECTIONS; i++) {
        // 各区画の中心座標
        sectionPositions[i] = MARGIN_STEPS + (long)(travelSteps * (2 * i + 1) / (2.0f * NUM_SECTIONS));
    }
    Serial.println("区画座標:");
    for (int i = 0; i < NUM_SECTIONS; i++) {
        Serial.printf("  区画%d: %ld steps\n", i, sectionPositions[i]);
    }
}

void loop() {
    if (!homingDone) return;

    stepper.run();

    if (motionMode == MODE_SINE) {
        sineMotionUpdate();
        return;
    }

    unsigned long now = millis();
    if (now - lastMoveTime >= MOVE_INTERVAL) {
        lastMoveTime = now;

        // 現在と異なる区画をランダムに選択
        int nextSection;
        do {
            nextSection = random(NUM_SECTIONS);
        } while (nextSection == currentSection);

        currentSection = nextSection;
        stepper.moveTo(sectionPositions[nextSection]);
        Serial.printf("→ 区画%d (%ld steps)\n", nextSection, sectionPositions[nextSection]);
    }
}
