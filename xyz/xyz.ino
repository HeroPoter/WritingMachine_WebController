//X + Y 双电机 + Z 四电机同步往返测试程序
#include <FastAccelStepper.h>

// ===== 引脚定义 =====
#define X_STEP_PIN        13
#define X_DIR_PIN         15

#define Y_LEFT_STEP_PIN   26
#define Y_LEFT_DIR_PIN    25
#define Y_RIGHT_STEP_PIN  14
#define Y_RIGHT_DIR_PIN   27

#define Z_STEP_PIN        33
#define Z_DIR_PIN         32

#define EN_PIN            21   // 四轴共用，低电平使能

// ===== 运动参数（只改这里）=====
// X 轴
const uint32_t X_SPEED_HZ     = 150000;    // Hz
const uint32_t X_ACCELERATION = 150000;   // 步/秒²
const int32_t  X_TRAVEL_STEPS = 14000;    // 步
const uint32_t X_PAUSE_MS     = 500;     // 到位停顿，毫秒

// Y 轴（双电机共用参数）
const uint32_t Y_SPEED_HZ     = 150000;    // Hz
const uint32_t Y_ACCELERATION = 150000;   // 步/秒²
const int32_t  Y_TRAVEL_STEPS = 14000;    // 步
const uint32_t Y_PAUSE_MS     = 500;     // 到位停顿，毫秒

// Z 轴
const uint32_t Z_SPEED_HZ     = 150000;    // Hz
const uint32_t Z_ACCELERATION = 150000;    // 步/秒²
const int32_t  Z_TRAVEL_STEPS = 500;    // 步
const uint32_t Z_PAUSE_MS     = 500;     // 到位停顿，毫秒

// false = 运动时使能、停止后失能；true = 全程使能
const bool EN_ALWAYS_ON = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *xStepper = NULL;
FastAccelStepper *yLeft    = NULL;
FastAccelStepper *yRight   = NULL;
FastAccelStepper *zStepper = NULL;

void setMotorEnable(bool en) {
  digitalWrite(EN_PIN, en ? LOW : HIGH);  // A4988 低电平使能
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("X + Y双电机 + Z 四电机同步往返测试开始...");
  Serial.print("X: 速度 "); Serial.print(X_SPEED_HZ);
  Serial.print(" Hz, 加速度 "); Serial.print(X_ACCELERATION);
  Serial.print(", 行程 0↔"); Serial.print(X_TRAVEL_STEPS);
  Serial.print(", 停顿 "); Serial.print(X_PAUSE_MS); Serial.println(" ms");

  Serial.print("Y: 速度 "); Serial.print(Y_SPEED_HZ);
  Serial.print(" Hz, 加速度 "); Serial.print(Y_ACCELERATION);
  Serial.print(", 行程 0↔"); Serial.print(Y_TRAVEL_STEPS);
  Serial.print(", 停顿 "); Serial.print(Y_PAUSE_MS); Serial.println(" ms");

  Serial.print("Z: 速度 "); Serial.print(Z_SPEED_HZ);
  Serial.print(" Hz, 加速度 "); Serial.print(Z_ACCELERATION);
  Serial.print(", 行程 0↔"); Serial.print(Z_TRAVEL_STEPS);
  Serial.print(", 停顿 "); Serial.print(Z_PAUSE_MS); Serial.println(" ms");

  Serial.print("EN模式: "); Serial.println(EN_ALWAYS_ON ? "全程使能" : "运动时使能");
  Serial.println("--------------------------------");

  engine.init();

  pinMode(EN_PIN, OUTPUT);
  setMotorEnable(EN_ALWAYS_ON);

  xStepper = engine.stepperConnectToPin(X_STEP_PIN);
  yLeft    = engine.stepperConnectToPin(Y_LEFT_STEP_PIN);
  yRight   = engine.stepperConnectToPin(Y_RIGHT_STEP_PIN);
  zStepper = engine.stepperConnectToPin(Z_STEP_PIN);

  if (xStepper == NULL || yLeft == NULL || yRight == NULL || zStepper == NULL) {
    Serial.println("电机初始化失败：STEP 引脚连接错误");
    while (1);
  }

  xStepper->setDirectionPin(X_DIR_PIN);
  yLeft->setDirectionPin(Y_LEFT_DIR_PIN);
  yRight->setDirectionPin(Y_RIGHT_DIR_PIN);
  zStepper->setDirectionPin(Z_DIR_PIN, false);  // Z 方向已翻转

  // 如果某个电机方向相反，加第二个参数 false 翻转，例如：
  // xStepper->setDirectionPin(X_DIR_PIN, false);
  // yRight->setDirectionPin(Y_RIGHT_DIR_PIN, false);

  xStepper->setSpeedInHz(X_SPEED_HZ);
  xStepper->setAcceleration(X_ACCELERATION);

  yLeft->setSpeedInHz(Y_SPEED_HZ);
  yLeft->setAcceleration(Y_ACCELERATION);
  yRight->setSpeedInHz(Y_SPEED_HZ);
  yRight->setAcceleration(Y_ACCELERATION);

  zStepper->setSpeedInHz(Z_SPEED_HZ);
  zStepper->setAcceleration(Z_ACCELERATION);

  xStepper->setCurrentPosition(0);
  yLeft->setCurrentPosition(0);
  yRight->setCurrentPosition(0);
  zStepper->setCurrentPosition(0);

  Serial.println("四轴已就绪，当前位置均为 0。");
}

// 四个电机同时移动到各自目标位置
void moveAllTo(int32_t xTarget, int32_t yTarget, int32_t zTarget) {
  if (!EN_ALWAYS_ON) {
    setMotorEnable(true);
  }

  xStepper->moveTo(xTarget, false);
  yLeft->moveTo(yTarget, false);
  yRight->moveTo(yTarget, false);
  zStepper->moveTo(zTarget, false);

  while (xStepper->isRunning() || yLeft->isRunning() ||
         yRight->isRunning() || zStepper->isRunning()) {
    delay(1);
  }
}

// 各轴按自己的停顿时间等待，全部到点后返回
void waitAllPauses() {
  uint32_t xReady = millis() + X_PAUSE_MS;
  uint32_t yReady = millis() + Y_PAUSE_MS;
  uint32_t zReady = millis() + Z_PAUSE_MS;

  while (millis() < xReady || millis() < yReady || millis() < zReady) {
    delay(1);
  }
}

void loop() {
  unsigned long cycleStart = millis();
  unsigned long t1, t2;

  // ---- 去程：0 → 各自行程 ----
  t1 = millis();
  moveAllTo(X_TRAVEL_STEPS, Y_TRAVEL_STEPS, Z_TRAVEL_STEPS);
  t2 = millis();
  unsigned long forwardTime = t2 - t1;

  // 各自停顿
  waitAllPauses();
  if (!EN_ALWAYS_ON) {
    setMotorEnable(false);
  }

  // ---- 回程：各自行程 → 0 ----
  t1 = millis();
  moveAllTo(0, 0, 0);
  t2 = millis();
  unsigned long backwardTime = t2 - t1;

  // 各自停顿
  waitAllPauses();
  if (!EN_ALWAYS_ON) {
    setMotorEnable(false);
  }

  // ---- 打印周期统计 ----
  unsigned long cycleTime = millis() - cycleStart;
  Serial.print("去程(X:");
  Serial.print(X_TRAVEL_STEPS);
  Serial.print(" Y:");
  Serial.print(Y_TRAVEL_STEPS);
  Serial.print(" Z:");
  Serial.print(Z_TRAVEL_STEPS);
  Serial.print("): ");
  Serial.print(forwardTime);
  Serial.print(" ms   回程: ");
  Serial.print(backwardTime);
  Serial.print(" ms   完整周期: ");
  Serial.print(cycleTime);
  Serial.println(" ms");
}