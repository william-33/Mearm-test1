#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>   // 板间软件串口：与遥控板(task_all_control)通信

/* ==================== 配置区 ==================== */

// —— 四个舵机引脚 ——
#define GRIPPER_PIN   6
#define Base_PIN      9
#define rArm_PIN      8
#define fArm_PIN      7

// —— 2 个摇杆的 4 个模拟轴 ——
#define JOY_1X_PIN    A0    // 摇杆1 X -> 底座 Base
#define JOY_1Y_PIN    A1    // 摇杆1 Y -> 大臂 rArm
#define JOY_2X_PIN    A2    // 摇杆2 X -> 爪子 Gripper
#define JOY_2Y_PIN    A3    // 摇杆2 Y -> 小臂 fArm
#define JOY_CENTER  513     // 摇杆中位理论值
#define JOY_DEAD    50      // 死区：偏移小于它算"没动"

// —— 爪子和速度 ——
#define GRIP_OPEN_ANGLE   81
#define GRIP_CLOSE_ANGLE  178
#define STEP_ANGLE   2      // 平滑移动时每步最多转几度
#define STEP_DELAY   25     // 每步停多少毫秒（速度手感）

// —— 串口 ——
#define BAUD_RATE         9600
#define LINK_BAUD         9600   // 板间串口（遥控板）

// —— 任务5：笔的三档高度 ——
#define Z_PEN_DOWN   5      // 落笔（纸面）高度
#define Z_TRAVEL     10     // 空程移动高度
#define Z_TEACH      7      // 示教高度
#define LINE_STEP    0.2
#define LINE_DELAY   40
#define CURVE_SAMPLES 60
#define TEACH_FRAME  30

// —— 任务3：录制参数 ——
#define REC_INTERVAL  100   // 采样间隔 ms（每秒 10 帧）
#define MAX_FRAMES    150   // 最长录制帧数
#define HOVER_DZ      5.0

// —— 任务一：PWM 脉宽 / 占空比 ——
#define MAX_PULSE_US   2400
#define MIN_PULSE_US   544
#define MAX_ANGLE      180
#define MIN_ANGLE      0
#define FRAME_US       20000UL   // PWM 周期 20ms（50Hz）

// —— 任务一：速度档位（5 档，值越大越慢）——
#define SPEED_LEVELS   5

// —— 任务一：摇杆灵敏度（偏移 / JOY_SENSE = 每帧转几度）——
#define JOY_SENSE     256

// —— 工作模式（用宏而非 enum，避免 Arduino 自动原型引用自定义类型失败）——
#define MODE_DEMO     0     // 演示模式：任务2 / 任务3 / 任务5
#define MODE_MANUAL   1     // 手动模式：任务一

/* ==================== 全局变量 ==================== */

const float L1 = 8;                     // 大臂
const float L2 = 8;                     // 小臂
const float H0 = 6;                     // 底座关节高度
const float L4 = 6.3;                   // 夹爪/笔长补偿
const float Base_OFF  = 4.6, Base_DIR  = 1;
const float rArm_OFF  = 135, rArm_DIR  = -1;
const float fArm_OFF  = -50, fArm_DIR  = 1;

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper;

// —— 板间串口（SoftwareSerial）：RX=10, TX=11 ——
// 接线要和遥控板"交叉"，两块板必须共地
SoftwareSerial link(10, 11);

// —— 统一舵机角账本（float：摇杆每帧增量很小，int 会被截断）——
float curBase = 90, currarm = 45, curfarm = 90;
float curgrip = GRIP_OPEN_ANGLE;

// —— IK 输出的三个几何角 ——
float baseangle, rarmangle, farmangle;

// —— 任务5：笔尖当前平面位置 ——
float curX = 0, curY = 16;
bool penheld = false;

// —— 平面坐标结构 ——
struct Point {
  float x;
  float y;
  float z;
};

// —— 任务5：笔位与待机位 ——
const float PEN_X = 2, PEN_Y = 17;   // 取笔点的平面坐标
const float PEN_Z = 5;                // 取笔时爪子下降的高度
const float PEN_HOVER = 8;            // 取笔前的悬停高度
Point pensite = {PEN_X, PEN_Y, PEN_Z};
Point HOME    = {0, 16, 8};           // 统一待机位

// —— 任务5：两个硬编码图形（厘米）——
const float L_x[2] = {0, 0};
const float L_y[2] = {17.5, 20};
const float V_x[3] = {-1.5, 0, 1.5};
const float V_y[3] = {20, 17.5, 20};

// —— 任务5：示教点（最多 5 个）——
float teachX[5];
float teachY[5];
int teachCount = 0;

// —— 任务2：A/B/C 取放点 ——
Point pickA  = { -7.0,  17.0, 2.0};
Point placeA = { 12.0,  16.0, 4.0};
Point pickB  = {-17.0,   7.0, 2.0};
Point placeB = {  9.0,  17.0, 4.0};
Point pickC  = { -6.0,  17.5, 2.0};
Point placeC = { 11.0,  15.0, 4.0};

// —— 任务3：循环计数与录制缓冲 ——
int  abcIndex  = 0;        // 0=下次夹A，1=夹B，2=夹C
bool recording = false;

struct Frame {
  byte b;   // 底座角
  byte r;   // 大臂角
  byte f;   // 小臂角
  byte g;   // 爪子
};
Frame rec[MAX_FRAMES];
int recLen = 0;

// —— 任务一：单关节限幅常量 ——
const int maxbase = 180, maxrArm = 120, maxfArm = 145, maxclaw = 178;
const int minbase =   0, minrArm =  20, minfArm =  68, minclaw =  81;

int toPos = 90;                              // b/r/f 的目标角
int speedIndex = 2;                          // 速度档（0 最快 … 4 最慢）
const int speedDelay[SPEED_LEVELS] = {5, 10, 15, 20, 25};

// —— 工作模式 ——
uint8_t workMode = MODE_DEMO;                // 上电默认演示模式

/* ==================== 通用工具函数 ==================== */

// 摇杆读数 -> 这一帧舵机角要转多少度，死区内返回 0（录制/示教用）
float joyDeltaAngle(int pin) {
  int off = analogRead(pin) - JOY_CENTER;
  if (abs(off) < JOY_DEAD) return 0;
  return off / 256.0;
}

// —— 统一指令源：遥控板(link) > USB 串口（仅演示模式使用）——
char readCommand()
{
  if (link.available() > 0) {                 // ① 遥控板 task_all_control
    char c = link.read();
    if (c != '\n' && c != '\r' && c != ' ') return c;
  }
  if (Serial.available() > 0) {               // ② USB 串口
    char c = Serial.read();
    if (c != '\n' && c != '\r' && c != ' ') return c;
  }
  return 0;
}

// 逆解：xyz -> 三个几何角
bool exchangeData(float x, float y, float z,
                  float &baseangle, float &rarmangle, float &farmangle)
{
  float tmp1;
  float tmp2;
  float r  = sqrtf(x*x + y*y) - L4;    // 减去夹爪/笔长，把"爪尖"折算成"腕关节"
  float H1 = z - H0;
  float L3 = sqrtf(H1*H1 + r*r);

  if (L3 > L1 + L2)       return false;    // 够不到
  if (L3 < fabs(L1 - L2)) return false;    // 太近
  if (r < 0)              return false;    // 目标比夹爪还近

  baseangle = atan2f(y, x) * 180.0 / PI;

  float tmp3 = (L1*L1 + L2*L2 - L3*L3) / (2*L1*L2);
  tmp3 = constrain(tmp3, -1.0, 1.0);
  float tmp4 = (L3*L3 + L1*L1 - L2*L2) / (2*L3*L1);
  tmp4 = constrain(tmp4, -1.0, 1.0);

  tmp1 = acosf(tmp4) * 180.0 / PI;
  tmp2 = atan2f(H1, r) * 180.0 / PI;
  rarmangle = tmp1 + tmp2;
  farmangle = (acosf(tmp3) * 180.0 / PI) + rarmangle;
  return true;
}

// 几何角 -> 舵机指令角
int baseToServo(float t1) { return constrain((int)(Base_OFF + Base_DIR * t1),   0, 180); }
int rarmToServo(float t2) { return constrain((int)(rArm_OFF + rArm_DIR * t2),  20, 120); }
int farmToServo(float t3) { return constrain((int)(fArm_OFF + fArm_DIR * t3),  68, 145); }

// 瞬时写入（画线用，不做插值）
void moveToAnglesFast(int x, int y, int z) {
  curBase = x;
  currarm = y;
  curfarm = z;
  Base.write((int)round(curBase));
  rArm.write((int)round(currarm));
  fArm.write((int)round(curfarm));
}

// 阻塞式平滑移动（三轴同步）
void moveToAngles(float baseangle, float rarmangle, float farmangle) {
  int s1 = baseToServo(baseangle);
  int s2 = rarmToServo(rarmangle);
  int s3 = farmToServo(farmangle);

  int st1 = (int)round(curBase);     // 出发角（记录一下，插值要用）
  int st2 = (int)round(currarm);
  int st3 = (int)round(curfarm);

  int d1 = abs(s1 - st1), d2 = abs(s2 - st2), d3 = abs(s3 - st3);
  int steps = max(d1, max(d2, d3)) / STEP_ANGLE;
  if (steps < 1) steps = 1;

  for (int i = 1; i <= steps; i++) {
    // 线性插值：从"出发角"朝"目标角"走 i/steps 的比例
    curBase = st1 + (s1 - st1) * i / steps;
    currarm = st2 + (s2 - st2) * i / steps;
    curfarm = st3 + (s3 - st3) * i / steps;

    Base.write((int)round(curBase));   // 三轴同一步内一起写 -> 同步
    rArm.write((int)round(currarm));
    fArm.write((int)round(curfarm));
    delay(STEP_DELAY);                 // 每步停一下，形成"平滑"效果
  }
}

// 爪子开合
void gripMoveTo(int target) {
  while ((int)round(curgrip) != target) {
    if (target > curgrip) {
      curgrip += STEP_ANGLE;                  // 往大走一小步
      if (curgrip > target) curgrip = target; // 防止越过目标
    } else {
      curgrip -= STEP_ANGLE;                  // 往小走一小步
      if (curgrip < target) curgrip = target;
    }
    Gripper.write((int)round(constrain(curgrip, 81, 178)));
    delay(STEP_DELAY);
  }
}

void gripOpen()  { gripMoveTo(GRIP_OPEN_ANGLE);  delay(300); } // 张开后等一下
void gripClose() { gripMoveTo(GRIP_CLOSE_ANGLE); delay(500); } // 闭合后多等一会, 夹稳

// 走到一个平面点（爪尖坐标）
bool moveToPoint(float x, float y, float z)
{
  if (!exchangeData(x, y, z, baseangle, rarmangle, farmangle)) {
    Serial.print(F("  [警告] 该点不可达, 已跳过: ("));
    Serial.print(x, 1); Serial.print(F(", "));
    Serial.print(y, 1); Serial.print(F(", "));
    Serial.print(z, 1); Serial.println(F(")"));
    return false;
  }
  moveToAngles(baseangle, rarmangle, farmangle);
  curX = x;          // 笔尖已移到 (x, y)
  curY = y;
  return true;
}

/* ==================== 任务一：小工具 ==================== */

// 从 "x10,y30,z20" 里取某个字母后面的整数
int getValue(String cmd, char letter)
{
  int start = cmd.indexOf(letter);
  if (start < 0) return -1;
  start = start + 1;
  int end = cmd.indexOf(',', start);
  if (end < 0) end = cmd.length();
  String value = cmd.substring(start, end);
  return value.toInt();
}

// 角度 -> 高电平脉宽(us)
unsigned long angleToPulse(int angle)
{
  return (unsigned long)angle * (MAX_PULSE_US - MIN_PULSE_US) / MAX_ANGLE + MIN_PULSE_US;
}

// 角度 -> 占空比(%)
float angleToDuty(int angle)
{
  return angleToPulse(angle) * 100.0 / FRAME_US;
}

// 限幅（带提示）
int jugetmentx(int x)
{
  if (x > maxbase)       { Serial.println(F("too high")); return maxbase; }
  else if (x < minbase)  { Serial.println(F("too low"));  return minbase; }
  else return x;
}
int jugetmenty(int y)
{
  if (y > maxrArm)       { Serial.println(F("too high")); return maxrArm; }
  else if (y < minrArm)  { Serial.println(F("too low"));  return minrArm; }
  else return y;
}
int jugetmentz(int z)
{
  if (z > maxfArm)       { Serial.println(F("too high")); return maxfArm; }
  else if (z < minfArm)  { Serial.println(F("too low"));  return minfArm; }
  else return z;
}

// 单轴平滑步进（带速度档），账本用统一的 float
void moveServo(Servo &servo, float &current, int target) {
  if ((int)round(current) == target) return;

  int step = (target > current) ? STEP_ANGLE : -STEP_ANGLE;
  while ((int)round(current) != target) {
    current += step;

    // 防止越过目标
    if ((step > 0 && current > target) || (step < 0 && current < target)) {
      current = target;
    }

    servo.write((int)round(current));
    delay(speedDelay[speedIndex]);
  }
}

// 回执
void ack(char cmd) {
  Serial.print(F("OK:"));
  Serial.print(cmd);
  Serial.print(F("  speed="));
  Serial.println(speedIndex);
}

// 打印某舵机当前角
void ackone(Servo &servo)
{
  Serial.println(servo.read());
}

// 任务一摇杆增量（含取反；与录制/示教的 joyDeltaAngle 方向约定不同，勿混用）
float joyDelta(int pin)
{
  int off = analogRead(pin) - JOY_CENTER;
  if (abs(off) < JOY_DEAD) return 0;
  return -(off / (float)JOY_SENSE);
}

// 摇杆实时操控：4 轴先算完再统一写（空闲时由两种模式共同调用）
bool joyMove() {
  float dB = joyDelta(JOY_1X_PIN);
  float dR = joyDelta(JOY_1Y_PIN);
  float dF = joyDelta(JOY_2Y_PIN);
  float dG = joyDelta(JOY_2X_PIN);

  if (dB == 0 && dR == 0 && dF == 0 && dG == 0) return false;  // 没动 -> 交回上层

  curBase = constrain(curBase + dB, minbase, maxbase);
  currarm = constrain(currarm + dR, minrArm, maxrArm);
  curfarm = constrain(curfarm + dF, minfArm, maxfArm);
  curgrip = constrain(curgrip + dG, minclaw, maxclaw);

  Base.write((int)round(curBase));
  rArm.write((int)round(currarm));
  fArm.write((int)round(curfarm));
  Gripper.write((int)round(curgrip));
  return true;
}

// 模式切换与菜单
void switchMode();
void printDemoMenu();
void printManualMenu();

/* ==================== 任务一：手动模式输入处理 ==================== */

// 单关节 / 开合爪 / 调速 / 回中 / 切模式
void handleManualChar(char cmd, bool fromSerial) {
  switch (cmd) {

    case 'b':                                   // 底座单关节（数值指令只走 USB）
      if (!fromSerial) break;
      toPos = Serial.parseInt();
      moveServo(Base, curBase, jugetmentx(toPos));
      ack(cmd);
      break;

    case 'r':                                   // 大臂单关节
      if (!fromSerial) break;
      toPos = Serial.parseInt();
      moveServo(rArm, currarm, jugetmenty(toPos));
      ack(cmd);
      break;

    case 'f':                                   // 小臂单关节
      if (!fromSerial) break;
      toPos = Serial.parseInt();
      moveServo(fArm, curfarm, jugetmentz(toPos));
      ack(cmd);
      break;

    case 'O':                                   // 张开爪子
      moveServo(Gripper, curgrip, GRIP_OPEN_ANGLE);
      ack(cmd);
      break;

    case 'S':                                   // 闭合爪子
      moveServo(Gripper, curgrip, GRIP_CLOSE_ANGLE);
      ack(cmd);
      break;

    case 'H':                                   // 加速（档位-1）
      if (speedIndex > 0) speedIndex--;
      ack(cmd);
      break;

    case 'L':                                   // 减速（档位+1）
      if (speedIndex < SPEED_LEVELS - 1) speedIndex++;
      ack(cmd);
      break;

    case 'i':                                   // 回中 + 状态
      moveServo(Gripper, curgrip, GRIP_OPEN_ANGLE);
      moveServo(fArm, curfarm, 90);
      moveServo(rArm, currarm, 45);
      moveServo(Base, curBase, 90);
      Serial.print(F("the base="));    ackone(Base);
      Serial.print(F("the rArm="));    ackone(rArm);
      Serial.print(F("the fArm="));    ackone(fArm);
      Serial.print(F("the Gripper=")); ackone(Gripper);
      break;

    case 'M': case 'm':                         // 切模式
      switchMode();
      break;

    case 'P': case 'p': case 'U': case 'u': case 'X': case 'R':
      Serial.println(F("(手动模式: 该过程键无效)"));
      break;

    default:
      if (cmd != '\n' && cmd != '\r' && cmd != ' ' && cmd != '\t') {
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
      }
      break;
  }
}

// 三轴整行 x,y,z
void handleManualXYZ(String cmd) {
  int x = getValue(cmd, 'x');
  int y = getValue(cmd, 'y');
  int z = getValue(cmd, 'z');

  if (x < 0 || y < 0 || z < 0) {
    Serial.print(F("Parse error: "));
    Serial.println(cmd);
    return;
  }

  x = jugetmentx(x);
  y = jugetmenty(y);
  z = jugetmentz(z);

  curBase = x; currarm = y; curfarm = z;      // ★ 账本同步
  Base.write(x);
  rArm.write(y);
  fArm.write(z);

  Serial.println(F("=== command OK ==="));
  Serial.print(F("x = ")); Serial.print(x);
  Serial.print(F(" deg, pulse = ")); Serial.print(angleToPulse(x));
  Serial.print(F(" us, duty = "));   Serial.print(angleToDuty(x), 2);
  Serial.println(F(" %"));

  Serial.print(F("y = ")); Serial.print(y);
  Serial.print(F(" deg, pulse = ")); Serial.print(angleToPulse(y));
  Serial.print(F(" us, duty = "));   Serial.print(angleToDuty(y), 2);
  Serial.println(F(" %"));

  Serial.print(F("z = ")); Serial.print(z);
  Serial.print(F(" deg, pulse = ")); Serial.print(angleToPulse(z));
  Serial.print(F(" us, duty = "));   Serial.print(angleToDuty(z), 2);
  Serial.println(F(" %"));
  Serial.println(F("=================="));
}

// 手动模式主循环：link 单字符 > USB 整行/单字符 > 空闲摇杆
void manualLoop() {
  // ① 板间串口（遥控板只发单字符 '1'..'4' / P / U / X / R）——最高优先
  if (link.available() > 0) {
    char c = link.read();
    if (c != '\n' && c != '\r' && c != ' ') handleManualChar(c, false);
    return;
  }

  // ② USB 串口：整行（三轴）或单字符（含 b/r/f + 数字）
  if (Serial.available() > 0) {
    char p = Serial.peek();
    if (p == 'x' || p == 'X') {
      String line = Serial.readStringUntil('\n');
      line.trim();
      bool hasDigit = false;
      for (int i = 0; i < (int)line.length(); i++) {
        if (line[i] >= '0' && line[i] <= '9') { hasDigit = true; break; }
      }
      if (hasDigit)               handleManualXYZ(line);
      else if (line.length() > 0) handleManualChar(line[0], true);
      return;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') return;
    handleManualChar(c, true);
    return;
  }

  // ③ 空闲——摇杆实时操控
  joyMove();
  delay(20);
}

/* ==================== 任务2 / 任务3 / 任务5 专用 ==================== */

// 打印步骤
void printStep(int n, const __FlashStringHelper* msg) {
  Serial.print(F("[")); Serial.print(n); Serial.print(F("/8] "));
  Serial.println(msg);
}

// 完整的"夹取 -> 放置"流程
void pickAndPlace(Point pick, Point place)
{
  printStep(1, F("悬停到夹取点上方"));
  moveToPoint(pick.x, pick.y, pick.z + HOVER_DZ);

  printStep(2, F("下降到夹取点"));
  moveToPoint(pick.x, pick.y, pick.z);

  printStep(3, F("闭合爪子(夹取)"));
  gripClose();

  printStep(4, F("抬升回悬停高度"));
  moveToPoint(pick.x, pick.y, pick.z + HOVER_DZ);

  printStep(5, F("平移到放置点上方"));
  moveToPoint(place.x, place.y, place.z + HOVER_DZ);

  printStep(6, F("下降到放置点"));
  moveToPoint(place.x, place.y, place.z);

  printStep(7, F("张开爪子(放置)"));
  gripOpen();

  printStep(8, F("回到休息位"));
  moveToPoint(HOME.x, HOME.y, HOME.z);
}

// 启动录制（第一层）
void toggleRecord() {
  if (!recording) {
    recording = true;
    recLen = 0;
    Serial.println(F("开始录制"));
    recordloop();
  }
}

// 录制核心：摇杆动、每 100ms 存一帧
void recordloop()
{
  while (recording) {

    char cmd = link.read();
    if (cmd == '2') {
      recording = false;
      Serial.print(F("=== 录制结束, 共 ")); Serial.print(recLen);
      Serial.print(F(" 帧 / ")); Serial.print(recLen / 10.0, 1);
      Serial.println(F(" 秒 ==="));
      break;
    }

    // 摇杆 -> 舵机角
    curBase = constrain(curBase + joyDeltaAngle(JOY_1X_PIN),  0, 180);
    currarm = constrain(currarm + joyDeltaAngle(JOY_1Y_PIN), 20, 120);
    curfarm = constrain(curfarm + joyDeltaAngle(JOY_2Y_PIN), 68, 145);
    curgrip = constrain(curgrip + joyDeltaAngle(JOY_2X_PIN), 81, 178);
    Base.write((int)round(curBase));
    rArm.write((int)round(currarm));
    fArm.write((int)round(curfarm));
    Gripper.write((int)round(curgrip));

    // 存帧
    rec[recLen].b = (byte)round(curBase);
    rec[recLen].r = (byte)round(currarm);
    rec[recLen].f = (byte)round(curfarm);
    rec[recLen].g = (byte)round(curgrip);
    recLen++;

    if (recLen >= MAX_FRAMES) {          // 录满自动停止，数组有界
      recording = false;
      Serial.println(F("!!! 已达最长录制, 自动结束并保存 !!!"));
      break;
    }
    delay(REC_INTERVAL);                 // 固定采样间隔，保证"录多久播多久"
  }
}

// 回放（直接写舵机角，与坐标系无关）
void playrecord()
{
  if (recLen == 0) {
    Serial.println(F("还没有录制数据, 请先按 2 录制"));
    return;
  }
  Serial.print(F("=== 播放录制动作, 共 ")); Serial.print(recLen);
  Serial.println(F(" 帧 ==="));
  for (int i = 0; i < recLen; i++) {
    curBase = rec[i].b;
    currarm = rec[i].r;
    curfarm = rec[i].f;
    curgrip = rec[i].g;
    Base.write((int)round(curBase));
    rArm.write((int)round(currarm));
    fArm.write((int)round(curfarm));
    Gripper.write((int)round(curgrip));
    delay(REC_INTERVAL);                 // 与录制同节奏
  }
  Serial.println(F("=== 播放完毕 ==="));
}

// 抬笔
void penUp() {
  moveToPoint(curX, curY, Z_TRAVEL);
  delay(300);
}

// 落笔
void penDown(float x, float y) {
  moveToPoint(x, y, Z_PEN_DOWN);
  delay(300);
}

// 画一条线段（在纸面坐标里插值）
bool drawLine(float x0, float y0, float x, float y, float z0) {
  float dx = x - x0;
  float dy = y - y0;
  float len = sqrt(dx*dx + dy*dy);
  int n = (int)(len / LINE_STEP);
  if (n < 1) n = 1;
  for (int i = 1; i <= n; i++) {
    char cmd3 = readCommand();           // 遥控板 / 串口 都能进来
    if (cmd3 == 'P' || cmd3 == 'p') {
      if (!pauseLoop()) return false;
    }
    if (cmd3 == 'X' || cmd3 == 'x') return false;

    float nowx = x0 + dx * i / n;
    float nowy = y0 + dy * i / n;
    delay(15);
    float t1, t2, t3;
    if (!exchangeData(nowx, nowy, z0, t1, t2, t3)) return false;
    moveToAnglesFast(baseToServo(t1), rarmToServo(t2), farmToServo(t3));
    curX = nowx;
    curY = nowy;
    delay(LINE_DELAY);
  }
  return true;
}

// 画一个由若干点连成的图形
bool drawShape(const float wx[], const float wy[], int n, const __FlashStringHelper* name)
{
  Serial.print(F("[任务] 绘制: ")); Serial.println(name);
  penUp();
  delay(500);
  if (!moveToPoint(wx[0], wy[0], Z_TRAVEL)) return false;
  penDown(wx[0], wy[0]);
  for (int i = 1; i < n; i++) {
    if (!drawLine(wx[i-1], wy[i-1], wx[i], wy[i], Z_PEN_DOWN)) return false;
  }
  delay(500);
  penUp();
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println(F("[完成] 已回待机位"));
  return true;
}

// 拿笔
void getpen()
{
  Serial.println(F("[拿笔] 张开爪子, 移向取笔点"));
  gripOpen();
  moveToPoint(pensite.x, pensite.y, PEN_HOVER);
  moveToPoint(pensite.x, pensite.y, pensite.z);
  Serial.println(F("[拿笔] 爪子已到位, 请调整笔位, 2 秒后自动夹紧"));
  delay(2000);
  Serial.println(F("[拿笔] 夹住铅笔"));
  gripClose();
  penheld = true;
  moveToPoint(pensite.x, pensite.y, PEN_HOVER);
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println(F("[拿笔] 完成, 笔已握好, 可以开始绘制"));
}

// 统一取消：抬笔回待机位
void cancelToHome() {
  penUp();
  delay(20);
  moveToPoint(HOME.x, HOME.y, HOME.z);

  while (Serial.available() > 0) { Serial.read(); }   // 清堆积的串口指令
  while (link.available() > 0)   { link.read();   }   // 板间串口也清一遍

  Serial.println(F("取消程序，回到休息位置"));
}

// 暂停（遥控板或串口都能恢复/取消）
bool pauseLoop()
{
  Serial.println(F("暂停（遥控板“继续”或本机 RESUME 键恢复；“取消”中止）"));
  while (true) {
    char cmd2 = readCommand();
    if (cmd2 == 'U' || cmd2 == 'u') { Serial.println(F("继续")); delay(200); return true; }
    if (cmd2 == 'X' || cmd2 == 'x') { Serial.println(F("取消")); return false; }
    delay(20);
  }
}

// 正解：舵机角 -> 笔尖平面坐标（示教存点用）
void forwardKinematics(float sBase, float sRarm, float sFarm, float &x, float &y, float &z)
{
  float s1 = constrain(sBase, 0, 180);
  float s2 = constrain(sRarm, 20, 120);
  float s3 = constrain(sFarm, 68, 145);

  float t1 = (s1 - Base_OFF) / Base_DIR;
  float t2 = (s2 - rArm_OFF) / rArm_DIR;
  float t3 = (s3 - fArm_OFF) / fArm_DIR;

  float r1 = t1 * PI / 180.0;
  float r2 = t2 * PI / 180.0;
  float r3 = t3 * PI / 180.0;
  float r4 = r3 - r2;

  float pr = L1*cos(r2) + L2*cos(PI - r2 - r4) + L4;

  float m1 = L1 * sin(r2);
  float m2 = L2 * sin(PI - r2 - r4);
  float pz = (L1 > L2) ? (L1 - L2) : (L2 - L1);

  x = pr * cos(r1);
  y = pr * sin(r1);
  z = pz + H0;
}

// 示教：摇杆把笔尖移到 5 个目标点，按记录键存点
bool teachMode() {
  teachCount = 0;
  Serial.println(F("=== 示教开始: 摇杆控舵机, 记录键存点(共5个) ==="));
  Serial.println(F("    摇杆1: 左右=底座 / 前后=大臂   摇杆2: 前后=小臂 (爪子已锁定)"));

  moveToPoint(HOME.x, HOME.y, Z_TEACH);      // 从待机位上方出发

  // —— 阶段1：采集 5 个示教点 ——
  while (teachCount < 5) {
    curBase = constrain(curBase + joyDeltaAngle(JOY_1X_PIN),   0, 180);
    currarm = constrain(currarm + joyDeltaAngle(JOY_1Y_PIN),  20, 120);
    curfarm = constrain(curfarm + joyDeltaAngle(JOY_2Y_PIN),  68, 145);
    Base.write((int)round(curBase));
    rArm.write((int)round(currarm));
    fArm.write((int)round(curfarm));

    char k = readCommand();                  // 遥控板 / 串口 都能存点
    if (k == 'R') {
      float tx, ty, tz;
      forwardKinematics(curBase, currarm, curfarm, tx, ty, tz);

      teachX[teachCount] = tx;
      teachY[teachCount] = ty;
      teachCount++;
      Serial.print(F("  示教点 ")); Serial.print(teachCount);
      Serial.print(F(": 坐标(")); Serial.print(tx, 1);
      Serial.print(F(", "));      Serial.print(ty, 1);
      Serial.print(F(") z="));    Serial.print(tz, 1);
      Serial.print(F("  舵机(")); Serial.print(curBase); Serial.print(F(", "));
      Serial.print(currarm);   Serial.print(F(", ")); Serial.print(curfarm);
      Serial.println(F(")"));
      delay(300);                            // 留出松手/防连记的时间
    }
    if (k == 'X') {                          // ★ 取消指令是 'X'
      teachCount = 0;
      return false;
    }
    delay(TEACH_FRAME);
  }

  // —— 阶段2：等待开始指令 ——
  Serial.println(F("5 点已存满! 再按记录键开始绘制 / 按取消键放弃"));
  while (true) {
    char k = readCommand();
    if (k == 'R') return true;               // 开始绘制
    if (k == 'X') {                          // ★ 放弃
      teachCount = 0;
      return false;
    }
    delay(30);
  }
}

// 五点折线
bool drawPolyline() {
  Serial.println(F("[任务]四 五点折线绘制"));
  moveToPoint(HOME.x, HOME.y, HOME.z);       // 先回待机位
  penUp();
  if (!moveToPoint(teachX[0], teachY[0], Z_TRAVEL)) return false;
  penDown(teachX[0], teachY[0]);             // 在第1点落笔

  for (int i = 1; i < 5; i++) {              // 4 段折线
    Serial.print(F("  段 ")); Serial.print(i); Serial.println(F("/4"));
    if (!drawLine(teachX[i-1], teachY[i-1], teachX[i], teachY[i], Z_PEN_DOWN)) return false;
  }
  penUp();
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println(F("[完成] 折线绘制结束, 已回待机位"));
  return true;
}

// Catmull-Rom 插值（曲线用）
float catmull(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5f * (2 * p1
                 + (-p0 + p2) * t
                 + (2*p0 - 5*p1 + 4*p2 - p3) * t2
                 + (-p0 + 3*p1 - 3*p2 + p3) * t3);
}

// 五点平滑曲线
bool drawCurve() {
  Serial.println(F("[任务]五 五点平滑曲线绘制"));
  float gx[7];
  float gy[7];
  for (int i = 0; i < 5; i++) {
    gx[i+1] = teachX[i];
    gy[i+1] = teachY[i];
  }
  gx[0] = teachX[0];   gx[6] = teachX[4];
  gy[0] = teachY[0];   gy[6] = teachY[4];

  moveToPoint(HOME.x, HOME.y, HOME.z);       // 先回待机位
  penUp();
  if (!moveToPoint(teachX[0], teachY[0], Z_TRAVEL)) return false;
  penDown(teachX[0], teachY[0]);

  float px = teachX[0];
  float py = teachY[0];
  for (int part = 1; part < 5; part++) {
    Serial.print(F("  曲线段 ")); Serial.print(part); Serial.println(F("/4"));
    for (int i = 1; i <= CURVE_SAMPLES; i++) {
      float t = (float)i / CURVE_SAMPLES;
      float x = catmull(gx[part-1], gx[part], gx[part+1], gx[part+2], t);
      float y = catmull(gy[part-1], gy[part], gy[part+1], gy[part+2], t);
      if (!drawLine(px, py, x, y, Z_PEN_DOWN)) return false;
      px = x;
      py = y;
    }
  }
  penUp();
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println(F("[完成] 曲线绘制结束, 已回待机位"));
  return true;
}

/* ==================== 演示模式（任务2/3/5）==================== */

// 演示模式主循环
void demoLoop() {
  bool ok = true;
  char cmd = readCommand();               // 遥控板 / USB 串口 二合一
  if (cmd == 0) { joyMove(); return; }    // ★ 空闲时：摇杆实时操控
  // 空闲时按到过程控制键不做事，避免误报"未知指令"
  if (cmd == 'P' || cmd == 'p' || cmd == 'U' || cmd == 'u' ||
      cmd == 'X' || cmd == 'x' || cmd == 'R') return;
  if (cmd == 'M' || cmd == 'm') { switchMode(); return; }

  Serial.print(F("[CMD] "));
  Serial.println(cmd);

  switch (cmd) {

    /* ================= 任务3 ================= */
    case '1':                             // 循环夹取 A -> B -> C -> A
      if (abcIndex == 0)      { Serial.println(F("执行任务: 夹取A")); pickAndPlace(pickA, placeA); }
      else if (abcIndex == 1) { Serial.println(F("执行任务: 夹取B")); pickAndPlace(pickB, placeB); }
      else                    { Serial.println(F("执行任务: 夹取C")); pickAndPlace(pickC, placeC); }
      abcIndex = (abcIndex + 1) % 3;
      break;

    case '2':                             // 录制 开始/结束
      toggleRecord();
      break;

    case '3':                             // 播放
      Serial.println(F("开始播放程序"));
      playrecord();
      break;

    case '4':                             // 回中
      Serial.println(F("Move to Home"));
      moveToPoint(HOME.x, HOME.y, HOME.z);
      break;

    /* ================= 任务2：分别夹取 ================= */
    case 'A': case 'a':                   // 夹取 A
      Serial.println(F("执行任务2: 夹取A"));
      pickAndPlace(pickA, placeA);
      break;

    case 'B': case 'b':                   // 夹取 B
      Serial.println(F("执行任务2: 夹取B"));
      pickAndPlace(pickB, placeB);
      break;

    case 'C': case 'c':                   // 夹取 C
      Serial.println(F("执行任务2: 夹取C"));
      pickAndPlace(pickC, placeC);
      break;

    /* ================= 任务5 ================= */
    case 'O':
    case 'o':
      if (!penheld) getpen();
      break;

    case 'L':
    case 'l':
      ok = drawShape(L_x, L_y, 2, F("直线"));
      break;

    case 'V':
    case 'v':
      ok = drawShape(V_x, V_y, 3, F("字母V"));
      break;

    case 'T':                             // 子任务4：五点示教 + 折线
      if (teachMode()) ok = drawPolyline();
      else             ok = false;
      break;

    case '5':                             // 子任务5：五点示教 + 曲线
      if (teachMode()) ok = drawCurve();
      else             ok = false;
      break;

    default:
      Serial.println(F("未知指令 ( 1/2/3/4 / A/B/C / O/L/V/T/5 , M=切模式 )"));
      break;
  }

  if (!ok) cancelToHome();

  Serial.println(F("------------------------------"));
}

/* ==================== 模式切换与菜单 ==================== */

// 切模式：先回中，保证账本与实际姿态一致
void switchMode() {
  moveToPoint(HOME.x, HOME.y, HOME.z);     // ★ 先回中（同步 float 账本）

  if (workMode == MODE_DEMO) {
    workMode = MODE_MANUAL;
    Serial.println(F(">>> 已切换到【手动模式】(任务一)"));
    printManualMenu();
  } else {
    workMode = MODE_DEMO;
    Serial.println(F(">>> 已切换到【演示模式】(任务2/3/5)"));
    printDemoMenu();
  }
}

void printDemoMenu() {
  Serial.println(F("----- 任务2: A/B/C  (分别夹取) -----"));
  Serial.println(F("   A=夹取A   B=夹取B   C=夹取C"));
  Serial.println(F("----- 任务3: 1/2/3/4 -----"));
  Serial.println(F("   1=循环夹取A,B,C  2=录制  3=播放  4=回中"));
  Serial.println(F("----- 任务5: O/L/V/T/5 -----"));
  Serial.println(F("   O=拿笔  L=直线  V=字母V"));
  Serial.println(F("   T=五点示教+折线   5=五点示教+曲线"));
  Serial.println(F("----- 过程控制（遥控板 / 串口敲字母） -----"));
  Serial.println(F("   P=暂停  U=继续  X=取消  R=示教记录"));
  Serial.println(F("   M=切到【手动模式】 (空闲推摇杆=实时操控)"));
}

void printManualMenu() {
  Serial.println(F("----- 手动模式（任务一） -----"));
  Serial.println(F("   USB串口: b/r/f<角度> 单关节     例 b90 / r45 / f90"));
  Serial.println(F("   USB串口: x<x>,y<y>,z<z> 三轴同步  例 x10,y30,z20"));
  Serial.println(F("   O=张开爪子  S=闭合爪子"));
  Serial.println(F("   H=加速  L=减速  i=回中并打印四关节角度"));
  Serial.println(F("   M=切回【演示模式】 (空闲推摇杆=实时操控)"));
}

/* ==================== 初始化 ==================== */
void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(50);      // 收敛 readStringUntil 的默认 1s 阻塞
  link.begin(LINK_BAUD);      // 打开板间串口，准备接收遥控板指令

  Base.attach(Base_PIN);
  rArm.attach(rArm_PIN);
  fArm.attach(fArm_PIN);
  Gripper.attach(GRIPPER_PIN);

  // ★ 本机没有按键，不需要 pinMode；过程控制从串口/遥控板进来

  Base.write(90);
  delay(10);
  rArm.write(45);
  delay(10);
  fArm.write(90);
  delay(10);
  Gripper.write(GRIP_OPEN_ANGLE);

  Serial.println(F("================================================="));
  Serial.println(F(" task1235 four-in-one ready.  (task1 + task2 + task3 + task5)"));
  Serial.println(F(" 当前模式: 【演示模式】   按 M 切到【手动模式】"));
  printDemoMenu();
  Serial.println(F("================================================="));
  delay(10);
}

/* ==================== 主循环 ==================== */
void loop() {
  if (workMode == MODE_DEMO) demoLoop();   // 演示模式：任务2/3/5
  else                       manualLoop(); // 手动模式：任务一
}
