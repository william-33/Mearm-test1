#include <Servo.h>
#include <math.h>
/* ==================== 配置区 (按实际硬件修改) ==================== */

#define GRIPPER_PIN   6    // 爪子舵机
#define Base_PIN     9    // 关节1 (底座旋转)                              记得要改引脚
#define rArm_PIN     8    // 关节2 (大臂)
#define fArm_PIN     7    // 关节3 (小臂)

// 爪子开 / 关 角度 
#define GRIP_OPEN_ANGLE   146
#define GRIP_CLOSE_ANGLE  56

// 串口波特率
#define BAUD_RATE         9600

//定义脉宽最值，角度最值，周期
#define MAX_PULSE_US   2400
#define MIN_PULSE_US   544
#define MAX_ANGLE      180
#define MIN_ANGLE      0
#define FRAME_US       20000UL // PWM 周期 20ms（50Hz）

#define SPEED_LEVELS   5
#define STEP_ANGLE     2

// —— 2 个摇杆的 4 个模拟轴 ——
#define JOY_1X_PIN    A0    // 摇杆1 X -> 底座 Base
#define JOY_1Y_PIN    A1    // 摇杆1 Y -> 大臂 rArm
#define JOY_2X_PIN    A2    // 摇杆2 X -> 小臂 fArm
#define JOY_2Y_PIN    A3    // 摇杆2 Y -> 夹爪 Gripper

// —— 摇杆参数 ——
#define JOY_CENTER    513   
#define JOY_DEAD      50   
#define JOY_SENSE     256    // 灵敏度：偏移 / 32 = 每圈转几度，数值越小越快,到时候自己挑

/* ==================== 全局变量 ==================== */

// 爪子当前角度 (记录以实现平滑步进)
int gripperAngle = GRIP_OPEN_ANGLE;

//确定各各舵机的最大值最小值（防止机器臂超出实际的移动角度）
const int maxbase = 180;      // 底座 上限
const int maxfArm = 130;      // 小臂 上限
const int maxrArm = 115;      // 大臂 上限
const int maxclaw = 146;      // 爪子上限

const int minbase = 0;        // 底座 下限
const int minfArm = 50;       // 小臂 下限
const int minrArm = 20;       // 大臂 下限
const int minclaw = 56;       // 爪子下限

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper;

int toPos = 90;

int curBase = 90;
int currArm = 20;
int curfArm = 90;

int speedIndex = 2;
const int speedDelay[SPEED_LEVELS] = {5, 10, 15, 20, 25};
/* ==================== 工具函数 ==================== */

//读取字符串并获得相应的值角度
int getValue(String cmd,char letter)
{
  int start =cmd.indexOf(letter);
  if (start<0) return -1;
  start=start+1;
  int end =cmd.indexOf(',',start);
  if(end<0) end=cmd.length();
  String value=cmd.substring(start,end);
  return value.toInt(); 
}

//计算高电平的脉宽
unsigned long int angleToPulse(int angle)
{
  return (unsigned long)angle*(MAX_PULSE_US-MIN_PULSE_US)/MAX_ANGLE+MIN_PULSE_US;
}

//计算占空比
float angleToDuty(int angle)
{
  return angleToPulse(angle)*100.0/FRAME_US;
}

//判断x是否在合理区间内
int jugetmentx(int x)
{
  if(x>maxbase)       {Serial.println("too high");return maxbase;}
  else if(x<minbase)  {Serial.println("too low");return minbase;}
  else return x;
}
//判断y是否在合理区间内
int jugetmenty(int y)
{
  if(y>maxrArm)       {Serial.println("too high");return maxrArm;}
  else if(y<minrArm)  {Serial.println("too low");return minrArm;}
  else return y;
}

//判断z是否在合理区间内
int jugetmentz(int z)
{
  if(z>maxfArm)       {Serial.println("too high");return maxfArm;}
  else if(z<minfArm)  {Serial.println("too low");return minfArm;}
  else return z;
}

void moveServo(Servo &servo, int &current, int target) {
  if (target == current) return;

  int step = (target > current) ? STEP_ANGLE : -STEP_ANGLE;
  while (current != target) {
    current += step;

    // 防止越过目标
    if ((step > 0 && current > target) || (step < 0 && current < target)) {
      current = target;
    }

    servo.write(current);
    delay(speedDelay[speedIndex]);
  }
}

// 回传确认信息
void ack(char cmd) {
  Serial.print("OK:");
  Serial.print(cmd);
  Serial.print("  speed=");
  Serial.println(speedIndex);
}

//在恢复为初始状态时显示出当前的机器信息
void ackone(Servo &servo)
{
  Serial.println(servo.read());
}

//是否移动，并赋值
float joyDelta(int pin)
{
  float off=analogRead(pin)-JOY_CENTER;
  if(abs(off)<JOY_DEAD) return 0;
  float angle=off/JOY_SENSE;
  Serial.print(off);
  Serial.print(",");
  Serial.println(angle);
  return angle;
}

//是否在进行移动，用bool来判断是否进行下一个的串口通信
bool joyMove(){
  int dB = joyDelta(JOY_1X_PIN);
  int dR = joyDelta(JOY_1Y_PIN);
  int dF = joyDelta(JOY_2X_PIN);
  int dG = joyDelta(JOY_2Y_PIN);

  if (dB == 0 && dR == 0 && dF == 0 && dG == 0) return false; 
  curBase += dB;
  currArm += dR;
  curfArm += dF;
  gripperAngle += dG;

  if (curBase < minbase) curBase = minbase;   // 限幅变量
  if (curBase > maxbase) curBase = maxbase;
  if (currArm < minrArm) currArm = minrArm;
  if (currArm > maxrArm) currArm = maxrArm;
  if (curfArm < minfArm) curfArm = minfArm;
  if (curfArm > maxfArm) curfArm = maxfArm;
  if (gripperAngle < minclaw) gripperAngle = minclaw;
  if (gripperAngle > maxclaw) gripperAngle = maxclaw;

  Base.write(curBase);
  rArm.write(currArm);
  fArm.write(curfArm);
  Gripper.write(gripperAngle);
  return true;                                // 有动作 -> 本圈摇杆接管
}


/* ==================== 初始化 ==================== */

void setup() {
  Serial.begin(BAUD_RATE);
  
  Base.attach(Base_PIN);
  rArm.attach(rArm_PIN);
  fArm.attach(fArm_PIN);
  Gripper.attach(GRIPPER_PIN);
  gripperAngle = GRIP_OPEN_ANGLE;

  Base.write(90);
  delay(10);
  rArm.write(20);
  delay(10);
  fArm.write(90);
  delay(10);
  Gripper.write(GRIP_OPEN_ANGLE);

  Serial.println("================================");
  Serial.println(" MeArm merged ready @9600 baud");
  Serial.println(" b/r/f<deg> : move one joint");
  Serial.println(" O/S        : gripper open / close");
  Serial.println(" H/L        : faster / slower");
  Serial.println(" i          : home + status");
  Serial.println(" x10,y30,z20: move 3 axes together");
  Serial.println("================================");
}

/* ==================== 主循环 ==================== */

void loop() {
  // 摇杆优先：本圈摇杆有动作
  if (joyMove()&&Serial.available()==0) return;
  delay(100);
  
  if (Serial.available() > 0) {
    char check=Serial.peek();
    if(check=='x'||check=='X'){
    String cmd=Serial.readStringUntil('\n');
    cmd.trim();
    int x=getValue( cmd, 'x');
    int y=getValue( cmd, 'y');
    int z=getValue( cmd, 'z');
    
    if(x<0||y<0||z<0) 
    {
      Serial.print("Parse error: ");
      Serial.println(cmd);
    }else
    {
      //调整合理角度
      x=jugetmentx(x);
      y=jugetmenty(y);
      z=jugetmentz(z);

      Base.write(x);
      rArm.write(y);
      fArm.write(z);
      curBase=x; currArm=y; curfArm=z;    // ★ 账本同步

      Serial.println("=== command OK ===");
      Serial.print("x = "); Serial.print(x);
      Serial.print(" deg, pulse = "); Serial.print(angleToPulse(x));
      Serial.print(" us, duty = ");   Serial.print(angleToDuty(x), 2);
      Serial.println(" %");

      Serial.print("y = "); Serial.print(y);
      Serial.print(" deg, pulse = "); Serial.print(angleToPulse(y));
      Serial.print(" us, duty = ");   Serial.print(angleToDuty(y), 2);
      Serial.println(" %");

      Serial.print("z = "); Serial.print(z);
      Serial.print(" deg, pulse = "); Serial.print(angleToPulse(z));
      Serial.print(" us, duty = ");   Serial.print(angleToDuty(z), 2);
      Serial.println(" %");
      Serial.println("==================");
    }
    }
    else{
      char cmd = Serial.read();
      switch (cmd) {
      case 'b':   // 基底移动
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxbase&&toPos>=minbase)
        {moveServo(Base,curBase,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'r':   // 后臂移动
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxrArm&&toPos>=minrArm)
        {moveServo(rArm,currArm,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'f':  // 前臂移动
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxfArm&&toPos>=minfArm)
        {moveServo(fArm,curfArm,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'O':  // 爪子张开
        moveServo(Gripper, gripperAngle, GRIP_OPEN_ANGLE);
        ack(cmd);
        break;

      case 'S':  // 爪子关闭
        moveServo(Gripper, gripperAngle, GRIP_CLOSE_ANGLE);
        ack(cmd);
        break;

      case 'H':  // 加速: 档位-1, 有下限保护
        if (speedIndex > 0) {
          speedIndex--;
        }
        ack(cmd);
        break;

      case 'L':  // 减速: 档位+1, 有上限保护
        if (speedIndex < SPEED_LEVELS - 1) {
          speedIndex++;
        }
        ack(cmd);
        break;

      case 'i':
      //初始化机器
        moveServo(Gripper, gripperAngle, GRIP_OPEN_ANGLE);
        moveServo(fArm,curfArm,90);
        moveServo(rArm,currArm,20);
        moveServo(Base,curBase,90);
      //显示状态
        Serial.print("the base=");
        ackone(Base);
        Serial.print("the rArm=");
        ackone(rArm);
        Serial.print("the fArm=");
        ackone(fArm);
        Serial.print("the Gripper=");
        ackone(Gripper);
        break;
      default:
        // 忽略空白字符与非法指令
        if (cmd != '\n' && cmd != '\r' && cmd != ' ' && cmd != '\t') {
          Serial.print("Unknown command: ");
          Serial.println(cmd);
        }
        break;
    }
  }
}
}