/*
 *  上位机 (电脑串口监视器) 发送单字符指令, 机械臂执行:
 *
 *    'b,topos'  -> 移动基底  （到达目标角度）
 *    'f,topos'  -> 移动小前臂（到达目标角度）
 *    'r,topos'  -> 移动大后臂（到达目标角度）
 *    'O'  ->  爪子张开
 *    'S'  ->  爪子关闭
 *    'H'  ->  提升整体运行速度 (速度档位 +1)
 *    'L'  ->  降低整体运行速度 (速度档位 -1)
 *
 *  收到指令后, 串口回传确认信息, 便于上位机/调试查看。
 *
 *  --------------------------------------------
 *    - 舵机信号线分别接下方 GRIPPER_PIN / Base_PIN / rArm_PIN / fArm_PIN
 *    - 舵机电源(VCC)建议独立供电(5~6V), 与 Arduino 共地(GND)
 *    - 棕色/黑色线 = GND, 红色 = VCC, 橙色/黄色 = 信号
 *  --------------------------------------------
 */

#include <Servo.h>

/* ==================== 配置区 (按实际硬件修改) ==================== */

// 舵机引脚定义
#define GRIPPER_PIN   9    // 爪子舵机
#define Base_PIN     6    // 关节1 (底座旋转)                              记得要改引脚
#define rArm_PIN     5    // 关节2 (大臂)
#define fArm_PIN     3    // 关节3 (小臂)

// 爪子开 / 关 角度 (单位: 度, 0~180, 按舵机实际行程调整)
#define GRIP_OPEN_ANGLE   90
#define GRIP_CLOSE_ANGLE  30

// 串口波特率
#define BAUD_RATE         9600

// 速度档位: 数组元素 = 每一步进之间的延时(ms), 值越小速度越快
#define SPEED_LEVELS      5
const int speedDelay[SPEED_LEVELS] = {5, 10, 15, 20, 25};

// 舵机动作步进角度 (每次移动的度数, 越小越平滑)
#define STEP_ANGLE        2

/* ==================== 全局变量 ==================== */

Servo gripper;
Servo Base;
Servo rArm;
Servo fArm;

// 当前速度档位 (0 ~ SPEED_LEVELS-1), 初始为中间档
int speedIndex = 2;

// 爪子当前角度 (记录以实现平滑步进)
int gripperAngle = GRIP_OPEN_ANGLE;
//各舵机当前角度以及目标角度
int toPos =90;
int fromPos=90;

//确定各各舵机的最大值最小值（防止机器臂超出实际的移动角度）
const int maxbase;            //先通过另一个程序确定再填写进入                                 记得和机械的同学沟通一下
const int maxfArm;
const int maxrArm;
const int maxclaw;

const int minbase;
const int minfArm;
const int minrArm;
const int minclaw;


/* ==================== 工具函数 ==================== */

// 平滑移动舵机到目标角度 (按步进 + 当前速度档位延时)
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


/* ==================== 初始化 ==================== */

void setup() {
  Serial.begin(BAUD_RATE);

  // 初始化舵机并归位
  gripper.attach(GRIPPER_PIN);
  delay(50);
  Base.attach(Base_PIN);
  delay(50);
  rArm.attach(rArm_PIN);
  delay(50);
  fArm.attach(fArm_PIN);
  delay(50);

  // 爪子初始为张开
  gripper.write(GRIP_OPEN_ANGLE);
  gripperAngle = GRIP_OPEN_ANGLE;
  delay(10);

  // 关节归中位 (可按需调整)
  Base.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  fArm.write(90);
  delay(10);

  Serial.println("Robot arm ready.");
  Serial.println("Commands: O=open  S=close  H=faster  L=slower");
  Serial.println("Commands:");
}

/* ==================== 主循环 ==================== */

void loop() {
  // 等待上位机发送指令
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'b':   // 基底移动
        fromPos=Base.read();
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxbase&&toPos>=minbase)
        {moveServo(Base,fromPos,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'r':   // 后臂移动
        fromPos=rArm.read();
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxrArm&&toPos>=minrArm)
        {moveServo(rArm,fromPos,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'f':  // 前臂移动
        fromPos=fArm.read();
        delay(10);
        toPos = Serial.parseInt();
        if(toPos<=maxfArm&&toPos>=minfArm)
        {moveServo(fArm,fromPos,toPos);
        ack(cmd);}
        else{Serial.println("out of the Limit!!");}
        break;
      
      case 'O':  // 爪子张开
        moveServo(gripper, gripperAngle, GRIP_OPEN_ANGLE);
        ack(cmd);
        break;

      case 'S':  // 爪子关闭
        moveServo(gripper, gripperAngle, GRIP_CLOSE_ANGLE);
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
