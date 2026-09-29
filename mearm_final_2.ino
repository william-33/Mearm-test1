#include <Servo.h>
/* ==================== 配置区 (按实际硬件修改) ==================== */

#define GRIPPER_PIN   9    // 爪子舵机
#define Base_PIN     6    // 关节1 (底座旋转)                              记得要改引脚
#define rArm_PIN     5    // 关节2 (大臂)
#define fArm_PIN     3    // 关节3 (小臂)

// 爪子开 / 关 角度 (单位: 度, 0~180, 按舵机实际行程调整)
#define GRIP_OPEN_ANGLE   90
#define GRIP_CLOSE_ANGLE  30

// 串口波特率
#define BAUD_RATE         9600

//定义脉宽最值，角度最值，周期
#define MAX_PULSE_US   2500
#define MIN_PULSE_US   500
#define MAX_ANGLE      180
#define MIN_ANGLE      0
#define FRAME_US       20000UL // PWM 周期 20ms（50Hz）

/* ==================== 全局变量 ==================== */

// 爪子当前角度 (记录以实现平滑步进)
int gripperAngle = GRIP_OPEN_ANGLE;

//确定各各舵机的最大值最小值（防止机器臂超出实际的移动角度）
const int maxbase;            //先通过另一个程序确定再填写进入                               记得和机械的同学沟通一下
const int maxfArm;
const int maxrArm;
const int maxclaw;

const int minbase;
const int minfArm;
const int minrArm;
const int minclaw;

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper;

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
  return angleToPulse(angle)/FRAME_US*100.0;
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

/* ==================== 初始化 ==================== */

void setup() {
  Serial.begin(BAUD_RATE);
  
  Base.attach(Base_PIN);
  rArm.attach(rArm_PIN);
  fArm.attach(fArm_PIN);
  Gripper.attach(GRIPPER_PIN);

  Base.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  fArm.write(90);
  delay(10);
  Gripper.write(GRIP_OPEN_ANGLE);

  
  Serial.println("Send a command like: x10,y30,z20");
  
}

/* ==================== 主循环 ==================== */

void loop() {
  if (Serial.available() > 0) {
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
}






















