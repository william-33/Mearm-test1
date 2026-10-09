#include <Servo.h>           
#include <math.h>            
#include <SoftwareSerial.h>

/* ==================== 配置区 ==================== */

// 四个舵机引脚,与任务二一致
#define Base_PIN       6
#define rArm_PIN       5
#define fArm_PIN       3
#define GRIPPER_PIN    9

#define BAUD_RATE     9600   //USB 调试串口波特率

SoftwareSerial link(10, 11); //修改记得
#define LINK_BAUD  9600

#define JOY_1X_PIN   A0       // 摇杆1 X 轴 -> 底座
#define JOY_1Y_PIN   A1       // 摇杆1 Y 轴 -> 大臂
#define JOY_2X_PIN   A2       // 摇杆2 X 轴 -> 小臂
#define JOY_2Y_PIN   A3       // 摇杆2 Y 轴 -> 爪子
#define JOY_CENTER  512      // 摇杆中位理论值
#define JOY_DEAD    50

const float L1 = ;   //记得填写
const float L2 = ;
const float H0 = ;

#define GRIP_OPEN_ANGLE   90 //也记得修改正确
#define GRIP_CLOSE_ANGLE  30

#define STEP_ANGLE   2
#define STEP_DELAY   15

#define REC_INTERVAL  100    // 采样间隔 ms（每秒 10 帧）
#define MAX_FRAMES    150    //最大帧数
#define HOVER_DZ      50.0
/* ==================== 全局变量 ==================== */

const float Base_OFF=90,Base_DIR=1;
const float rArm_OFF=90,rArm_DIR=1;
const float fArm_OFF=90,fArm_DIR=1;

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper; 

int  abcIndex  = 0;       // 按键1循环计数：0=下次夹A，1=夹B，2=夹C
bool recording = false;

struct Frame {
  byte b;   // 底座角
  byte r;   // 大臂角
  byte f;   // 小臂角
  byte g;   // 爪子
};
Frame rec[MAX_FRAMES];    // 录制缓冲区
int recLen = 0;           // 本次已录了多少帧

/* ==================== 按键一（直接用任务二的） ==================== */

// 记录三个关节舵机"当前"的角度（用于平滑步进时知道从哪出发）
int curBase = 90, currarm = 90, curfarm = 90;
int curgrip =90;

float baseangle=90;
float rarmangle=90;
float farmangle=90;

struct Point {
  float x;
  float y;
  float z;
};

Point pickA  = {100, -60, 30}; // 物体A 在哪里
Point placeA = { 60,  80, 60}; // 物体A 要放到哪里
Point pickB  = {150,   0, 30}; //同理 
Point placeB = { 90, -90, 60};
Point pickC  = {100,  60, 30};
Point placeC = { 60, -80, 60};

Point HOME   = {150, 0, 100};    // 任务结束后的"休息位"

/* ==================== 工具函数（按键一的） ==================== */
//转变xyz到具体的角度
void exchangeData(float x,float y,float z,float &baseangle,float &rarmangle,float &farmangle)
{
  float tmp1;
  float tmp2;
  float r=sqrt(x*x+y*y);
  float H1=z-H0;
  float L3=sqrt(H1*H1+r*r);
  baseangle=atan2(y,x)* 180.0 / PI;
  float tmp3=(L1*L1+L2*L2-L3*L3)/(2*L1*L2);
  tmp3=constrain(tmp3,-1.0,1.0);
  farmangle=acos(tmp3)* 180.0 / PI;
  float tmp4=(L3*L3+L1*L1-L2*L2)/(2*L3*L1);
  tmp4=constrain(tmp4,-1.0,1.0);
  tmp1=acos(tmp4)* 180.0 / PI;
  tmp2=atan2(H1,r)* 180.0 / PI;
  rarmangle=(tmp1)+(tmp2);
}

//转变具体的角度变为指令的角度
int baseToServo(float baseangle)     { return Base_OFF  + Base_DIR *baseangle ; }
int rarmToServo(float rarmangle)     { return rArm_OFF  + rArm_DIR * rarmangle; }
int farmToServo(float farmangle)     { return fArm_OFF  + fArm_DIR * farmangle; }

//阻塞式平滑移动（三轴同步）
void moveToAngles(float baseangle, float rarmangle, float farmangle) {
  int s1 =baseToServo( baseangle);
  int s2 =rarmToServo( rarmangle);
  int s3 =farmToServo( farmangle);

  int st1 = curBase;                 // 出发角（记录一下，插值要用）
  int st2 = currarm;
  int st3 = curfarm;

  int d1 = abs(s1 - st1), d2 = abs(s2 - st2), d3 = abs(s3 - st3);
  int steps = max(d1, max(d2, d3)) / STEP_ANGLE;
  if (steps < 1) steps = 1;
  
  for (int i = 1; i <= steps; i++) {
    // 线性插值：从"出发角"朝"目标角"走 i/steps 的比例
    curBase = st1 + (s1 - st1) * i / steps;
    currarm  = st2 + (s2 - st2) * i / steps;
    curfarm  = st3 + (s3 - st3) * i / steps;

    Base.write(curBase);             // 三轴同一步内一起写 -> 同步
    rArm.write(currarm);
    fArm.write(curfarm);
    delay(STEP_DELAY);               // 每步停一下，形成"平滑"效果
  }
}

//爪子开合
void gripMoveTo(int target) {
  static int cur = GRIP_OPEN_ANGLE;   // static：函数结束后仍记住爪子当前角度
  while (cur != target) {
    if (target > cur) {
      cur += STEP_ANGLE;              // 往大走一小步
      if (cur > target) cur = target; // 防止越过目标
    } else {
      cur -= STEP_ANGLE;              // 往小走一小步
      if (cur < target) cur = target;
    }
    Gripper.write(constrain(cur, 0, 180));
    delay(STEP_DELAY);
  }
}

void gripOpen()  { gripMoveTo(GRIP_OPEN_ANGLE);  delay(300); } // 张开后等一下
void gripClose() { gripMoveTo(GRIP_CLOSE_ANGLE); delay(500); } // 闭合后多等一会, 夹稳

//一个过程的一小步
void moveToPoint(float x,float y,float z)
{
  exchangeData(x,y,z,baseangle,rarmangle,farmangle);
  moveToAngles(baseangle,  rarmangle,  farmangle);
}

//打印出来
void printStep(int n, const char* msg) {
  Serial.print("["); Serial.print(n); Serial.print("/8] ");
  Serial.println(msg);}

//进行舵机的移动(完整的过程)
void pickAndPlace(Point pick, Point place)
{
  printStep(1, "悬停到夹取点上方");
  moveToPoint(pick.x, pick.y, pick.z + HOVER_DZ);
  
  printStep(2, "下降到夹取点");
  moveToPoint(pick.x, pick.y, pick.z);

  printStep(3, "闭合爪子(夹取)");
  gripClose();

  printStep(4, "抬升回悬停高度");
  moveToPoint(pick.x, pick.y, pick.z + HOVER_DZ);

  printStep(5, "平移到放置点上方");
  moveToPoint(place.x, place.y, place.z + HOVER_DZ);

  printStep(6, "下降到放置点");
  moveToPoint(place.x, place.y, place.z);

  printStep(7, "张开爪子(放置)");
  gripOpen();

  printStep(8, "回到休息位");
  moveToPoint(HOME.x, HOME.y, HOME.z);
}

/* ==================== 工具函数（按键二，三的） ==================== */

//读取按键按下,确实是有移动机器
float truemove(int pin)
{
  int raw=analogRead(pin);
  float off=raw-JOY_CENTER;
  if(abs(off)<JOY_DEAD) return 0;
  return off / 128.0;
}

//改变recording 的值，启动录制(第一层函数)
void toggleRecord(){
  if(!recording)
  {
    recording=true;
    recLen=0;
    Serial.println("开始录制");
    recordloop();
  }
} 

//开始录制的程序，核心
void recordloop()
{
  while(recording)
  {
    if(link.available()>0)
    {
      char cmd=link.read();
      if(cmd=='2')
      {
        recording=false;
        Serial.print("=== 录制结束, 共 "); Serial.print(recLen);
        Serial.print(" 帧 / "); Serial.print(recLen / 10.0, 1);
        Serial.println(" 秒 ===");
        break;
      }
    }
    //移动机器
    curBase  = constrain(curBase  + truemove(JOY_1X_PIN), 0, 180);
    currarm  = constrain(currarm  + truemove(JOY_1Y_PIN), 0, 180);
    curfarm  = constrain(curfarm  + truemove(JOY_2X_PIN), 0, 180);
    curgrip  = constrain(curgrip  + truemove(JOY_2Y_PIN), 0, 180);
    Base.write(curBase);
    rArm.write(currarm);
    fArm.write(curfarm);
    Gripper.write(curgrip);

    //开始记录
    rec[recLen].b = curBase;
    rec[recLen].r = currarm;
    rec[recLen].f = curfarm;
    rec[recLen].g = curgrip;
    recLen++;

    // 录满自动停止：数组有界，不越界
    if (recLen >= MAX_FRAMES) {
      recording = false;
      Serial.println("!!! 已达最长录制 15 秒, 自动结束并保存 !!!");
      break;
    }

    delay(REC_INTERVAL);   // 5. 固定采样间隔，保证"录多久播多久"
  }
}

//开始播放；录制的移动
void playrecord()
{
  if (recLen == 0) {
    Serial.println("还没有录制数据, 请先按按键2录制");
    return;
  }
  else{
  Serial.print("=== 播放录制动作, 共 "); Serial.print(recLen);
  Serial.println(" 帧 ===");
  for(int i=0;i<recLen;i++)
  {
    curBase = rec[i].b;
    currarm = rec[i].r;
    curfarm = rec[i].f;
    curgrip = rec[i].g;
    Base.write(curBase);
    rArm.write(currarm);
    fArm.write(curfarm);
    Gripper.write(curgrip);
    delay(REC_INTERVAL);       // 与录制同节奏
  }
  Serial.println("=== 播放完毕 ===");
}}
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

  Serial.println("===============================");
  Serial.println(" Keys: 1=pick:A,B,C  2=record  3=play  4=home");
  Serial.println("===============================");
}

/* ==================== 主循环 ==================== */
void loop() {
  if(link.available()>0)
  {
    char cmd=link.read();
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') {
      return;
    }

    Serial.print("[CMD] ");
    Serial.println(cmd);

    // 3. 分发任务,判断是哪一个按键，并且依照所按的按键，进行任务展开
    switch(cmd)
    {
      case '1':// 循环夹取 A -> B -> C -> A 
      if(abcIndex == 0)    
      {Serial.println("执行任务: 夹取A");
      pickAndPlace(pickA, placeA);}

      else if(abcIndex == 1)
      {Serial.println("执行任务: 夹取B");
      pickAndPlace(pickB, placeB);}

      else if(abcIndex == 2)
      {Serial.println("执行任务: 夹取C");
      pickAndPlace(pickC, placeC);}

      abcIndex=(abcIndex+1)%3; // 0->1->2->0 循环
      Serial.println("DONE");
      break;

      case '2' : // 录制 开始/结束 
      toggleRecord();
      Serial.println("DONE");
      break;

      case '3': // 播放 
      Serial.println("开始播放程序");
      playrecord();
      Serial.println("DONE");
      break;

      case '4' :// 回中
      Serial.println("Move to Home ");
      moveToPoint(HOME.x,HOME.y,HOME.z);
      Serial.println("DONE");
      break;

      default:
        Serial.println("未知指令 (只认 1/2/3/4)");
        break;
    }
    Serial.println("------------------------------");
  }
}
