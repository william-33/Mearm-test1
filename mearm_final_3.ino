#include <Servo.h>
#include <math.h>
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

//控制机械臂的平滑移动
#define STEP_ANGLE   2     // 每步最多转 2 度
#define STEP_DELAY   15  

#define HOVER_DZ 50.0   // 悬停高度：在目标点正上方 50mm 处移动，防止横扫撞倒物体

/* ==================== 全局变量 ==================== */

//确定各个部分的初始角度值
const float Base_OFF=90,Base_DIR=1;
const float rArm_OFF=90,rArm_DIR=1;
const float fArm_OFF=90,fArm_DIR=1;

//各个部分的角度
float baseangle=90;
float rarmangle=90;
float farmangle=90;

// 记录三个关节舵机"当前"的角度（用于平滑步进时知道从哪出发）
int curBase = 90, currarm = 90, curfarm = 90;

//确定好各个臂的长度
const float L1=;      //大臂       记得测量实际的长度，90.0
const float L2=;      //小臂
const float H0=;      //底座

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper;

//建立结构体
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

/* ==================== 工具函数 ==================== */

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
  if(H1<0)rarmangle=(tmp1)-(tmp2);
  else if(H1>0)rarmangle=(tmp1)+(tmp2);
  else if(H1==0)rarmangle=tmp1;
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

  Serial.println("======================================");
  Serial.println(" Robot arm pick & place ready (IK)");
  Serial.println(" Send  A / B / C  to pick object A/B/C");
  Serial.println("======================================");

}
/* ==================== 主循环 ==================== */

void loop()
  {if (Serial.available() > 0) {
    char cmd = Serial.read();   // 只读一个字符

    // 2. 忽略串口监视器自动附带的换行/回车/空格
    if (cmd == '\n' || cmd == '\r' || cmd == ' ' || cmd == '\t') {
      return;
    }

    Serial.print("[CMD] ");
    Serial.println(cmd);

    // 3. 根据字符执行对应的"夹取-放置"任务
    switch (cmd) {
      case 'A': case 'a':
        pickAndPlace(pickA, placeA);
        Serial.println("DONE: A");
        break;
      case 'B': case 'b':
        pickAndPlace(pickB, placeB);
        Serial.println("DONE: B");
        break;
      case 'C': case 'c':
        pickAndPlace(pickC, placeC);
        Serial.println("DONE: C");
        break;
      default:
        // 收到不认识的字符，打印帮助信息
        Serial.println("Unknown command. Send A / B / C.");
        break;
    }
    Serial.println("------------------------------");
  }
 }
    