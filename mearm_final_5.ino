#include <Servo.h>
#include <math.h>
/* ==================== 配置区 (按实际硬件修改) ==================== */
#define LINE_STEP  1
#define LINE_DELAY  40 

#define GRIPPER_PIN   6    // 爪子舵机
#define Base_PIN     9    // 关节1 (底座旋转)                              ###记得改引脚###
#define rArm_PIN     8    // 关节2 (大臂)
#define fArm_PIN     7    // 关节3 (小臂)
#define PIN_PAUSE   4    // 暂停键
#define PIN_RESUME  12    // 继续键
#define PIN_CANCEL  13    // 取消键

// 爪子开 / 关 角度 
#define GRIP_OPEN_ANGLE   146
#define GRIP_CLOSE_ANGLE  56

// —— 2 个摇杆的 4 个模拟轴 ——
#define JOY_1X_PIN    A0    // 摇杆1 X -> 底座 Base
#define JOY_1Y_PIN    A1    // 摇杆1 Y -> 大臂 rArm
#define JOY_2X_PIN    A2    // 摇杆2 X -> 小臂 fArm
#define JOY_2Y_PIN    A3    // 摇杆2 Y -> 夹爪 Gripper
// 串口波特率
#define BAUD_RATE         9600

#define Z_PEN_DOWN   //待定
#define Z_TRAVEL     //待定

//控制机械臂的平滑移动
#define STEP_ANGLE   2     // 每步最多转 2 度
#define STEP_DELAY   15
/* ==================== 全局变量 ==================== */
struct Point{
  float x;
  float y;
  float z;
};

const float L_x[2] = {100, 160};
const float L_y[2] = {0, 0};                   // ####记得填写修改####
const float V_x[3] = {100, 130, 160};
const float V_y[3] = {-30, 30, -30};

Point pensite ={,,};
Point HOME = {150, 0, 100};   // 待机位                 ####记得修改####
const float PEN_HOVER = ;

Servo Base;
Servo rArm;
Servo fArm;
Servo Gripper;

float baseangle,rarmangle,farmangle;
int curBase = 90, currarm = 20, curfarm = 90;

float curX = 150, curY = 0;//在指定平面的坐标，要改                ####记得修改####

const float L1 = 80.0; //                             ####记得修改####
const float L2 = 120.0;
const float H0 = 0.0;

const float Base_OFF = 90, Base_DIR = 1;
const float rArm_OFF  = 20, rArm_DIR  = 1;
const float fArm_OFF  = 90, fArm_DIR  = 1;


/* ==================== 工具函数 ==================== */
//读取按键
char ReadKeyEdge()
{
  static unsigned long LastMs= 0;
  if(millis()-LastMs<30) return 0;
  if(digitalRead(PIN_PAUSE)==LOW) {LastMs=millis(); return 'P';}
  if(digitalRead(PIN_RESUME)==LOW){LastMs=millis(); return 'U';}
  if(digitalRead(PIN_CANCEL)==LOW){LastMs=millis(); return 'C';}
  return 0;
}

//转变xyz到具体的角度
bool exchangeData(float x,float y,float z,float &baseangle,float &rarmangle,float &farmangle)
{
  float tmp1;
  float tmp2;
  float r=sqrt(x*x+y*y);
  float H1=z-H0;
  float L3=sqrt(H1*H1+r*r);
  if (L3 > L1 + L2)       return false;   
  if (L3 < fabs(L1 - L2)) return false; 
  baseangle=atan2(y,x)* 180.0 / PI;
  float tmp3=(L1*L1+L2*L2-L3*L3)/(2*L1*L2);
  tmp3=constrain(tmp3,-1.0,1.0);
  float tmp4=(L3*L3+L1*L1-L2*L2)/(2*L3*L1);
  tmp4=constrain(tmp4,-1.0,1.0);
  tmp1=acos(tmp4)* 180.0 / PI;
  tmp2=atan2(H1,r)* 180.0 / PI;
  rarmangle=(tmp1)+(tmp2);
  farmangle=(acos(tmp3)* 180.0 / PI)+rarmangle;
  return true;
}

//画出线段的
bool drawLine(float x0,float y0,float x,float y,float z0){
 int i=1;
 float nowx=x0;
 float nowy=y0;
 float dx= x- x0;
 float dy= y- y0;
 float len=sqrt(dx*dx+dy*dy);
 int n=(int)(len/LINE_STEP);
 if(n<1) n=1;
 for(i=1;i<=n;i++){
  char cmd3=ReadKeyEdge();
  if(cmd3=='P')
   {
     if(!pauseLoop()) return false;
   }
  if (cmd3=='C') return false;
   nowx=nowx+dx*i/n;
   nowy=nowy+dy*i/n;

   float t1,t2,t3;
   if (!exchangeData(nowx,nowy,z0,t1,t2,t3)) return false; 
   moveToAnglesFast(baseToServo(t1),rarmToServo(t2),farmToServo(t3));
   
    curX = nowx;
    curY = nowy;
    delay(LINE_DELAY); 
 }
 return true;
}

//转变具体的角度变为指令的角度
int baseToServo(float t1)     { return constrain((int)(Base_OFF  + Base_DIR  * t1), 0, 180); }
int rarmToServo(float t2)     { return constrain((int)(rArm_OFF  + rArm_DIR  * t2), 20, 115); }
int farmToServo(float t3)     { return constrain((int)(fArm_OFF  + fArm_DIR  * t3), 50, 130); }

//用于画直线快速移动
void moveToAnglesFast(int x,int y,int z){
 curBase = x; 
 currarm = y; 
 curfarm = z;
 Base.write(curBase);
 rArm.write(currarm);
 fArm.write(curfarm);
}

//正常的阻塞式平滑移动（三轴同步）
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
    Gripper.write(constrain(cur, 56, 146));
    delay(STEP_DELAY);
  }
}

void gripOpen()  { gripMoveTo(GRIP_OPEN_ANGLE);  delay(300); } // 张开后等一下
void gripClose() { gripMoveTo(GRIP_CLOSE_ANGLE); delay(500); } // 闭合后多等一会, 夹稳

//一个过程的一小步
bool moveToPoint(float x,float y,float z)
{
  if(!exchangeData(x,y,z,baseangle,rarmangle,farmangle))return false;
  moveToAngles(baseangle,  rarmangle,  farmangle);
  curX = x;        // 笔尖已移到 (x, y)
  curY = y;
  return true;
}

//笔的抬起
void penUp() {
  moveToPoint(curX, curY, Z_TRAVEL);
  delay(300);
}

//笔的放下
void penDown(float x, float y) {
  moveToPoint(x, y, Z_PEN_DOWN);
  delay(300);
}

bool drawShape(const float wx[], const float wy[], int n, const char* name)
{
  Serial.print("[任务] 绘制: "); Serial.println(name);
  penUp();
  delay(10);
  if (!moveToPoint(wx[0], wy[0], Z_TRAVEL)) return false;
  penDown(wx[0], wy[0]);
  for (int i = 1; i < n; i++) 
  {
    if (!drawLine(wx[i - 1], wy[i - 1], wx[i], wy[i], Z_PEN_DOWN)) {
      return false;
    }
  }
  penUp();
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println("[完成] 已回待机位");
  return true;
}

//拿笔
void getpen()
{
  Serial.println("[拿笔] 张开爪子, 移向取笔点");
  gripOpen();
  moveToPoint(pensite.x,pensite.y,PEN_HOVER);
  moveToPoint(pensite.x,pensite.y,pensite.z);
  Serial.println("[拿笔] 夹住铅笔");
  gripClose();
  moveToPoint(pensite.x,pensite.y,PEN_HOVER);
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println("[拿笔] 完成, 笔已握好, 可以开始绘制");
}

//判断ok是否是真的，统一取消的窗口
void cancelToHome(){
  penUp();
  delay(20);
  moveToPoint(HOME.x, HOME.y, HOME.z);
  
  while (Serial.available() > 0)      // 清空任务期间可能堆积的串口指令，防止退出后误触发
  {   Serial.read();
  }

  Serial.println("取消程序，回到休息位置");
}

//暂停
bool pauseLoop()
{
  Serial.println("暂停");
  while(true){
  char cmd2=ReadKeyEdge();
  if(cmd2=='U') {Serial.println("继续");delay(200);return true;} 
  if(cmd2=='C') {Serial.println("取消");return false;}
  
  delay(20);}
  
}
/* ==================== 初始化 ==================== */
void setup() {
  Serial.begin(BAUD_RATE);
  
  Base.attach(Base_PIN);
  rArm.attach(rArm_PIN);
  fArm.attach(fArm_PIN);
  Gripper.attach(GRIPPER_PIN);
  
  
  pinMode(PIN_PAUSE, INPUT_PULLUP);
  pinMode(PIN_RESUME, INPUT_PULLUP);
  pinMode(PIN_CANCEL, INPUT_PULLUP);

  Base.write(90);
  delay(10);
  rArm.write(20);
  delay(10);
  fArm.write(90);
  delay(10);
  Gripper.write(GRIP_OPEN_ANGLE);

  Serial.println("==========================================");
  Serial.println(" Robot arm drawing ready. (v1: task 1-3)");
  Serial.println(" Commands: L=line  N/Z/V=letter  S=triangle");
  Serial.println(" Keys: PAUSE(4) RESUME(12) CANCEL(13)");
  Serial.println("==========================================");
  delay(10);

  getpen();
}

/* ==================== 主循环 ==================== */
void loop() {
  if (Serial.available()>0){
    bool ok = true; 
    char cmd=Serial.read();
    if(cmd == '\n' || cmd == '\r' || cmd == ' ')  return;
    Serial.print("[CMD] ");
    Serial.println(cmd);
    switch(cmd){
      case 'L':
      case 'l':
        ok=drawShape(L_x, L_y, 2, "直线");
        break;
      case 'V':
      case 'v':
        ok = drawShape(V_x, V_y, 3, "字母V");
        break;
      default:
        Serial.println("未知指令 (L/V)");
        break;
    }

if(!ok){cancelToHome();}

 Serial.println("------------------------------");
  }
}
