#include <Servo.h>
#include <math.h>
/* ==================== 配置区 (按实际硬件修改) ==================== */
#define LINE_STEP  1
#define LINE_DELAY  40 
#define CURVE_SAMPLES 30

#define GRIPPER_PIN   6    // 爪子舵机
#define Base_PIN     9    // 关节1 (底座旋转)                              ###记得改引脚###
#define rArm_PIN     8    // 关节2 (大臂)
#define fArm_PIN     7    // 关节3 (小臂)
//待定的
#define PIN_REC     2     // 记录键（示教存点 / 存满后开始绘制）
#define PIN_PAUSE   4     // 暂停键
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

#define JOY_CENTER  513  // 摇杆中位理论值
#define JOY_DEAD    50   // 死区：偏移小于它算"没动"

#define Z_PEN_DOWN   //待定
#define Z_TRAVEL     //待定
#define Z_TEACH      //待定

//控制机械臂的平滑移动
#define STEP_ANGLE   2     // 每步最多转 2 度
#define STEP_DELAY   15

#define X_MIN 90.0
#define X_MAX 180.0
#define Y_MIN -80.0
#define Y_MAX 80.0

#define TEACH_FRAME  30

/* ==================== 全局变量 ==================== */
bool penheld= false;

struct Point{
  float x;
  float y;
  float z;
};

const float L_x[2] = {100, 160};
const float L_y[2] = {0, 0};                   // ####记得填写修改####
const float V_x[3] = {100, 130, 160};
const float V_y[3] = {-30, 30, -30};

const float PEN_X = 150.0, PEN_Y = 0.0;   // 取笔点的平面坐标（占位值，实测后改）
const float PEN_Z = 40.0;                 // 取笔时爪子下降到的高度（爪口对准笔身）
const float PEN_HOVER = ;                 // 取笔前的悬停高度

Point pensite ={PEN_X, PEN_Y, PEN_Z};
Point HOME = {150, 0, 100};   // 待机位                 ####记得修改####

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

// 示教点（最多 5 个）
float teachX[5];
float teachY[5];
int teachCount = 0;

/* ==================== 工具函数 ==================== */
// 摇杆读数 -> 这一帧舵机角要转多少度，死区内返回 0
float joyDeltaAngle(int pin) {
  int off = analogRead(pin) - JOY_CENTER;
  if (abs(off) < JOY_DEAD) return 0;
  return off /256.0 ;   
}
//读取按键
char ReadKeyEdge()
{
  static unsigned long LastMs= 0;
  if(millis()-LastMs<30) return 0;
  if(digitalRead(PIN_PAUSE)==LOW) {LastMs=millis(); return 'P';}
  if(digitalRead(PIN_RESUME)==LOW){LastMs=millis(); return 'U';}
  if(digitalRead(PIN_CANCEL)==LOW){LastMs=millis(); return 'C';}
  if(digitalRead(PIN_REC)==LOW){LastMs=millis(); return 'R';}
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
  penheld=true;
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

//正解
void forwardKinematics(float sBase,float sRarm,float sFarm,float &x,float &y,float &z)
{
  float s1=constrain(sBase,0,180);
  float s2=constrain(sRarm,20,115);
  float s3=constrain(sFarm,50,130);

  float t1 = (s1 - BASE_OFF) / BASE_DIR;
  float t2 = (s2 - rArm_OFF) / rArm_DIR;
  float t3 = (s3 - fArm_OFF) / fArm_DIR;

  float r1 = t1 * PI / 180.0;
  float r2 = t2 * PI / 180.0;
  float r3 = t3 * PI / 180.0;


  float pz = L1 * sin(r2) + L2 * sin(r2 + r3);
  //等待确认

  x = pr * cos(r1);
  y = pr * sin(r1);
  z = pz + H0;
}

//示教过程
bool teachMode() {
  teachCount = 0;
  Serial.println("=== 示教开始: 摇杆控舵机, 记录键存点(共5个) ===");
  Serial.println("    摇杆1: 左右=底座 / 前后=大臂   摇杆2: 左右=小臂 (爪子已锁定)");

  // 从待机位上方出发
  moveToPoint(HOME.x, HOME.y, Z_TEACH);

  // —— 阶段1：采集 5 个示教点 ——
  while (teachCount < 5) {
    // 1. 摇杆 -> 三个舵机角（爪子不动，保持夹笔）
    curBase = constrain(curBase + joyDeltaAngle(JOY_1X_PIN), 0, 180);
    currarm = constrain(currarm + joyDeltaAngle(JOY_1Y_PIN), 20, 115);
    curfarm = constrain(curfarm + joyDeltaAngle(JOY_2X_PIN), 50, 130);
    Base.write(curBase);
    rArm.write(currarm);
    fArm.write(curfarm);

    // 2. 按键：记录存点 / 取消退出
    char k = ReadKeyEdge();
    if (k == 'R') {
      // 用 FK 把当前三个舵机角反算成笔尖的纸面坐标
      float tx, ty, tz;
      forwardKinematics(curBase, currarm, curfarm, tx, ty, tz);

                                                                           //等待确定
      if (tx < X_MIN || tx > X_MAX || ty < Y_MIN || ty > Y_MAX) {
        Serial.println("  x 超出安全矩形, 本次不记录, 请把笔尖挪回来");
      } else if (tz > Z_TEACH + 20 || tz < Z_PEN_DOWN - 10) {
        Serial.print("  x 笔尖高度不合适 (z=");
        Serial.print(tz, 1);
        Serial.println("), 请调到贴近纸面再存");
      } else {
        teachX[teachCount] = tx;
        teachY[teachCount] = ty;
        teachCount++;
        Serial.print("  示教点 "); Serial.print(teachCount);
        Serial.print(": 坐标("); Serial.print(tx, 1);
        Serial.print(", "); Serial.print(ty, 1);
        Serial.print(") z="); Serial.print(tz, 1);
        Serial.print("  舵机("); Serial.print(curBase); Serial.print(", ");
        Serial.print(currarm); Serial.print(", "); Serial.print(curfarm);
        Serial.println(")");
        delay(300);                        // 留出松手时间并把姿态稳住（连记已由边沿检测挡住）
      }
    }
    if (k == 'C') {
      teachCount = 0;
      return false;
    }
    delay(TEACH_FRAME);

    // —— 阶段2：等待开始指令 ——
  Serial.println("5 点已存满! 再按记录键开始绘制 / 按取消键放弃");
  while (true) {
    char k = ReadKeyEdge();
    if (k == 'R') return true;           // 开始绘制
    if (k == 'C') {
      teachCount = 0;
      return false;                      // 放弃
     }
    delay(30);
    } 
}

//示教过程的复现
bool drawPolyline() {
  Serial.println("[任务]四 五点折线绘制");
  moveToPoint(HOME.x, HOME.y, HOME.z);              // 先回待机位
  penUp();
  if (!moveToPoint(teachX[0], teachY[0], Z_TRAVEL)) return false;
  penDown(teachX[0], teachY[0]);                    // 在第1点落笔

  for (int i = 1; i < 5; i++) {                     // 4 段折线
    Serial.print("  段 "); Serial.print(i); Serial.println("/4");
    if (!drawLine(teachX[i - 1], teachY[i - 1],
                  teachX[i], teachY[i], Z_PEN_DOWN)) {
      return false;
    }
  }
  penUp();
  moveToPoint(HOME.x, HOME.y, HOME.z);
  Serial.println("[完成] 折线绘制结束, 已回待机位");
  return true;
}

//通过"投票"（加权混合）决定，所以拐弯处会圆滑过渡而不是硬折
float catmull(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5f * (2 * p1
                 + (-p0 + p2) * t
                 + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
}

bool drawCurve(){
  Serial.println("[任务]五 五点平滑曲线绘制");
  
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
  pinMode(PIN_REC, INPUT_PULLUP);

  Base.write(90);
  delay(10);
  rArm.write(20);
  delay(10);
  fArm.write(90);
  delay(10);
  Gripper.write(GRIP_OPEN_ANGLE);

  Serial.println("==========================================");
  Serial.println(" Robot arm drawing ready. (v1: task 1-5)");
  Serial.println("=====first:please input O(to get pen)====");
  Serial.println(" Commands: L=line  V=letter  S=triangle");
  Serial.println("         T=teach+polyline  C=teach+curve");
  Serial.println(" Keys: REC(2) PAUSE(4) RESUME(12) CANCEL(13)");
  Serial.println("==========================================");
  delay(10);

 
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
      case 'O':
      case 'o':
      if(!penheld) getpen();
      break;
      case 'L':
      case 'l':
        ok=drawShape(L_x, L_y, 2, "直线");
        break;
      case 'V':
      case 'v':
        ok = drawShape(V_x, V_y, 3, "字母V");
        break;
      case 'T':   // 子任务4
        if (teachMode()) {
        ok = drawPolyline();
        } else {
          ok = false;
        }
        break;
      case 'C':   // 子任务5：五点示教 + 平滑曲线
        if (teachMode()) {
        ok = drawCurve();
        } else {
          ok = false;
        }
        break;
      default:
        Serial.println("未知指令 (L/V)");
        break;
    }

if(!ok){cancelToHome();}

 Serial.println("------------------------------");
  }
}
