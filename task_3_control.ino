#include <SoftwareSerial.h>   // 软件串口库：把普通数字脚当成串口用

/* ==================== 配置区 ==================== */

// —— 4 个按键的引脚（占位值，以电路组实际接线为准） ——
#define KEY1_PIN  2    // 按键1：循环夹取 A/B/C
#define KEY2_PIN  4    // 按键2：录制 开始/结束
#define KEY3_PIN  7    // 按键3：播放
#define KEY4_PIN  8    // 按键4：回中                  
//联调顺序建议：先单独测本板（看 USB 回显），再单独测机械臂板.
//用电脑发 1/2/3/4，都正常了再把两块板接起来联调。
#define KEY_ACTIVE  LOW

#define LINK_BAUD  9600    

#define DEBOUNCE_MS  20

// —— 板间串口（SoftwareSerial）：RX=10, TX=11 ——以电路组为准
// 注意 RX/TX 要和对方"交叉"连接：我的 TX(11) 接对方的 RX(10)
SoftwareSerial link(10, 11); //修改记得
/* ==================== 工具函数 ==================== */
//按键读取（含消抖、防连发）
bool keyPressed(int pin)
{
  if(digitalRead(pin)==KEY_ACTIVE)//确认一次
  {
    delay(DEBOUNCE_MS);
    if (digitalRead(pin) == KEY_ACTIVE)//zai确认一次
    {
    while(digitalRead(pin)==KEY_ACTIVE){delay(5);}//按住期间一直等
    return true;
    }
  }
  return false;
}

/* ==================== 初始化 ==================== */
void setup() {
  Serial.begin(9600); 

  pinMode(KEY1_PIN, INPUT_PULLUP);
  pinMode(KEY2_PIN, INPUT_PULLUP);
  pinMode(KEY3_PIN, INPUT_PULLUP);
  pinMode(KEY4_PIN, INPUT_PULLUP);

  link.begin(LINK_BAUD);     // 打开板间串口（给机械臂发指令用）
    
  Serial.println("===============================");
  Serial.println(" Remote board ready.");
  Serial.println(" Keys: 1=pick:A,B,C  2=record  3=play  4=home");
  Serial.println("===============================");
}

/* ==================== 主循环 ==================== */
void loop() {
 if (keyPressed(KEY1_PIN)) {
    link.write('1');                          // 发给机械臂
    Serial.println("sent: 1 (pick A/B/C)");   // 本地回显
  }
  if (keyPressed(KEY2_PIN)) {
    link.write('2');
    Serial.println("sent: 2 (record)");
  }
  if (keyPressed(KEY3_PIN)) {
    link.write('3');
    Serial.println("sent: 3 (play)");
  }
  if (keyPressed(KEY4_PIN)) {
    link.write('4');
    Serial.println("sent: 4 (home)");
  }
}

