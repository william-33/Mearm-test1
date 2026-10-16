#include <SoftwareSerial.h>   // 软件串口库：把普通数字脚当成串口用

/* ==================== 配置区 ==================== */

// —— 4 个按键的引脚（占位值，以电路组实际接线为准） ——
#define KEY1_PIN  2    // 按键1：任务3=循环夹取A/B/C   任务5=暂停
#define KEY2_PIN  4    // 按键2：任务3=录制开始/结束    任务5=继续
#define KEY3_PIN  7    // 按键3：任务3=播放             任务5=取消
#define KEY4_PIN  8    // 按键4：任务3=回中             任务5=示教记录                  

//用电脑发 1/2/3/4，都正常了再把两块板接起来联调。
#define KEY_ACTIVE  LOW

#define LINK_BAUD  9600    

#define DEBOUNCE_MS  20

#define MODE_TASK3  3
#define MODE_TASK5  5
int taskMode = MODE_TASK3; // 上电默认进任务3

// 注意 RX/TX 要和对方"交叉"连接：我的 TX(11) 接对方的 RX(10)
SoftwareSerial link(10, 11); //修改记得


/* ==================== 工具函数 ==================== */
//按键扫描（非阻塞，按下沿触发一次）

int scanKey()
{
  const int pins[4] = {KEY1_PIN, KEY2_PIN, KEY3_PIN, KEY4_PIN};
  static unsigned long lastMs = 0;
  static bool wasDown = false;

  bool anyDown = false;
  for (int i = 0; i < 4; i++) {
    if (digitalRead(pins[i]) == KEY_ACTIVE) anyDown = true;
  }

  if (millis() - lastMs < 30) return 0;      // 时间门：消抖 + 防连发
  if (!anyDown) { wasDown = false; return 0; }
  if (wasDown) return 0;                     // 还按着不放，不再触发
  wasDown = true; lastMs = millis();

  for (int i = 0; i < 4; i++) {
    if (digitalRead(pins[i]) == KEY_ACTIVE) return i + 1;
  }
  return 0;
}

//打印当前模式
void printMode()
{
  if (taskMode == MODE_TASK3) {
    Serial.println(">>> 当前模式: 任务3   (1=夹取A/B/C  2=录制  3=播放  4=回中)");
  } else {
    Serial.println(">>> 当前模式: 任务5   (1=暂停  2=继续  3=取消  4=示教记录)");
  }
}

//串口切模式：监视器里敲 3 或 5
void updateMode()
{
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '3') { taskMode = MODE_TASK3; printMode(); }
    else if (c == '5') { taskMode = MODE_TASK5; printMode(); }
    // 其它字符（含换行/回车）忽略
  }
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
  Serial.println("Type 3 or 5 in the monitor to switch mode.");
  printMode();
  Serial.println("===============================");
}

/* ==================== 主循环 ==================== */
void loop() {
  updateMode();                 // ① 先看串口有没有切模式

  int k = scanKey();            // ② 一次扫描拿到按下的键号(1~4)
  if (k == 0) return;

  if (taskMode == MODE_TASK3) {
    /* ---------- 任务3 模式：1/2/3/4 ---------- */
    if      (k == 1) { link.write('1'); Serial.println("sent: 1 (pick A/B/C)"); }
    else if (k == 2) { link.write('2'); Serial.println("sent: 2 (record)");    }
    else if (k == 3) { link.write('3'); Serial.println("sent: 3 (play)");      }
    else if (k == 4) { link.write('4'); Serial.println("sent: 4 (home)");      }
  } else {
    /* ---------- 任务5 模式：P 暂停 / U 继续 / X 取消 / R 示教记录 ---------- */
    if      (k == 1) { link.write('P'); Serial.println("sent: P (pause)");  }
    else if (k == 2) { link.write('U'); Serial.println("sent: U (resume)"); }
    else if (k == 3) { link.write('X'); Serial.println("sent: X (cancel)"); }
    else             { link.write('R'); Serial.println("sent: R (teach record)"); }
  }
}

