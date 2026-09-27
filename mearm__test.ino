#include <Servo.h>
Servo base, fArm, rArm, claw;
int basePos=90;      //初始化
int fArmPos=90;
int rArmPos=90;
int clawPos=90;

const int maxbase;                                                // 记得和机械的同学沟通一下
const int maxfArm;
const int maxrArm;
const int maxclaw;

const int minbase;
const int minfArm;
const int minrArm;
const int minclaw;

void setup() {
  Serial.begin(9600);
  base.attach(2);                                                  //记得要改引脚量
  delay(200);
  fArm.attach(4);
  delay(200);
  rArm.attach(6);
  delay(200);
  claw.attach(8);
  delay(200);
  Serial.println("please tell me your choice");
}

void loop() {
  if(Serial.available>0)//确定有输入
  {
    char serialCmd = Serial.read();//读取第一位，看是哪一部分
    delay(10);
    armdatecmd(serialCmd);
  }


  base.write(basePos); //最后改变
  delay(10);
  fArm.write(fArmPos);
  delay(10);
  rArm.write(rArmPos);
  delay(10);
  claw.write(clawPos);
  delay(10);
}


void armdatecmd(char serialCmd)
{
 Serial.print("serialCmd: ");//在串口监视器处看到具体参数
 Serial.println(serialCmd);

 int servodate=Serial.parseInt();
 switch(serialCmd) //判断输入的是什么
 {
  case 'b':
  {
    basePos=servodate;
    Serial.print(" Set base servo value=");
    Serial.println(servodate);
    break;
  }
  case 'r':
  {
    rArmPos=servodate;
    Serial.print(" Set rArm servo value=");
    Serial.println(servodate);
    break;
  }
  case 'f':
  {
    fArmPos=servodate;
    Serial.print(" Set fArm servo value=");
    Serial.println(servodate);
    break;
  }
  case 'c':
  {
    clawPos=servodate;
    Serial.print(" Set claw servo value=");
    Serial.println(servodate);
    break;
  }
  default :
  Serial.println("Unknown Command.");
 }
}

















