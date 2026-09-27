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
  base.attach();
  delay(100);
  rArm.attach();
  delay(100);
  fArm.attach();
  delay(100);
  claw.attach();
  delay(100);


   base.write(90); //初始化
  delay(10);
  fArm.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  claw.write(90);
  delay(10);

  Serial.println("please tell me your choice  ");
}

void loop() {
  if(Serial.available()>0)
  {
    char serialCmd = Serial.read();
    delay(15);
    armdatecmd(serialCmd);
  }
}


void armdatecmd(char serialCmd)
{
 Serial.print("serialCmd: ");//在串口监视器处看到具体参数
 Serial.println(serialCmd);

int fromPos;//确定初始的位置
int toPos;//确定目标的位置


 int servodate=Serial.parseInt();
 switch(serialCmd) //判断输入的是什么
 {
  case 'b':
  {
    fromPos=base.read();
    toPos=servodate;
    if(fromPos<=toPos)
    {for(int i=fromPOs;i<=toPos;i++)
    {
      base.write(i);
      delay(15);
    }
    Serial.print(" Set base servo value=");
    Serial.println(servodate);
    break;
    }
    else
    {for(int i=fromPOs;i>=toPos;i--)
    {
      base.write(i);
      delay(15);
    }
    Serial.print(" Set base servo value=");
    Serial.println(servodate);
    break;

    }
  }
  case 'r':
  {
    fromPos=rArm.read();
    toPos=servodate;
    rArmPos=servodate;
    Serial.print(" Set rArm servo value=");
    Serial.println(servodate);
    break;
  }
  case 'f':
  {
    fromPos=fArm.read();
    toPos=servodate;
    fArmPos=servodate;
    Serial.print(" Set fArm servo value=");
    Serial.println(servodate);
    break;
  }
  case 'c':
  {
    fromPos=claw.read();
    toPos=servodate;
    clawPos=servodate;
    Serial.print(" Set claw servo value=");
    Serial.println(servodate);
    break;
  }
  case 'o':
  {
    Serial.println("");
    Serial.println("");
    Serial.println("#####now the Mearm condition######");
    Serial.print("  the base");Serial.println(base.read());
    Serial.print("  the rArm");Serial.println(rArm.read());
    Serial.print("  the fArm");Serial.println(fArm.read());
    Serial.print("  the claw");Serial.println(claw.read());
  }
  default :
  Serial.println("Unknown Command.");
 }
}