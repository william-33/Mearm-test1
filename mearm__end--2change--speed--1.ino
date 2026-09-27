#include <Servo.h>
Servo base, fArm, rArm, claw;
int basePos=90;      //初始化
int fArmPos=90;
int rArmPos=90;
int clawPos=90;
int DSD=15;

const int maxbase;                           // 记得和机械的同学沟通一下
const int maxfArm;
const int maxrArm;
const int maxclaw;

const int minbase;
const int minfArm;
const int minrArm;
const int minclaw;

//初始化
void setup() {
  Serial.begin(9600);
  base.attach();              //初始化引脚
  delay(100);
  rArm.attach();
  delay(100);
  fArm.attach();
  delay(100);
  claw.attach();
  delay(100);



//初始化最初的样子
  base.write(90); 
  delay(10);
  fArm.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  claw.write(90);
  delay(10);

  Serial.println("please tell me your choice  ");
  Serial.println("1.improve the DSD please input 'H' ");
  Serial.println("1. down   the DSD please input 'L' ");
}
}


void loop() {
  if(Serial.available()>0)
  {
    char serialCmd = Serial.read();//读取第一位看是哪一个部分
    delay(15);
    armdatecmd(serialCmd);//开始判断输入是否正确
  }
}


//判断输入是否正确
void armdatecmd(char serialCmd)
{
  if(serialCmd='b'||serialCmd='r'||serialCmd='f'||serialCmd='c')
  {
    int servodate=Serial.parseInt();
    servocmd(serialCmd,servodate,DSD);//正确的话才执行下一步的改变变化
  }
  else 
  {
    switch(serialCmd)//查看当前的状态
    {
      case 'o'://展现状态信息
      {
        reportcondition();
        break;
      }
      default :
      {
        Serial.println("Unknown Command");
      }
    }
  }
}


//展现窗口现在的机械信息
void reportcondition()
{
  Serial.println(""); 
  Serial.println("####now the conditon:####");
  Serial.print("   base  ==");Serial.println(base.read()); 
  Serial.print(" rear Arm==");Serial.println(rArm.read());
  Serial.print("front Arm==");Serial.println(fArm.read()); 
  Serial.print("  claw   ==");Serial.println(claw.read()); 
}


//主要的显示变化的过程的程序
void servocmd(char serialCmd ,int servodate,int DSD)
{
  //展现操作的对象以及对应的速度检查
  Servo servotmp;
  int fromPos;
  Serial.println("");
  Serial.println("####Servo Command:####");
  Serial.print("   name    :"); Serial.println(serialCmd);
  Serial.print("    to     :"); Serial.println(servodate);
  Serial.print("with speed :"); Serial.println(DSD);


//对的名称对象进行相应的动作
  switch(serialCmd)
  {
  case 'b'://是base的话
  {
    if(servodate>=minbase&&servodate<=maxbase)//判断角度的合理
    {
      servotmp=base;
      fromPos=base.read();
      break;//切换成功tmp，进入下一步
    }
    else
    {
      Serial.println("###Warning: Servo Value out of the Limit!!###");
      return;//回去主函数，结束。
    }
  }
  case 'r':
  {
    if(servodate>=minrArm&&servodate<=maxrArm)//判断角度的合理
    {
      servotmp=rArm;
      fromPos=rArm.read();
      break;//切换成功tmp，进入下一步
    }
    else
    {
      Serial.println("###Warning: Servo Value out of the Limit!!###");
      return;//回去主函数，结束。
    }
  }
  case 'f':
  {
    if(servodate>=minfArm&&servodate<=maxfArm)//判断角度的合理
    {
      servotmp=fArm;
      fromPos=fArm.read();
      break;//切换成功tmp，进入下一步
    }
    else
    {
      Serial.println("###Warning: Servo Value out of the Limit!!###");
      return;//回去主函数，结束。
    }
  }
  case 'c':
  {
    if(servodate>=minclaw&&servodate<=maxclaw)//判断角度的合理
    {
      servotmp=claw;
      fromPos=claw.read();
      break;//切换成功tmp，进入下一步
    }
    else
    {
      Serial.println("###Warning: Servo Value out of the Limit!!###");
      return;//回去主函数，结束。
    }
  }
 }


//在确定了目标正确，改变的目标角度也正确，进行下一步的操作。
//先判断是角度变小还是变大再进行改变。
if(fromPos<=servodate)
{
  for(int i=fromPOs;i<=servodate;i++)
    {
      servotmp.write(i);
      delay(DSD);
    }
}
else
{
  for(int i=fromPOs;i>=servodate;i--)
    {
      servotmp.write(i);
      delay(DSD);
    }
}







