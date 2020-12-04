/*
Simple script to move my tiny 6dof robotic arm
简单的脚本，移动我的小6自由度机械臂
*/
#include <math.h>

#define PI 3.1415926535897932384626433832795

//driver for the axis 1 //驱动轴1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1

//enable pin for the axis 3, 2 and 1  //使能引脚
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

double curPos1 = 0.0;
double curPos2 = 0.0;
double curPos3 = 0.0;
double curPos4 = 0.0;
double curPos5 = 0.0;
double curPos6 = 0.0;

boolean PULstat1 = 0;
boolean PULstat2 = 0;
boolean PULstat3 = 0;
boolean PULstat4 = 0;
boolean PULstat5 = 0;
boolean PULstat6 = 0;

//robot geometry  //机器人结构参数
const double dl1 = 360.0/200.0/32.0/4.8;  //轴1齿轮减速比 = 96/20 = 4.8
const double dl2 = 360.0/200.0/32.0/4.0;  //轴2齿轮减速比 = 4
const double dl3 = 360.0/200.0/32.0/5.0;  //轴3齿轮减速比 = 5
const double dl4 = 360.0/200.0/32.0/2.8;  //轴4齿轮减速比 = 56/20 = 2.8
const double dl5 = 360.0/200.0/32.0/2.1;  //轴5齿轮减速比 = 42/20 = 2.1
const double dl6 = 360.0/200.0/32.0/1.0;  //轴6齿轮减速比 = 1
const double r1 = 47.0;
const double r2 = 110.0;
const double r3 = 26.0; 
const double d1 = 133.0;
const double d3 = 0.0;
const double d4 = 117.50;
const double d6 = 28.0;

void setup()
{
  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);
  
  digitalWrite(PUL1_PIN, LOW); // gear ratio = 96/20 = 4.8  //轴1齿轮减速比 = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW); // LOW = negative direction  //低电平为反向
  
  digitalWrite(PUL2_PIN, LOW); // gear ratio = 4            //轴2齿轮减速比 = 4
  digitalWrite(DIR2_PIN, LOW); // LOW = positive direction  //低电平为正向
  
  digitalWrite(PUL3_PIN, LOW); // gear ratio = 5            //轴3齿轮减速比 = 5
  digitalWrite(DIR3_PIN, LOW); // LOW = negative direction  //低电平为反向
  
  digitalWrite(PUL4_PIN, LOW); // gear ratio = 56/20 = 2.8  //轴4齿轮减速比 = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW); // LOW = positive direction  //低电平为正向
  
  digitalWrite(PUL5_PIN, LOW); // gear ratio = 42/20 = 2.1  //轴5齿轮减速比 = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW); // LOW = positive direction  //低电平为正向
  
  digitalWrite(PUL6_PIN, LOW); // gear ratio = 1            //轴6齿轮减速比 = 1
  digitalWrite(DIR6_PIN, LOW); // LOW = positive direction  //低电平为正向

  // all joints disabled!      //所有关节关闭使能
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH); 

  //Serial.begin(9600);
}

void loop()
{
  // enable all joints        //开启所有关节使能  //以下阶段为机械臂初始化，所有关节从折叠状态回到原点位置
  delay(10000);
  digitalWrite(EN321_PIN, LOW);
  digitalWrite(EN4_PIN, LOW);
  digitalWrite(EN5_PIN, LOW);
  digitalWrite(EN6_PIN, LOW);
  delay(1000);
  // go to the home position (all joints equal to 0)  //回到原点位置（所有关节归零）
  // joint #2   //关节 #2
  digitalWrite(DIR2_PIN, HIGH);
  int delValue = 4000;    //速度（脉冲周期us）
  int incValue = 7;       //增量值（加速度/减速度阶段脉冲周期增量值）
  int accRate = 530;      //加速度（加速度/减速度阶段步数）
  int totSteps = 2791*2;  //总步数
  for (int i = 0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL2_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL2_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #3     //关节 #3
  digitalWrite(DIR3_PIN, HIGH);
  delValue=4000;  //速度（脉冲周期us）
  incValue=7;     //增量值（加速度/减速度阶段脉冲周期增量值）
  accRate=530;    //加速度（加速度/减速度阶段步数）
  totSteps=6569;  //总步数
  for (int i=0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #5       //关节 #5
  digitalWrite(DIR5_PIN, HIGH);
  delValue=4000;    //速度（脉冲周期us）
  incValue=7;       //增量值（加速度/减速度阶段脉冲周期增量值）
  accRate=530;      //加速度（加速度/减速度阶段步数）
  totSteps=90/dl5;  //总步数
  for (int i=0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration    //加速阶段
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration   //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL5_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL5_PIN, LOW);
    delayMicroseconds(delValue);
  }
  /******* 以上阶段为机械臂初始化，所有关节从折叠状态回到原点位置 *******/
  /*--------------------------------------GoGoGo-----------------------------------------*/
  /*------------------------------------开始自动运行--------------------------------------*/
  curPos1=0.0;
  curPos2=0.0;
  curPos3=0.0;
  curPos4=0.0;
  curPos5=90.0;
  curPos6=0.0;
  float Xhome[6]={164.5, 0.0, 241.0, 90.0, 180.0, -90.0}; //{x, y, z, ZYZ Euler angles}
  
  float X1[6]={164.5, 0.0, 141.0, 90.0, 180.0, -90.0};
  float X11[6]={164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0};
  float X12[6]={164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0};
  float X13[6]={164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0};
  float X14[6]={164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0};
  float X15[6]={164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0};
  float X16[6]={164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0};
  float X17[6]={164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0};
  float X18[6]={164.5+50.0, 0.0, 141.0, 90.0, 180.0, -90.0};
  
  float X2[6]={264.5, 0.0, 141.0, 0.0, 90.0, 0.0};
  float X3[6]={164.5, 100.0, 141.0, 90.0, 90.0, 0.0};
  float X4[6]={164.5, -100.0, 141.0, 90.0, -90.0, 0.0};
  
  float Jhome[6], J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
  InverseK(Xhome, Jhome);
  InverseK(X1, J1);
  InverseK(X11, J11);
  InverseK(X12, J12);
  InverseK(X13, J13);
  InverseK(X14, J14);
  InverseK(X15, J15);
  InverseK(X16, J16);
  InverseK(X17, J17);
  InverseK(X18, J18);
  InverseK(X2, J2);
  InverseK(X3, J3);
  InverseK(X4, J4);

  goStrightLine(Jhome, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  float velG=0.25e-4;
  goStrightLine(J1, J11, 0.25e-4, 0.75e-10, 0.0, 0.5*velG);
  goStrightLine(J11, J12, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J12, J13, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J13, J14, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J14, J15, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J15, J16, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J16, J17, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J17, J1, 0.25e-4, 0.75e-10, 0.5*velG, 0.0);

  goStrightLine(J1, J18, 0.25e-4, 0.75e-10, 0.0, 0.8*velG);
  goStrightLine(J18, J14, 0.25e-4, 0.75e-10, 0.8*velG, 0.0);
  goStrightLine(J14, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);
  
  goStrightLine(J1, J2, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J2, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  goStrightLine(J1, J3, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J3, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  goStrightLine(J1, J4, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J4, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);
  
  goStrightLine(J1, Jhome, 0.25e-4, 0.75e-10, 0.0, 0.0);

  /*-----------------------------------GoGoGoBack----------------------------*/
  // come back from home position to fold position  //从家的位置回到折叠位置
  // joint #5       //关节 #5
  digitalWrite(DIR5_PIN, LOW);
  delValue=4000;    //速度（脉冲周期us）
  incValue=7;       //增量值（加速度/减速度阶段脉冲周期增量值）
  accRate=530;      //加速度（加速度/减速度阶段步数）
  totSteps=90/dl5;  //总步数
  for (int i=0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL5_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL5_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #3 //关节 #3
  digitalWrite(DIR3_PIN, LOW);
  delValue=4000;  //速度（脉冲周期us）
  incValue=7;     //增量值（加速度/减速度阶段脉冲周期增量值）
  accRate=530;    //加速度（加速度/减速度阶段步数）
  totSteps=6569;  //总步数
  for (int i=0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #2 //关节 #2
  digitalWrite(DIR2_PIN, LOW);
  delValue=4000;  //速度（脉冲周期us）
  incValue=7;     //增量值（加速度/减速度阶段脉冲周期增量值）
  accRate=530;    //加速度（加速度/减速度阶段步数）
  totSteps=2791*2;//总步数
  for (int i=0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration  //加速阶段s
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration  //没有适当的加速度/减速度空间
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration  //加速阶段
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration //减速阶段
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL2_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL2_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // all joints disabled! //所有关节使能关闭
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH); 
  // wait 15 minutes      //等待15分钟
  delay(900000);
}

/* 走直线函数 */
void goStrightLine(float* xfi, float* xff, float vel0, float acc0, float velini, float velfin){
  float lmax = max(abs(xff[0]-xfi[0]),abs(xff[1]-xfi[1]));
  lmax = max(lmax,abs(xff[2]-xfi[2]));
  lmax = max(lmax,abs(xff[3]-xfi[3]));
  lmax = max(lmax,abs(xff[4]-xfi[4]));
  lmax = max(lmax,abs(xff[5]-xfi[5]));
  unsigned long preMil = micros();
  double l = 0.0;
  vel0 = min(vel0,sqrt(lmax*acc0+0.5*velini*velini+0.5*velfin*velfin));
  unsigned long curMil = micros();
  unsigned long t = 0;
  double tap = vel0/acc0-velini/acc0;
  double lap = velini*tap+acc0*tap*tap/2.0;
  double lcsp = lmax-(vel0*vel0/2.0/acc0-velfin*velfin/2.0/acc0);
  double tcsp = (lcsp-lap)/vel0+tap;
  double tfin = vel0/acc0-velfin/acc0+tcsp;
  while (curMil-preMil<=tfin){
    t = curMil-preMil;
    //acceleration phase  //加速阶段
    if (t<=tap) {
      l = velini*t+acc0*t*t/2.0;
    }
    //contant maximum speed phase //确定最大速度
    if (t>tap && t<=tcsp) {
      l = lap+vel0*(t-tap);
    }
    //deceleration phase  //减速阶段
    if (t>tcsp) {
      l = lcsp+vel0*(t-tcsp)-acc0*(t-tcsp)*(t-tcsp)/2.0;
    }
  
    //trajectory x and y as a function of l //轨迹x和y是l的函数
    float Xx[6];
    Xx[0]=xfi[0]+(xff[0]-xfi[0])/lmax*l;
    Xx[1]=xfi[1]+(xff[1]-xfi[1])/lmax*l;
    Xx[2]=xfi[2]+(xff[2]-xfi[2])/lmax*l;
    Xx[3]=xfi[3]+(xff[3]-xfi[3])/lmax*l;
    Xx[4]=xfi[4]+(xff[4]-xfi[4])/lmax*l;
    Xx[5]=xfi[5]+(xff[5]-xfi[5])/lmax*l;
    
    goTrajectory(Xx);
    curMil = micros();
  }
}

/* 走的轨迹函数 */
void goTrajectory(float* Jf){
  //execution //执行
  int delF=2;
  // joint #1 //关节 #1
  if (Jf[0]-curPos1>0.0) { // positive direction of rotation  //正旋转方向
    digitalWrite(DIR1_PIN, HIGH);
    while (Jf[0]-curPos1>dl1/2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 + dl1/2.0;
      if (Jf[0]-curPos1>dl1/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR1_PIN, LOW);
    while (-Jf[0]+curPos1>dl1/2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 - dl1/2.0;
      if (-Jf[0]+curPos1>dl1/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #2 //关节 #2
  if (Jf[1]-curPos2>0.0) { // positive direction of rotation  //正旋转方向
    digitalWrite(DIR2_PIN, HIGH);
    while (Jf[1]-curPos2>dl2/2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 + dl2/2.0;
      if (Jf[1]-curPos2>dl2/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR2_PIN, LOW);
    while (-Jf[1]+curPos2>dl2/2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 - dl2/2.0;
      if (-Jf[1]+curPos2>dl2/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #3 //关节 #3
  if (Jf[2]-curPos3>0.0) { // positive direction of rotation  //正旋转方向
    digitalWrite(DIR3_PIN, LOW);
    while (Jf[2]-curPos3>dl3/2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 + dl3/2.0;
      if (Jf[2]-curPos3>dl3/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR3_PIN, HIGH);
    while (-Jf[2]+curPos3>dl3/2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 - dl3/2.0;
      if (-Jf[2]+curPos3>dl3/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #4 //关节 #4
  if (Jf[3]-curPos4>0.0) { // positive direction of rotation  //正旋转方向
    digitalWrite(DIR4_PIN, HIGH);
    while (Jf[3]-curPos4>dl4/2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 + dl4/2.0;
      if (Jf[3]-curPos4>dl4/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR4_PIN, LOW);
    while (-Jf[3]+curPos4>dl4/2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 - dl4/2.0;
      if (-Jf[3]+curPos4>dl4/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #5 //关节 #5
  if (Jf[4]-curPos5>0.0) { //positive direction of rotation  //正旋转方向
    digitalWrite(DIR5_PIN, HIGH);
    while (Jf[4]-curPos5>dl5/2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 + dl5/2.0;
      if (Jf[4]-curPos5>dl5/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR5_PIN, LOW);
    while (-Jf[4]+curPos5>dl5/2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 - dl5/2.0;
      if (-Jf[4]+curPos5>dl5/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #6 //关节 #6
  if (Jf[5]-curPos6>0.0) { // positive direction of rotation  //正旋转方向
    digitalWrite(DIR6_PIN, HIGH);
    while (Jf[5]-curPos6>dl6/2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 + dl6/2.0;
      if (Jf[5]-curPos6>dl6/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR6_PIN, LOW);
    while (-Jf[5]+curPos6>dl6/2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 - dl6/2.0;
      if (-Jf[5]+curPos6>dl6/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
}

/* 
 * 函数名：inverse kinematics 逆向运动学（IK）函数
 * 备  注：已知机械臂末端在直角坐标系下的坐标XYZ欧拉角ABC，求解机械臂各关节的转动角度
 * 输  入: Xik - pos value for the calculation of the inverse kinematics      //输入:Xik 点的值用于逆运动学的计算
 * 输  出：Jfk - joints value for the calculation of the inversed kinematics  //输出:Jfk 关节的值，用于计算逆运动学
 */
void InverseK(float* Xik, float* Jik)
{ 
  // from deg to rad    //角度制转到弧度制
  // Xik(4:6)=Xik(4:6)*pi/180;
  Xik[3]=Xik[3]*PI/180.0;
  Xik[4]=Xik[4]*PI/180.0;
  Xik[5]=Xik[5]*PI/180.0;
  // Denavit-Hartenberg matrix  //DH坐标系矩阵
  float theta[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0};      // theta=[0; -90+0; 0; 0; 0; 0];    //θ
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0};  // alfa=[-90; 0; -90; 90; -90; 0];  //a
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0};               // r=[47; 110; 26; 0; 0; 0];        //r
  float d[6]={d1, 0.0, d3, d4, 0.0, d6};                // d=[133; 0; 7; 117.5; 0; 28];     //d
  // from deg to rad  //角度制转到弧度制
  MatrixScale(theta, 6, 1, PI/180.0);   // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0);    // alfa=alfa*pi/180;
  // work frame   //工作坐标系
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  // tool frame   //工具坐标系
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  // work frame transformation matrix //工作坐标系变换矩阵
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  // tool frame transformation matrix //工具坐标系变换矩阵
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  // total transformation matrix      //全部的变换矩阵
  float Twt[16];
  pos2tran(Xik, Twt); // Twt=pos2tran(Xik);
  // find T06 //找到 T06
  float inTwf[16], inTtf[16], Tw6[16], T06[16];
  invtran(Twf, inTwf); // inTwf=invtran(Twf);
  invtran(Ttf, inTtf); // inTtf=invtran(Ttf);
  MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6); // Tw6=Twt*inTtf;
  MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06); // T06=inTwf*Tw6;
  // positon of the spherical wrist //球形手腕的位置
  float Xsw[3];
  // Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
  Xsw[0]=T06[0*4 + 3]-d[5]*T06[0*4 + 2];
  Xsw[1]=T06[1*4 + 3]-d[5]*T06[1*4 + 2];
  Xsw[2]=T06[2*4 + 3]-d[5]*T06[2*4 + 2];
  // joints variable    //关节变量
  // Jik=zeros(6,1);  
  // first joint        //第一个关节
  Jik[0]=atan2(Xsw[1],Xsw[0])-atan2(d[2],sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])); // Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  // second joint       //第二个关节
  Jik[1] = PI/2.0
  -acos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
  -atan((Xsw[2]-d[0])/(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])); // Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  // third joint        //第三个关节
  Jik[2]=PI
  -acos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*sqrt(r[2]*r[2]+d[3]*d[3])))
  -atan(d[3]/r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  // last three joints  //最后三个关节
  float T01[16], T12[16], T23[16], T02[16], T03[16], inT03[16], T36[16];
  DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  MatrixMultiply(T01, T12, 4, 4, 4, T02); // T02=T01*T12;
  MatrixMultiply(T02, T23, 4, 4, 4, T03); // T03=T02*T23;
  invtran(T03, inT03); // inT03=invtran(T03);
  MatrixMultiply(inT03, T06, 4, 4, 4, T36); // T36=inT03*T06;
  // forth joint      //第四个关节
  Jik[3]=atan2(-T36[1*4+2], -T36[0*4+2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
  // fifth joint      //第五个关节
  Jik[4]=atan2(sqrt(T36[0*4+2]*T36[0*4+2]+T36[1*4+2]*T36[1*4+2]), T36[2*4+2]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  // sixth joints     //第六个关节
  Jik[5]=atan2(-T36[2*4+1], T36[2*4+0]); // Jik(6)=atan2(-T36(3,2),T36(3,1));
  // rad to deg       //角度制转弧度制
  MatrixScale(Jik, 6, 1, 180.0/PI); // Jik=Jik/pi*180;
}

/* 
 * 函数名：forward kinematics 正向运动学（FK）函数
 * 备  注：已知机械臂末端在直角坐标系下的坐标XYZ欧拉角ABC，求解机械臂各关节的转动角度
 * 输  入：Jfk - joints value for the calculation of the forward kinematics //Jfk - 输入关节值用于正运动学的计算
 * 输  出：Xfk - pos value for the calculation of the forward kinematics    //Xfk - 输出点坐标值用于正运动学的计算
 */
void ForwardK(float* Jfk, float* Xfk)
{
  // Denavit-Hartenberg matrix  //DH坐标系矩阵
  float theTemp[6] = {0.0, -90.0, 0.0, 0.0, 0.0, 0.0};  //临时变量
  float theta[6];                                       //θ
  MatrixAdd(theTemp, Jfk, 6, 1, theta); // theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];  //矩阵加法程序
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa=[-90; 0; -90; 90; -90; 0];               //a
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];                                  //r
  float d[6]={d1, 0.0, d3, d4, 0.0, d6}; // d=[133; 0; 7; 117.5; 0; 28];                                //d
  // from deg to rad  //角度制转到弧度制
  MatrixScale(theta, 6, 1, PI/180.0); // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0); // alfa=alfa*pi/180;
  // work frame   //工作坐标系
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  // tool frame   //工件坐标系
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  // work frame transformation matrix //工作坐标系变换矩阵
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  // tool frame transformation matrix //工件坐标系变换矩阵
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  // DH homogeneous transformation matrix //DH齐次变换矩阵
  float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
  DH1line(theta[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1),alfa(1),r(1),d(1));
  DH1line(theta[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2),alfa(2),r(2),d(2));
  DH1line(theta[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3),alfa(3),r(3),d(3));
  DH1line(theta[3], alfa[3], r[3], d[3], T34); // T34=DH1line(theta(4),alfa(4),r(4),d(4));
  DH1line(theta[4], alfa[4], r[4], d[4], T45); // T45=DH1line(theta(5),alfa(5),r(5),d(5));
  DH1line(theta[5], alfa[5], r[5], d[5], T56); // T56=DH1line(theta(6),alfa(6),r(6),d(6));
  float Tw1[16], Tw2[16], Tw3[16], Tw4[16], Tw5[16], Tw6[16], Twt[16];
  MatrixMultiply(Twf, T01, 4, 4, 4, Tw1);       // Tw1=Twf*T01;
  MatrixMultiply(Tw1, T12, 4, 4, 4, Tw2);       // Tw2=Tw1*T12;
  MatrixMultiply(Tw2, T23, 4, 4, 4, Tw3);       // Tw3=Tw2*T23;
  MatrixMultiply(Tw3, T34, 4, 4, 4, Tw4);       // Tw4=Tw3*T34;
  MatrixMultiply(Tw4, T45, 4, 4, 4, Tw5);       // Tw5=Tw4*T45;
  MatrixMultiply(Tw5, T56, 4, 4, 4, Tw6);       // Tw6=Tw5*T56;
  MatrixMultiply(Tw6, Ttf, 4, 4, 4, Twt);       // Twt=Tw6*Ttf;
  // calculate pos from transformation matrix   //从变换矩阵计算点坐标
  tran2pos(Twt, Xfk); // Xfk=tran2pos(Twt);
  // Xfk(4:6)=Xfk(4:6)/pi*180;
  Xfk[3]=Xfk[3]/PI*180.0;
  Xfk[4]=Xfk[4]/PI*180.0;
  Xfk[5]=Xfk[5]/PI*180.0;
}

void invtran(float* Titi, float* Titf)
{
  // finding the inverse of the homogeneous transformation matrix //求齐次变换矩阵的逆
  // first row  //第1行
  Titf[0*4 + 0] = Titi[0*4 + 0];
  Titf[0*4 + 1] = Titi[1*4 + 0];
  Titf[0*4 + 2] = Titi[2*4 + 0];
  Titf[0*4 + 3] = -Titi[0*4 + 0]*Titi[0*4 + 3]-Titi[1*4 + 0]*Titi[1*4 + 3]-Titi[2*4 + 0]*Titi[2*4 + 3];
  // second row //第2行
  Titf[1*4 + 0] = Titi[0*4 + 1];
  Titf[1*4 + 1] = Titi[1*4 + 1];
  Titf[1*4 + 2] = Titi[2*4 + 1];
  Titf[1*4 + 3] = -Titi[0*4 + 1]*Titi[0*4 + 3]-Titi[1*4 + 1]*Titi[1*4 + 3]-Titi[2*4 + 1]*Titi[2*4 + 3];
  // third row  //第3行
  Titf[2*4 + 0] = Titi[0*4 + 2];
  Titf[2*4 + 1] = Titi[1*4 + 2];
  Titf[2*4 + 2] = Titi[2*4 + 2];
  Titf[2*4 + 3] = -Titi[0*4 + 2]*Titi[0*4 + 3]-Titi[1*4 + 2]*Titi[1*4 + 3]-Titi[2*4 + 2]*Titi[2*4 + 3];
  // forth row  //第4行
  Titf[3*4 + 0] = 0.0;
  Titf[3*4 + 1] = 0.0;
  Titf[3*4 + 2] = 0.0;
  Titf[3*4 + 3] = 1.0;
}

void tran2pos(float* Ttp, float* Xtp)
{
  // pos from homogeneous transformation matrix //点坐标来自齐次变换矩阵
  Xtp[0] = Ttp[0*4 + 3];
  Xtp[1] = Ttp[1*4 + 3];
  Xtp[2] = Ttp[2*4 + 3];
  Xtp[4] = atan2(sqrt(Ttp[2*4 + 0]*Ttp[2*4 + 0] + Ttp[2*4 + 1]*Ttp[2*4 + 1]),Ttp[2*4 + 2]);
  Xtp[3] = atan2(Ttp[1*4 + 2]/sin(Xtp[4]),Ttp[0*4 + 2]/sin(Xtp[4]));
  Xtp[5] = atan2(Ttp[2*4 + 1]/sin(Xtp[4]),-Ttp[2*4 + 0]/sin(Xtp[4]));
}

void pos2tran(float* Xpt, float* Tpt)
{ 
  // pos to homogeneous transformation matrix //从点坐标到齐次变换矩阵
  // first row  //第1行
  Tpt[0*4 + 0] = cos(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])-sin(Xpt[3])*sin(Xpt[5]);
  Tpt[0*4 + 1] = -cos(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])-sin(Xpt[3])*cos(Xpt[5]);
  Tpt[0*4 + 2] = cos(Xpt[3])*sin(Xpt[4]);
  Tpt[0*4 + 3] = Xpt[0];
  // second row //第2行
  Tpt[1*4 + 0] = sin(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])+cos(Xpt[3])*sin(Xpt[5]);
  Tpt[1*4 + 1] = -sin(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])+cos(Xpt[3])*cos(Xpt[5]);
  Tpt[1*4 + 2] = sin(Xpt[3])*sin(Xpt[4]);
  Tpt[1*4 + 3] = Xpt[1];
  // third row  //第3行
  Tpt[2*4 + 0] = -sin(Xpt[4])*cos(Xpt[5]);
  Tpt[2*4 + 1] = sin(Xpt[4])*sin(Xpt[5]);
  Tpt[2*4 + 2] = cos(Xpt[4]);
  Tpt[2*4 + 3] = Xpt[2];
  // forth row  //第4行
  Tpt[3*4 + 0] = 0.0;
  Tpt[3*4 + 1] = 0.0;
  Tpt[3*4 + 2] = 0.0;
  Tpt[3*4 + 3] = 1.0;
}

void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh)
{
  // creats Denavit-Hartenberg homogeneous transformation matrix  //建立德纳维-哈滕伯格齐次变换矩阵
  // first row  //第1行
  Tdh[0*4 + 0] = cos(thetadh);
  Tdh[0*4 + 1] = -sin(thetadh)*cos(alfadh);
  Tdh[0*4 + 2] = sin(thetadh)*sin(alfadh);
  Tdh[0*4 + 3] = rdh*cos(thetadh);
  // second row //第2行
  Tdh[1*4 + 0] = sin(thetadh);
  Tdh[1*4 + 1] = cos(thetadh)*cos(alfadh);
  Tdh[1*4 + 2] = -cos(thetadh)*sin(alfadh);
  Tdh[1*4 + 3] = rdh*sin(thetadh);
  // third row  //第3行
  Tdh[2*4 + 0] = 0.0;
  Tdh[2*4 + 1] = sin(alfadh);
  Tdh[2*4 + 2] = cos(alfadh);
  Tdh[2*4 + 3] = ddh;
  // forth row  //第4行
  Tdh[3*4 + 0] = 0.0;
  Tdh[3*4 + 1] = 0.0;
  Tdh[3*4 + 2] = 0.0;
  Tdh[3*4 + 3] = 1.0;
}

/*
 * MatrixPrint
 * 串口打印矩阵信息
 */
void MatrixPrint(float* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

/*
 * MatrixCopy
 * 复制矩阵
 */
void MatrixCopy(float* A, int n, int m, float* B)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      B[n * i + j] = A[n * i + j];
    }
}

/*
 * MatrixMultiply
 * Matrix Multiplication Routine
 * C = A*B
 * 矩阵的乘法例程
 */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
}

/*
 * MatrixAdd
 * Matrix Addition Routine
 * 矩阵的加法例程
 */
void MatrixAdd(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] + B[n * i + j];
}

/*
 * MatrixSubtract
 * Matrix Subtraction Routine
 * 矩阵的减法例程
 */
void MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A-B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] - B[n * i + j];
}

/*
 * MatrixTranspose
 * Matrix Transpose Routine
 * 矩阵转置例程
 */
void MatrixTranspose(float* A, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
}

/*
 * MatrixScale
 * Matrix Scale
 * 矩阵比例变化程序
 */
void MatrixScale(float* A, int m, int n, float k)
{
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      A[n * i + j] = A[n * i + j] * k;
}
