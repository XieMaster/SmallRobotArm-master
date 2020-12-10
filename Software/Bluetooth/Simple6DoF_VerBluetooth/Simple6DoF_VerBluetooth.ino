/*
 * Simple script to move my tiny 6dof robotic arm
 */
#include <math.h>
//#include <SoftwareSerial.h>

#define PI 3.1415926535897932384626433832795

//driver for the axis 1
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

//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

double curPos1 = 0.0;
double curPos2 = -78.51;
double curPos3 = 73.90;
double curPos4 = 0.0;
double curPos5 = -90.0;
double curPos6 = 0.0;

boolean PULstat1 = 0;
boolean PULstat2 = 0;
boolean PULstat3 = 0;
boolean PULstat4 = 0;
boolean PULstat5 = 0;
boolean PULstat6 = 0;

//robot geometry
const double dl1 = 360.0/200.0/32.0/4.8;
const double dl2 = 360.0/200.0/32.0/4.0;
const double dl3 = 360.0/200.0/32.0/5.0;
const double dl4 = 360.0/200.0/32.0/2.8;
const double dl5 = 360.0/200.0/32.0/2.1;
const double dl6 = 360.0/200.0/32.0/1.0;
const double r1 = 47.0;
const double r2 = 110.0;
const double r3 = 26.0; 
const double d1 = 133.0;
const double d3 = 0.0;
const double d4 = 117.50;
const double d6 = 28.0;

//SoftwareSerial Bluetooth(29, 31); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)
String dataIn = ""; //variable to store the bluetooth command
double futPos1 = 0.0;
double futPos2 = 0.0;
double futPos3 = 0.0;
double futPos4 = 0.0;
double futPos5 = 0.0;
double futPos6 = 0.0;
double curSpeed = 0.3*0.5e-4;
double curFinalSpeed = 0.0;
int index = 0; //index corresonding to the robot position
float Joint1[50], Joint2[50], Joint3[50], Joint4[50], Joint5[50], Joint6[50], MaxSpeed[50], InSpeed[50], FinSpeed[50];

void setup()
{
  //Bluetooth.begin(38400); // Default baud rate of the Bluetooth module
  //Bluetooth.setTimeout(1);
  Serial1.begin(9600);
  //Serial1.setTimeout(3);
  delay(20);

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
  
  digitalWrite(PUL1_PIN, LOW); // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL2_PIN, LOW); // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL3_PIN, LOW); // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL4_PIN, LOW); // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL5_PIN, LOW); // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL6_PIN, LOW); // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW); //LOW = positive direction

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH); 

  //Serial.begin(9600);
}

void loop()
{
  //Serial1.write("Hello");
  /* Check for incoming data  检查输入数据 */
  if (Serial1.available() > 0) {
    dataIn = Serial1.readString();  // Read the data as string  //读取串口字符串数据

    //Serial.println(dataIn);
    if (dataIn == "enable") {
      digitalWrite(EN321_PIN, LOW);
      digitalWrite(EN4_PIN, LOW);
      digitalWrite(EN5_PIN, LOW);
      digitalWrite(EN6_PIN, LOW);
    }
    
    if (dataIn == "disable") {
      digitalWrite(EN321_PIN, HIGH);
      digitalWrite(EN4_PIN, HIGH);
      digitalWrite(EN5_PIN, HIGH);
      digitalWrite(EN6_PIN, HIGH);
    }

    if (dataIn.startsWith("s6")) {  //如果收到的字符串以“S6”开头
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos6 = dataInS.toFloat();  //字符串浮点数
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, curPos4, curPos5, futPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);  //走直线
    }

    if (dataIn.startsWith("s5")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos5 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, curPos4, futPos5, curPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s4")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos4 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, futPos4, curPos5, curPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s3")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos3 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, futPos3, curPos4, curPos5, curPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s2")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos2 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, futPos2, curPos3, curPos4, curPos5, curPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }
    
    if (dataIn.startsWith("s1")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos1 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={futPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    /* set the maximum speed for the move 设置移动的最大速度 */
    if (dataIn.startsWith("ss")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      curSpeed = (dataInS.toFloat()/100)*0.5e-4;
    }

    /* If button "SAVE" is pressed  如果按下“保存”按钮 */
    if (dataIn.startsWith("save")) {
      String dataInS = dataIn.substring(4, dataIn.length());
      InSpeed[index] = curFinalSpeed;
      curFinalSpeed = (dataInS.toFloat()/100)*0.5e-4;
      Joint1[index] = curPos1;  // save position into the array 保存位置到数组中
      Joint2[index] = curPos2;
      Joint3[index] = curPos3;
      Joint4[index] = curPos4;
      Joint5[index] = curPos5;
      Joint6[index] = curPos6;
      MaxSpeed[index] = curSpeed;
      FinSpeed[index] = curFinalSpeed;
      index++;                        // Increase the array index 增加数组索引
    }

    if ( dataIn == "reset") {
      index = 0;  // Index to 0
    }

    if (dataIn.startsWith("run")) {   //如果收到的字符串以“run”开头
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={Joint1[0], Joint2[0], Joint3[0], Joint4[0], Joint5[0], Joint6[0]};
      goStrightLine(Jinitial, Jfinal, MaxSpeed[0], 0.75e-10, 0.0, 0.0);
      for (int i = 0; i <= index - 2; i++) {  // Run through all steps(index) //运行所有步骤(索引)
        float Jinitial[6]={Joint1[i], Joint2[i], Joint3[i], Joint4[i], Joint5[i], Joint6[i]};
        float Jfinal[6]={Joint1[i+1], Joint2[i+1], Joint3[i+1], Joint4[i+1], Joint5[i+1], Joint6[i+1]};
        goStrightLine(Jinitial, Jfinal, MaxSpeed[i+1], 0.75e-10, InSpeed[i+1], FinSpeed[i+1]);
      }
    }
  }
}

/*
 * goStrightLine
 * 走直线函数
 */
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
    //contant maximum speed phase //最大速度阶段
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

/*
 * goTrajectory
 * 走的轨迹函数
 * 
 */
void goTrajectory(float* Jf){
  
  //execution 执行
  int delF=2;
  // joint #1 //关节 #1
  if (Jf[0]-curPos1>0.0) { // positive direction of rotation  //正选择方向
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
  if (Jf[1]-curPos2>0.0) { // positive direction of rotation  //正选择方向
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
  if (Jf[2]-curPos3>0.0) { // positive direction of rotation  //正选择方向
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
  if (Jf[3]-curPos4>0.0) { // positive direction of rotation  //正选择方向
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
  if (Jf[4]-curPos5>0.0) { // positive direction of rotation  //正选择方向
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
  if (Jf[5]-curPos6>0.0) { // positive direction of rotation  //正选择方向
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
 * 函 数 名：InverseK
 *          inverse kinematics 逆向运动学
 * 输入参数：Xik - pos value for the calculation of the inverse kinematics
 * 输出参数：Jfk - joints value for the calculation of the inversed kinematics
 */
void InverseK(float* Xik, float* Jik)
{
  /* from deg to rad 角度制转弧度制 */ 
  Xik[3]=Xik[3]*PI/180.0;
  Xik[4]=Xik[4]*PI/180.0;
  Xik[5]=Xik[5]*PI/180.0;
  /* Denavit-Hartenberg matrix  D-H矩阵 */
  float theta[6] = {0.0, -90.0, 0.0, 0.0, 0.0, 0.0};      // theta = [0; -90+0; 0; 0; 0; 0];
  float alfa[6]  = {-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa  = [-90; 0; -90; 90; -90; 0];
  float r[6]     = {r1, r2, r3, 0.0, 0.0, 0.0};           // r     = [47; 110; 26; 0; 0; 0];
  float d[6]     = {d1, 0.0, d3, d4, 0.0, d6};            // d     = [133; 0; 7; 117.5; 0; 28];
  /* from deg to rad  角度转弧度 */
  MatrixScale(theta, 6, 1, PI/180.0); 
  MatrixScale(alfa, 6, 1, PI/180.0);  
  /* work frame 工作坐标 */
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  /* tool frame 工具坐标 */
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  /* work frame transformation matrix 工作坐标变换矩阵 */
  float Twf[16];
  pos2tran(Xwf, Twf);  
  /* tool frame transformation matrix 工具坐标变换矩阵 */
  float Ttf[16];  
  pos2tran(Xtf, Ttf);  
  /* total transformation matrix 总变换矩阵 */
  float Twt[16];
  pos2tran(Xik, Twt);  
  /* find T06 找到T06 */
  float inTwf[16], inTtf[16], Tw6[16], T06[16];
  invtran(Twf, inTwf);  
  invtran(Ttf, inTtf);  
  MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6); 
  MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06); 
  /* positon of the spherical wrist 球形手腕的位置 */
  float Xsw[3];
  // Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
  Xsw[0]=T06[0*4 + 3]-d[5]*T06[0*4 + 2];
  Xsw[1]=T06[1*4 + 3]-d[5]*T06[1*4 + 2];
  Xsw[2]=T06[2*4 + 3]-d[5]*T06[2*4 + 2];
  // joints variable
  // Jik=zeros(6,1);
  /* first joint 第1个关节 */
  Jik[0]=atan2(Xsw[1],Xsw[0])-atan2(d[2],sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])); // Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  /* second joint 第2个关节 */
  Jik[1]=PI/2.0
  -acos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
  -atan((Xsw[2]-d[0])/(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])); // Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  /* third joint 第3个关节 */
  Jik[2]=PI
  -acos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*sqrt(r[2]*r[2]+d[3]*d[3])))
  -atan(d[3]/r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  /* last three joints 最后3个关节 */
  float T01[16], T12[16], T23[16], T02[16], T03[16], inT03[16], T36[16];
  DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  MatrixMultiply(T01, T12, 4, 4, 4, T02); // T02=T01*T12;
  MatrixMultiply(T02, T23, 4, 4, 4, T03); // T03=T02*T23;
  invtran(T03, inT03); // inT03=invtran(T03);
  MatrixMultiply(inT03, T06, 4, 4, 4, T36); // T36=inT03*T06;
  /* forth joint  第4个关节 */
  Jik[3]=atan2(-T36[1*4+2], -T36[0*4+2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
  /* fifth joint  第5个关节 */
  Jik[4]=atan2(sqrt(T36[0*4+2]*T36[0*4+2]+T36[1*4+2]*T36[1*4+2]), T36[2*4+2]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  /* sixth joints 第6个关节 */
  Jik[5]=atan2(-T36[2*4+1], T36[2*4+0]); // Jik(6)=atan2(-T36(3,2),T36(3,1));
  /* rad to deg  弧度转角度 */
  MatrixScale(Jik, 6, 1, 180.0/PI); 
}

/*
 * 函 数 名：ForwardK
 *          forward kinematics 正向运动学
 * 输入参数：Jfk - joints value for the calculation of the forward kinematics [j1,j2,j3,j4,j5,j6]
 * 输出参数：Xfk - pos value for the calculation of the forward kinematics    [X,Y,Z,A,B,C]
 */
void ForwardK(float* Jfk, float* Xfk)
{
  /* Denavit-Hartenberg matrix */
  float theTemp[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0};
  float theta[6];
  MatrixAdd(theTemp, Jfk, 6, 1, theta);   // theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; 
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];
  float d[6]={d1, 0.0, d3, d4, 0.0, d6};  // d=[133; 0; 7; 117.5; 0; 28];
  /* from deg to rad */
  MatrixScale(theta, 6, 1, PI/180.0);     // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0);      // alfa=alfa*pi/180;
  /* work frame */
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  /* tool frame */
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  /* work frame transformation matrix */
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  /* tool frame transformation matrix */
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  /* DH homogeneous transformation matrix */
  float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
  DH1line(theta[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1),alfa(1),r(1),d(1));
  DH1line(theta[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2),alfa(2),r(2),d(2));
  DH1line(theta[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3),alfa(3),r(3),d(3));
  DH1line(theta[3], alfa[3], r[3], d[3], T34); // T34=DH1line(theta(4),alfa(4),r(4),d(4));
  DH1line(theta[4], alfa[4], r[4], d[4], T45); // T45=DH1line(theta(5),alfa(5),r(5),d(5));
  DH1line(theta[5], alfa[5], r[5], d[5], T56); // T56=DH1line(theta(6),alfa(6),r(6),d(6));
  float Tw1[16], Tw2[16], Tw3[16], Tw4[16], Tw5[16], Tw6[16], Twt[16];
  MatrixMultiply(Twf, T01, 4, 4, 4, Tw1); // Tw1=Twf*T01;
  MatrixMultiply(Tw1, T12, 4, 4, 4, Tw2); // Tw2=Tw1*T12;
  MatrixMultiply(Tw2, T23, 4, 4, 4, Tw3); // Tw3=Tw2*T23;
  MatrixMultiply(Tw3, T34, 4, 4, 4, Tw4); // Tw4=Tw3*T34;
  MatrixMultiply(Tw4, T45, 4, 4, 4, Tw5); // Tw5=Tw4*T45;
  MatrixMultiply(Tw5, T56, 4, 4, 4, Tw6); // Tw6=Tw5*T56;
  MatrixMultiply(Tw6, Ttf, 4, 4, 4, Twt); // Twt=Tw6*Ttf;
  // calculate pos from transformation matrix
  tran2pos(Twt, Xfk); // Xfk=tran2pos(Twt);
  /* 弧度转为角度 *//* 欧拉角 */
  Xfk[3]=Xfk[3]/PI*180.0; 
  Xfk[4]=Xfk[4]/PI*180.0; 
  Xfk[5]=Xfk[5]/PI*180.0;
}

void invtran(float* Titi, float* Titf)
{
  // finding the inverse of the homogeneous transformation matrix //求齐次变换矩阵的逆
  // first row    //第1行
  Titf[0*4 + 0] = Titi[0*4 + 0];
  Titf[0*4 + 1] = Titi[1*4 + 0];
  Titf[0*4 + 2] = Titi[2*4 + 0];
  Titf[0*4 + 3] = -Titi[0*4 + 0]*Titi[0*4 + 3]-Titi[1*4 + 0]*Titi[1*4 + 3]-Titi[2*4 + 0]*Titi[2*4 + 3];
  // second row   //第2行
  Titf[1*4 + 0] = Titi[0*4 + 1];
  Titf[1*4 + 1] = Titi[1*4 + 1];
  Titf[1*4 + 2] = Titi[2*4 + 1];
  Titf[1*4 + 3] = -Titi[0*4 + 1]*Titi[0*4 + 3]-Titi[1*4 + 1]*Titi[1*4 + 3]-Titi[2*4 + 1]*Titi[2*4 + 3];
  // third row    //第3行
  Titf[2*4 + 0] = Titi[0*4 + 2];
  Titf[2*4 + 1] = Titi[1*4 + 2];
  Titf[2*4 + 2] = Titi[2*4 + 2];
  Titf[2*4 + 3] = -Titi[0*4 + 2]*Titi[0*4 + 3]-Titi[1*4 + 2]*Titi[1*4 + 3]-Titi[2*4 + 2]*Titi[2*4 + 3];
  // forth row    //第4行
  Titf[3*4 + 0] = 0.0;
  Titf[3*4 + 1] = 0.0;
  Titf[3*4 + 2] = 0.0;
  Titf[3*4 + 3] = 1.0;
}

void tran2pos(float* Ttp, float* Xtp)
{
  // pos from homogeneous transformation matrix //来自齐次变换矩阵的点坐标
  Xtp[0] = Ttp[0*4 + 3];
  Xtp[1] = Ttp[1*4 + 3];
  Xtp[2] = Ttp[2*4 + 3];
  Xtp[4] = atan2(sqrt(Ttp[2*4 + 0]*Ttp[2*4 + 0] + Ttp[2*4 + 1]*Ttp[2*4 + 1]),Ttp[2*4 + 2]);
  Xtp[3] = atan2(Ttp[1*4 + 2]/sin(Xtp[4]),Ttp[0*4 + 2]/sin(Xtp[4]));
  Xtp[5] = atan2(Ttp[2*4 + 1]/sin(Xtp[4]),-Ttp[2*4 + 0]/sin(Xtp[4]));
}

void pos2tran(float* Xpt, float* Tpt)
{
  // pos to homogeneous transformation matrix //点坐标到齐次变换矩阵
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
  // creats Denavit-Hartenberg homogeneous transformation matrix  //建立D-H齐次变换矩阵
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
 * 函 数 名：MatrixPrint
 *          Matrix Print 矩阵的串口打印函数
 * 输入参数：A = input matrix (m x n)
 *          m = number of rows in A 
 *          n = number of columns in A
 *          label = 
 * 输出参数：
 */
void MatrixPrint(float* A, int m, int n, String label)
{
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
 * 函 数 名：MatrixCopy
 *          Matrix Copy 矩阵的复制函数
 * 输入参数：A = input matrix (m x n)
 *          m = number of rows in A 
 *          n = number of columns in A
 * 输出参数：B = output matrix = A(m x n)
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
 * 函 数 名：MatrixMultiply
 *          Matrix Multiplication Routine 矩阵的乘法
 * 输入参数：A = input matrix (m x p)
 *          B = input matrix (p x n)
 *          m = number of rows in A 
 *          p = number of columns in A = number of rows in B
 *          n = number of columns in B
 * 输出参数：C = output matrix = A*B (m x n)
 */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
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
 * 函 数 名：MatrixAdd
 *          Matrix Addition Routine 矩阵的加法
 * 输入参数：A = input matrix (m x n)
 *          B = input matrix (m x n)
 *          m = number of rows in A = number of rows in B
 *          n = number of columns in A = number of columns in B
 * 输出参数：C = output matrix = A+B (m x n)
 */
void MatrixAdd(float* A, float* B, int m, int n, float* C)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] + B[n * i + j];
}

/*
 * 函 数 名：MatrixSubtract
 *          Matrix Subtraction Routine 矩阵的减法
 * 输入参数：A = input matrix (m x n)
 *          B = input matrix (m x n)
 *          m = number of rows in A = number of rows in B
 *          n = number of columns in A = number of columns in B
 * 输出参数：C = output matrix = A-B (m x n)
 */
void MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] - B[n * i + j];
}

/*
 * 函 数 名：MatrixTranspose
 *          Matrix Transpose Routine 矩阵的转置程序
 * 输入参数：A = input matrix (m x n)
 *          m = number of rows in A 
 *          n = number of columns in A 
 * 输出参数：C = output matrix = the transpose of A (n x m)
 */
void MatrixTranspose(float* A, int m, int n, float* C)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
}

/*
 * 函 数 名：MatrixScale
 *          Matrix scaling operation 矩阵的比例运算
 * 输入参数：A = input matrix (m x n)
 *          m = number of rows in A
 *          n = number of columns in A
 *          K = the Matrix scale factor
 * 输出参数：A = input matrix A(m x n) * K
 */
void MatrixScale(float* A, int m, int n, float k)
{
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      A[n * i + j] = A[n * i + j] * k;
}

