int U_Pin = 2;  //定义中断引脚为数字端口2
const byte RATE_SIZE = 10; //存储速度数据数组的长度
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //上次记录的时间
int beatAvg;      //平均速度
int Val = 0;      //设置变量Val，计数
int val0 = 0;     //记录上次的Val
float time;  //设置变量time，计时
float p,i,d;  //定义pid参数
String P,I,D,Target;
int Speed;  //设置变量Speed，存储转速
int target;  //目标转速
int e1,e2,e3,pwmout;  //定义误差，和输出的PWM占空比
int duty;  //占空比
int cnt=0;
boolean state = false;  //定义判断是否开启PID的状态量
String Duty;
String comdata = "";  //定义字符串存储串口
String senddata = "";  //定义向串口发送的数据

void setup(){

  Serial.begin(115200);  //定义串口波特率为115200，变量初始化
  p=0;
  i=0;
  d=0;
  e1=0;
  e2=0;
  e3=0;
  pwmout=50;  
  pinMode(6,OUTPUT);  //定义引脚模式
  pinMode(7,OUTPUT);
  digitalWrite(7,LOW);
  analogWrite(6,200);  //电机启动
  attachInterrupt(0,count,CHANGE);    //引脚电平发生改变时触发
}

void loop(){
  while (Serial.available() > 0)       //读取串口数据
    {
        comdata += char(Serial.read());
        delay(2);
    }
  if(comdata.indexOf("ratio") != -1){  //读取串口发送的PWM占空比并驱动电机
      Duty=comdata.substring(comdata.indexOf("o")+1,comdata.length());
      duty=Duty.toInt();
      analogWrite(6,duty);
      comdata="";
  }
  else if(comdata.indexOf("close") != -1){  //电机停止
      analogWrite(6,0);
      comdata="";
  }
  else if(comdata.indexOf("targ") != -1){  //读取目标转速及PID参数量
        
      if (comdata.indexOf("P") != -1 && comdata.indexOf("I") !=-1){
        Target=comdata.substring(comdata.indexOf("g")+1,comdata.indexOf("P"));
        target = Target.toInt();
        P=comdata.substring(comdata.indexOf("P")+1,comdata.indexOf("I"));
        p=P.toFloat();
      }  
      if (comdata.indexOf("I") != -1 && comdata.indexOf("D") !=-1){
        I=comdata.substring(comdata.indexOf("I")+1,comdata.indexOf("D"));
        i=I.toFloat();
      }
      if (comdata.indexOf("D") != -1){
        D=comdata.substring(comdata.indexOf("D")+1,comdata.length());
        d=D.toFloat();
      }      
      comdata="";
  }
  else if(comdata.indexOf("start") != -1){  //开启PID控制
      state = true;
      comdata="";
  }
  else if(comdata.indexOf("stop") != -1){  //停止PID控制
      state = false;
      comdata="";
  }      
  int v = Val-val0;
  long delta = millis() - lastBeat;
  lastBeat = millis();
  val0=Val;
  Speed = int(v*60000/40/delta);  //计算转速
  rates[rateSpot++] = (byte)Speed; //存储到数组
  rateSpot %= RATE_SIZE;
  beatAvg = 0;
  for (byte x = 0 ; x < RATE_SIZE ; x++){  //对速度数组累加求平均
        beatAvg += rates[x];
  }
  beatAvg /= RATE_SIZE;
  if(state){
    cnt++;
    if(cnt>1){  //200ms进行一次PID控制
      e3=target-beatAvg;
      pwmout+= p*(e3-e2)+i*(e3)+d*(e3-2*e2+e1); 
      e1=e2;
      e2=e3;
      if(pwmout<50){
        pwmout=50;
      } 
      else if(pwmout>255){
        pwmout=255;
      }
      analogWrite(6,pwmout);
      cnt=0;
    }
  }
  Serial.println(beatAvg);  
  delay(100);
}

void count(){  //中断计数
  Val += 1;
} 
