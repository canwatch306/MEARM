#include<Servo.h> //导入舵机库
Servo myservo; //建立舵机对象
String comdata = ""; //定义字符串存储串口
String senddata = ""; //定义向串口发送的数据
String D0,D1;
int angle; //定义舵机转角
int datamin,datamax; //定义标定的最小值和最大值
boolean status=false;
void setup(){
  myservo.attach(9); //舵机连接引脚9
  Serial.begin(115200); //设置串口波特率9600
  pinMode(A0,INPUT);
  angle=0; //初始化角度,标定值
  datamin=0;
  datamax=0;
}
void loop(){
  while (Serial.available() > 0)       //读取串口数据
    {
        comdata += char(Serial.read());
        delay(2);
    }
    if (comdata.indexOf("auto") != -1){
        if (comdata.indexOf("n") != -1 && comdata.indexOf("x") !=-1){
          D0 = comdata.substring(comdata.indexOf("n")+1,comdata.indexOf("x"));                 
        }        
        if (comdata.indexOf("x") != -1){    
          D1 = comdata.substring(comdata.indexOf("x")+1,comdata.length());                
        }
        datamin=D0.toInt();
        datamax=D1.toInt();
        comdata="";
    }
    else if(comdata.indexOf("stop") != -1){
        status = false;
        comdata="";
    }
    else if(comdata.indexOf("status") != -1){
        status = true;
        comdata="";
    }
    else if(comdata.indexOf("hand") != -1){
        String Angle = comdata.substring(comdata.indexOf("d")+1,comdata.length());
        servocontrol(Angle.toInt(),20);
        comdata="";      
    }
    if(status){
      if(analogRead(A0)<datamin){
        angle=0;
      }
      else if(analogRead(A0)>datamax){
        angle=180;
      }
      else{
        angle = int(180*(analogRead(A0)-datamin)/(datamax-datamin));        
      }
      myservo.write(angle);
      comdata=""; 
      senddata = senddata +"sens"+ analogRead(A0) + "/" +myservo.read();
      Serial.println(senddata);
      senddata = "";
      delay(10);         
    }
  senddata = senddata +"sens"+ analogRead(A0) + "/" +myservo.read();
  Serial.println(senddata);
  senddata = "";
  delay(100);    
}
void servocontrol(int toPos,int servoDelay){       
        int fromPos = myservo.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i++){
            myservo.write(i);
            senddata = senddata+"sens" + analogRead(A0) + "/" +myservo.read();
            Serial.println(senddata);
            senddata = "";
            delay (servoDelay);
          }
        }  
        else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i--){
              myservo.write(i);
              senddata = senddata +"sens"+ analogRead(A0) + "/" +myservo.read();
              Serial.println(senddata);
              senddata = "";
              delay (servoDelay);
            }
           }        
}