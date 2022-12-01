const int TrigPin = 6;  //定义超声波发送引脚
const int EchoPin = 12;  //定义超声波接收引脚
float distance;  //定义距离
int alarmdis;  //定义预警距离
int frequency;  //定义PWM频率
float duty;  //定义占空比
boolean state=false;  
String Frequency,Distance;
String comdata = "";  //定义字符串存储串口
String senddata = "";  //定义向串口发送的数据
void setup(){
  Serial.begin(115200);  //定义串口通讯波特率,3,4,6,11,12引脚为输出,12引脚为输入,预设PWM频率为2000
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  frequency = 2000;
}
void loop(){
  while (Serial.available() > 0)       //读取串口数据
    {
        comdata += char(Serial.read());
        delay(2);
    }
  if(comdata.indexOf("hz") != -1){  //读取串口发送的目标PWM频率
      Frequency=comdata.substring(comdata.indexOf("z")+1,comdata.length());
      frequency=Frequency.toInt();
      comdata="";
  }
  else if(comdata.indexOf("start") != -1){  //输出指定频率的PWM驱动蜂鸣器
      PWM(0.5,frequency);
      digitalWrite(4,HIGH);      
  }
  else if(comdata.indexOf("close") != -1){  //关闭蜂鸣器
      close();
      digitalWrite(4,LOW);
      comdata="";      
  }
  else if(comdata.indexOf("stop") != -1){  //停止传感器控制
      state=false;
      comdata="";      
  }   
  else if(comdata.indexOf("Dis") != -1){  //设置预警距离并开启预警
      Distance=comdata.substring(comdata.indexOf("s")+1,comdata.length());
      alarmdis=Distance.toInt();
      state=true;
      comdata="";
  }    
  digitalWrite(TrigPin, LOW); //发送10us 的高电平信号激励超声波传感器
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  distance = pulseIn(EchoPin, HIGH)/58.00;  //检测脉冲宽度，并计算出距离
  if(distance<2000){
    if(state){
        if(distance<=alarmdis){
            duty = 0.03 +0.3*(alarmdis-distance)/(alarmdis-0);  //距离控制占空比,控制蜂鸣器音量
            PWM(duty,frequency);            
            senddata=senddata + "D" +distance + "F" +frequency;
            Serial.println(senddata);
            digitalWrite(4,HIGH);            
            senddata = "";     
        }
        else{
            close();
            digitalWrite(4,LOW);
            senddata=senddata + "D" +distance;
            Serial.println(senddata);
            senddata = "";             
        }
        delay(180);        
    }
    else{
      senddata=senddata + "D" +distance;
      Serial.println(senddata);
      senddata = "";
      delay(180);      
    }

  }
  comdata="";
  delay(20);    
}

void PWM(float ratio,int f){  //产生频率f,占空比ratio的PWM
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //Set Timer2 to varying top limit fast PWM mode
  TCCR2B = _BV(WGM22) | _BV(CS22);//another way to set prescaler CS2=fff(16MHz/64)
  
  OCR2A = int((16000000/64/f)-1); //Top value A(PWM频率为16MHz/64/(OCR2A+1))
  OCR2B = int((OCR2A*ratio)-1); //Toggle value B, Output at pin 3(占空比为(OCR2B+1)/(OCR2A+1))
}

void close(){  //关闭PWM
  TCCR2A = _BV(COM2A0) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20); //Set Timer2 to varying top limit fast PWM mode
  TCCR2B = _BV(WGM22) | _BV(CS22);//another way to set prescaler CS2=fff(16MHz/64)
}