#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>                //使用servo库
String C1,C2;
int c1,c2;
int bangle,rangle,fangle,cangle;
boolean status = false;
boolean status1 = false;
int mode = 0;
String send = ""; 
MPU6050 accelgyro;
unsigned long time1,time2;
unsigned long now, lastTime = 0;
float dt;                                   //微分时间
 
int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    //角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量
 
float pi = 3.1415926;
float AcceRatio = 16384.0;                  //加速度计比例系数
float GyroRatio = 131.0;                    //陀螺仪比例系数
 
uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum,aaz_sum;                      //x,y轴采样和
 
float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //加速度计协方差计算队列
float Px=1, Rx, Kx, Sx, Vx, Qx;             //x轴卡尔曼变量
float Py=1, Ry, Ky, Sy, Vy, Qy;             //y轴卡尔曼变量
float Pz=1, Rz, Kz, Sz, Vz, Qz;             //z轴卡尔曼变量

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int data1=0;

Servo base, fArm, rArm, claw ;    //创建4个servo对象
String comdata = "";
String angle;
String b_angle, r_angle, f_angle, c_angle;
String senddata = "";
int redi = 0;
int yellowi = 0;
const int baseMin = 20;
const int baseMax = 160;
const int rArmMin = 60;
const int rArmMax = 140;
const int fArmMin = 50;
const int fArmMax = 160;
const int clawMin = 60;
const int clawMax = 100;
const int bpwm = 6;
const int rpwm = 9;
const int fpwm = 10;
const int cpwm = 11;
int DSD = 15;          //舵机延时,控制舵机速度
int redposition[3][4]{
{120,130,95,95},
{120,118,108,95},
{120,110,120,95}
};
int yellowposition[3][4]{
{40,130,95,95},
{40,118,108,95},
{40,110,120,95}
};

void setup() {
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);    
  base.attach(bpwm);   // base 伺服舵机连接引脚6 舵机代号'b'
  delay(200);          // 稳定性等待
  rArm.attach(rpwm);     // rArm 伺服舵机连接引脚9 舵机代号'r'
  delay(200);          // 稳定性等待
  fArm.attach(fpwm);      // fArm 伺服舵机连接引脚10  舵机代号'f'
  delay(200);          // 稳定性等待
  claw.attach(cpwm);      // claw 伺服舵机连接引脚11  舵机代号'c'
  delay(200);
  //机械臂初始位置
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);  
  baseservo(90,DSD); 
  delay(30);
  clawservo(95,DSD);  
  delay(30); 
 
  Serial.begin(115200);  

}

void loop() {
  while (Serial.available() > 0){
        comdata += char(Serial.read());
        delay(2);
    }
    if (comdata.indexOf("auto") != -1){
        if (comdata.indexOf("y") != -1 && comdata.indexOf("z") !=-1){
        C1 = comdata.substring(comdata.indexOf("y")+1,comdata.indexOf("z"));                   
        }
        if (comdata.indexOf("z") != -1){
        C2 = comdata.substring(comdata.indexOf("z")+1,comdata.length());                    
        }
        c1 = C1.toInt();
        c2 = C2.toInt();
        comdata="";                            
    }    
    else if (comdata.indexOf("stop") != -1){
        status = false;
        comdata="";
    }
    else if (comdata.indexOf("status") != -1){
        status1 = true;
        comdata="";
    }
    else if (comdata.indexOf("over") != -1){
        status1 = false;
        comdata="";
    }
    else if (comdata.indexOf("mode1") != -1){
        mode = 0; 
        comdata="";   
    }
    else if (comdata.indexOf("mode2") != -1){
        mode = 1;
        comdata="";        
    }
    if (mode == 1){
        if (comdata.indexOf("b") != -1 && comdata.indexOf("r") !=-1){
          angle = comdata.substring(comdata.indexOf("b")+1,comdata.indexOf("r"));
          base.write(angle.toInt());
        }
        if (comdata.indexOf("f") != -1 && comdata.indexOf("c") !=-1){
          angle = comdata.substring(comdata.indexOf("f")+1,comdata.indexOf("c"));
          fArm.write(angle.toInt());
        }
        if (comdata.indexOf("r") != -1 && comdata.indexOf("f") !=-1){
          angle = comdata.substring(comdata.indexOf("r")+1,comdata.indexOf("f"));
          rArm.write(angle.toInt());
        }
        if (comdata.indexOf("c") != -1){
          angle = comdata.substring(comdata.indexOf("c")+1,comdata.indexOf("c")+3);
          claw.write(angle.toInt());
        }
        comdata="";
    }               
    else if (status){
      mpu6050();    
      if(status1){
        if(analogRead(A0)<c1){
          cangle = 95;
        }
        else if(analogRead(A0)>c2){
          cangle = 65;
        }
        else{
          cangle = int(95 - 30*(analogRead(A0)-c1)/(c2-c1));          
        }
        bangle = 90+int(yaw);
        rangle = 100+int(agx);
        fangle = 110+int(agy);
        if (bangle > 160){
          bangle = 160;          
        }
        else if (bangle < 20){
          bangle = 20;         
        }
        if (rangle > 140){      
          rangle = 140;        
        }
        else if (rangle < 60){          
          rangle = 60;          
        }
        if (fangle > 140){      
          fangle = 140;        
        }
        else if (fangle < 80){          
          fangle = 80;          
        }        
        base.write(bangle);
        delay(1);
        rArm.write(rangle);
        delay(1);
        fArm.write(fangle);
        delay(1);
        claw.write(cangle);
        delay(1);        
        send = send + "sens" +"X"+int(agx) +"Z"+int(yaw)+"P"+analogRead(A0)+"W"+int(agy)+ "b" + base.read() + "r"+ rArm.read() + "f" + fArm.read() + "c" + claw.read();
        Serial.println(send);
        send ="";        
      }
      else{
        send = send + "sens" +"X"+int(agx) +"Z"+int(yaw)+"P"+analogRead(A0)+"W"+int(agy)+ "b" + base.read() + "r"+ rArm.read() + "f" + fArm.read() + "c" + claw.read();
        Serial.println(send);
        send ="";
        delay(50);        
      }      
      comdata="";                  
    }    
    else if (comdata.indexOf("start") != -1){
      start();
      c1 = 0;
      c2 = 0;
      status = true;
      comdata="";
    }
    else if (comdata.indexOf("reco") != -1){
      reco();
      comdata="";
      redi = 0;
      yellowi = 0;     
      delay(1000);
      Serial.println("ready");
    }
    else if (comdata.indexOf("red") != -1){
      red();
      redi++;
      if(redi>2){
      redi=0;        
      }      
      comdata="";
      delay(1000);
      Serial.println("ready");
    }
    else if (comdata.indexOf("yellow") != -1){
      yellow();
      yellowi++;
      if(yellowi>2){
      yellowi=0;        
      }            
      comdata="";
      delay(1000);
      Serial.println("ready");
    }
    else if (comdata.length() > 0){
        farmservo(140,DSD);
        delay(30);
        rarmservo(70,DSD);
        delay(30);
        baseservo(90,DSD); 
        delay(300);
        if (comdata.indexOf("b") != -1 && comdata.indexOf("r") !=-1){
          angle = comdata.substring(comdata.indexOf("b")+1,comdata.indexOf("r"));
          baseservo(angle.toInt(),DSD);
        }
        if (comdata.indexOf("f") != -1 && comdata.indexOf("c") !=-1){
          angle = comdata.substring(comdata.indexOf("f")+1,comdata.indexOf("c"));
          farmservo(angle.toInt(),DSD);
        }
        if (comdata.indexOf("r") != -1 && comdata.indexOf("f") !=-1){
          angle = comdata.substring(comdata.indexOf("r")+1,comdata.indexOf("f"));
          rarmservo(angle.toInt(),DSD);
        }
        if (comdata.indexOf("c") != -1){
          angle = comdata.substring(comdata.indexOf("c")+1,comdata.length());
          clawservo(angle.toInt(),DSD);
        }
        comdata="";
    }
    else{
      sendangle();
      delay(100);
    }
}      
void start(){
  time1 = millis();
  Serial.begin(115200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  accelgyro.initialize();                 //初始化
  unsigned short times = 200;             //采样次数
    for(int i=0;i<times;i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
        axo += ax; ayo += ay; azo += az;      //采样和
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    
    axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
    gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移
}

void mpu6050(){
    float a;
    float b;
    time2 = millis();
    unsigned long now = millis();             //当前时间(ms)
    dt = (now - lastTime) / 1000.0;           //微分时间(s)
    lastTime = now;                           //上一次采样时间(ms) 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
 
    float accx = ax / AcceRatio;              //x轴加速度
    float accy = ay / AcceRatio;              //y轴加速度
    float accz = az / AcceRatio;              //z轴加速度
 
    aax = atan(accy / accz) * (-180) / pi;    //y轴对于z轴的夹角
    aay = atan(accx / accz) * 180 / pi;       //x轴对于z轴的夹角
    aaz = atan(accz / accy) * 180 / pi;       //z轴对于y轴的夹角
 
    aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;

    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //角度调幅至0-90°
    aays[n_sample-1] = aay;                        //此处应用实验法取得合适的系数
    aay_sum += aay * n_sample;                     //本例系数为9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;

 
    float gyrox = - (gx-gxo) / GyroRatio * dt; //x轴角速度
    float gyroy = - (gy-gyo) / GyroRatio * dt; //y轴角速度

    agx += gyrox;                             //x轴角速度积分
    agy += gyroy;                             //x轴角速度积分

    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //测量值平均值运算
        a_x[i-1] = a_x[i];                      //即加速度平均值
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];

    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 //x轴加速度平均值
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 //y轴加速度平均值

 
    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              //得到方差
    Ry = Ry / 9;                        

  
    Px = Px + 0.0025;                         // 0.0025在下面有说明...
    Kx = Px / (Px + Rx);                      //计算卡尔曼增益
    agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px;                       //更新p值
 
    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  


 // === 读取加速度计数据 === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === 读取重力加速度计 === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = GyroZ;
  if ((time2-time1)<=3000){                      //获得第3秒的数据
  yaw = yaw + GyroZ * elapsedTime;
  a = yaw;
  }
  if ((time2-time1)<=2000){                     //获得第2秒的数据
   b = yaw;
  }
  if ((time2-time1)>3000){
  yaw = yaw + (GyroZ - (a-b)) * elapsedTime; //修正公式 
  }  
}

void sendangle(){
  senddata = senddata + "b" + base.read() + "r"+ rArm.read() + "f" + fArm.read() + "c" + claw.read();
  Serial.println(senddata);
  senddata = "";
}

void baseservo(int toPos,int servoDelay){       
    if(toPos >= baseMin && toPos <= baseMax){
        int fromPos = base.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i++){
            base.write(i);
            sendangle();
            delay (servoDelay);
          }
        }  else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i--){
              base.write(i);
              sendangle();
              delay (servoDelay);
            }
           } 
    }        
} 
       
void rarmservo(int toPos,int servoDelay){       
    if(toPos >= rArmMin && toPos <= rArmMax){
        int fromPos = rArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i=i+2){
            if(toPos-i ==3){
              i=toPos;              
            }
            rArm.write(i);
            sendangle();
            delay (servoDelay);
          }
        }  else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i=i-2){
            if(i-toPos ==3){
              i=toPos;              
            }              
              rArm.write(i);
              sendangle();
              delay (servoDelay);
            }
           } 
    }        
}

void farmservo(int toPos,int servoDelay){       
    if(toPos >= fArmMin && toPos <= fArmMax){
        int fromPos = fArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i++){
            fArm.write(i);
            sendangle();
            delay (servoDelay);
          }
        }  else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i--){
              fArm.write(i);
              sendangle();
              delay (servoDelay);
            }
           } 
    }        
}

void clawservo(int toPos,int servoDelay){
    if(toPos >= clawMin && toPos <= clawMax){
        int fromPos = claw.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i++){
            claw.write(i);
            sendangle();
            delay (servoDelay);
          }
        }  else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i--){
              claw.write(i);
              sendangle();
              delay (servoDelay);
            }
           } 
    }  
}

void reco(){
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);
  baseservo(90,DSD); 
  delay(30);
  clawservo(95,DSD);
  delay(300);
}

void red(){
  farmservo(98,DSD);
  delay(30);
  rarmservo(130,DSD);
  delay(30);
  clawservo(65,DSD);
  delay(300);
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);
  baseservo(redposition[redi][0],DSD);
  delay(30);
  farmservo(redposition[redi][2],DSD);
  delay(30);
  rarmservo(redposition[redi][1],DSD);
  delay(30);
  clawservo(redposition[redi][3],DSD);
  delay(300);
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);
  baseservo(90,DSD); 
  delay(30);
  clawservo(95,DSD);
  delay(300);
}

void yellow(){
  farmservo(98,DSD);
  delay(30);
  rarmservo(130,DSD);
  delay(30);
  clawservo(65,DSD);
  delay(300);
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);
  baseservo(yellowposition[yellowi][0],DSD);
  delay(30);
  farmservo(yellowposition[yellowi][2],DSD);
  delay(30);
  rarmservo(yellowposition[yellowi][1],DSD);
  delay(30);
  clawservo(yellowposition[yellowi][3],DSD);
  delay(300);
  farmservo(145,DSD);
  delay(30);
  rarmservo(70,DSD);
  delay(30);
  baseservo(90,DSD); 
  delay(30);
  clawservo(95,DSD);
  delay(300);
}