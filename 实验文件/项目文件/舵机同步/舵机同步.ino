#include "Wire.h"  
#include "I2Cdev.h"  //导入I2C库
#include "MPU6050.h"  //导入MPU6050库
#include<Servo.h>  
Servo myservo;
String Angle;
String comdata = "";
String send = "";
boolean state = false; 
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


void setup()
{
  myservo.attach(9);
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
 
void loop()
{
  while (Serial.available() > 0){  //读取上位机发送的信息
        comdata += char(Serial.read());
        delay(2);
  }
  if (comdata.indexOf("start") != -1){  //开始传感器控制舵机
    state = true;
    comdata="";
  }
  else if(comdata.indexOf("stop") != -1){  //停止传感器控制舵机
    state = false;
    comdata="";    
  }
  else if(comdata.indexOf("hand") != -1){  //手动控制舵机
        String Angle = comdata.substring(comdata.indexOf("d")+1,comdata.length());
        servocontrol(Angle.toInt(),10);
        comdata="";
  }  
  mpu6050();  
  if(state){  //若state为true则为传感器控制舵机模式，否则只是采集数据
    myservo.write(int(yaw)+90);
    send = send +"X" + int(agx) +"Y"+int(agy)+"Z"+int(yaw) +"S"+myservo.read();
    Serial.println(send);                      //输出y轴数据
    send ="";     
    delay(10);        
  }
  else{
    send = send +"X" + int(agx) +"Y"+int(agy)+"Z"+int(yaw) +"S"+myservo.read();
    Serial.println(send);                      //输出y轴数据
    send ="";
    delay(100); 
  }
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
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58)
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
void servocontrol(int toPos,int servoDelay){       
        int fromPos = myservo.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
          for (int i=fromPos; i<=toPos; i++){
            myservo.write(i);
            send = send +"X" + int(agx) +"Y"+int(agy)+"Z"+int(yaw) +"S"+myservo.read();
            Serial.println(send);
            send = "";
            delay (servoDelay);
          }
        }  
        else { //否则“起始角度值”大于“目标角度值”
            for (int i=fromPos; i>=toPos; i--){
              myservo.write(i);
              send = send +"X" + int(agx) +"Y"+int(agy)+"Z"+int(yaw) +"S"+myservo.read();
              Serial.println(send);
              send = "";
              delay (servoDelay);
            }
           }        
}
