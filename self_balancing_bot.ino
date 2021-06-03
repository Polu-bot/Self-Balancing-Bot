#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <geometry_msgs/Twist.h>
#define EN_L 9
#define IN1_L 10
#define IN2_L 11
#define EN_R 8
#define IN1_R 12
#define IN2_R 13
double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


//Set up the ros node and publisher
std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;

ros::NodeHandle nh2;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;


void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

                           
void setup()
{

  nh.initNode();
  nh.advertise(imu);

  Motors_init();
  nh2.initNode();
  nh2.subscribe(sub);
 
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

long publisher_timer;

void loop()
{
  MotorL(w_l*10);
  MotorR(w_r*10);

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers  String AX = String(mpu6050.getAccX());
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  String AX = String(AcX);
  String AY = String(AcY);
  String AZ = String(AcZ);
  String GX = String(GyX);
  String GY = String(GyY);
  String GZ = String(GyZ);
  String tmp = String(Tmp);

  String data = "A" + AX + "B"+ AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
  Serial.println(data);
  int length = data.indexOf("G") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  
  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    imu_msg.data = data_final;
    imu.publish(&imu_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
    nh2.spinOnce();
  }

  
}

void Motors_init(){
 pinMode(EN_L, OUTPUT);
 pinMode(EN_R, OUTPUT);
 pinMode(IN1_L, OUTPUT);
 pinMode(IN2_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(EN_L, LOW);
 digitalWrite(EN_R, LOW);
 digitalWrite(IN1_L, LOW);
 digitalWrite(IN2_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
}
void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, HIGH);
     digitalWrite(IN2_L, LOW);
 }
 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);
 }
 if (Pulse_Width1 == 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, LOW);
 }
}
void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);
 }
 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(Pulse_Width2);
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);
 }
 if (Pulse_Width2 == 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, LOW);
 }
}
