#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// 实例化一个 mpu6050 陀螺仪
MPU6050 mpu6050(Wire);

// 定义引脚
#define BEEP 45
#define RGB 44
#define SOUND A1
#define TEMP A0
#define TX0 1
#define RX0 0
#define TX2 16
#define RX2 17

// motor
#define EN_PWMB 10
#define EN_PWMA 11

#define EN_DIRA2 49
#define EN_DIRA1 48

#define EN_DIRB1 47
#define EN_DIRB2 46

// 编码器
#define ENB_A 18
#define ENB_B 43

#define ENA_A 19
#define ENA_B 42

// led 灯
#define LIGHT2 A2
#define LIGHT1 A3

// 陀螺仪接的 I2C
#define scl_pin 21
#define sda_pin 20

// 定义初始化参数
float Balance_Angle_raw = -1.4;
const int leftMotorPwmOffset = 13;
const int rightMotorPwmOffset = 13;

int z_turn_spd = 60;
float ENERGY = 6; 
float kp = 18, ki = 0.6, kd = 0.8;
float angle_kp = -2; 

float Keep_Angle, bias, integrate;
float AngleX, AngleY, AngleZ, GyroX, GyroY, GyroZ;
int vertical_PWM, angle_PWM, PWM, L_PWM, R_PWM;

char flag = 's';

SoftwareSerial Bt(RX0, TX0);

void motor(int left_EN, int right_EN){
    if (left_EN == 0){
      analogWrite(EN_PWMA, 0);
    }
    if (right_EN == 0){
      analogWrite(EN_PWMB, 0);
    }
    if (left_EN < 0){
      digitalWrite(EN_DIRA1, LOW);
      digitalWrite(EN_DIRA2, HIGH);
      analogWrite(EN_PWMA, 0 - left_EN);
    }
    if (right_EN < 0) {
      digitalWrite(EN_DIRB1, LOW);
      digitalWrite(EN_DIRB2, HIGH);
      analogWrite(EN_PWMB, 0 - right_EN);
    }
    if (left_EN > 0){
      digitalWrite(EN_DIRA1, HIGH);
      digitalWrite(EN_DIRA2, LOW);
      analogWrite(EN_PWMA, left_EN);
    }
    if (right_EN > 0){
      digitalWrite(EN_DIRB1, HIGH);
      digitalWrite(EN_DIRB2, LOW);
      analogWrite(EN_PWMB, right_EN); 
    }
}

void serial_debug()
{
  if (Bt.available() > 0){
    char DATA = Bt.read();
    delay(5);
    switch (DATA)
    {
      case 'u':
        Keep_Angle += 0.1;
        break;
      case 'd':
        Keep_Angle -= 0.1;
        break; // 调节机械平衡点, u 前倾 , b 后仰 

     /* 调试直立环 */ 
     case  '0':
        kp -= 1;
        break;
      case '1':
        kp += 1;
        break;
      case '2':
        ki -= 0.1; 
        break;
      case '3':
        ki += 0.1;
        break;
      case '4':
        kd -= 0.1;
        break;
      case '5':
        kd += 0.1;
        break;
      case '6':
        angle_kp -= 0.2;
        break;
      case '7':
        angle_kp += 0.2;
        break;  // 转向环比例
      case 's':
        flag = 's';
        Keep_Angle = Balance_Angle_raw;
        z_turn_spd = 0;
        break;
      case 'a':
         flag = 'a';  // 前进
         Keep_Angle = Balance_Angle_raw + ENERGY;
         z_turn_spd = 0;
         break;
      case 'b':
         flag = 'b';
         Keep_Angle = Balance_Angle_raw - ENERGY;
         z_turn_spd = 0;
         break;
      case 'r':
          flag = 'r';
          z_turn_spd = -60;
          break;
      case 'l':
          flag = 'l';
          z_turn_spd = 60;
          break;
    }

    if (kp < 0)
      kp = 0;
    if (ki < 0)
      ki = 0;
    if (kd < 0)
      kd = 0;

    Bt.print("Keep_Angle: ");
    Bt.println(Keep_Angle);
    Bt.print("kp: ");
    Bt.print(kp);
    Bt.print("\tki: ");
    Bt.print(ki);
    Bt.print("\tkd: ");
    Bt.print(kd);
    Bt.print("\tangle_kp: ");
    Bt.println(angle_kp);
    Bt.println("-----------------------------------------------------");

  }
}

void angle_pwm_calculation(){
    GyroZ = mpu6050.getGyroZ();  // 获取陀螺仪角速度
    angle_PWM = angle_kp * (GyroZ - z_turn_spd);
    angle_PWM = constrain(angle_PWM, -130, 130);
}


void vertical_pwm_calculation(){
  AngleX=  mpu6050.getAngleX();
  GyroX = mpu6050.getGyroX();
  bias = AngleX - Keep_Angle; // 计算角度偏差
  integrate += bias;  //偏差的积分, integrate 为全局变量,一直积累.
  integrate = constrain(integrate, -1000, 1000); // 限定误差积分的最大和最小值
   /* 直立环 PID计算 PWM 通过陀螺仪返回数据计算,前倾陀螺仪Y 轴为正, 后仰陀螺仪 Y 轴为负, 前倾车前进,后仰车后退,保持直立. */ 
   vertical_PWM = kp * bias + ki * integrate + kd * GyroX; 
}


void motor_control(){
   /* 补偿右轮偏差 */ 
   if (PWM > 0) {
    L_PWM = PWM + leftMotorPwmOffset; 
    R_PWM = PWM + rightMotorPwmOffset;
   }
   if (PWM < 0){
    L_PWM = PWM - leftMotorPwmOffset;
    R_PWM = PWM - rightMotorPwmOffset;
   }
   if (PWM = 0){
    L_PWM = 0;
    R_PWM = 0;
   }

  /* 转向判断 */  
  if (flag == 's'){
     L_PWM = L_PWM;
     R_PWM = R_PWM; 
  }
  if (flag == 'l'){
    L_PWM += angle_PWM;
    R_PWM -= angle_PWM; 
  }
  if (flag == 'r'){
    L_PWM += angle_PWM;
    R_PWM -= angle_PWM; 
  }
  L_PWM = constrain(L_PWM, -255, 255);
  R_PWM = constrain(R_PWM, -255, 255);

  motor(L_PWM, R_PWM); 

  if (AngleX > 50 || AngleX < -50){
     motor(0, 0);  //停车
  }
}

void setup() {
  Serial.begin(9600);
  Bt.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Keep_Angle = Balance_Angle_raw;
  motor(0, 0);
  delay(50); 
}

void loop() {
   /* 串口调试 PID + 控制 */ 
   serial_debug();

  /* 陀螺仪刷新 */ 
  mpu6050.update();

  // Serial.print("angleX : ");
  // Serial.print(mpu6050.getAngleX());
  /* PWM 计算 */
  angle_pwm_calculation();  // 转向环计算 PWM 值
  vertical_pwm_calculation(); // 直立环PWM 计算

  PWM = vertical_PWM; 
  
  /* 马达输出 */
  motor_control(); 
  // Serial.print("\tangleY : ");
  // Serial.println(mpu6050.getAngleY());
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());
}
