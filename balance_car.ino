/* 
* 代码头文件
*/
#include <Wire.h>
#include <MPU6050_tockn.h>
// left pwm 补偿: 30 right pwm 补偿: 30
// init mpu6050
MPU6050 mpu6050(Wire);


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

int pwm = 0;

// 定义电机的方式,输入参数是两个 pwm 信号.
void motor(int left_EN, int right_EN) {
  if (left_EN == 0) {
    analogWrite(EN_PWMA, 0);
  }

  if (left_EN < 0) {
    digitalWrite(EN_DIRA1, LOW);
    digitalWrite(EN_DIRA2, HIGH);
    analogWrite(EN_PWMA, 0 - left_EN);
  }
  if (left_EN > 0) {
    digitalWrite(EN_DIRA1, HIGH);
    digitalWrite(EN_DIRA2, LOW);
    analogWrite(EN_PWMA, left_EN);
  }
  if (right_EN == 0) {
    analogWrite(EN_PWMB, 0);
  }
  if (right_EN < 0) {
    digitalWrite(EN_DIRB1, LOW);
    digitalWrite(EN_DIRB2, HIGH);
    analogWrite(EN_PWMB, 0 - right_EN);
  }
  if (right_EN > 0) {
    digitalWrite(EN_DIRB1, HIGH);
    digitalWrite(EN_DIRB2, LOW);
    analogWrite(EN_PWMB, right_EN);
  }
}

// 串口调试
void serial_debug() {
  if (Serial.available() > 0) {
    delay(5);
    char DATA = Serial.read();
    switch (DATA) {
      case '1':
        pwm++;
        Serial.print("PWM: ");
        Serial.println(pwm);
        break;
      case '0':
        pwm--;
        Serial.print("PWM: ");
        Serial.println(pwm);
        break;
    }
  }
}

void init_car() {
  pinMode(EN_DIRA1, OUTPUT);
  pinMode(EN_DIRA2, OUTPUT);
  pinMode(EN_DIRB1, OUTPUT);
  pinMode(EN_DIRB2, OUTPUT);
}


void setup() {
  init_car();
  Serial.begin(9600);
  // Wire.begin();
  // mpu6050.
  //   // 设置MPU6050的陀螺仪满量程范围
  //   mpu6050.setFullScaleGyroRange(GYRO_FS_SEL);
  // delay(5);
  // Serial.begin(9600);
  // pinMode(BEEP, OUTPUT);
  // digitalWrite(BEEP, HIGH);
  // delay(1000);
  // digitalWrite(BEEP, LOW);
  // Serial.println("BEE;P test ok!");
  // Serial.println("Car init....OK");

  // if (mpu6050.testConnection()) {
  //   Serial.println("MPU6050 connection successful");
  // } else {
  //   Serial.println("MPU6050 connection failed");
  // }
}




void loop() {
  serial_debug();
   motor(pwm, pwm);
   
}

// void car_move(int dir, int pwma, int pwmb, int interval) {
//   for (int i = 0; i < interval; i++) {
//     if (dir > 0) {
//       digitalWrite(EN_DIRA1, HIGH);
//       digitalWrite(EN_DIRA2, LOW);
//       analogWrite(EN_PWMA, pwma);
//       digitalWrite(EN_DIRB1, HIGH);
//       digitalWrite(EN_DIRB2, LOW);
//       analogWrite(EN_PWMB, pwmb);
//       // Serial.println("forward");
//       delay(10);
//     }

//     if (dir < 0) {
//       digitalWrite(EN_DIRA1, LOW);
//       digitalWrite(EN_DIRA2, HIGH);
//       analogWrite(EN_PWMA, pwma);
//       digitalWrite(EN_DIRB1, LOW);
//       digitalWrite(EN_DIRB2, HIGH);
//       analogWrite(EN_PWMB, pwmb);
//       delay(10);
//       // Serial.println("Backward");
//     }
//   }
// }
