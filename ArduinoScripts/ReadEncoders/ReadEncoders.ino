#define ENCODER_OPTIMIZE_INTERRUPTS //最佳化encoder的中斷
#include <Encoder.h>

#define encoder1_pin1 22
#define encoder1_pin2 24
#define encoder2_pin1 26
#define encoder2_pin2 28
#define encoder5_pin1 34
#define encoder5_pin2 36
#define encoder6_pin1 30
#define encoder6_pin2 32

#define encoder3_pin1 23
#define encoder3_pin2 25
#define encoder4_pin1 27
#define encoder4_pin2 29
#define encoder7_pin1 31
#define encoder7_pin2 33
#define encoder8_pin1 35
#define encoder8_pin2 37

#define motor1A_pin 38
#define motor1B_pin 40
#define motor2A_pin 42
#define motor2B_pin 44
#define motor5A_pin 46
#define motor5B_pin 48
#define motor6A_pin 50
#define motor6B_pin 52

#define motor3A_pin 39
#define motor3B_pin 41
#define motor4A_pin 43
#define motor4B_pin 45
#define motor7A_pin 47
#define motor7B_pin 49
#define motor8A_pin 51
#define motor8B_pin 53

#define motor1Pwm_pin 2
#define motor2Pwm_pin 3
#define motor3Pwm_pin 4
#define motor4Pwm_pin 5
#define motor5Pwm_pin 6
#define motor6Pwm_pin 7
#define motor7Pwm_pin 8
#define motor8Pwm_pin 9

#define revoluteCount 3895.870773854245



int pwm1 =0, pwm2 =0, pwm3 =0, pwm4 =0, pwm5 =0, pwm6 =0, pwm7 =0, pwm8 =0;
long motorPosition1 = 0, motorPosition2 = 0, motorPosition3 = 0, motorPosition4 = 0;
long motorPosition5 = 0, motorPosition6 = 0, motorPosition7 = 0, motorPosition8 = 0;
bool isStop = 0;

float Ang1 = 0, Ang2 = 0, Ang3 = 0, Ang4 = 0;
float Ang5 = 0, Ang6 = 0, Ang7 = 0, Ang8 = 0;


Encoder Encoder1(encoder1_pin1, encoder1_pin2);
Encoder Encoder2(encoder2_pin1, encoder2_pin2);
Encoder Encoder3(encoder3_pin1, encoder3_pin2);
Encoder Encoder4(encoder4_pin1, encoder4_pin2);
Encoder Encoder5(encoder5_pin1, encoder5_pin2);
Encoder Encoder6(encoder6_pin1, encoder6_pin2);
Encoder Encoder7(encoder7_pin1, encoder7_pin2);
Encoder Encoder8(encoder8_pin1, encoder8_pin2);




void motor_control(int pin1, int pin2, int pwmPin, int pwm) {
  if (!isStop) {
    if (pwm > 0) {
      digitalWrite(pin1, 0);
      digitalWrite(pin2, 1);
    }
    else if (pwm < 0) {
      digitalWrite(pin1, 1);
      digitalWrite(pin2, 0);
    } else {
      digitalWrite(pin1, 0);
      digitalWrite(pin2, 0);
    }
    analogWrite(pwmPin, abs(pwm));
  }
  else {
    digitalWrite(pin1, 0);
    digitalWrite(pin2, 0);
    analogWrite(pwmPin, 0);
  }
}

union FloatsVsBytes{
    float Fs[8];
    byte Bs[32];
  };

union IntsVsBytes{
    float Is[8];
    byte Bs[32];
  };

FloatsVsBytes floatConvert;
IntsVsBytes intConvert;

void setup() {
  pinMode(motor1A_pin, OUTPUT);
  pinMode(motor1B_pin, OUTPUT);
  pinMode(motor2A_pin, OUTPUT);
  pinMode(motor2B_pin, OUTPUT);
  pinMode(motor3A_pin, OUTPUT);
  pinMode(motor3B_pin, OUTPUT);
  pinMode(motor4A_pin, OUTPUT);
  pinMode(motor4B_pin, OUTPUT);
  pinMode(motor5A_pin, OUTPUT);
  pinMode(motor5B_pin, OUTPUT);
  pinMode(motor6A_pin, OUTPUT);
  pinMode(motor6B_pin, OUTPUT);
  pinMode(motor7A_pin, OUTPUT);
  pinMode(motor7B_pin, OUTPUT);
  pinMode(motor8A_pin, OUTPUT);
  pinMode(motor8B_pin, OUTPUT);  
  motor_control(motor1A_pin, motor1B_pin, motor1Pwm_pin, 0);
  motor_control(motor2A_pin, motor2B_pin, motor2Pwm_pin, 0);
  motor_control(motor3A_pin, motor3B_pin, motor3Pwm_pin, 0);
  motor_control(motor4A_pin, motor4B_pin, motor4Pwm_pin, 0);
  motor_control(motor5A_pin, motor5B_pin, motor5Pwm_pin, 0);
  motor_control(motor6A_pin, motor6B_pin, motor6Pwm_pin, 0);
  motor_control(motor7A_pin, motor7B_pin, motor7Pwm_pin, 0);
  motor_control(motor8A_pin, motor8B_pin, motor8Pwm_pin, 0);
  analogWriteResolution(12);    
  Encoder1.write((int)(revoluteCount/4.0));
  Encoder2.write((int)(-revoluteCount/4.0));
  Encoder3.write((int)(revoluteCount/4.0));
  Encoder4.write((int)(-revoluteCount/4.0));
  Serial2.begin(115200);
}

void loop() {
  motorPosition1 = Encoder1.read();
  motorPosition2 = Encoder2.read();
  motorPosition3 = Encoder3.read();
  motorPosition4 = Encoder4.read();
  motorPosition5 = Encoder5.read();
  motorPosition6 = Encoder6.read();
  motorPosition7 = Encoder7.read();
  motorPosition8 = Encoder8.read();
  Ang1 = (motorPosition1 / revoluteCount) * 360.0;
  Ang2 = (motorPosition2 / revoluteCount) * 360.0;
  Ang3 = (motorPosition3 / revoluteCount) * 360.0;
  Ang4 = (motorPosition4 / revoluteCount) * 360.0;
  Ang5 = (motorPosition5 / revoluteCount) * 360.0;
  Ang6 = (motorPosition6 / revoluteCount) * 360.0;
  Ang7 = (motorPosition7 / revoluteCount) * 360.0;
  Ang8 = (motorPosition8 / revoluteCount) * 360.0;
  floatConvert.Fs[0] = Ang1;
  floatConvert.Fs[1] = Ang2;
  floatConvert.Fs[2] = Ang3;
  floatConvert.Fs[3] = Ang4;
  floatConvert.Fs[4] = Ang5;
  floatConvert.Fs[5] = Ang6;
  floatConvert.Fs[6] = Ang7;
  floatConvert.Fs[7] = Ang8;
  //send motor angle
Serial2.write(0xff);
for(int index =0;index<32;index++)
{
  Serial2.write(floatConvert.Bs[index]);
  }
  //read pwn value
  while(Serial2.available())
  {
      int data = Serial2.read();
      if(data == 0xff)
      {
        for(int index =0;index<32;index++){
          intConvert.Bs[index] = Serial2.read();
          }
        }
    }
  motor_control(motor1A_pin, motor1B_pin, motor1Pwm_pin, intConvert.Is[0]);
  motor_control(motor2A_pin, motor2B_pin, motor2Pwm_pin, intConvert.Is[1]);
  motor_control(motor3A_pin, motor3B_pin, motor3Pwm_pin, intConvert.Is[2]);
  motor_control(motor4A_pin, motor4B_pin, motor4Pwm_pin, intConvert.Is[3]);
  motor_control(motor5A_pin, motor5B_pin, motor5Pwm_pin, intConvert.Is[4]);
  motor_control(motor6A_pin, motor6B_pin, motor6Pwm_pin, intConvert.Is[5]);
  motor_control(motor7A_pin, motor7B_pin, motor7Pwm_pin, intConvert.Is[6]);
  motor_control(motor8A_pin, motor8B_pin, motor8Pwm_pin, intConvert.Is[7]); 
  
}
