//#define USE_USBCON //使用藍牙的話註解此行,使用USB連線取消註解此行
#define ENCODER_OPTIMIZE_INTERRUPTS //最佳化encoder的中斷
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <quadruped_control/MotorAngles.h>

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


//create ros node
ros::NodeHandle  nh;
double setpoint1 = 90, setpoint2 = -90, setpoint3 = 90.0, setpoint4 = -90.0;
double setpoint5 = 0, setpoint6 = 0, setpoint7 = 0, setpoint8 = 0;
long motorPosition1 = 0, motorPosition2 = 0, motorPosition3 = 0, motorPosition4 = 0;
long motorPosition5 = 0, motorPosition6 = 0, motorPosition7 = 0, motorPosition8 = 0;
bool isStop = 0;

void motor_setpoint_cb( const  quadruped_control::MotorAngles & cmd_msg) {
  setpoint1 = (double) cmd_msg.ang1;
  setpoint2 = (double) cmd_msg.ang2;
  setpoint3 = (double) cmd_msg.ang3;
  setpoint4 = (double) cmd_msg.ang4;
  setpoint5 = (double) cmd_msg.ang5;
  setpoint6 = (double) cmd_msg.ang6;
  setpoint7 = (double) cmd_msg.ang7;
  setpoint8 = (double) cmd_msg.ang8;
}

void Stop_cb( const std_msgs::Empty& toggle_msg) {
  isStop = !isStop;
}

long pre_time =0;
double Kp1 = 400, Ki1 = 0, Kd1 = 10;
double Kp2 = 400, Ki2 = 0, Kd2 = 10;
double Kp3 = 500, Ki3 = 0, Kd3 = 15;
double Kp4 = 500, Ki4 = 0, Kd4 = 15;
double Kp5 = 400, Ki5 = 0, Kd5 = 10;
double Kp6 = 400, Ki6 = 0, Kd6 = 10;
double Kp7 = 500, Ki7 = 0, Kd7 = 25;
double Kp8 = 500, Ki8 = 0, Kd8 = 25;
double Input1 = 0, Output1 = 0, Input2 = 0, Output2 = 0, Input3 = 0, Output3 = 0, Input4 = 0, Output4 = 0;
double Input5 = 0, Output5 = 0, Input6 = 0, Output6 = 0, Input7 = 0, Output7 = 0, Input8 = 0, Output8 = 0;
double pre_output1 =0, pre_output2 =0, pre_output3 =0, pre_output4 =0, pre_output5 =0, pre_output6 =0, pre_output7 =0, pre_output8 =0;

PID motorPID1(&Input1, &Output1, &setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID motorPID2(&Input2, &Output2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID motorPID3(&Input3, &Output3, &setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID motorPID4(&Input4, &Output4, &setpoint4, Kp4, Ki4, Kd4, DIRECT);
PID motorPID5(&Input5, &Output5, &setpoint5, Kp5, Ki5, Kd5, DIRECT);
PID motorPID6(&Input6, &Output6, &setpoint6, Kp6, Ki6, Kd6, DIRECT);
PID motorPID7(&Input7, &Output7, &setpoint7, Kp7, Ki7, Kd7, DIRECT);
PID motorPID8(&Input8, &Output8, &setpoint8, Kp8, Ki8, Kd8, DIRECT);
void Gain_cb( const geometry_msgs::Point& gain_msg) {
Kp1 = gain_msg.x, Ki1 = gain_msg.y, Kd1 = gain_msg.z;
Kp2 = gain_msg.x, Ki2 = gain_msg.y, Kd2 = gain_msg.z;
Kp3 = gain_msg.x, Ki3 = gain_msg.y, Kd3 = gain_msg.z;
Kp4 = gain_msg.x, Ki4 = gain_msg.y, Kd4 = gain_msg.z;
Kp5 = gain_msg.x, Ki5 = gain_msg.y, Kd5 = gain_msg.z;
Kp6 = gain_msg.x, Ki6 = gain_msg.y, Kd6 = gain_msg.z;
Kp7 = gain_msg.x, Ki7 = gain_msg.y, Kd7 = gain_msg.z;
Kp8 = gain_msg.x, Ki8 = gain_msg.y, Kd8 = gain_msg.z;
motorPID1.SetTunings(Kp1, Ki1, Kd1);
motorPID2.SetTunings(Kp2, Ki2, Kd2);
motorPID3.SetTunings(Kp3, Ki3, Kd3);
motorPID4.SetTunings(Kp4, Ki4, Kd4);
motorPID5.SetTunings(Kp5, Ki5, Kd5);
motorPID6.SetTunings(Kp6, Ki6, Kd6);
motorPID7.SetTunings(Kp7, Ki7, Kd7);
motorPID8.SetTunings(Kp8, Ki8, Kd8);
/*
Serial2.print(Kp1);
Serial2.print('\t');
Serial2.print(Ki1);
Serial2.print('\t');
Serial2.print(Kd1);
Serial2.print('\n');
*/
}
// create ros subscriber
ros::Subscriber<quadruped_control::MotorAngles> sub1("MotorSetpoint", &motor_setpoint_cb);
ros::Subscriber<std_msgs::Empty> sub2("StopMotors", &Stop_cb);
ros::Subscriber<geometry_msgs::Point> sub3("Gains", &Gain_cb);
//create ros publisher
quadruped_control::MotorAngles angles;
//ros::Publisher pub1("motorAng", &angles);



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
  Encoder3.write((int)(revoluteCount/4.0));
  Encoder4.write((int)(-revoluteCount/4.0));
  Encoder1.write((int)(revoluteCount/4.0));
  Encoder2.write((int)(-revoluteCount/4.0));
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  //nh.advertise(pub1);
  //turn the PID on
  motorPID1.SetMode(AUTOMATIC);
  motorPID2.SetMode(AUTOMATIC);
  motorPID3.SetMode(AUTOMATIC);
  motorPID4.SetMode(AUTOMATIC);
  motorPID5.SetMode(AUTOMATIC);
  motorPID6.SetMode(AUTOMATIC);
  motorPID7.SetMode(AUTOMATIC);
  motorPID8.SetMode(AUTOMATIC);
  int resolution = 4095;
  motorPID1.SetOutputLimits(-resolution, resolution);
  motorPID2.SetOutputLimits(-resolution, resolution);
  motorPID3.SetOutputLimits(-resolution, resolution );
  motorPID4.SetOutputLimits(-resolution, resolution );
  motorPID5.SetOutputLimits(-resolution, resolution );
  motorPID6.SetOutputLimits(-resolution, resolution );
  motorPID7.SetOutputLimits(-resolution, resolution );
  motorPID8.SetOutputLimits(-resolution, resolution );
  motorPID1.SetSampleTime(10);
  motorPID2.SetSampleTime(10);
  motorPID3.SetSampleTime(10);
  motorPID4.SetSampleTime(10);
  motorPID5.SetSampleTime(10);
  motorPID6.SetSampleTime(10);
  motorPID7.SetSampleTime(10);
  motorPID8.SetSampleTime(10);

  //Serial2.begin(9600);
      
}

void loop() {
  if(nh.connected())
  {
    /*
  Serial1.print("Setpoints: \t");
  Serial1.print(setpoint1);
  Serial1.print('\t');
  Serial1.print(setpoint2);
  Serial1.print('\t');
  Serial1.print(setpoint3);
  Serial1.print('\t');
  Serial1.print(setpoint4);
  Serial1.print('\t');
  Serial1.print(setpoint5);
  Serial1.print('\t');
  Serial1.print(setpoint6);
  Serial1.print('\t');
  Serial1.print(setpoint7);
  Serial1.print('\t');
  Serial1.print(setpoint8);
  Serial1.print('\n');
  Serial1.print("Inputs: \t");
  Serial1.print(Input1);
  Serial1.print('\t');
  Serial1.print(Input2);
  Serial1.print('\t');
  Serial1.print(Input3);
  Serial1.print('\t');
  Serial1.print(Input4);
  Serial1.print('\t');
  Serial1.print(Input5);
  Serial1.print('\t');
  Serial1.print(Input6);
  Serial1.print('\t');
  Serial1.print(Input7);
  Serial1.print('\t');
  Serial1.print(Input8);  
  Serial1.print('\n');
  Serial1.print("Outputs: \t");
  Serial1.print(Output1);
  Serial1.print('\t');
  Serial1.print(Output2);
  Serial1.print('\t');
  Serial1.print(Output3);
  Serial1.print('\t');
  Serial1.print(Output4);
  Serial1.print('\t');
  Serial1.print(Output5);
  Serial1.print('\t');
  Serial1.print(Output6);
  Serial1.print('\t');
  Serial1.print(Output7);
  Serial1.print('\t');
  Serial1.print(Output8);
  Serial1.print('\n');
  Serial1.print('\n');
  */
  motorPosition1 = Encoder1.read();
  motorPosition2 = Encoder2.read();
  motorPosition3 = Encoder3.read();
  motorPosition4 = Encoder4.read();
  motorPosition5 = Encoder5.read();
  motorPosition6 = Encoder6.read();
  motorPosition7 = Encoder7.read();
  motorPosition8 = Encoder8.read();  
  Input1 = (double) (motorPosition1 / revoluteCount) * 360.0;
  Input2 = (double) (motorPosition2 / revoluteCount) * 360.0;
  Input3 = (double) (motorPosition3 / revoluteCount) * 360.0;
  Input4 = (double) (motorPosition4 / revoluteCount) * 360.0;
  Input5 = (double) (motorPosition5 / revoluteCount) * 360.0;
  Input6 = (double) (motorPosition6 / revoluteCount) * 360.0;
  Input7 = (double) (motorPosition7 / revoluteCount) * 360.0;
  Input8 = (double) (motorPosition8 / revoluteCount) * 360.0;

  angles.ang1 = Input1;
  angles.ang2 = Input2;
  angles.ang3 = Input3;
  angles.ang4 = Input4;
  angles.ang5 = Input5;
  angles.ang6 = Input6;
  angles.ang7 = Input7;
  angles.ang8 = Input8;

  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();
  motorPID5.Compute();
  motorPID6.Compute();
  motorPID7.Compute();
  motorPID8.Compute();
  pre_output1= 0.5*Output1 + 0.5*pre_output1;
  pre_output2= 0.5*Output2 + 0.5*pre_output2;
  pre_output3= 0.5*Output3 + 0.5*pre_output3;
  pre_output4= 0.5*Output4 + 0.5*pre_output4;
  pre_output5= 0.5*Output5 + 0.5*pre_output5;
  pre_output6= 0.5*Output6 + 0.5*pre_output6;
  pre_output7= 0.5*Output7 + 0.5*pre_output7;
  pre_output8= 0.5*Output8 + 0.5*pre_output8; 
  motor_control(motor1A_pin, motor1B_pin, motor1Pwm_pin, (int) pre_output1);
  motor_control(motor2A_pin, motor2B_pin, motor2Pwm_pin, (int) pre_output2);
  motor_control(motor3A_pin, motor3B_pin, motor3Pwm_pin, (int)pre_output3);
  motor_control(motor4A_pin, motor4B_pin, motor4Pwm_pin, (int)pre_output4);
  motor_control(motor5A_pin, motor5B_pin, motor5Pwm_pin, (int)pre_output5);
  motor_control(motor6A_pin, motor6B_pin, motor6Pwm_pin, (int)pre_output6);
  motor_control(motor7A_pin, motor7B_pin, motor7Pwm_pin, (int)pre_output7);
  motor_control(motor8A_pin, motor8B_pin, motor8Pwm_pin, (int)pre_output8); 
  }else{
  motor_control(motor1A_pin, motor1B_pin, motor1Pwm_pin, 0);
  motor_control(motor2A_pin, motor2B_pin, motor2Pwm_pin, 0);
  motor_control(motor3A_pin, motor3B_pin, motor3Pwm_pin, 0);
  motor_control(motor4A_pin, motor4B_pin, motor4Pwm_pin, 0);  
  motor_control(motor5A_pin, motor5B_pin, motor5Pwm_pin, 0);
  motor_control(motor6A_pin, motor6B_pin, motor6Pwm_pin, 0);
  motor_control(motor7A_pin, motor7B_pin, motor7Pwm_pin, 0);
  motor_control(motor8A_pin, motor8B_pin, motor8Pwm_pin, 0);
    }
    long now_time = micros();
    /*
    if(now_time - pre_time >200)
    {
      pub1.publish(&angles);  
      pre_time = now_time;
      }*/

nh.spinOnce();
}
