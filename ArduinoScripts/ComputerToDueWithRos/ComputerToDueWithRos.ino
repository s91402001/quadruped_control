//#define USE_USBCON //使用藍牙的話註解此行,使用USB連線取消註解此行
#define USE_TEENSY_HW_SERIAL
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
#include <quadruped_control/MotorAngles.h>
#define revoluteCount 3895.870773854245


//create ros node
class NewHardware : public ArduinoHardware
{
  public:
  NewHardware() :ArduinoHardware(&Serial1,115200){};
  };
  ros::NodeHandle_<NewHardware,3,1,4096,4096> nh;
//ros::NodeHandle  nh;
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

// create ros subscriber
ros::Subscriber<quadruped_control::MotorAngles> sub1("MotorSetpoint", &motor_setpoint_cb);
ros::Subscriber<std_msgs::Empty> sub2("StopMotors", &Stop_cb);
ros::Subscriber<geometry_msgs::Point> sub3("Gains", &Gain_cb);
//create ros publisher
quadruped_control::MotorAngles angles;
ros::Publisher pub1("motorAng", &angles);


void setup() {
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(pub1);
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
}

void loop() {

  //read motor angle value
  while(Serial2.available())
  {
      int data = Serial2.read();
      if(data == 0xff)
      {
        for(int index =0;index<32;index++){
          floatConvert.Bs[index] = Serial2.read();
          }
        }
    }

  angles.ang1 = floatConvert.Fs[0];
  angles.ang2 = floatConvert.Fs[1];
  angles.ang3 = floatConvert.Fs[2];
  angles.ang4 = floatConvert.Fs[3];
  angles.ang5 = floatConvert.Fs[4];
  angles.ang6 = floatConvert.Fs[5];
  angles.ang7 = floatConvert.Fs[6];
  angles.ang8 = floatConvert.Fs[8];
  Input1 =(double) floatConvert.Fs[0]; 
  Input2 =(double) floatConvert.Fs[1]; 
  Input3 =(double) floatConvert.Fs[2]; 
  Input4 =(double) floatConvert.Fs[3]; 
  Input5 =(double) floatConvert.Fs[4]; 
  Input6 =(double) floatConvert.Fs[5]; 
  Input7 =(double) floatConvert.Fs[6]; 
  Input8 =(double) floatConvert.Fs[7];    
  if(nh.connected())
  {
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
  intConvert.Is[0] = pre_output1;
  intConvert.Is[1] = pre_output2;
  intConvert.Is[2] = pre_output3;
  intConvert.Is[3] = pre_output4;
  intConvert.Is[4] = pre_output5;
  intConvert.Is[5] = pre_output6;
  intConvert.Is[6] = pre_output7;
  intConvert.Is[7] = pre_output8;
//Send PWM value
 Serial2.write(0xff);
for(int index =0;index<32;index++)
{
  Serial2.write(intConvert.Bs[index]);
  }
  }else{
   intConvert.Is[0] =0;
   intConvert.Is[1] = 0;
   intConvert.Is[2] = 0;
   intConvert.Is[3] = 0;
   intConvert.Is[4] = 0;
   intConvert.Is[5] = 0;
   intConvert.Is[6] = 0;
   intConvert.Is[7] = 0;   
//Send PWM value
 Serial2.write(0xff);
for(int index =0;index<32;index++)
{
  Serial2.write(intConvert.Bs[index]);
  }
    }
nh.spinOnce();
long now_time = millis();
if(now_time - pre_time >5 )
  {
    pub1.publish(&angles);
    pre_time = now_time;
  }   
}
