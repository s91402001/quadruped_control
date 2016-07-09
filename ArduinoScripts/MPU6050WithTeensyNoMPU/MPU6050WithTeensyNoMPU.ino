//Motor Libraries
#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#define revoluteCount 3264.0
//ROS Libraries
//#define USE_USBCON //使用藍牙的話註解此行,使用USB連線取消註解此行
#define USE_TEENSY_HW_SERIAL
#define ENCODER_OPTIMIZE_INTERRUPTS //最佳化encoder的中斷
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//create ros node
class NewHardware : public ArduinoHardware
{
  public:
    NewHardware() : ArduinoHardware(&Serial1, 115200) {};
};
ros::NodeHandle_<NewHardware, 3, 2, 4096, 1024 * 8> nh;

//Motors control variables
double setpoint =0, Input=0, Output=0,pre_output=0;
double kp = 500, kd = 10, ki = 0.1;
int pawlAng=80;
long motorPosition=0;
bool isStop = 0;
PID motorPID(&Input, &Output, &setpoint, kp, ki, kd, DIRECT);
Encoder encoder(4,5);
Servo servo1;
Servo servo2;
//=============================================================
//===                                             MOTOR CONTROL FUNCTION                         ===
//=============================================================
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
//=================================================================
//===                                          SUBSCRIBER CALLBACKS                                               ===
//=================================================================
void motor_setpoint_cb(const  std_msgs::Float32 & cmd_msg){
  setpoint = cmd_msg.data;
}
void servo_setpoint_cb(const  std_msgs::Float32 & cmd_msg){
  pawlAng = (int)cmd_msg.data;
}

// create ros subscriber
ros::Subscriber<std_msgs::Float32> SpringSub("SpringMotorSetpoint", &motor_setpoint_cb);
ros::Subscriber<std_msgs::Float32> ServoSub("ServoSetpoint", &servo_setpoint_cb);
//create ros publisher
geometry_msgs::Point orientation;
ros::Publisher OrientationPub("BodyOrientation", &orientation);
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // Motor setting
    pinMode(6,OUTPUT);//Servo1
    pinMode(7,OUTPUT);//Servo2
    pinMode(8,OUTPUT);//Motor1
    pinMode(9,OUTPUT);//Motor2
    pinMode(10,OUTPUT);//Motor PWM
    analogWriteResolution(12);
    motorPID.SetMode(AUTOMATIC);
    int resolution = 4095;
    motorPID.SetOutputLimits(-resolution, resolution);
    motorPID.SetSampleTime(10);
    servo1.attach(6);
    servo2.attach(7);
    servo1.write(0-pawlAng+155);
    servo2.write(pawlAng);
    //ROS initialize
    nh.getHardware() -> setBaud(115200);
    nh.initNode();
    nh.subscribe(SpringSub);
    nh.subscribe(ServoSub);
    nh.advertise(OrientationPub);
    Serial.println("End setup");
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
long blinkT =0;
void loop() {
        orientation.x = (float) millis();
        orientation.y = (float) 0;
        orientation.z = (float) 0;
        if(nh.connected()) {
          motorPosition = encoder.read();
          Input = (double) (motorPosition / revoluteCount) * 360.0;
          motorPID.Compute();
          pre_output= 0.5*Output + 0.5*pre_output;
          motor_control(8,9, 10, (int) pre_output);
          servo1.write(pawlAng);
          servo2.write(0-pawlAng+192);
          }else {
              motor_control(8,9, 10, 0);
          }
        // blink LED to indicate activity
        long nowT = millis();
        if(nowT-blinkT > 200){
          blinkState = !blinkState;
          blinkT=nowT;
        }
        nh.spinOnce();
        OrientationPub.publish(&orientation);
        digitalWrite(LED_PIN, blinkState);
}
