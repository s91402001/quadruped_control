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
//MPU6050 Libraries
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer



// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
int pawlAng=47;
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
    //MPU setting
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(149.8 / 2.0);
    mpu.setYGyroOffset(-24.02 / 2.0);
    mpu.setZGyroOffset(20.17 / 2.0);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready!"));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
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
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    //get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        orientation.x = (float) (ypr[0] * 180 / M_PI);
        orientation.y = (float) (ypr[1] * 180 / M_PI);
        orientation.z = (float) (ypr[2] * 180 / M_PI);
        /*Serial.print("ypr\t");
        Serial.print(orientation.x);
        Serial.print("\t");
        Serial.print(orientation.y);
        Serial.print("\t");
        Serial.println(orientation.z);
        Serial.println(setpoint);
        Serial.println(motorPosition);*/
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
}
