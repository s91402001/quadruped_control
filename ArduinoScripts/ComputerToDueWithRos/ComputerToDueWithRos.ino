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
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include <i2c_t3.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = true;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

//create ros node
class NewHardware : public ArduinoHardware
{
  public:
    NewHardware() : ArduinoHardware(&Serial1, 115200) {};
};
ros::NodeHandle_<NewHardware, 3, 2, 4096, 1024 * 8> nh;
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

long pre_time = 0;
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
double pre_output1 = 0, pre_output2 = 0, pre_output3 = 0, pre_output4 = 0, pre_output5 = 0, pre_output6 = 0, pre_output7 = 0, pre_output8 = 0;

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

union FloatsVsBytes {
  float Fs[8];
  byte Bs[32];
};

union IntsVsBytes {
  int Is[8];
  byte Bs[32];
};

FloatsVsBytes floatConvert;
IntsVsBytes intConvert;

//MPU 6050 setting
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// create ros subscriber
ros::Subscriber<quadruped_control::MotorAngles> sub1("MotorSetpoint", &motor_setpoint_cb);
ros::Subscriber<std_msgs::Empty> sub2("StopMotors", &Stop_cb);
ros::Subscriber<geometry_msgs::Point> sub3("Gains", &Gain_cb);
//create ros publisher
quadruped_control::MotorAngles angles;
geometry_msgs::Point orientation;
ros::Publisher pub1("motorAng", &angles);
ros::Publisher pub2("BodyOrientation", &orientation);
double gyro_LSB = 65.5;
double Xang = 0.0, Yang = 0.0, Zang = 0.0;
void setup() {
  Serial.begin(9600);
  //MPU 6050 setting
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(1);//65.5 = 1 degree/s   range = +- 500 degree/s
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  devStatus = accelgyro.dmpInitialize();
  accelgyro.setXGyroOffset(149.8 / 2.0);
  accelgyro.setYGyroOffset(-24.02 / 2.0);
  accelgyro.setZGyroOffset(20.17 / 2.0);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    accelgyro.setDMPEnabled(true);
    mpuIntStatus = accelgyro.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }


  pinMode(20, INPUT);
  Serial.println("begin setup");

  Serial2.begin(115200);
  //
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

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


  Serial.println("End setup");
}

void loop() {

  //read motor angle value
  if(Serial2.available())
  {
    int data = Serial2.read();
    if (data == 0xff)
    {
      for (int index = 0; index < 32; index++) {
        floatConvert.Bs[index] = Serial2.read();
      }
    }
  }
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();

  // get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /* Serial.print("ypr\t");
     Serial.print(ypr[0] * 180/M_PI);
     Serial.print("\t");
     Serial.print(ypr[1] * 180/M_PI);
     Serial.print("\t");
     Serial.println(ypr[2] * 180/M_PI);*/
  }

  angles.ang1 = floatConvert.Fs[0];
  angles.ang2 = floatConvert.Fs[1];
  angles.ang3 = floatConvert.Fs[2];
  angles.ang4 = floatConvert.Fs[3];
  angles.ang5 = floatConvert.Fs[4];
  angles.ang6 = floatConvert.Fs[5];
  angles.ang7 = floatConvert.Fs[6];
  angles.ang8 = floatConvert.Fs[8];
  Input1 = (double) floatConvert.Fs[0];
  Input2 = (double) floatConvert.Fs[1];
  Input3 = (double) floatConvert.Fs[2];
  Input4 = (double) floatConvert.Fs[3];
  Input5 = (double) floatConvert.Fs[4];
  Input6 = (double) floatConvert.Fs[5];
  Input7 = (double) floatConvert.Fs[6];
  Input8 = (double) floatConvert.Fs[7];
  if (nh.connected())
  {
    motorPID1.Compute();
    motorPID2.Compute();
    motorPID3.Compute();
    motorPID4.Compute();
    motorPID5.Compute();
    motorPID6.Compute();
    motorPID7.Compute();
    motorPID8.Compute();
    pre_output1 = 0.5 * Output1 + 0.5 * pre_output1;
    pre_output2 = 0.5 * Output2 + 0.5 * pre_output2;
    pre_output3 = 0.5 * Output3 + 0.5 * pre_output3;
    pre_output4 = 0.5 * Output4 + 0.5 * pre_output4;
    pre_output5 = 0.5 * Output5 + 0.5 * pre_output5;
    pre_output6 = 0.5 * Output6 + 0.5 * pre_output6;
    pre_output7 = 0.5 * Output7 + 0.5 * pre_output7;
    pre_output8 = 0.5 * Output8 + 0.5 * pre_output8;
    intConvert.Is[0] = (int)pre_output1;
    intConvert.Is[1] =  (int)pre_output2;
    intConvert.Is[2] =  (int)pre_output3;
    intConvert.Is[3] = (int) pre_output4;
    intConvert.Is[4] =  (int)pre_output5;
    intConvert.Is[5] = (int) pre_output6;
    intConvert.Is[6] =  (int)pre_output7;
    intConvert.Is[7] =  (int)pre_output8;
    //Send PWM value
    Serial2.write(0xff);
    for (int index = 0; index < 32; index++)
    {
      Serial2.write(intConvert.Bs[index]);
    }
  } else {
    intConvert.Is[0] = 0;
    intConvert.Is[1] = 0;
    intConvert.Is[2] = 0;
    intConvert.Is[3] = 0;
    intConvert.Is[4] = 0;
    intConvert.Is[5] = 0;
    intConvert.Is[6] = 0;
    intConvert.Is[7] = 0;
    //Send PWM value
    Serial2.write(0xff);
    for (int index = 0; index < 32; index++)
    {
      Serial2.write(intConvert.Bs[index]);
    }
  }
  long now_time = millis();
  nh.spinOnce();
  if (now_time - pre_time > 200 )
  {
    /*
    }
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double delta_time = (double)(now_time-pre_time)/1000.0;
    Xang += (double)gx/gyro_LSB*delta_time;
    Yang += (double)gy/gyro_LSB*delta_time;
    Zang += (double)gz/gyro_LSB*delta_time;*/
    orientation.x = (float) (ypr[0] * 180 / M_PI);
    orientation.y = (float) (ypr[1] * 180 / M_PI);
    orientation.z = (float) (ypr[2] * 180 / M_PI);
    Serial.print(ypr[0] * 180 / M_PI); Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI); Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    pub1.publish(&angles);

    pub2.publish(&orientation);
    pre_time = now_time;
  }
}
