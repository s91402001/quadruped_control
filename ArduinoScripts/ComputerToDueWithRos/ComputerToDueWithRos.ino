//#define USE_USBCON //使用藍牙的話註解此行,使用USB連線取消註解此行
//#define USE_TEENSY_HW_SERIAL
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
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
long pre_time =0;

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
//MPU 6050 setting
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// create ros subscriber
//create ros publisher
geometry_msgs::Point orientation;
ros::Publisher pub1("BodyOrientation", &orientation);
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
  //
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  nh.advertise(pub1);
    Serial.println("End setup");
}

void loop() {

  //read motor angle value
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

  long now_time = millis();
  nh.spinOnce();
  if (now_time - pre_time > 100 )
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
    pub1.publish(&orientation);
    pre_time = now_time;
  }
}
