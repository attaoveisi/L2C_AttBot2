#include <Arduino.h>
//#include <BasicLinearAlgebra.h>
//#include <math.h>
//#include <EEPROM.h>
//#include <string.h>
#include <Time.h>
//#include <DFRobot_sim808.h>
//#include <ArduinoLog.h>
//#include <SoftwareSerial.h>
#include <QMC5883L.h>

// ROS includes
//#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/tf.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

// OLD VERSION:
// Frame names: wheel speed 
//char base_link[] = "base_link";
//char odom[]      = "odom";
// char base_footprint[]      = "base_footprint";
// nav_msgs::Odometry odomMsg;
// ros::Publisher odom("odom", &odomMsg);
// geometry_msgs::TransformStamped t;
// tf::TransformBroadcaster tfBroadcaster;



sensor_msgs::NavSatFix gpsMsg;
ros::Publisher gps("gps", &gpsMsg);

// // Publisher object for IMU_Raw
// std_msgs::String imumsg_raw;
// ros::Publisher imu_raw("imu_raw", &imumsg_raw);
// ros::NodeHandle nh_imu_raw;

// Publisher object for filtered IMU
sensor_msgs::Imu imumsg_filtered;
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
float orientation_covariance[9];
float angular_velocity_covariance[9];
float linear_acceleration_covariance[9];
ros::Publisher imu_data("imu_data", &imumsg_filtered);

// #include <SD.h>
// #include <SPI.h>
// File Logger;
// int pinCS = 53; // SD card digital pin
// #define Logger_Sampling_Time_ms 1000
// unsigned long currentMillis_SD = 0;
// unsigned long previousMillis_SD = 0;
// String Logger_file_name = "Logged_";
// char *Logger_file_name_ptr, Logger_file_name_char;

int Logger_name_year;
int Logger_name_month;
int Logger_name_day;
int Logger_name_hour;
int Logger_name_min;
int Logger_name_second;

// Compass setup
QMC5883L compass;
int heading_compass = 0;

#define GSM_MESSAGE_LENGTH 160
char GSM_message[GSM_MESSAGE_LENGTH];
int GSM_messageIndex = 0;
char GSM_MESSAGE[300];
char GSM_lat[12];
char GSM_lon[12];
char GSM_wspeed[12];
char GSM_heading[12];
#define GSM_phone "015758752522"
char GSM_datetime[24];
#define PIN_TX 15
#define PIN_RX 14
// SoftwareSerial mySerial(PIN_TX,PIN_RX);
//The content of messages sent
//#define GSM_Initial_MESSAGE  "Hello Master, This is AttBot at your service. Tell me what to do!"
// DFRobot_SIM808 sim808(&mySerial);//Connect RX,TX,PWR,

// uncomment "OUTPUT_READABLE_GPS_GSM" if you want to see the GPS data sent to you per Msg
//#define OUTPUT_READABLE_GPS_GSM

float GPS_la = 0.0;
float GPS_lo = 0.0;
float GPS_ws = 0.0;
float GPS_alt = 0.0;
float GPS_heading = 0.0;
uint16_t GPS_year = 0;
uint8_t GPS_month = 0;
uint8_t GPS_day = 0;
uint8_t GPS_hour = 0;
uint8_t GPS_minute = 0;
uint8_t GPS_second = 0;
uint8_t GPS_centisecond = 0;
#define GSM_Sampling_Time_ms 100000
unsigned long currentMillis_GSM = 0;
unsigned long previousMillis_GSM = 0;
#define GPS_Sampling_Time_ms 500
unsigned long currentMillis_GPS = 0;
unsigned long previousMillis_GPS = 0;

#define ENCODEROUTPUT 12 // Please insert your motor encoder output pulse per rotation
#define HALLSEN_FR_A 11 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_FL_A 10 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RR_A 9 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RL_A 8 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)

volatile long encoderValue_FR = 0;
volatile long encoderValue_FL = 0;
volatile long encoderValue_RR = 0;
volatile long encoderValue_RL = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
float rpm_FR = 0.0;
float rpm_FL = 0.0;
float rpm_RR = 0.0;
float rpm_RL = 0.0;
float vx_FR = 0.0;
float vx_FL = 0.0;
float vx_RR = 0.0;
float vx_RL = 0.0;
float v_right = 0.0;
float v_left = 0.0;
float vx = 0.0;
float vy = 0.0;
float vth = 0.0;
float dt = 0.0;
float delta_x = 0.0;
float delta_y = 0.0;
float delta_th = 0.0;
float x_driven = 0.0;
float y_driven = 0.0;
float th_driven = 0.0;
float lengthBetweenTwoWheels = 130/1000; //130 mm

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float ax_raw_SI, ay_raw_SI, az_raw_SI;
float gx_raw_SI, gy_raw_SI, gz_raw_SI;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
#include "MPU6050_6Axis_MotionApps20.h"

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 gyroReal;   // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float aa_SI[3];         // [x, y, z]            accel sensor measurements
float aaReal_SI[3];     // [x, y, z]            gravity-free accel sensor measurements
float gyroReal_SI[3];   // [x, y, z]            Gyro sensor measurements
float aaWorld_SI[3];    // [x, y, z]            world-frame accel sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void updateEncoder_FR()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_FR++;
}

void updateEncoder_FL()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_FL++;
}

void updateEncoder_RR()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_RR++;
}

void updateEncoder_RL()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_RL++;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication
  // mySerial.begin(57600);
  //Serial.begin(57600);     //open serial and set the baudrate

  compass.init();
  compass.setSamplingRate(50);

  //Connect to ROS
  nh.initNode();

  // OLD VERSION: Advertise odometry and transform
  // nh.advertise(odom);
  // tfBroadcaster.init(nh);

  

  // // Advertise imu raw data  
  // nh.advertise(imu_raw);

  //advertise GPS  data
  nh.advertise(gps);

  //advertise IMU filtered data
  nh.advertise(imu_data);

  // Logger_name_year = year();
  // Logger_name_month = month();
  // Logger_name_day = day();
  // Logger_name_hour = hour();
  // Logger_name_min = minute();
  // Logger_name_second = second();
  // String Logger_name_year_str = String(Logger_name_year);
  // Logger_file_name += Logger_name_year_str;
  // String Logger_name_month_str = String(Logger_name_month);
  // Logger_file_name += Logger_name_month_str;
  // String Logger_name_day_str = String(Logger_name_day);
  // Logger_file_name += Logger_name_day_str;
  // String Logger_name_hour_str = String(Logger_name_hour);
  // Logger_file_name += Logger_name_hour_str;
  // String Logger_name_min_str = String(Logger_name_min);
  // Logger_file_name += Logger_name_min_str;
  // String Logger_name_second_str = String(Logger_name_second);
  // Logger_file_name += Logger_name_second_str;
  // Logger_file_name += ".txt";
  // Logger_file_name_ptr = &Logger_file_name_char;
  // Logger_file_name.toCharArray(Logger_file_name_ptr, 50);
  //Serial.println(Logger_file_name_char);

  // pinMode(pinCS, OUTPUT);
  
  // SD Card Initialization
  // if (SD.begin())
  // {
  //   Serial.println("SD card is ready to use.");
  // } else
  // {
  //   Serial.println("SD card initialization failed");
  //   return;
  // }

  //******** Initialize sim808 module *************
  // while(!sim808.init())
  // {
  //     Serial.print("Sim808 init error\r\n");
  //     delay(1000);
  // }
  // delay(3000);

  // if( sim808.attachGPS())
  //     Serial.println("Open the GPS power success");
  // else 
  //     Serial.println("Open the GPS power failure");
      
  // Serial.println("Init Success, please send SMS message to me!");

  //******** test phone number and text **********
  // sim808.sendSMS(GSM_phone,GSM_Initial_MESSAGE);

    // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  pinMode(HALLSEN_FR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_FL_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RL_A, INPUT_PULLUP); // Set hall sensor A as input pullup

  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FR_A), updateEncoder_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FL_A), updateEncoder_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RR_A), updateEncoder_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RL_A), updateEncoder_RL, RISING);

  // Serial.print("\n\n");
  // Serial.println("Measuring DC Motor's RPM");

  // join I2C bus (I2Cdev library doesn't do this automatically)
   
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  // get raw IMU data

  // Compass readings
  //heading_compass = compass.readHeading();

  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    // Revolutions per minute (RPM) = (total encoder pulse in 1s / motor encoder output) x 60s
    rpm_FR = (float)(encoderValue_FR * 60 / ENCODEROUTPUT);
    rpm_FL = (float)(encoderValue_FL * 60 / ENCODEROUTPUT);
    rpm_RR = (float)(encoderValue_RR * 60 / ENCODEROUTPUT);
    rpm_RL = (float)(encoderValue_RL * 60 / ENCODEROUTPUT);
    vx_FR = rpm_FR*2*PI/60*65/2.0f/1000.0f;
    vx_FL = rpm_FL*2*PI/60*65/2.0f/1000.0f;
    vx_RR = rpm_RR*2*PI/60*65/2.0f/1000.0f;
    vx_RL = rpm_RL*2*PI/60*65/2.0f/1000.0f;
    v_right = (vx_FR+vx_RR)/2.0;
    v_left = (vx_FL+vx_RL)/2.0;
    vx = (v_right + v_left) /2.0;
    vy = 0;
    vth = ((v_right - v_left)/lengthBetweenTwoWheels);
    dt = currentMillis - previousMillis;
    delta_x = (vx * cos(th_driven)) * dt;
    delta_y = (vx * sin(th_driven)) * dt;
    delta_th = vth * dt;
    x_driven += delta_x;
    y_driven += delta_y;
    th_driven += delta_th;
    while (th_driven > PI) {
      th_driven -= 2.0 * PI;
    }
    while (th_driven < -PI) {
      th_driven += 2.0 * PI;
    }
  
    previousMillis = currentMillis;
    // Serial.print('Front right wheel: ');
    //ySerial.println(rpm_FR);
    // Serial.print(" RPM \n");
    // Serial.print('Front left wheel: ');
    //Serial.println(rpm_FL);
    // Serial.print(" RPM \n");
    // Serial.print('Rear right wheel: ');
    //Serial.println(rpm_RR);
    // Serial.print(" RPM \n");
    // Serial.print('Rear left wheel: ');
    //Serial.println(rpm_RL);
    // Serial.print(" RPM \n");
    //Serial.println("----");

    // Reset the encoders 
    encoderValue_FR = 0;
    encoderValue_FL = 0;
    encoderValue_RR = 0;
    encoderValue_RL = 0;
  }

  // if programming failed, don't try to do anything
  // if (!dmpReady) {
  //   return;
  // }

  // read a packet from FIFO
  if (1) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_QUATERNION
          // display quaternion values in easy matrix form: w x y z
          // mpu.dmpGetQuaternion(&q, fifoBuffer);
          // Serial.print("quat\t");
          // Serial.print(q.w);
          // Serial.print("\t");
          // Serial.print(q.x);
          // Serial.print("\t");
          // Serial.print(q.y);
          // Serial.print("\t");
          // Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
          // display Euler angles in degrees
          // mpu.dmpGetQuaternion(&q, fifoBuffer);
          // mpu.dmpGetEuler(euler, &q);
          // Serial.print("euler\t");
          // Serial.print(euler[0] * 180/M_PI);
          // Serial.print("\t");
          // Serial.print(euler[1] * 180/M_PI);
          // Serial.print("\t");
          // Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          // mpu.dmpGetQuaternion(&q, fifoBuffer);
          // mpu.dmpGetGravity(&gravity, &q);
          // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //Serial.print("ypr\t");
          //Serial.print(ypr[0] * 180/M_PI);
          //Serial.print("\t");
          // Serial.print(ypr[2] * 180/M_PI);
          // Serial.print("/");
          // Serial.println(ypr[1] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
          // display real acceleration, adjusted to remove gravity
          
          aaReal_SI[0] = 9.81;
          aaReal_SI[1] = 9.81;
          aaReal_SI[2] = 9.81;
          gyroReal_SI[0] = 131.0;
          gyroReal_SI[1] = 131.0;
          gyroReal_SI[2] =131.0;
          // Serial.print("areal\t");
          // Serial.print(aaReal_SI[0]);
          // Serial.print("\t");
          // Serial.print(aaReal_SI[1]);
          // Serial.print("\t");
          // Serial.println(aaReal_SI[2]);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
          // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          // mpu.dmpGetQuaternion(&q, fifoBuffer);
          // mpu.dmpGetAccel(&aa, fifoBuffer);
          // mpu.dmpGetGravity(&gravity, &q);
          // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          // Serial.print("aworld\t");
          // Serial.print(aaWorld.x);
          // Serial.print("\t");
          // Serial.print(aaWorld.y);
          // Serial.print("\t");
          // Serial.println(aaWorld.z);
      #endif
  
      #ifdef OUTPUT_TEAPOT
          // display quaternion values in InvenSense Teapot demo format:
          // teapotPacket[2] = fifoBuffer[0];
          // teapotPacket[3] = fifoBuffer[1];
          // teapotPacket[4] = fifoBuffer[4];
          // teapotPacket[5] = fifoBuffer[5];
          // teapotPacket[6] = fifoBuffer[8];
          // teapotPacket[7] = fifoBuffer[9];
          // teapotPacket[8] = fifoBuffer[12];
          // teapotPacket[9] = fifoBuffer[13];
          // Serial.write(teapotPacket, 14);
          // teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
      #endif

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);

      // // Preparing the ROS Message with ROS (imu topic)
      // String AX_raw = String(ax_raw);
      // String AY_raw = String(ay_raw);
      // String AZ_raw = String(az_raw);
      // String GX_raw = String(gx_raw);
      // String GY_raw = String(gy_raw);
      // String GZ_raw = String(gz_raw);
      // String data_imu_raw = "A" + AX_raw + "B"+ AY_raw + "C" + AZ_raw + "D" + GX_raw + "E" + GY_raw + "F" + GZ_raw + "G" ;
      // int length_of_G = data_imu_raw.indexOf("G") +2;
      // char data_final_imu_raw[length_of_G+1];
      // data_imu_raw.toCharArray(data_final_imu_raw, length_of_G+1);
      
      // //publishing raw imu data
      // imumsg_raw.data = data_final_imu_raw;
      // imu_raw.publish(&imumsg_raw);

      //publish imu filtered data
      imumsg_filtered.header.stamp = nh.now();
      imumsg_filtered.header.frame_id = "odom";
      imumsg_filtered.orientation.w = q.w;
      imumsg_filtered.orientation.x = q.x;
      imumsg_filtered.orientation.y = q.y;
      imumsg_filtered.orientation.z = q.z;
      imumsg_filtered.linear_acceleration.x = aaReal_SI[0];
      imumsg_filtered.linear_acceleration.y = aaReal_SI[1];
      imumsg_filtered.linear_acceleration.z = aaReal_SI[2];
      imumsg_filtered.linear_acceleration_covariance[0] = -1;
      imumsg_filtered.angular_velocity.x = gyroReal_SI[0];
      imumsg_filtered.angular_velocity.y = gyroReal_SI[1];
      imumsg_filtered.angular_velocity.z = gyroReal_SI[2];
      imumsg_filtered.angular_velocity_covariance[0] = -1;
      imu_data.publish(&imumsg_filtered);
      delay(1);
  }
 
  //check the sampling time of acquising GPS data
  // currentMillis_GSM = millis();
  // if (currentMillis_GSM - previousMillis_GSM > GSM_Sampling_Time_ms) {
  //   //*********** Detecting unread SMS ************************
  //   GSM_messageIndex = sim808.isSMSunread();

  //   //*********** At least, there is one UNREAD SMS ***********
  //   if (GSM_messageIndex > 0)
  //   { 
  //     // Serial.print("messageIndex: ");
  //     // Serial.println(GSM_messageIndex);
      
  //     sim808.readSMS(GSM_messageIndex, GSM_message, GSM_MESSAGE_LENGTH, GSM_phone, GSM_datetime);
                  
  //     //***********In order not to full SIM Memory, is better to delete it**********
  //     sim808.deleteSMS(GSM_messageIndex);
  //     // Serial.print("From number: ");
  //     // Serial.println(GSM_phone);  
  //     // Serial.print("Datetime: ");
  //     // Serial.println(GSM_datetime);        
  //     // Serial.print("Recieved Message: ");
  //     // Serial.println(GSM_message);     
  //   }
  //   previousMillis_GSM = currentMillis_GSM;
  // }

  //check the sampling time of acquising GPS data
  currentMillis_GPS = millis();
  if (currentMillis_GPS - previousMillis_GPS > GPS_Sampling_Time_ms) {
    //*********** IF GPS data is acquised ***********
    // if(sim808.getGPS()){
      // Serial.print(sim808.GPSdata.year);
      // Serial.print("/");
      // Serial.print(sim808.GPSdata.month);
      // Serial.print("/");
      // Serial.print(sim808.GPSdata.day);
      // Serial.print(" ");
      // Serial.print(sim808.GPSdata.hour);
      // Serial.print(":");
      // Serial.print(sim808.GPSdata.minute);
      // Serial.print(":");
      // Serial.print(sim808.GPSdata.second);
      // Serial.print(":");
      // Serial.println(sim808.GPSdata.centisecond);
      // Serial.print("latitude :");
      // Serial.println(sim808.GPSdata.lat);
      // Serial.print("longitude :");
      // Serial.println(sim808.GPSdata.lon);
      // Serial.print("speed_kph :");
      // Serial.println(sim808.GPSdata.speed_kph);
      // Serial.print("heading :");
      // Serial.println(sim808.GPSdata.heading);
      // Serial.println();

      // GPS_la = sim808.GPSdata.lat;
      // GPS_lo = sim808.GPSdata.lon;
      // GPS_ws = sim808.GPSdata.speed_kph;
      // GPS_alt = sim808.GPSdata.altitude;
      // GPS_heading = sim808.GPSdata.heading;
      // GPS_year = sim808.GPSdata.year;
      // GPS_month = sim808.GPSdata.month;
      // GPS_day = sim808.GPSdata.day;
      // GPS_hour = sim808.GPSdata.hour;
      // GPS_minute = sim808.GPSdata.minute;
      // GPS_second = sim808.GPSdata.second;
      // GPS_centisecond = sim808.GPSdata.centisecond;

    //   #ifdef OUTPUT_READABLE_GPS_GSM
    //     dtostrf(GPS_la, 6, 2, GSM_lat); //put float value of la into char array of lat. 6 = number of digits before decimal sign. 2 = number of digits after the decimal sign.
    //     dtostrf(GPS_lo, 6, 2, GSM_lon); //put float value of lo into char array of lon
    //     dtostrf(GPS_ws, 6, 2, GSM_wspeed);  //put float value of ws into char array of wspeed
    //     sprintf(GSM_MESSAGE, "Latitude : %s\nLongitude : %s\nWind Speed : %s kph\nMy Module Is Working. Atta Oveisi. Try With This Link.\nh_odomttp://www.latlong.net/Show-Latitude-Longitude.html\nh_odomttp://maps.google.com/maps?q=%s,%s\n", GSM_lat, GSM_lon, GSM_wspeed, GSM_lat, GSM_lon);    
    //     Serial.println("Sim808 init success");
    //     Serial.println("Start to send message ...");
    //     Serial.println(GSM_MESSAGE);
    //     Serial.println(GSM_phone);
    //     sim808.sendSMS(GSM_phone,GSM_MESSAGE);
    //   #endif
    //   //************* Turn off the GPS power ************
    //   sim808.detachGPS();    
    // }
    previousMillis_GPS = currentMillis_GPS;
  }
  //publish GPS data
  gpsMsg.header.stamp = nh.now();
  gpsMsg.header.frame_id = "base_footprint";
  gpsMsg.latitude = GPS_la;
  gpsMsg.longitude = GPS_lo;
  gpsMsg.altitude = GPS_alt;
  gps.publish(&gpsMsg);
  delay(1);

  // // check the SD logging sampling time in ms
  // currentMillis_SD = millis();
  // if (currentMillis_SD - previousMillis_SD > Logger_Sampling_Time_ms) {
  //   // Write the data to the logger
  //   // Logger = SD.open(Logger_file_name_char, FILE_WRITE);
  //   Logger = SD.open("Log.txt", FILE_WRITE);
  //   if (Logger) {    
  //     Logger.print(GPS_year);
  //     Logger.print(",");
  //     Logger.print(GPS_month);
  //     Logger.print(",");
  //     Logger.println(GPS_day);
  //     Logger.print(",");
  //     Logger.print(GPS_hour);
  //     Logger.print(",");
  //     Logger.print(GPS_minute);
  //     Logger.print(",");
  //     Logger.print(GPS_second);
  //     Logger.print(",");
  //     Logger.print(GPS_centisecond);
  //     Logger.print(",");
  //     Logger.print(currentMillis);
  //     Logger.print(",");  
  //     Logger.print(GPS_la);
  //     Logger.print(",");
  //     Logger.print(GPS_lo);
  //     Logger.print(",");
  //     Logger.print(GPS_alt);
  //     Logger.print(",");
  //     Logger.print(GPS_ws);
  //     Logger.print(",");
  //     Logger.print(GPS_heading);
  //     Logger.print(",");
  //     Logger.print(rpm_FR);
  //     Logger.print(",");
  //     Logger.print(rpm_FL);
  //     Logger.print(",");
  //     Logger.print(rpm_RR);
  //     Logger.print(",");
  //     Logger.print(rpm_RL);
  //     Logger.print(",");
  //     Logger.print(vx_FR);
  //     Logger.print(",");
  //     Logger.print(vx_FL);
  //     Logger.print(",");
  //     Logger.print(vx_RR);
  //     Logger.print(",");
  //     Logger.print(vx_RL);
  //     Logger.print(",");
  //     Logger.print(v_right);
  //     Logger.print(",");
  //     Logger.print(v_left);
  //     Logger.print(",");
  //     Logger.print(vx);
  //     Logger.print(",");
  //     Logger.print(vy);
  //     Logger.print(",");  
  //     Logger.print(vth);
  //     Logger.print(",");
  //     Logger.print(dt);
  //     Logger.print(",");
  //     Logger.print(x_driven);
  //     Logger.print(",");
  //     Logger.print(y_driven);
  //     Logger.print(",");
  //     Logger.print(th_driven);
  //     Logger.print(",");
  //     Logger.print(ypr[0] * 180/M_PI);
  //     Logger.print(",");
  //     Logger.print(ypr[1] * 180/M_PI);
  //     Logger.print(",");
  //     Logger.print(ypr[2] * 180/M_PI);
  //     Logger.print(",");
  //     Logger.print(aaReal_SI[0]);
  //     Logger.print(",");
  //     Logger.print(aaReal_SI[1]);
  //     Logger.print(",");
  //     Logger.print(aaReal_SI[2]);
  //     Logger.print(",");
  //     Logger.print(gyroReal_SI[0]);
  //     Logger.print(",");
  //     Logger.print(gyroReal_SI[1]);
  //     Logger.print(",");
  //     Logger.print(gyroReal_SI[2]);
  //     Logger.print(",");
  //     Logger.println(heading_compass);
  //     Logger.close(); // close the file
  //   }
  //   else {
  //     Serial.println("error opening Logger.txt");
  //   }
  //   previousMillis_SD = currentMillis_SD;
  // }

  // // Publishing odom from wheel speed sensors
  // odomMsg.header.stamp          = nh.now();
  // odomMsg.header.frame_id       = "odom";
  // odomMsg.child_frame_id        = "base_link";
  
  // odomMsg.pose.pose.position.x  = x_driven;
  // odomMsg.pose.pose.position.y  = y_driven;
  // odomMsg.pose.pose.position.z  = 0.0;
  // odomMsg.pose.pose.orientation = tf::createQuaternionFromYaw(th_driven);

  // odomMsg.twist.twist.linear.x  = vx;
  // odomMsg.twist.twist.linear.y  = 0;
  // odomMsg.twist.twist.angular.z = vth;

  // odom.publish(&odomMsg);

  // // Broadcasting wheel speed tf base_link->odom ncccc
  // t.header.stamp            = nh.now();
  // t.header.frame_id         = "odom";
  // t.child_frame_id          = "base_link";

  // t.transform.translation.x = x_driven;
  // t.transform.translation.y = y_driven;
  // t.transform.translation.z = 0.0;
  // t.transform.rotation      = tf::createQuaternionFromYaw(-th_driven);
  // tfBroadcaster.sendTransform(t);

  nh.spinOnce();
  delay(10);
}
