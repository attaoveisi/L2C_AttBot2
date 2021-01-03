#include <Arduino.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//create an IMU object
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (5)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//create a GPS object
EasyTransferI2C ET_GPS_data; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  double GPS_la;
  double GPS_lo;
  // double GPS_ws;
  double GPS_alt;
  //double GPS_heading;
  // uint16_t GPS_year;
  // uint8_t GPS_month;
  // uint8_t GPS_day;
  // uint8_t GPS_hour;
  // uint8_t GPS_minute;
  // uint8_t GPS_second;
  // uint8_t GPS_centisecond;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE GPS_data;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

bool state_GPS = LOW;
int LED_GPS = 4;
bool state_IMU = LOW;
int LED_IMU = 5;
void stateChange(bool & state, int LED){
  state = !state;
  digitalWrite(LED, state);  
}

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

//#define _ODOM_PROXY

#ifdef _ODOM_PROXY
#include <rosproxy_msgs/Odometry.h>
#include <rosproxy_msgs/RequestOdometryCovariances.h>
#else
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#endif

#ifdef _ODOM_PROXY
// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER
#endif

ros::NodeHandle nh;

//NEW VERSION:
#ifdef _ODOM_PROXY
rosproxy_msgs::Odometry odom_msg;
ros::Publisher odom_pub("rosproxy/odom", &odom_msg);
#else
geometry_msgs::TransformStamped trans_msg;
nav_msgs::Odometry odom_msg;
tf::TransformBroadcaster broadcaster;
ros::Publisher odom_pub("odom", &odom_msg);
#endif

#ifdef _ODOM_COVAR_SERVER
void odom_covar_callback(const rosproxy_msgs::RequestOdometryCovariancesRequest& req, rosproxy_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
ros::ServiceServer<rosproxy_msgs::RequestOdometryCovariancesRequest, rosproxy_msgs::RequestOdometryCovariancesResponse> odom_covar_server("rosproxy/odom_covar_srv",&odom_covar_callback);
#endif

#define NORMALIZE(z) (atan2(sin(z), cos(z)))

sensor_msgs::NavSatFix gpsMsg;
ros::Publisher gps("gps", &gpsMsg);

// Publisher object for filtered IMU
sensor_msgs::Imu imumsg_filtered;
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
double orientation_covariance[9];
double angular_velocity_covariance[9];
double linear_acceleration_covariance[9];
ros::Publisher imu_data("imu_data", &imumsg_filtered);

double GPS_la = 0.1;
double GPS_lo = 0.1;
double GPS_ws = 0.1;
double GPS_alt = 0.1;
double GPS_heading = 0.1;
uint16_t GPS_year = 1;
uint8_t GPS_month = 1;
uint8_t GPS_day = 1;
uint8_t GPS_hour = 1;
uint8_t GPS_minute = 1;
uint8_t GPS_second = 1;
uint8_t GPS_centisecond = 1;
#define GPS_Sampling_Time_ms 100
unsigned long currentMillis_GPS = 0;
unsigned long previousMillis_GPS = 0;

double ENCODEROUTPUT = 20.0; // Please insert your motor encoder output pulse per rotation
#define HALLSEN_RA 3 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_FA 2 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)

double encoderValue_F = 0.0;
double encoderValue_R = 0.0;
double encoderValueOld_F = 0.0;
double encoderValueOld_R = 0.0;
double encoderValueTemp_F = 0.0;
double encoderValueTemp_R = 0.0;

void updateEncoder_FA()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_F += 1.0;
}

void updateEncoder_RA()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_R += 1.0;
}
double radii = 65.0/2.0;
unsigned long interval = 40;
double interval_d = 40.0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
double factor_rpm_mps = ((2.0*3.14)*((radii)/1000.0))/60.0;
double rpm_F = 0.0;
double rpm_R = 0.0;
double vx_F = 0.0;
double vx_R = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dt = 0.0;
double delta_x = 0.0;
double delta_y = 0.0;
double delta_th = 0.0;
double x_driven = 0.0;
double y_driven = 0.0;
double th_driven = 0.0;
double lengthBetweenTwoWheels = 130.0/1000.0; //130 mm

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
double ax_raw_SI, ay_raw_SI, az_raw_SI;
double gx_raw_SI, gy_raw_SI, gz_raw_SI;

double aa_SI[3];         // [x, y, z]            accel sensor measurements
double aaReal_SI[3];     // [x, y, z]            gravity-free accel sensor measurements
double gyroReal_SI[3];   // [x, y, z]            Gyro sensor measurements
double aaWorld_SI[3];    // [x, y, z]            world-frame accel sensor measurements
double euler[3];         // [psi, theta, phi]    Euler angle container
double ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double quat_SI[4];

void receive(int numBytes){}

uint8_t displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t bno_system, bno_gyro, bno_accel, bno_mag;
  bno_system = bno_gyro = bno_accel = bno_mag = 0;
  bno.getCalibration(&bno_system, &bno_gyro, &bno_accel, &bno_mag);
  return bno_system;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(57600);

  pinMode(LED_GPS, OUTPUT); // Declare the LED as an output
  pinMode(LED_IMU, OUTPUT); // Declare the LED as an output

  while(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    stateChange(state_IMU, LED_IMU);
    delay(1000);
    stateChange(state_IMU, LED_IMU);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(HALLSEN_FA, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RA, INPUT_PULLUP); // Set hall sensor A as input pullup

  attachInterrupt(digitalPinToInterrupt(HALLSEN_FA), updateEncoder_FA, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RA), updateEncoder_RA, RISING);
  
  Wire1.begin(I2C_SLAVE_ADDRESS);

  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc. 
  ET_GPS_data.begin(details(GPS_data), &Wire1);
  
  //define handler function on receiving data
  Wire1.onReceive(receive);

  //Connect to ROS
  nh.initNode();

  //NEW VERSION:
  #ifdef _ODOM_PROXY
  nh.advertise(odom_pub);
  #else
    broadcaster.init(nh);
    nh.advertise(odom_pub);
  #endif

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // CBA Set Odom covariances  
  #ifdef _ODOM_PROXY
    odom_msg.pose_covariance[0] = 0.001;
    odom_msg.pose_covariance[1] = 0.001;
    odom_msg.pose_covariance[2] = 1000000;
    odom_msg.pose_covariance[3] = 1000000;
    odom_msg.pose_covariance[4] = 1000000;
    odom_msg.pose_covariance[5] = 1000;

    odom_msg.twist_covariance[0] = 0.001;
    odom_msg.twist_covariance[1] = 0.001;
    odom_msg.twist_covariance[2] = 1000000;
    odom_msg.twist_covariance[3] = 1000000;
    odom_msg.twist_covariance[4] = 1000000;
    odom_msg.twist_covariance[5] = 1000;
  #else
    trans_msg.header.seq = 0;
    trans_msg.header.frame_id = "odom";
    trans_msg.child_frame_id = "base_link";

    memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1000000;
    odom_msg.pose.covariance[21] = 1000000;
    odom_msg.pose.covariance[28] = 1000000;
    odom_msg.pose.covariance[35] = 1000;

    memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[14] = 1000000;
    odom_msg.twist.covariance[21] = 1000000;
    odom_msg.twist.covariance[28] = 1000000;
    odom_msg.twist.covariance[35] = 1000;
  #endif

  //nh.advertise(gps);

  //advertise IMU filtered data
  nh.advertise(imu_data);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  /* Get a new sensor event */ 
  sensors_event_t imu_event; 
  bno.getEvent(&imu_event);

  imu::Quaternion quat_imu = bno.getQuat();
  imu::Vector<3> lgyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  aaReal_SI[0] = laccel.x();
  aaReal_SI[1] = laccel.y();
  aaReal_SI[2] = laccel.z();
  gyroReal_SI[0] = lgyro.x();
  gyroReal_SI[1] = lgyro.y();
  gyroReal_SI[2] = lgyro.z();
  quat_SI[0] = quat_imu.w();
  quat_SI[1] = quat_imu.x();
  quat_SI[2] = quat_imu.y();
  quat_SI[3] = quat_imu.z();
  Serial.println(quat_imu.z());
  delay(BNO055_SAMPLERATE_DELAY_MS);

  //publish imu filtered data
  imumsg_filtered.header.stamp = nh.now();
  imumsg_filtered.header.frame_id = "odom";
  imumsg_filtered.orientation.w = quat_SI[0];
  imumsg_filtered.orientation.x = quat_SI[1];
  imumsg_filtered.orientation.y = quat_SI[2];
  imumsg_filtered.orientation.z = quat_SI[3];
  imumsg_filtered.linear_acceleration.x = aaReal_SI[0];
  imumsg_filtered.linear_acceleration.y = aaReal_SI[1];
  imumsg_filtered.linear_acceleration.z = aaReal_SI[2];
  imumsg_filtered.linear_acceleration_covariance[0] = -1;
  imumsg_filtered.angular_velocity.x = gyroReal_SI[0];
  imumsg_filtered.angular_velocity.y = gyroReal_SI[1];
  imumsg_filtered.angular_velocity.z = gyroReal_SI[2];
  imumsg_filtered.angular_velocity_covariance[0] = -1;
  uint8_t bno_system_out = displayCalStatus();
  if (bno_system_out>0){
    imu_data.publish(&imumsg_filtered);
    delay(2);
    digitalWrite(LED_IMU, HIGH);
  }else{
    stateChange(state_IMU, LED_IMU);
  }

  if(ET_GPS_data.receiveData()){
    digitalWrite(LED_GPS, HIGH);
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out. 
    GPS_la = GPS_data.GPS_la;
    GPS_lo = GPS_data.GPS_lo;
    // GPS_ws = GPS_data.GPS_ws;
    GPS_alt = GPS_data.GPS_alt;
    // GPS_heading = GPS_data.GPS_heading;
    // Serial.println(GPS_alt);
    // GPS_year = GPS_data.GPS_year;
    // GPS_month = GPS_data.GPS_month;
    // GPS_day = GPS_data.GPS_day;
    // GPS_hour = GPS_data.GPS_hour;
    // GPS_minute = GPS_data.GPS_minute;
    // GPS_second = GPS_data.GPS_second;
    // GPS_centisecond = GPS_data.GPS_centisecond;
  }else{
    stateChange(state_GPS, LED_GPS);
  }
  
  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    rpm_F = (double)(encoderValue_F/(interval_d/1000.0) * 60.0 / ENCODEROUTPUT);
    rpm_R = (double)(encoderValue_R/(interval_d/1000.0) * 60.0 / ENCODEROUTPUT);
    vx_F = abs(rpm_F*factor_rpm_mps); // rpm_FR*2*PI/60*65/2.0/1000
    vx_R = abs(rpm_R*factor_rpm_mps);
    vx = (vx_F + vx_R)/2.0;
    vy = 0.0;
    vth = 0.0;
    dt = millis() - currentMillis;
    delta_x = (vx * cos(th_driven)) * dt/1000.0;
    delta_y = (vx * sin(th_driven)) * dt/1000.0;
    delta_th = vth * dt/1000.0;
    x_driven += delta_x;
    y_driven += delta_y;
    th_driven += delta_th;
    while (th_driven > PI) {
      th_driven -= 2.0 * PI;
    }
    while (th_driven < -PI) {
      th_driven += 2.0 * PI;
    }

    // Reset the encoders 
    encoderValue_F = 0.0;
    encoderValue_R = 0.0;
  
  }
  
  //publish GPS data
  // gpsMsg.header.stamp = nh.now();
  // gpsMsg.header.frame_id = "map";
  // gpsMsg.latitude = GPS_la;
  // gpsMsg.longitude = GPS_lo;
  // gpsMsg.altitude = GPS_alt;
  // gps.publish(&gpsMsg);
  // //nh.spinOnce();
  // delay(2);


  //New version:
  geometry_msgs::Quaternion quat_ros = tf::createQuaternionFromYaw(th_driven);
  #ifdef _ODOM_PROXY
    odom_msg.header.seq++;
    odom_msg.header.stamp = nh.now();
    odom_msg.pose.position.x = x_driven;
    odom_msg.pose.position.y = y_driven;
    odom_msg.pose.position.z = 0.0;
    odom_msg.pose.orientation.x = quat.x;
    odom_msg.pose.orientation.y = quat.y;
    odom_msg.pose.orientation.z = quat.z;
    odom_msg.pose.orientation.w = quat.w;
    odom_msg.twist.linear.x = vx;
    odom_msg.twist.linear.y = vy;
    odom_msg.twist.linear.z = 0.0;
    odom_msg.twist.angular.x = 0.0;
    odom_msg.twist.angular.y = 0.0;
    odom_msg.twist.angular.z = vth;
    odom_pub.publish( &odom_msg );
  #else
    trans_msg.header.seq++;
    trans_msg.header.stamp = nh.now();
    trans_msg.transform.translation.x = x_driven;
    trans_msg.transform.translation.y = y_driven;
    trans_msg.transform.translation.z = 0.0;
    trans_msg.transform.rotation = quat_ros;
    broadcaster.sendTransform(trans_msg);

    odom_msg.header.seq++;
    odom_msg.header.stamp = nh.now();
    odom_msg.pose.pose.position.x = x_driven;
    odom_msg.pose.pose.position.y = y_driven;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat_ros;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vth;
    odom_pub.publish( &odom_msg );
  #endif
  nh.spinOnce();
  delay(2);
}