#define USB_USBCON
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <GY_85.h>

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

//create an IMU object
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (5)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

bool state_odom = LOW;
int LED_odom = 4;
bool state_IMU = LOW;
int LED_IMU = 5;
void stateChange(bool & state, int LED){
  state = !state;
  digitalWrite(LED, state);  
}

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

// Publisher object for filtered IMU
sensor_msgs::Imu imumsg_filtered;
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
double orientation_covariance[9];
double angular_velocity_covariance[9];
double linear_acceleration_covariance[9];
ros::Publisher imu_data("imu_data", &imumsg_filtered);

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
  digitalWrite(LED_odom, HIGH);
}

void updateEncoder_RA()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_R += 1.0;
  digitalWrite(LED_odom, HIGH);
}

double radii = 137.2/2.0;
double lr = 14.4/200.0;
double lf = 14.2/200.0;
double beta;
double fwa;
unsigned long interval = 40;
double interval_d = 40.0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
double factor_rpm_mps = ((2.0*PI)*((radii)/1000.0))/60.0;
double rpm_F = 0.0;
double rpm_R = 0.0;
double vx_F = 0.0;
double vx_R = 0.0;
double vx_m = 0.0;
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
double degreeToRad = PI / 180;

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

GY_85 GY85;     //create the object

double zeroValue[5] = { 0, 0, 0, 0, 0}; // Found by experimenting

/* All the angles start at 0.0 degrees */
double gyroXangle = 0.0;
double gyroYangle = 0.0;
double gyroZangle = 0.0;

double accXangle = 0.0;
double accYangle = 0.0;
double accZangle = 0.0;

double compAngleX_deg = 0.0;
double compAngleY_deg = 0.0;
double compAngleZ_deg = 0.0;

double compAngleX_rad = 0.0;
double compAngleY_rad = 0.0;
double compAngleZ_rad = 0.0;

unsigned long timer;

uint8_t buffer[2]; // I2C buffer

double ax_fw;
double ay_fw; 
double az_fw;
double ax_fw_old;
double ay_fw_old; 
double az_fw_old;

int cx_fw;
int cy_fw;
int cz_fw;

float gx_fw;
float gy_fw;
float gz_fw;
float gt_fw;

double getAngle(const int x_val, const int y_val) {
  double accXval = (double)x_val;
  double accYval = (double)y_val;
  double angle = (atan2(accXval, accYval) + PI) * RAD_TO_DEG;
  return angle;
}

double fwa_calibration_cycle;
double fwa_calibration_start = 5000.0;
double fwa_calibration_end = 10000.0;
double fwa_calibration_seq = 1.0;
double axy_calibration_seq = 1.0;
double quat_SI_z_mean = 0.0;
double compAngleZ_rad_mean = 0.0;
double ax_fw_mean = 0.0;
double ay_fw_mean = 0.0;

double thresh(double val, double threshold){
  if (val < threshold){
    val = 0.0;
  }
  return val;
}

// for avoiding numerical issues
double reverse_thresh(double new_val, double old_val, double threshold){
  if (abs(new_val-old_val) > threshold){
    return old_val;
  }else
  {
    return new_val;
  }
}

#define WINDOW_SIZE 10
int maf_INDEX_x = 0; //moving average filter
double maf_SUM_x = 0.0;
double maf_READINGS_x[WINDOW_SIZE];
double maf_AVERAGED_x = 0.0;
int maf_INDEX_y = 0; //moving average filter
double maf_SUM_y = 0.0;
double maf_READINGS_y[WINDOW_SIZE];
double maf_AVERAGED_y = 0.0;
int maf_INDEX_z = 0; //moving average filter
double maf_SUM_z = 0.0;
double maf_READINGS_z[WINDOW_SIZE];
double maf_AVERAGED_z = 0.0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  //Serial.begin(57600);

  pinMode(LED_odom, OUTPUT); // Declare the LED as an output
  pinMode(LED_IMU, OUTPUT); // Declare the LED as an output

  digitalWrite(LED_IMU, HIGH);
  while(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    stateChange(state_IMU, LED_IMU);
    delay(1000);
    stateChange(state_IMU, LED_IMU);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Wire.begin();
  delay(10);
  GY85.init();
  delay(10);

  pinMode(HALLSEN_FA, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RA, INPUT_PULLUP); // Set hall sensor A as input pullup

  attachInterrupt(digitalPinToInterrupt(HALLSEN_FA), updateEncoder_FA, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RA), updateEncoder_RA, RISING);

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

  //advertise IMU filtered data
  nh.advertise(imu_data);
  while(!nh.connected()) {nh.spinOnce();}
  timer = micros();
  //Serial.println("Setup is finished!");

  fwa_calibration_cycle = millis();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  digitalWrite(LED_odom, LOW);

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
  //Serial.println(quat_imu.z());

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
  imu_data.publish(&imumsg_filtered);

  // Raw data from GY-85:
  // ITG3205 (3-Achsen-Drehratensensor)
  // ADXL345 (3-Achsen-Beschleunigungssensor)
  // HMC5883L (3-Achsen Digitalkompass)
  ax_fw = (double)(GY85.accelerometer_x( GY85.readFromAccelerometer() ));
  ay_fw = (double)(GY85.accelerometer_y( GY85.readFromAccelerometer() ));
  az_fw = (double)(GY85.accelerometer_z( GY85.readFromAccelerometer() ));

  ax_fw = reverse_thresh(ax_fw, ax_fw_old, 5000.0);
  ax_fw_old = ax_fw;
  ay_fw = reverse_thresh(ay_fw, ay_fw_old, 5000.0);
  ay_fw_old = ay_fw;
  az_fw = reverse_thresh(az_fw, az_fw_old, 5000.0);
  az_fw_old = az_fw;
  
  maf_SUM_x = maf_SUM_x - maf_READINGS_x[maf_INDEX_x];
  maf_READINGS_x[maf_INDEX_x] = ax_fw;
  maf_SUM_x = maf_SUM_x + ax_fw;
  maf_INDEX_x += 1;
  ax_fw = maf_SUM_x/WINDOW_SIZE; 

  maf_SUM_y = maf_SUM_y - maf_READINGS_y[maf_INDEX_y];
  maf_READINGS_y[maf_INDEX_y] = ay_fw;
  maf_SUM_y = maf_SUM_y + ay_fw;
  maf_INDEX_y += 1;
  ay_fw = maf_SUM_y/WINDOW_SIZE; 

  maf_SUM_z = maf_SUM_z - maf_READINGS_z[maf_INDEX_z];
  maf_READINGS_z[maf_INDEX_z] = az_fw;
  maf_SUM_z = maf_SUM_z + az_fw;
  maf_INDEX_z += 1;
  az_fw = maf_SUM_z/WINDOW_SIZE; 
  
  if(maf_INDEX_x == WINDOW_SIZE){
    maf_INDEX_x = 0;
    maf_INDEX_y = 0;
    maf_INDEX_z = 0;
  }
    
  cx_fw = GY85.compass_x( GY85.readFromCompass() );
  cy_fw = GY85.compass_y( GY85.readFromCompass() );
  cz_fw = GY85.compass_z( GY85.readFromCompass() );
  
  gx_fw = GY85.gyro_x( GY85.readGyro() )/14.375; // 14.375 is the sensitivity (see datasheet)
  gx_fw = thresh(gx_fw, 0.02);
  gy_fw = GY85.gyro_y( GY85.readGyro() )/14.375;
  gy_fw = thresh(gy_fw, 0.02);
  gz_fw = GY85.gyro_z( GY85.readGyro() )/14.375;
  gz_fw = thresh(gz_fw, 0.02);
  gt_fw = GY85.temp  ( GY85.readGyro() )/14.375;

  gyroXangle += gx_fw * ((double)(micros() - timer) / 1000000); // Without any filter
  gyroYangle += gy_fw * ((double)(micros() - timer) / 1000000); // Without any filter
  gyroZangle += gz_fw * ((double)(micros() - timer) / 1000000); // Without any filter

  // taking the mean value in the first couple of seconds out
  if ((millis()-fwa_calibration_cycle > fwa_calibration_start) && (millis()-fwa_calibration_cycle < fwa_calibration_end)){
    ax_fw_mean += ax_fw;
    ay_fw_mean += ay_fw;
    axy_calibration_seq += 1.0;
  }else{
    ax_fw_mean = ax_fw_mean/axy_calibration_seq;
    ay_fw_mean = ay_fw_mean/axy_calibration_seq;
    ax_fw = ax_fw - ax_fw_mean;
    ay_fw = ay_fw - ay_fw_mean;
    axy_calibration_seq = 1.0;
  }

  accXangle = getAngle(ax_fw, az_fw);
  accYangle = getAngle(ay_fw, az_fw);
  accZangle = getAngle(ax_fw, ay_fw);

  compAngleX_deg = (0.94 * (compAngleX_deg + (gx_fw * (double)(micros() - timer) / 1000000))) + (0.06 * accXangle);
  compAngleY_deg = (0.94 * (compAngleY_deg + (gy_fw * (double)(micros() - timer) / 1000000))) + (0.06 * accYangle);
  compAngleZ_deg = (0.94 * (compAngleZ_deg + (gz_fw * (double)(micros() - timer) / 1000000))) + (0.06 * accZangle);
  compAngleX_rad = degreeToRad*compAngleX_deg;
  compAngleY_rad = degreeToRad*compAngleY_deg;
  compAngleZ_rad = degreeToRad*compAngleZ_deg;
  while (compAngleX_rad > PI) {
      compAngleX_rad -= 2.0 * PI;
  }
  while (compAngleX_rad < -PI) {
      compAngleX_rad += 2.0 * PI;
  }
  while (compAngleY_rad > PI) {
      compAngleY_rad -= 2.0 * PI;
  }
  while (compAngleY_rad < -PI) {
      compAngleY_rad += 2.0 * PI;
  }
  while (compAngleZ_rad > PI) {
      compAngleZ_rad -= 2.0 * PI;
  }
  while (compAngleZ_rad < -PI) {
      compAngleZ_rad += 2.0 * PI;
  }
  
  // taking the mean value in the first couple of seconds out
  if ((millis()-fwa_calibration_cycle > fwa_calibration_start) && (millis()-fwa_calibration_cycle < fwa_calibration_end)){
    quat_SI_z_mean += quat_imu.z();
    compAngleZ_rad_mean += compAngleZ_rad;
    fwa_calibration_seq += 1.0;
    fwa = (quat_imu.z() - compAngleZ_rad);
  }else{
    quat_SI_z_mean = quat_SI_z_mean/fwa_calibration_seq;
    compAngleZ_rad_mean = compAngleZ_rad_mean/fwa_calibration_seq;
    fwa_calibration_seq = 1.0;
    fwa = (quat_imu.z() - compAngleZ_rad)- abs(quat_SI_z_mean-compAngleZ_rad_mean);
  }
  
  while (fwa > PI) {
      fwa -= 2.0 * PI;
  }
  while (fwa < -PI) {
      fwa += 2.0 * PI;
  }
  // Serial.print(compAngleZ_rad); Serial.print("\t");
  // Serial.print(quat_imu.z()); Serial.print("\t");
  // Serial.print(fwa); Serial.print("\n");

  timer = micros();

  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    rpm_F = (double)(encoderValue_F/(interval_d/1000.0) * 60.0 / ENCODEROUTPUT);
    rpm_R = (double)(encoderValue_R/(interval_d/1000.0) * 60.0 / ENCODEROUTPUT);
    vx_F = abs(rpm_F*factor_rpm_mps); // rpm_FR*2*PI/60*65/2.0/1000
    vx_R = abs(rpm_R*factor_rpm_mps);
    vx_m = (vx_F + vx_R)/2.0;
    beta = atan((lf)/(lf+lr)*tan(fwa));
    vth = vx_m/(lf+lr)*cos(beta)*tan(fwa);
    vx = vx_m*cos(th_driven+beta);
    vy = vx_m*sin(th_driven+beta);
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
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  // Check the calibration status
  uint8_t bno_system_out = displayCalStatus();
  if (bno_system_out > 2){ // 0..3 with 3 meaning fully calibrated 
    digitalWrite(LED_IMU, HIGH);
  }else{
    stateChange(state_IMU, LED_IMU);
  }
}