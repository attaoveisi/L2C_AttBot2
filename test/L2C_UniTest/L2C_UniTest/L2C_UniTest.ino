#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <EEPROM.h>

#include <IRremote.h>

////////// IR REMOTE CODES //////////
#define FD 16736925  // FORWARD
#define BK 16754775 // BACK
#define LT 16720605 // LEFT
#define RT 16761405 // RIGHT
#define SP 16712445 // STOP
#define UNKNOWN_F 5316027     // FORWARD
#define UNKNOWN_B 2747854299  // BACK
#define UNKNOWN_L 1386468383  // LEFT
#define UNKNOWN_R 553536955   // RIGHT
#define UNKNOWN_S 3622325019  // STOP
#define KEY1 16738455
#define KEY2 16750695
#define KEY3 16756815
#define KEY4 16724175
#define KEY5 16718055
#define KEY6 16743045
#define KEY7 16716015
#define KEY8 16726215
#define KEY9 16734885
#define KEY0 16730805
#define KEY_STAR 16728765
#define KEY_HASH 16732845

#define RECV_PIN  A7
#define carSpeedSLAM 200  //  speed of car >=0 to <=255 when SLAMing
#define A8  A8
#define A9  A9
#define A10  A10
#define A11  A11
#define A12  A12
#define A13  A13

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;
unsigned long preMillis;

#define ENCODEROUTPUT 12 // Please insert your motor encoder output pulse per rotation
#define HALLSEN_FR_A 7 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_FL_A 6 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RR_A 4 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RL_A 5 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)

volatile long encoderValue_FR = 0;
volatile long encoderValue_FL = 0;
volatile long encoderValue_RR = 0;
volatile long encoderValue_RL = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
int rpm_FR = 0;
int rpm_FL = 0;
int rpm_RR = 0;
int rpm_RL = 0;

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

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

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
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

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

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

long duration;
long distanceCm_new = 0, distanceCm_old = 0;
long ultraSound_center = 0, ultraSound_left = 0, ultraSound_right = 0;

#define ENA 8
#define ENB 9
#define IN1 10
#define IN2 11
#define IN3 12
#define IN4 13

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication
  Serial.begin(9600);     //open serial and set the baudrate

    // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  pinMode(A8, OUTPUT);
  pinMode(A9, INPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, INPUT);
  pinMode(A12, OUTPUT);
  pinMode(A13, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  stop();

  pinMode(HALLSEN_FR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_FL_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RL_A, INPUT_PULLUP); // Set hall sensor A as input pullup

  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FR_A), updateEncoder_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FL_A), updateEncoder_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RR_A), updateEncoder_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RL_A), updateEncoder_RL, RISING);

  Serial.print("\n\n");
  Serial.println("Measuring DC Motor's RPM");

  irrecv.enableIRIn();

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-79);
  mpu.setYGyroOffset(60);
  mpu.setZGyroOffset(-9);
  mpu.setYAccelOffset(-1160);
  mpu.setXAccelOffset(738); 
  mpu.setZAccelOffset(1539); 


  // make sure it worked (returns 0 if so)
  if (1) {
  //if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    // Revolutions per minute (RPM) = (total encoder pulse in 1s / motor encoder output) x 60s
    rpm_FR = (float)(encoderValue_FR * 60 / ENCODEROUTPUT);
    rpm_FL = (float)(encoderValue_FL * 60 / ENCODEROUTPUT);
    rpm_RR = (float)(encoderValue_RR * 60 / ENCODEROUTPUT);
    rpm_RL = (float)(encoderValue_RL * 60 / ENCODEROUTPUT);

    // Serial.print('Front right wheel: ');
    Serial.print(rpm_FR);
    // Serial.print(" RPM \n");
    // Serial.print('Front left wheel: ');
    Serial.print(rpm_FL);
    // Serial.print(" RPM \n");
    // Serial.print('Rear right wheel: ');
    Serial.print(rpm_RR);
    // Serial.print(" RPM \n");
    // Serial.print('Rear left wheel: ');
    Serial.print(rpm_RL);
    // Serial.print(" RPM \n");

    // Reset the encoders 
    encoderValue_FR = 0;
    encoderValue_FL = 0;
    encoderValue_RR = 0;
    encoderValue_RL = 0;
  }

  long ultraSound_center = getDistance(A8,A9,ultraSound_center); // Gets distance from the sensor and this function is repeatedly called while we are at the first example in order to print the lasest results from the distance sensor
  long ultraSound_left = getDistance(A10,A11,ultraSound_left);
  long ultraSound_right = getDistance(A12,A13,ultraSound_right);
  // Serial.println(ultraSound_left);
  // Serial.println(ultraSound_center);
  // Serial.println(ultraSound_right);
  // Serial.println("----");
  long minEuclDistCm = minEuclDist(ultraSound_center, ultraSound_left, ultraSound_right);
  // Serial.println(minEuclDistCm);
  // Serial.println("----");
  // Serial.println("----");

  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            // Serial.print(ypr[2] * 180/M_PI);
            // Serial.print("/");
            // Serial.println(ypr[1] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
  
  //go forward
  // digitalWrite(IN1,HIGH); 
  // digitalWrite(IN2,LOW);  
  // digitalWrite(IN3,LOW);  
  // digitalWrite(IN4,HIGH);
  // int vel_Rq = 127;
  //Handling the too much steering and too less speed
  // for (int stw_Rq = 127; stw_Rq >= -127; stw_Rq--)
  // {
  //   Speed_Steering_Clamp(vel_Rq,stw_Rq);
  //   analogWrite(ENB,vel_Rq-stw_Rq);
  //   analogWrite(ENA,vel_Rq+stw_Rq);
  //   delay(10);
  // }
  // //stop
  // analogWrite(ENB,0); //speed = 0
  // analogWrite(ENA,0);  
  // delay(1000);
  // if (irrecv.decode(&results)){ 
  //   preMillis = millis();
  //   val = results.value;
  //   Serial.println(val);
  //   irrecv.resume();
  //   switch(val){
  //     case FD: 
  //     case UNKNOWN_F: forward(); break;
  //     case BK: 
  //     case UNKNOWN_B: back(); break;
  //     case LT: 
  //     case UNKNOWN_L: left(); break;
  //     case RT: 
  //     case UNKNOWN_R: right();break;
  //     case SP: 
  //     case UNKNOWN_S: stop(); break;
  //     default: break;
  //   }
  // }
  // else{
  //   if(millis() - preMillis > 500){
  //     stop();
  //     preMillis = millis();
  //   }
  // }
  forward();
}

long getDistance(const int &trigPin,const int &echoPin, long &distanceCm_old) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm_old = distanceCm_new;
  distanceCm_new= duration*0.034/2;
  if (abs((distanceCm_new-distanceCm_old) >= 100) || (distanceCm_new >= 600))
  {
    distanceCm_new = distanceCm_old;
  }
  return distanceCm_new;
}

long minEuclDist(const long &ultraSound_center, const long &ultraSound_left,const long &ultraSound_right){
  long minEuclDistCm = max(0,min(min(ultraSound_center, ultraSound_left),ultraSound_right));
  return minEuclDistCm;
}

void Speed_Steering_Clamp(int &vel_Rq, int &stw_Rq)
{
  if (vel_Rq+stw_Rq >= 255)
    {
      vel_Rq = vel_Rq-(255-(vel_Rq+stw_Rq));
    }
    else if (vel_Rq-stw_Rq < 0)
    {
      vel_Rq = vel_Rq+abs(vel_Rq-stw_Rq);
    }
}

void forward(){ 
  digitalWrite(ENA,carSpeedSLAM);
  digitalWrite(ENB,carSpeedSLAM);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  //Serial.println("go forward!");
}
void back(){
  digitalWrite(ENA,carSpeedSLAM);
  digitalWrite(ENB,carSpeedSLAM);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  //Serial.println("go back!");
}
void left(){
  analogWrite(ENA,carSpeedSLAM);
  analogWrite(ENB,carSpeedSLAM);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  //Serial.println("go left!");
}
void right(){
  analogWrite(ENA,carSpeedSLAM);
  analogWrite(ENB,carSpeedSLAM);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  //Serial.println("go right!");
}
void stop(){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  //Serial.println("STOP!");  
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
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
