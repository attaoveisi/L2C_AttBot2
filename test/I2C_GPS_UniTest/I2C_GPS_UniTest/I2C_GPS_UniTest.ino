#include <EasyTransferI2C.h>
#include <Wire.h>

//create object
EasyTransferI2C ET_GPS_data; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  double GPS_la;
  double GPS_lo;
  double GPS_ws;
  double GPS_alt;
  double GPS_heading;
  uint16_t GPS_year;
  uint8_t GPS_month;
  uint8_t GPS_day;
  uint8_t GPS_hour;
  uint8_t GPS_minute;
  uint8_t GPS_second;
  uint8_t GPS_centisecond;
};

void receive(int numBytes) {}


//give a name to the group of data
RECEIVE_DATA_STRUCTURE GPS_data;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

double GPS_la = 0.1;
double GPS_lo = 0.1;
double GPS_ws = 0.0;
double GPS_alt = 0.1;
double GPS_heading = 0.0;
uint16_t GPS_year = 0;
uint8_t GPS_month = 0;
uint8_t GPS_day = 0;
uint8_t GPS_hour = 0;
uint8_t GPS_minute = 0;
uint8_t GPS_second = 0;
uint8_t GPS_centisecond = 0;
#define GPS_Sampling_Time_ms 500
unsigned long currentMillis_GPS = 0;
unsigned long previousMillis_GPS = 0;

void setup() {

  Wire.begin(I2C_SLAVE_ADDRESS);
  Serial.begin(57600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc. 
  ET_GPS_data.begin(details(GPS_data), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive);
}

void loop() {

  if(ET_GPS_data.receiveData()){
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out. 
    GPS_la = GPS_data.GPS_la;
    GPS_lo = GPS_data.GPS_lo;
    GPS_ws = GPS_data.GPS_ws;
    GPS_alt = GPS_data.GPS_alt;
    GPS_heading = GPS_data.GPS_heading;
    GPS_year = GPS_data.GPS_year;
    GPS_month = GPS_data.GPS_month;
    GPS_day = GPS_data.GPS_day;
    GPS_hour = GPS_data.GPS_hour;
    GPS_minute = GPS_data.GPS_minute;
    GPS_second = GPS_data.GPS_second;
    GPS_centisecond = GPS_data.GPS_centisecond;
  }
}
