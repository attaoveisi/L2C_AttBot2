/* Start of Code */

#include <Wire.h>

/* The I2C address of the module */
#define HMC5803L_Address 0x0D

/* Register address for the X Y and Z data */
#define X 3
#define Y 7
#define Z 5

void setup() 
{
Serial.begin(9600); 
Wire.begin();
/* Initialise the module */ 
Init_HMC5803L();
}

void loop() 
{
/* Read each sensor axis data and output to the serial port */
Serial.print(HMC5803L_Read(X));
Serial.print(" ");
Serial.print(HMC5803L_Read(Y));
Serial.print(" ");
Serial.println(HMC5803L_Read(Z));

int xvalue = HMC5803L_Read(X);

int yvalue = HMC5803L_Read(Y);
/* Wait a little before reading again */
delay(10);
}


/* This function will initialise the module and only needs to be run once
after the module is first powered up or reset */
void Init_HMC5803L(void)
{
/* Set the module to 8x averaging and 15Hz measurement rate */
Wire.beginTransmission(HMC5803L_Address);
Wire.write(0x00);
Wire.write(0x70);

/* Set a gain of 5 */
Wire.write(0x01);
Wire.write(0xA0);
Wire.endTransmission();
}


/* This function will read once from one of the 3 axis data registers
and return the 16 bit signed result. */
int HMC5803L_Read(byte Axis)
{
int Result;

/* Initiate a single measurement */
Wire.beginTransmission(HMC5803L_Address);
Wire.write(0x02);
Wire.write(0x01);
Wire.endTransmission();
delay(6);

/* Move modules the resiger pointer to one of the axis data registers */
Wire.beginTransmission(HMC5803L_Address);
Wire.write(Axis);
Wire.endTransmission();

/* Read the data from registers (there are two 8 bit registers for each axis) */ 
Wire.requestFrom(HMC5803L_Address, 2);
Result = Wire.read() << 8;
Result |= Wire.read();

return Result;
}

/* End of Code */
