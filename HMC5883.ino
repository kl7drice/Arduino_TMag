/* HMC5883.ino
   An Arduino code example for interfacing with the HMC5883 and the TMP102
   from Sparkfun.  Note that the magnetometer breakout does not have pullup
   resistors, but the temperature breakout does, so using them together
   works well.  Original code examples were from Sparkfun, merged and modified
   by Don Rice.
   It was necessary to change wire.send() to wire.write() and wire.receive()
   to wire.read() for current Arduino library usage.

   Use Arduino 3.3V and ground for power, and:
   Analog input 4 I2C SDA
   Analog input 5 I2C SCL

   HMC5883 Setup:
   Configuration register A (CRA): default
     MA = 00 (1 sample average)
     DO = 100 (15 Hz output rate)
     MS = 00 (normal measurement, no bias current)
   Configuration register B (CRB): default
     GN = 001 (0.92 mG/LSb, +/- 1.3 Ga nominal range)
   Mode register (MR)
     MR = 01 by default for single measurement mode
        = 00 in setup() for continuous measurement mode

   History:
   2015-Mar-13 Cleaned up code and added averaging.
   2015-Mar-12 Combined and tested Sparkfun samples
*/
#include <Wire.h> //I2C Arduino Library

#define magAddress    0x1E // 0011110b, I2C 7bit address of HMC5883
#define tmp102Address 0x48 // 1001000b, default I2C address of TMP102
#define NAVG            10 // Data points to average for output
#define LOOPDELAY      250 // Delay between sensor readings
//#define SHOWALL          1 // Show each measurement
int npts;
float ts, xs, ys, zs;

void setup()
{
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(magAddress); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  // Start averaging count
  npts = 0;
  Serial.print( "Sensor delay=" );
  Serial.print( LOOPDELAY );
  Serial.print( " Averaging Count=" );
  Serial.println( NAVG );

} /* setup */

void loop()
{
  int x, y, z; //triple axis data
  float t, celsius, fahrenheit, xa, ya, za;
  double angle, bmag;
  
  // Loop delay
  delay(LOOPDELAY);

  // Fetch data from sensors
  getMagXYZ( &x, &y, &z );
  t = getTemperature();

  // update sums for average
  if( npts > 0 )
  {
    xs += x;
    ys += y;
    zs += z;
    ts += t;
  }
  else
  {
    xs = x;
    ys = y;
    zs = z;
    ts = t;
  }
#ifdef SHOWALL
  Serial.print( npts );
  Serial.print( "x: " );
  Serial.print( x );
  Serial.print( "/" );
  Serial.print( xs );
  Serial.print( " y: " );
  Serial.print( y );
  Serial.print( "/" );
  Serial.print( ys );
  Serial.print( " z: " );
  Serial.print( z );
  Serial.print( "/" );
  Serial.println( zs );
#endif
  if( ++npts < NAVG ) return;

  //Print out values of each magnetometer axis; mult by 92 for nT
  xa = 92*xs/(float)npts;
  Serial.print( "x: " );
  Serial.print( xa );
  ya = 92*ys/(float)npts;
  Serial.print( "  y: " );
  Serial.print( ya );
  za = 92*zs/(float)npts;
  Serial.print( "  z: " );
  Serial.print( za );
  angle = atan2( (double)ya, (double)xa )*57.295779513082320876798154814105;
  Serial.print( " nT; ang: " );
  Serial.print( angle );
  bmag = sqrt( xa*xa+ya*ya+za*za );
  Serial.print( " mag: " );
  Serial.print( bmag );
  
  //Print temperature values
  celsius = ts/(float)npts;
  Serial.print( " T_C: " );
  Serial.print( celsius );

  fahrenheit = (1.8 * celsius) + 32.0;  
  Serial.print( " T_F: " );
  Serial.println( fahrenheit );
  
  // Restart averaging
  npts = 0;
} /* loop */

/* Fetch magnetic field X, Y, Z from HMC5883 sensor */
int getMagXYZ( int *x, int *y, int *z )
{
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(magAddress);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(magAddress, 6);
  if(Wire.available() < 6 ) return 0; /* read failed */
  *x = Wire.read()<<8; //X msb
  *x |= Wire.read(); //X lsb
  *z = Wire.read()<<8; //Z msb
  *z |= Wire.read(); //Z lsb
  *y = Wire.read()<<8; //Y msb
  *y |= Wire.read(); //Y lsb
  return 1;
} /* getMagXYZ */

/* Return TMP102 temperature in degrees Celsius */
float getTemperature()
{
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
} /* getTemperature */
