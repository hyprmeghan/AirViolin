/* 
 MMA8452Q Basic Example Code
 Nathan Seidle
 SparkFun Electronics
 November 5, 2012
 
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This example code shows how to read the X/Y/Z accelerations and basic functions of the MMA5842. It leaves out
 all the neat features this IC is capable of (tap, orientation, and inerrupts) and just displays X/Y/Z. See 
 the advanced example code to see more features.
 
 Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA -------^^(330)^^------- A4
 SCL -------^^(330)^^------- A5
 GND ---------------------- GND
 
 The MMA8452 is 3.3V so we recommend using 330 or 1k resistors between a 5V Arduino and the MMA8452 breakout.
 
 The MMA8452 has built in pull-up resistors for I2C so you do not need additional pull-ups.
 */

#include <Wire.h> // Used for I2C

// GYRO

//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = (1<<0);
char DLPF_CFG_1 = (1<<1);
char DLPF_CFG_2 = (1<<2);
char DLPF_FS_SEL_0 = (1<<3);
char DLPF_FS_SEL_1 = (1<<4);

//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.
char itgAddress = 0x69;

// END GYRO

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

#define BUFF_SIZE 20
#define UNDEF -1
#define STR1 52
#define STR2 50
#define FIN1 10
#define FIN2 9
#define MOV 8
const float THRESHOLDS[] = {0.30,0.45,0.45,0.45};
struct angleData {
  float angle;
  float accelX;
  float accelY;
  float accelZ;
  int next;
  int string; 
};

struct buffer{
  angleData data[BUFF_SIZE];
  int first;
  int current;
  int nbData;
  float motion;
  int numStr[4];
};

float previous = UNDEF;
boolean initDone;
float accelG[3];
float x;
float y;
float z;
int count;
boolean hold = true;
int stringHold = 0;
float movement = 0;
float prev = 0;
int mostProb = UNDEF;
int prevStr = UNDEF;
struct buffer buff1;

void setup()
{
  Serial.begin(57600);
  Serial.println("MMA8452 Basic Example");
  initDone = true;
  Wire.begin(); //Join the bus as a master
  initMMA8452(); //Test and intialize the MMA8452
  
  pinMode(STR1, OUTPUT);
  pinMode(STR2, OUTPUT);
  pinMode(FIN1, OUTPUT);
  pinMode(FIN2, OUTPUT);
  pinMode(MOV, OUTPUT);
  // GRYRO

  //Read the WHO_AM_I register and print the result
    char id=0; 
    id = itgRead(itgAddress, 0x00);  
    //Serial.print("ID: ");
    //Serial.println(id, HEX);
    
    //Configure the gyroscope
    //Set the gyroscope scale for the outputs to +/-2000 degrees per second
    itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
    //Set the sample rate to 100 hz
    itgWrite(itgAddress, SMPLRT_DIV, 9);

  // END GYRO
  
    resetBuff();
}

void loop()
{  
 delay(500);
  int accelCount[3];  // Stores the 12-bit signed value
  readAccelData(accelCount);  // Read the x/y/z adc values
// Now we'll calculate the accleration value into actual g's
  float accelGCurr[3];  // Stores the real accel value in g's
   for (int i = 0 ; i < 3 ; i++)
  {
    accelGCurr[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
  }
 int string;
  x = accelGCurr[0];
  y = accelGCurr[1];
  z = accelGCurr[2];
  //Serial.println("accelRead");
 float angle; 
 angle = degrees(atan2(y, z));

 if (angle <= -2.5 && angle > -152.5){
  string = 1; 
 }
 else if (angle <= 27.5 && angle > -2.5){
  string = 2; 
 }
 else if (angle <= 57.5 && angle > 27.5){
  string = 3; 
 }
 else if ((angle <= 180 && angle > 57.5) || (angle <= -125.5 && angle >= -180)){
 string = 4;
 }
 else
 string = 0;
float mot = motion(y);
if(previous != UNDEF)
       if(abs(max(previous,movement)/min(previous,movement)) > 4)
           count = 7;
previous = movement;  
if(count-- <= 0)
{
  fillBuff(string,angle,x,y,z,mot);
  count = 0;
}
 
 if(buff1.nbData == BUFF_SIZE)
 {
   movement = buff1.motion;
         
   hold = false;
   angleData accelData;
    getFirst(&accelData);
    
        stringHold = maxStrings();
        if(prevStr == UNDEF)
          prevStr = stringHold;
        int tempHold = stringHold;
        if(mostProb == UNDEF)
          mostProb = stringHold;
        else if((int)abs(mostProb-stringHold) == 1 && (prevStr != stringHold))
          stringHold = mostProb;
        else if((int)abs(mostProb-stringHold) == 2)
        {
          mostProb = mostProb+stringHold/2;
          stringHold = mostProb;
        }
        else if((int)abs(mostProb-stringHold) == 3)
        {
          if(mostProb == 1)
            mostProb = 2;
          else if(mostProb == 4)
            mostProb = 3;
          stringHold = mostProb;
        }
        else
        {
          mostProb = stringHold;
        }
        stringHold = prevStr;
        prevStr = tempHold;
        hold = true;
        resetBuff();
      
 }
 int move = 0;
 if(movement > THRESHOLDS[stringHold-1])
 {
  digitalWrite(MOV,HIGH);
  move = 1;
 }
 else
 {
  digitalWrite(MOV,LOW);
  move = 0;
 }
 
 Serial.println(mostProb);
 Serial.println(stringHold);
 
  if(hold)
 {
 switch(stringHold)
 {case 1: digitalWrite(STR1,LOW);
 digitalWrite(STR2, LOW);
 Serial.println("STR1");
 //if(move == 1)
  //tone(7, 110,250);
 break;
 case 2: digitalWrite(STR1,HIGH);
 digitalWrite(STR2, LOW);
 Serial.println("STR2");
  //if(move == 1)
  //tone(7, 220,250);
 break;
 case 3: digitalWrite(STR1,LOW);
 digitalWrite(STR2, HIGH);
 Serial.println("STR3");
  //if(move == 1)
  //tone(7, 440,250);
 break;
 case 4: digitalWrite(STR1,HIGH);
 digitalWrite(STR2, HIGH);
 Serial.println("STR4");
  //if(move == 1)
  //tone(7, 880,250);
 break;
 default:break;
 }
 }

  

  //Create variables to hold the output rates.
   int xRate, yRate, zRate;
 //Serial.println("endloop"); 
}

void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {  
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    Serial.println("MMA8452Q is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);

  //The default data rate is 800Hz and we don't modify it in this example code

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}
  //GYRO
  
  //This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int readX(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_XOUT_L);  
  
  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int readY(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_YOUT_L);  
  
  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);  
  
  return data;
  
  //END GYRO
}


void fillBuff(int string,float angle, float x, float y, float z, float mot)
{
   //Serial.println("fillBuff In");
    if(buff1.nbData < BUFF_SIZE)
    {
        buff1.data[buff1.current].angle = angle;
        buff1.data[buff1.current].accelX = x;
        buff1.data[buff1.current].accelY = y; 
        buff1.data[buff1.current].accelZ = z;
        buff1.data[buff1.current].string = string;
        buff1.data[buff1.current].next = (buff1.current+1)%BUFF_SIZE;
        buff1.nbData ++;
        buff1.numStr[string-1]++;
        buff1.current = (buff1.current + 1)%BUFF_SIZE;
        buff1.motion += mot;
    }
    //Serial.println("fillBuff Out");
}

void getFirst(struct angleData* data)
{
  //Serial.println("getFirst In");
    (*data) = buff1.data[buff1.first];
    buff1.first = buff1.data[buff1.first].next;
    buff1.numStr[buff1.data[buff1.first].string-1]--;
    buff1.nbData --;
    //Serial.println("getFirst Out");
}
float motion(float val)
{
  //Serial.println("motion In");
     float mov,mov2;
    mov = abs(val);
       
       mov2= mov;
      mov = abs(prev - mov);
      prev= mov2;
 //Serial.println("maxString Out");
    return mov;
}
void resetBuff()
{
  //Serial.println("resetbuff In");
    buff1.first = 0;
  buff1.nbData = 0;
  for(int* str = buff1.numStr; str < buff1.numStr + 4; str++)
    *str = 0;
  buff1.current = 0;
  buff1.motion = 0;
  //Serial.println("resetbuff out");
}
void keepString(int i)
{
  buff1.first = 0;
  buff1.nbData = buff1.numStr[i-1];
  for(int j = 0; j<4; j++)
  { 
    if(j!=i-1)
    buff1.numStr[j] = 0;
  }
  buff1.current = 0;
  buff1.motion = buff1.motion*buff1.numStr[i-1]/20;
  //Serial.print("string ");
  //Serial.println(i);
  //Serial.print("data ");
  //Serial.println(buff1.nbData);
}
int maxStrings()
{
  //Serial.println("maxString In");
  int result = 0;
  int maximum = 0;
  int i=0;
  for(int* str = buff1.numStr; str < buff1.numStr + 4; str++)
    {
      i++;
      if(*str > maximum)
      {
        maximum = *str;
        result = i;
      }
    }
    return result;
}

