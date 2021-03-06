
/*
Air Violin Sensor Processsing

Meghan Jimenez and Antoine Billig

Receives and processes data from flex sensors (analog input) and a 3-axis accelerometer 
(over I2C). Outputs information on string to LEDs and information on finger to buzzers.
*/

#include <Wire.h> 

#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

#define BUFF_SIZE 50
#define UNDEF -1
#define STR1 12
#define STR2 11
#define FIN1 10
#define FIN2 9
#define FIN3 13
#define MOT1 7
#define MOT2 6
#define MOT3 5
#define MOT4 4
#define MOV 8
#define OFF 0
const float THRESHOLDS[] = {0.25,0.3,0.3,0.5};
const float FINGER_TRESH[] = {850,865,790,800};
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

int accelCount[3];
float previous = UNDEF;
boolean initDone;
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
int prevStr2 = UNDEF;
struct buffer buff1;
int sensor1;
 int sensor2;
 int sensor3;
 int sensor4;
 
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
  pinMode(FIN3, OUTPUT);
  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);
  pinMode(MOV, OUTPUT);
  resetBuff();
} 


void loop()
{  
  delay(3);
   sensor1 = analogRead(0);
 sensor2 = analogRead(1);
 sensor3 = analogRead(2);
 sensor4 = analogRead(3);
 
 if (sensor4 > FINGER_TRESH[3]){
   leftHand(4);
  } 
 else if (sensor3 > FINGER_TRESH[2]){ 
   leftHand(3);
  }
  
 else if (sensor2 > FINGER_TRESH[1]){
   leftHand(2);
  }
 
 else if (sensor1 > FINGER_TRESH[0]){
   leftHand(1);
 }

else
{
  leftHand(OFF);
}
 
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
  z = accelGCurr[2];
  
 float angle; 
 angle = degrees(atan2(x, z));
 
 readAccelData(accelCount);  // Read the x/y/z adc values
  for (int i = 0 ; i < 3 ; i++)
  {
    accelGCurr[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
  }
  x = accelGCurr[0];
  y = accelGCurr[1];
  z = accelGCurr[2];
  if(z == 0)
  {
    z = 0.0001;
  }
  if(y == 0)
  {
    y = 0.0001;
  }
 angle = degrees(atan2(y, z));

 if (angle <= -19.5 && angle > -152.5){
  string = 1; 
 }
 else if (angle <= 20.5 && angle > -19.5){
  string = 2; 
 }
 else if (angle <= 78.5 && angle > 20.5){
  string = 3; 
 }
 else {
   string = 4;
 }

float mot = motion(y);
if(movement == 0)
  movement = .001;

//unused now
/*if(previous != UNDEF)
       if(abs(max(previous,movement)/min(previous,movement)) > 4)
           count = 7;*/
previous = movement;  
//if(count <= 0)
if(true)
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
          if(prevStr2 == UNDEF)
          prevStr2 = prevStr;
        int tempHold = stringHold;
        if(mostProb == UNDEF)
          mostProb = stringHold;
        
        if(prevStr+ stringHold == 2)
        {
            mostProb = 1;
            stringHold = 1;
        }
        if(prevStr+stringHold == 3)
        {
          if(mostProb == 1)
          {
            stringHold = 1;
            if(prevStr == 1)
              mostProb = 1;
            else
              mostProb = 2;
          }
          else if(mostProb == 2)
          {
            stringHold = 2;
            if(prevStr == 1)
              mostProb = 1;
            else
              mostProb = 2;
          }
          else
          {
          mostProb = 2;
          stringHold = 2;
          }
          mostProb= 2;
          stringHold = 2;
        }
        else if(prevStr+stringHold == 4)
        {  
          mostProb = 2;
          stringHold = 2;
        }
        else if(prevStr+stringHold == 5)
        {
          if(mostProb == 2)
          {
            if(prevStr == 2 && prevStr2 == 2)
            stringHold = 2;
            else if(prevStr == 3 && prevStr2 == 2)
            {
              mostProb = 2;
              stringHold = 2;
            }
            else if(prevStr == 3 && prevStr2 == 3)
            {
              mostProb = 3;
              stringHold = 3;
            }
            else
            { 
              stringHold =2;
              mostProb = 2;
            }
          }
          else if(mostProb == 3)
          {
            if(prevStr == 3 && prevStr2 == 3)
            {
              stringHold = 3;
              mostProb =3;
            }
            else if(prevStr == 2 && prevStr2 == 3)
            {
               mostProb = 3;
               stringHold = 3;
            
            }
            else if(prevStr2 == 2 && prevStr== 2)
            {
              mostProb = 3;
              stringHold = 2;
            }
            else
            {
              stringHold = 3;
              mostProb = 3;
            }
          }
          else if(previous <= 2)
          {
            mostProb = 2;
            stringHold = 2;
            
          }
          else
          {
            mostProb = 3;
            stringHold = 3;
          }
        }
        else if(prevStr + stringHold == 6)
        {
          mostProb = 3;
          stringHold = 3;
        }
        else if(prevStr + stringHold == 7)
        {
          mostProb = 3;
          stringHold = 3;
        }
        else if(prevStr + stringHold == 8)
        {
          mostProb = 4;
          stringHold = 4;
        }
        prevStr2 = prevStr;
        prevStr = tempHold;
        
        hold = true;
        resetBuff();
      
 }
 int move = 0;
 
 //movement is mean of all differences between the motions that occured while the bbuffer was being filled
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
 
 //if the buffer is currently being filled uses the 'held' string
 if(hold)
 {
 strings(stringHold);
 
 }
  
  
  
}




void readAccelData(int * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  readRegisters(0x01, 6, &rawData[0]);  // Read the six raw data registers into data array
  
  /* loop to calculate 12-bit ADC and g value for each axis */
  for (int i=0; i<6; i+=2)
  {
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {  // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
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

//fills the buffer thanks to the given parameters
void fillBuff(int string,float angle, float x, float y, float z, float mot)
{
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
        buff1.motion += abs(mot);
    }
   
}

//returns the first element of the buffer
void getFirst(struct angleData* data)
{
  (*data) = buff1.data[buff1.first];
  buff1.first = buff1.data[buff1.first].next;
  buff1.numStr[buff1.data[buff1.first].string-1]--;
  buff1.nbData --;
}

//returns the difference in motion since the last mesure
float motion(float val)
{
  float mov,mov2;
  mov = abs(val);
  mov2= mov;
  mov = abs(prev - mov);
  prev= mov2;
  return mov;
}
void resetBuff()
{
  buff1.first = 0;
  buff1.nbData = 0;
  for(int* str = buff1.numStr; str < buff1.numStr + 4; str++)
    *str = 0;
  buff1.current = 0;
  buff1.motion = 0;
}

//not used now
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
  buff1.motion = buff1.motion*buff1.numStr[i-1]/BUFF_SIZE;
  Serial.print("string ");
  Serial.println(i);
  Serial.print("data ");
  Serial.println(buff1.nbData);
}

//most used string in the current buffer
int maxStrings()
{
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

//string encoding
void strings(int id)
{
  digitalWrite(STR1,LOW);
  digitalWrite(STR2,LOW);
  switch(id)
  {
    case 2:
    digitalWrite(STR1,HIGH);
    break;
    case 3:
    digitalWrite(STR2,HIGH);
    break;
    case 4:
    digitalWrite(STR1,HIGH);
    digitalWrite(STR2,HIGH);
    break;
    default:
    break;
  }
}

//motor + finger encoding
void leftHand(int id)
{
  motors(id);
  fingers(id);
}

//motors encoding
void motors(int id)
{
   switch(id)
  {
    case 1:
    digitalWrite(MOT1,HIGH);
    digitalWrite(MOT2,LOW);
    digitalWrite(MOT3,LOW);
    digitalWrite(MOT4,LOW);
    break;
    case 2:
    digitalWrite(MOT2,HIGH);
    digitalWrite(MOT1,LOW);
    digitalWrite(MOT3,LOW);
    digitalWrite(MOT4,LOW);
    break;
    case 3:
    digitalWrite(MOT3,HIGH);
    digitalWrite(MOT2,LOW);
    digitalWrite(MOT1,LOW);
    digitalWrite(MOT4,LOW);
    break;
    case 4:
    digitalWrite(MOT4,HIGH);
    digitalWrite(MOT2,LOW);
    digitalWrite(MOT3,LOW);
    digitalWrite(MOT1,LOW);
    break;
    default:
    digitalWrite(MOT1,LOW);
    digitalWrite(MOT2,LOW);
    digitalWrite(MOT3,LOW);
    digitalWrite(MOT4,LOW);
    break;
  }
}


//fingers encoding
void fingers(int id)
{
  switch(id)
  {
    case 1:
    digitalWrite(FIN1,LOW);
    digitalWrite(FIN2,LOW);
    digitalWrite(FIN3,LOW);
    break;
    case 2:
    digitalWrite(FIN1,HIGH);
    digitalWrite(FIN2,LOW);
    digitalWrite(FIN3,LOW);
    break;
    case 3:
    digitalWrite(FIN1,LOW);
    digitalWrite(FIN2,HIGH);
    digitalWrite(FIN3,LOW);
    break;
    case 4:
    digitalWrite(FIN1,HIGH);
    digitalWrite(FIN2,HIGH);
    digitalWrite(FIN3,LOW);
    break;
    default:
    digitalWrite(FIN3,HIGH);
    digitalWrite(FIN2,LOW);
    digitalWrite(FIN1,LOW);
    break;
  }
  
}
 
