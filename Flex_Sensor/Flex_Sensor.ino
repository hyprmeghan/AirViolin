/*
Flex_Sensor
Meghan Jimenez
Antoine Billig

Based on code from SparkFun.com by Mike Grusin
https://www.sparkfun.com/tutorials/270

Uses a flex sensor to determine if a finger is "down" on the string of an air violin

Current circuit:
Voltage divider with 15K resistor, 3.3V source
*/

void setup(){
  //initialize serial communications
 Serial.begin(9600); 
 pinMode(22,OUTPUT);
 
}

int downThresh1 = 800;  //point at which we assume the finger is down
boolean down1 = false; //straight at 400, bent value depends on where you bend
                       //we need to put these in the glove to determine final values

void loop(){
 //Read sensor, print value
 int sensor1;
 int sensor2;
 int sensor3;
 int sensor4;
 sensor1 = analogRead(0);
 sensor2 = analogRead(1);
 sensor3 = analogRead(2);
 sensor4 = analogRead(3);
 Serial.print(sensor1); 
 Serial.print('\t');
 Serial.print(sensor2); 
 Serial.print('\t');
 Serial.print(sensor3); 
 Serial.print('\t');
 Serial.print(sensor4); 
 Serial.println();
 
//Check if the finger is down  
if (sensor2 > downThresh1){
 down1 = true;
 digitalWrite(22,HIGH);
 Serial.print("Finger 1 is down");
 Serial.println();
}
else
{
 down1 = false;
 digitalWrite(22,LOW);
}
 
 //print and delay for next read
 Serial.println();
 delay(100);
}
