//Air Violin Sound Output
//Meghan Jimenez and Antoine Billig

/*
Reads in processed data from the master arduino which has flex sensor
and accelerometer input. Produces the relevant note based on the current
string and finger being played. Right hand must be in motion for any 
sound to be played.
*/

//include header file of pitch frequencies
#include "pitches.h"

//Define pin 13 as sound out
#define SOUT 13

void setup(){
  Serial.begin(57600); // setup serial connection
  
  pinMode(SOUT, OUTPUT); //make SOUT an output
}

void loop(){
  //read in processed sensor information from 
  //the other arduino
  int STR1 = analogRead(0);
  int STR2 = analogRead(1);
  int FIN1 = analogRead(2);
  int FIN2 = analogRead(3);
  int FIN3 = analogRead(4);
  int MOV = analogRead(5);
  
  //Determine current string and finger
  int string;
  int finger;
  
  //String 1 - 4
  
  if (!STR1 && !STR2){
    string = 1;
  }
  
  else if (STR1 && !STR2){
    string = 2;
  }
  
  else if (!STR1 && STR2){
    string = 3;
  }
  
  else if (STR1 && STR2){
    string = 4;
  }
 
 //Finger 0 - 4 
  if (FIN3){
    finger = 0;
  }
  
  else if (!FIN1 && !FIN2){
    finger = 1;
  }
  
  else if (FIN1 && !FIN2){
    finger = 2;
  }
  
  else if (!FIN1 && FIN2){
    finger = 3;
  }
  
  else if (FIN1 && FIN2){
    finger = 4;
  }
  
  /*
  //Print current finger values
  Serial.println(FIN1);
  Serial.println('\t');
  Serial.println(FIN2);
  Serial.println('\t');
  Serial.println(FIN3);
  Serial.println('\t');
  */
  
  //if the right hand is in motion
  //determine which note should be
  //played based on current string
  //and finger
  
  if(MOV){ 
    if (string == 1 && finger == 0){
       tone(SOUT, NOTE_G3); 
    }
    else if (string == 1 && finger == 1){
      tone(SOUT, NOTE_A3);
    }
    
    else if (string == 1 && finger == 2){
      tone(SOUT, NOTE_B3);
    }
    
    else if (string == 1 && finger == 3){
      tone(SOUT, NOTE_C4);
    }
    
    else if (string == 1 && finger == 4){
      tone(SOUT, NOTE_D4);
    }
    
    else if (string == 2 && finger == 0){
      tone(SOUT, NOTE_D4);
    }
    
    else if (string == 2 && finger == 1){
      tone(SOUT, NOTE_E4);
    }
    
    else if (string == 2 && finger == 2){
      tone(SOUT, NOTE_F4);
    }
    
    else if (string == 2 && finger == 3){
      tone(SOUT, NOTE_G4);
    }
    
    else if (string == 2 && finger == 4){
      tone(SOUT, NOTE_A4);
    }
    
    else if (string == 3 && finger == 0){
      tone(SOUT, NOTE_A4);
    }
    
    else if (string == 3 && finger == 1){
      tone(SOUT, NOTE_B4);
    }
    
    else if (string == 3 && finger == 2){
      tone(SOUT, NOTE_C5);
    }
    
    else if (string == 3 && finger == 3){
      tone(SOUT, NOTE_D5);
    }
    
    else if (string == 3 && finger == 4){
      tone(SOUT, NOTE_E5);
    }
    
    else if (string == 4 && finger == 0){
      tone(SOUT, NOTE_E5);
    }
    
    else if (string == 4 && finger == 1){
      tone(SOUT, NOTE_F5);
    }
    
    else if (string == 4 && finger == 2){
      tone(SOUT, NOTE_G5);
    }
    
    else if (string == 4 && finger == 3){
      tone(SOUT, NOTE_A5);
    }
    
    else if (string == 4 && finger == 4){
      tone(SOUT, NOTE_B5);
    }
    
    else{
     noTone(SOUT); 
    }
  }
  
  //if the right hand is not in motion
  //play no sound
  /*else{
     noTone(SOUT); 
  }
  */
}
