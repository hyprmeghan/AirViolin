//Meghan Jimenez and Antoine Billig

#include "pitches.h"

#define LED1 12
#define LED2 11
#define LED3 10
#define LED4 9
#define LED5 8
#define SOUT 13

void setup(){
  Serial.begin(57600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  
  pinMode(SOUT, OUTPUT);
}

void loop(){
  int STR1 = analogRead(0);
  int STR2 = analogRead(1);
  int FIN1 = analogRead(2);
  int FIN2 = analogRead(3);
  int FIN3 = analogRead(4);
  int MOV = analogRead(5);
  
  Serial.println(FIN1);
  Serial.println('\t');
  Serial.println(FIN2);
  Serial.println('\t');
  Serial.println(FIN3);
  Serial.println('\t');
  
  if (FIN3){
    noTone(8);
  }
  
  else if (!STR1 && !STR2 && MOV){
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
    if (!FIN1 && !FIN2){
      tone(SOUT, NOTE_A3);
    }
    else if (FIN1 && !FIN2){
      tone(SOUT, NOTE_B3);
    }
    else if (!FIN1 && FIN2){
      tone(SOUT, NOTE_C4);
    }
    else if (FIN1 && FIN2){
      tone(SOUT, NOTE_D4);
    }
    delay(200);
  }
  else if (STR1 && !STR2 && MOV){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  if (!FIN1 && !FIN2){
      tone(SOUT, NOTE_E4);
    }
    else if (FIN1 && !FIN2 && MOV){
      tone(SOUT, NOTE_F4);
    }
    else if (!FIN1 && FIN2){
      tone(SOUT, NOTE_G4);
    }
    else if (FIN1 && FIN2){
      tone(SOUT, NOTE_A4);
    }
    
    delay(100);
  }
  else if (STR2 && !STR1 && MOV){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, LOW);
  if (!FIN1 && !FIN2){
      tone(SOUT, NOTE_B4);
    }
    else if (FIN1 && !FIN2){
      tone(SOUT, NOTE_C5);
    }
    else if (!FIN1 && FIN2){
      tone(SOUT, NOTE_D5);
    }
    else if (FIN1 && FIN2){
      tone(SOUT, NOTE_E5);
    }
    
    delay(100);
  }
  else if (STR2 && STR1 && MOV){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, HIGH);
  if (!FIN1 && !FIN2){
      tone(SOUT, NOTE_FS5);
    }
    else if (FIN1 && !FIN2){
      tone(SOUT, NOTE_G5);
    }
    else if (!FIN1 && FIN2){
      tone(SOUT, NOTE_A5);
    }
    else if (FIN1 && FIN2){
      tone(SOUT, NOTE_B5);
    }
    
    delay(100);
  }
  
  else{
  noTone(SOUT);
  }
  
  if (MOV){
  digitalWrite(LED5, HIGH);
  }
  
  //delay(50);
}
