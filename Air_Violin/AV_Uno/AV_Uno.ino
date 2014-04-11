//Meghan Jimenez and Antoine Billig

#define LED1 12
#define LED2 11
#define LED3 10
#define LED4 9
#define LED5 8

void setup(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
}

void loop(){
  int STR1 = analogRead(0);
  int STR2 = analogRead(1);
  int FIN1 = analogRead(2);
  int FIN2 = analogRead(3);
  int MOV = analogRead(4);
  
  if (!STR1 && !STR2){
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  }
  else if (STR1 && !STR2){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  }
  else if (STR2 && !STR1){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, LOW);
  }
  else if (STR2 && STR1){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, HIGH);
  }
  else if (MOV){
  digitalWrite(LED5, HIGH);
  }
}
