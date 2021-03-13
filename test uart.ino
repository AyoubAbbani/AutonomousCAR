#include <SoftwareSerial.h>
#include <stdlib.h>
#include<Servo.h>
Servo monservo;
    
const int EnableL = 5;
const int HighL = 6;       // Left side motors
const int LowL =7;

const int EnableR = 10;
const int HighR = 8;       // right side motors
const int LowR = 9;

const int trig = 13;
const int echo = 12;    //ultrasonic variables
long cm=0;
int distanceMesuree =0;


const int D0 = 0;       //Raspberry pin 21    LSB
const int D1 = 1;       //Raspberry pin 22
const int D2 = 2;       //Raspberry pin 23
const int D3 = 3;       //Raspberry pin 24    MSB

int a,b,c,d,data;
char num;

void setup() {

pinMode(11,OUTPUT);
pinMode(EnableL, OUTPUT);
pinMode(HighL, OUTPUT);
pinMode(LowL, OUTPUT);
monservo.attach(11);
monservo.write(70);

pinMode(EnableR, OUTPUT);
pinMode(HighR, OUTPUT);
pinMode(LowR, OUTPUT);

pinMode(D0, INPUT_PULLUP);
pinMode(D1, INPUT_PULLUP);
pinMode(D2, INPUT_PULLUP);
pinMode(D3, INPUT_PULLUP);

//setup ultras

 //pinMode(trig,OUTPUT);
 //digitalWrite(trig,LOW);
 //pinMode(echo,INPUT);
 //pinMode(4,OUTPUT);
 //digitalWrite(4, LOW);
 
 Serial.begin(9600);


}

void Data()
{
   a = digitalRead(D0);
   b = digitalRead(D1);
   c = digitalRead(D2);
   d = digitalRead(D3);

   data = 8*d+4*c+2*b+a;
}

void Forward()
{
   monservo.write(70);
   delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}


void Backward()
{
 monservo.write(70);
 delay(10);
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,255);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,255);
  
}

void Stop()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  
}

void Left1()
{
  monservo.write(66);
  delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}

void Left2()
{
   monservo.write(55);
   delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}


void Left3()
{
  monservo.write(50);
  delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  Serial.begin(9600);
}

void Right1()
{
  monservo.write(78);
  delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);  //200
  
}
void Right2()
{
  monservo.write(85);
  delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);   //160
  
}

void Right3()
{
  monservo.write(92);
  delay(10);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,255);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);   //100
  
}

boolean LaVoieEstLibre()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  cm =pulseIn(echo,HIGH)/58;
  

  if(cm < 8)  
  {return true;}
  else
  {return false;}   
}


void loop() 
{
 
//If the data is available serially, then receive it
  if(Serial.available()>0){
    num = Serial.read();
    
  if(num=='0')
     {
        Forward();
        Serial.print("forward =  \t ");Serial.println(num);
       delay(10);
       }
   else if(num=='1')
   {
     Right1();
     Serial.print("right1 =  \t ");Serial.println(num);
     delay(10);
   }
     
  else if(num=='2')
   {
     Right2();
     Serial.print("Right2 =  \t ");Serial.println(num);
     delay(10);
   }
     
  else if(num=='3')
   {
     Right3();
     Serial.print("Right3 =  \t ");Serial.println(num);
     delay(10);
   }
     
  else if(num=='4')
   {
     Left1();
     Serial.print("Left1 =  \t ");Serial.println(num);
     delay(10);
   }
    
  else if(num=='5')
   {
     Left2();
     Serial.print("Left2 =  \t ");Serial.println(num);
     delay(10);
   }
    
  else if(num=='6')
   {
     Left3();
     Serial.print("Left3 =  \t ");Serial.println(num);
     delay(10);
   }   
  else if(num=='7')
   {
     analogWrite(EnableL,0);
     analogWrite(EnableR,0);
     delay(4000);
     Serial.print("Stop sign =  \t ");Serial.println(num);
     analogWrite(EnableL,255);
     analogWrite(EnableR,255);
     delay(1000);
     
   }   
  else if (num=='8')
   {
     Stop();
     Serial.print("Red light =  \t ");Serial.println(num);
   }else if (num=='9')
   {
     Stop();
     Serial.print("Pedestrian =  \t ");Serial.println(num);
   }else if (num=='A')
   {
     Stop();
     Serial.print("Stop Obstacle =  \t ");Serial.println(num);
   }
   
   


}
}
