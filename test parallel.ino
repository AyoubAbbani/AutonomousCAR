#include<Servo.h>
Servo monservo;
    
const int EnableL = 5;
const int HighL = 6;       // Left side motors
const int LowL =7;

const int EnableR = 10;
const int HighR = 8;       // right side motors
const int LowR = 9;




const int D0 = 0;       //Raspberry pin 21    LSB
const int D1 = 1;       //Raspberry pin 22
const int D2 = 2;       //Raspberry pin 23
const int D3 = 3;       //Raspberry pin 24    MSB

int a,b,c,d,data;


void setup() {

pinMode(11,OUTPUT);
pinMode(EnableL, OUTPUT);
pinMode(HighL, OUTPUT);
pinMode(LowL, OUTPUT);
monservo.attach(11);
monservo.write(64);

pinMode(EnableR, OUTPUT);
pinMode(HighR, OUTPUT);
pinMode(LowR, OUTPUT);

pinMode(D0, INPUT_PULLUP);
pinMode(D1, INPUT_PULLUP);
pinMode(D2, INPUT_PULLUP);
pinMode(D3, INPUT_PULLUP);


 
 //Serial.begin(9600);


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
   monservo.write(64);
  
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}


void Backward()
{
 monservo.write(64);
 
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,245);

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
  monservo.write(58);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}

void Left2()
{
   monservo.write(53);
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}


void Left3()
{
  monservo.write(50);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);
  
}

void Right1()
{
  monservo.write(70);
 
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);  //200
  
}
void Right2()
{
  monservo.write(76);
 
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);   //160
  
}

void Right3()
{
  monservo.write(82);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,245);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,255);   //100
  
}



void loop() 
{
 
  Data();

if(data==0)
     {
        Forward();
       }
   else if(data==1)
   {
     Right1();
     
   }
     
  else if(data==2)
   {
     Right2();
      }
     
  else if(data==3)
   {
     Right3();
       }
     
  else if(data==4)
   {
     Left1();
     
   }
    
  else if(data==5)
   {
     Left2();
    
   }
    
  else if(data==6)
   {
     Left3();
    
   }   
  else if(data==7)
   {
     analogWrite(EnableL,0);
     analogWrite(EnableR,0);
     delay(4000);
     
     analogWrite(EnableL,255);
     analogWrite(EnableR,255);
     delay(1000);
     
   }   
  else  if(data==8)
   {
     analogWrite(EnableL,0);
     analogWrite(EnableR,0);
     delay(4000);
     
     analogWrite(EnableL,255);
     analogWrite(EnableR,255);
     delay(1000);
     
   } 
   
   else if(data==9)
   {
     analogWrite(EnableL,0);
     analogWrite(EnableR,0);
     delay(5000);

     Left3();
     delay(10000);

     Forward();
     delay(10000);

     Right3();
     delay(10000);
     
     Forward();
     delay(10000);
     
      } 
   
   else if(data==10)
   {
     Stop();
   }
   else if (data>7)
   {
     Stop();
   }
   
   


}
