#define trigPin 4
#define echoPin 13
const int EnableL = 5;
const int HighL = 6;       // LEFT SIDE MOTOR
const int LowL =7;

const int EnableR = 10;
const int HighR = 8;       //RIGHT SIDE MOTOR
const int LowR =9;

const int D0 = 11;       //Raspberry pin 21    LSB
const int D1 = 12;       //Raspberry pin 22
const int D2 = 2;       //Raspberry pin 23
const int D3 = 3;       //Raspberry pin 24    MSB

long sure =0;
long mesafe = 0;

int a,b,c,d,data;
const int max = 120;
const int low = 10;
const int medium = 25;
const int high = 45;

void setup() {
pinMode(EnableL, OUTPUT);
pinMode(HighL, OUTPUT);
pinMode(LowL, OUTPUT);

pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

pinMode(EnableR, OUTPUT);
pinMode(HighR, OUTPUT);
pinMode(LowR, OUTPUT);

pinMode(D0, INPUT_PULLUP);
pinMode(D1, INPUT_PULLUP);
pinMode(D2, INPUT_PULLUP);
pinMode(D3, INPUT_PULLUP);

delay(300);

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
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,max-20);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,max-20);
  
}
void Stop()
{

  analogWrite(EnableL,0);
  analogWrite(EnableR,0);
  
}
void Left1()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,high);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,max);
  
}
void Left2()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,medium);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,max);
  
}
void Left3()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,low);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,max);
  
}
void Right1()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,max);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,high);  //200
  
}
void Right2()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,max);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,medium);   //160
  
}
void Right3()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,max);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,low);   //100
  
}
void Stop_sign(){
    analogWrite(EnableL,0);
    analogWrite(EnableR,0);
    delay(3000);

    analogWrite(EnableL,110);
    analogWrite(EnableR,110);
    delay(800);
}
void Car(){

  analogWrite(EnableL, 0);
  analogWrite(EnableR, 0);            //stop
  delay(1000);

  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);        //left
  analogWrite(EnableL, 150);
  analogWrite(EnableR, 150);
  delay(500);

  analogWrite(EnableL, 0);
  analogWrite(EnableR, 0);            //stop
  delay(200);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);           //forward
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableL, 140);
  analogWrite(EnableR, 140);
  delay(600);

  analogWrite(EnableL, 0);           //stop
  analogWrite(EnableR, 0);
  delay(200);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  digitalWrite(HighR, HIGH);         //right
  digitalWrite(LowR, LOW);
  analogWrite(EnableL, 150);
  analogWrite(EnableR, 150);
  delay(500);

  analogWrite(EnableL, 0);               //stop
  analogWrite(EnableR, 0);
  delay(1000);

  
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  digitalWrite(HighR, LOW);       // forward
  digitalWrite(LowR, HIGH);
  analogWrite(EnableL, 135);
  analogWrite(EnableR, 135);
  delay(1000);

    analogWrite(EnableL, 0);               //stop
  analogWrite(EnableR, 0);
  delay(1000);

    digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  digitalWrite(HighR, HIGH);         //right
  digitalWrite(LowR, LOW);
  analogWrite(EnableL, 150);
  analogWrite(EnableR, 150);
  delay(500);

    analogWrite(EnableL, 0);               //stop
  analogWrite(EnableR, 0);
  delay(200);

  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);           //forward
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableL, 135);
  analogWrite(EnableR, 135);
  delay(500);

  analogWrite(EnableL, 0);               //stop
  analogWrite(EnableR, 0);
  delay(1000);

    digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);        //left
  analogWrite(EnableL, 160);
  analogWrite(EnableR, 160);
  delay(500);

    analogWrite(EnableL, 0);               //stop
  analogWrite(EnableR, 0);
  delay(1000);
}
void loop() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  sure = pulseIn(echoPin, HIGH);

  mesafe = (sure / 2) * 0.0348;

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
    Stop();
  }
   else if(data==8) 
  {
    Stop_sign();
  }
  else if(data==9){
    Car();
  }
  else if (data == 10){
    Stop();
  }
  if (mesafe <= 15) {

    Car();
  }

}
