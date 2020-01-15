/*  SandRover Kshitij 2020
 *   AMCS Department
 *   BATCH:2018-2023
 *   14/01/2020
 */
#include <Servo.h>

//initialize pin numbers
int channel_1 = 5;
int channel_2 = 6;
int channel_5 = 10;

int channel_1_input;
int channel_2_input;
int channel_5_input;

float x;
float y;

int motorrhs_data1 = 4;   //rhs data 1
int motorrhs_data2 = 2;   //rhs data 2
int motorlhs_data1 = 8;   //lhs data 1
int motorlhs_data2 = 12;  //lhs data 2
int ENABLE1 = 3;          //rhs enable  
int ENABLE2 = 11;         //lhs enable

float speedunit = 150;    
bool isStopped = false;

void setup() {
  Serial.begin(9600);
  pinMode(channel_1,INPUT);
  pinMode(channel_2,INPUT);
  pinMode(channel_5, INPUT);

  pinMode(motorlhs_data1,OUTPUT);
  pinMode(motorlhs_data2,OUTPUT);
  pinMode(ENABLE1,OUTPUT);
  pinMode(ENABLE2,OUTPUT);
  pinMode(motorrhs_data1,OUTPUT);
  pinMode(motorrhs_data2,OUTPUT);
  pinMode(3, OUTPUT);  
  
  digitalWrite(3, HIGH);
  
}
/*
 * Channel 1 --> Left and Right [L_AVG:1038 , R_AVG:1791]
 * Channel 2 -->Forward and Backward [F_AVG:1743 , B_AVG:1034]
 * Channel 5 -->Speed [ MAX:2001 , MIN:1397]
 */

float normalize(int channel, float inp)
{
  switch(channel)
  {
    case 1: 
      return neutralize(((inp-1414.5)/376.5)*100, -20, 20); //neutralize(((inp-((1038+1791)/2.0))/((1791-1038)/2.0))*100, -20, 20); 
    case 2:
      return neutralize(((inp-1388.5)/354.5)*100, -20, 20); //neutralize(((inp-((1034+1743)/2.0))/((1743-1034)/2.0))*100, -20, 20); 
    case 3:
      return ((inp - 1135)/(float)(1767-1135)*100.0*1.8);
    case 5:
      return neutralize((((inp - 1397)/(604))*100), 0, 0);//neutralize((((inp - 980)/(2001-1397)/2)*100), 0, 0);
  }
}

float neutralize(float inp, int lim_l, int lim_r)
{
  if(inp >= lim_l && inp <= lim_r)
  {
    return 0;
  }else if(inp > 100)
  {
    return 100;
  }else if(inp < -100)
  {
    return -100;
  }
  else
  {
    return inp;
  }
}

void setDirection(int state, int motorpin_1, int motorpin_2)
{
  switch(state)
  {
    case 0:  // 0->Stop
        digitalWrite(motorpin_1,LOW);
        digitalWrite(motorpin_2,LOW);
        break;

    case -1:  //-1-> Reverse
        digitalWrite(motorpin_1,LOW);
        digitalWrite(motorpin_2,HIGH);
        break;

    case 1:   //1->Forward
        digitalWrite(motorpin_1,HIGH);
        digitalWrite(motorpin_2,LOW);
        break;
  }
}

void rotate(float Rwheel_mag,float Lwheel_mag)
{
    analogWrite(ENABLE1,abs((Rwheel_mag*speedunit)/100.0));
    analogWrite(ENABLE2,abs((Lwheel_mag*speedunit)/100.0));  
}

void rpm_calc()
{
    if(y==0)          
    {
      if(x==0)        //STOP
      {
        isStopped = true;
        setDirection(0,motorrhs_data1,motorrhs_data2);
        setDirection(0,motorlhs_data1,motorlhs_data2);
      }
      else
      {
        if(x>0)       //RIGHT
        {
          setDirection(-1,motorrhs_data1,motorrhs_data2);
          setDirection(1,motorlhs_data1,motorlhs_data2);
          rotate(x,x);
        }
        else          //LEFT
        {
          setDirection(1,motorrhs_data1,motorrhs_data2);
          setDirection(-1,motorlhs_data1,motorlhs_data2);
          rotate(x,x);
        }
      }
    }
    else if(y>0)
    {
      if(x>=0)      //QUAD-1
      {
          setDirection(1,motorrhs_data1,motorrhs_data2);
          setDirection(1,motorlhs_data1,motorlhs_data2);
          rotate(max(y-x,0),y);
      }
      else          //QUAD-2
      {
          setDirection(1,motorrhs_data1,motorrhs_data2);
          setDirection(1,motorlhs_data1,motorlhs_data2);
          rotate(y,max(y+x,0));
      }
    }
    else
    {
      if(x>=0)    //QUAD-4
      {
          setDirection(-1,motorrhs_data1,motorrhs_data2);
          setDirection(-1,motorlhs_data1,motorlhs_data2);
          rotate(max(-y-x,0),-y);
      }
      else        //QUAD-3
      {
          setDirection(-1,motorrhs_data1,motorrhs_data2);
          setDirection(-1,motorlhs_data1,motorlhs_data2);
          rotate(-y,max(-y+x,0));
      }      
    }
}

void loop() {
   channel_1_input = pulseIn(channel_1, 25000);
   channel_2_input = pulseIn(channel_2, 25000);
   channel_5_input = pulseIn(channel_5, 25000)*0.6+800;

   speedunit = ((normalize(5, channel_5_input) * 255)/100);
    x = normalize(1,channel_1_input);
   Serial.println(x);
   y = normalize(2,channel_2_input);
   Serial.println(y);
   rpm_calc();
   

}
