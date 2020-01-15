/*  DroidBlitz Kshitij 2020
 *   AMCS Department
 *   BATCH:2018-2023
 *   14/01/2020
 */
#include <Servo.h>

//initialize pin numbers
int channel_1 = 5;
int channel_2 = 6;
int channel_3 = 7;
int channel_5 = 13;
int downAnglemax = 0;
int upAnglemax = 0;
int channel_1_input;
int channel_2_input;
int channel_3_input;
int channel_5_input;

float x;
float y;

int motorrhs_data1 = 2;   //rhs data 1
int motorrhs_data2 = 4;   //rhs data 2
int motorlhs_data1 = 8;   //lhs data 1
int motorlhs_data2 = 12;  //lhs data 2
int ENABLE1 = 3;          //rhs enable  
int ENABLE2 = 11;         //lhs enable
int servopinA = 9;
int servopinB = 10;
float prevAngleA = 0;
float prevAngleB = 0;
float currentAngleA;
int angleabs = 0;

Servo servoA;
Servo servoB;

float speedunit = 255;    
bool isStopped = false;

void setup() {
  Serial.begin(9600);
  pinMode(channel_1,INPUT);
  pinMode(channel_2,INPUT);
  pinMode(channel_3,INPUT);
  pinMode(channel_5, INPUT);
  
  pinMode(motorlhs_data1,OUTPUT);
  pinMode(motorlhs_data2,OUTPUT);
  pinMode(ENABLE1,OUTPUT);
  pinMode(ENABLE2,OUTPUT);
  pinMode(motorrhs_data1,OUTPUT);
  pinMode(motorrhs_data2,OUTPUT);
  pinMode(3, OUTPUT);  
  servoA.attach(servopinA);
  servoB.attach(servopinB);
  
  digitalWrite(3, HIGH);
  
}
/*
 * Channel 1 --> Left and Right [L_AVG:1038 , R_AVG:1791]
 * Channel 2 -->Forward and Backward [F_AVG:1743 , B_AVG:1034]
 * Channel 5 -->Speed [ MAX:2001 , MIN:1397]
 */
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

float normalize(int channel, float inp)
{
  switch(channel)
  {
    case 1: 
      return neutralize(((inp-1418)/355.0)*100, -20, 20); //neutralize(((inp-((1063+1773)/2.0))/((1773-1063)/2.0))*100, -20, 20); 
    case 2:
      return neutralize(((inp-1395.5)/310.5)*100, -20, 20); //neutralize(((inp-((1085+1706)/2.0))/((1706-1085)/2.0))*100, -20, 20); 
    case 3:
       angleabs = (((inp - 1100)/(float)(1730-1100)*100.0*1.8));

//        angleabs = (inp - 1415)/3.5;
//       Serial.println(inp);
//       Serial.println(angleabs);
//       Serial.print("\n");
       return angleabs;
//       if(angleabs<0){
//         angleabs = 0;
//       }else if (angleabs>180)
//       {
//          angleabs = 180;
//       }
//       return 180-angleabs;
    case 5:
      return neutralize((((inp - 1392)/(602))*100), 0, 0);//neutralize((((inp - 1392)/(1994-1392)/2)*100), 0, 0);
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
          rotate(-x,-x);
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

void driveCrane()
{
  float pos_s1;
  float pos_s2;
  if( prevAngleA > currentAngleA )
  {
    for(pos_s1 = prevAngleA, pos_s2 = prevAngleB; pos_s1 >= currentAngleA; pos_s1 -= 2, pos_s2 -= 2)
    {
      servoB.write( pos_s2);
      delay( 15 );
      servoA.write( pos_s1 );
      delay( 15 );
//      Serial.print(pos_s1);
//      Serial.print("\t");
//      Serial.println(pos_s2);
    }
  }
  else 
  {
    for(pos_s1 = prevAngleA, pos_s2 = prevAngleB; pos_s1 <= currentAngleA; pos_s1 += 2, pos_s2 += 2)
    {
      servoB.write( pos_s2);
      delay( 15 );
      servoA.write( pos_s1 );
      delay( 15 );
//      Serial.print(pos_s1);
//      Serial.print("\t");
//      Serial.println(pos_s2);
    }
  }
  prevAngleA = pos_s1;
  prevAngleB = pos_s2;
    
}
void loop() {
   channel_1_input = pulseIn(channel_1, 25000);
   channel_2_input = pulseIn(channel_2, 25000);
   channel_3_input = pulseIn(channel_3, 25000);
   channel_5_input = pulseIn(channel_5, 25000)*0.6+800;
   //Serial.println(channel_3_input);

   speedunit = ((normalize(5, channel_5_input) * 255)/100);
    x = normalize(1,channel_1_input);
   y = normalize(2,channel_2_input);
   currentAngleA = normalize(3,channel_3_input);
   //Serial.print(currentAngleA);
   //Serial.print("\n");
   rpm_calc();
   if(abs(prevAngleA-currentAngleA)>=15 && currentAngleA>=25)
   {
     driveCrane();
   }

}
