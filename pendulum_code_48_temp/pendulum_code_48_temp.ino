
//testing to see if github works
/* TEMP  note to SElf
position = amplitude*cos(Time*360*Period)     //I do believe

*TO DO
Make own AccelMAX on motor controller very hight,  then use the sinusoidal equation above to modulate the position  
Make variable for idealPosition (at this time) defined by the sinusoidal, and then continually modulate the motorSpeed value toward this position
Make variable for idealPeriod then modulate the amplitudes and speeds toward this ideal
***make pos=pos+(rotation*1000);
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <TimerOne.h>

#define rxPin 4  // pin 3 connects to smcSerial TX  (not used in this example)
#define txPin 5  // pin 4 connects to smcSerial RX

// some motor limit IDs
#define FORWARD_ACCELERATION 5
#define REVERSE_ACCELERATION 9

//define slave i2c address
#define THIS_ADDRESS 8
#define ADDRESS 8

SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);

//create object
EasyTransferI2C ETin; 
EasyTransferI2C ETout; 

const int posPin=A3;   //analog input pin for position sensor
const int mSpeedPin = A0;  // Analog input pin for motor sensor1
const int mAmpPin = A1;  // Analog input pin for motor sensor2
//const int centerPin = A2;  //analog input pin to determine center 
const int accelPin = A2;  //analog input pin to determine center height

const float pi = 3.141592; 

//const int decelPin = A4;    ///*********** On UNO this conclicts with i2c pins
//const int centerPin =A5;    ///*********** On UNO this conclicts with i2c pins


const int  interuptPin1 =  2;
const int interuptPin2 =3;


int maxAccel= 3000;  //500;   //1000;  //0 to 3200          //faster maxAccel so that idealPos can control speed
int  maxDecel= maxAccel; //500;    //1000;  //0 to 3200

byte  maxAccel_b1= maxAccel & 0x7F;  //first byte of int
byte  maxAccel_b2= maxAccel >> 7;  //second byte of int
byte  maxDecel_b1= maxDecel & 0x7F;  //first byte of int
byte  maxDecel_b2= maxDecel >> 7;  //second byte of int

long pos= 1;   //position value read by sensor  and multiplied by rotation value
long pos_mapped=0;   //mapped to degrees from 0 to 359
long posPrev=pos;
long posPrev2=pos;
int mSpeed = 50;        // value read from the pot
int speedLeft=50;       //speed going left
int speedRight=50;      //speed going right
int mAmp = 150;        // value read from pot.  amplitude of wave
long rotation=100;  // counts number of rotations    //ofset intitial value to avoid problems when this goes negative
long offset = rotation * 1000;   //initial position offset determined by initial rotation value,   meant to avoid problems when negative
long posActual;   //value equal to offsetPos-offset;

long idealCenter = 500 + offset;      // determines center of pendulum motion
long center= idealCenter;   // this is the measured center of swing
long centerPrev=center;
long centerOff = center - idealCenter;     // how much the actual center is off from the ideal, value can be pos or neg.     
int interupt1 = 0;
int interupt2 =0;
long interuptCount=0;
/*int mCount1=0;   //counts number of tiggers of mSensor1
int mCount2=0;  //counts number of tiggers of mSensor2
int mCountBoth=0;  //counts number of times that both mSense1 and mSense2 are simultaneously triggered
int mCountHeight=42;  //adds counts height based on mCountBoth and dirSense of height//   42 is height of first hall sensor
*/
int mDir=1;     //direction of motor  (1 or -1)
 
 long leftMaxPos;   //maximum position of left swing (after decel) ,   initial value or right swing
 long rightMaxPos;   //maximum position of right swing (after decel) ,   initial value or left swing
 long idealLeftMax;   // ideal max position of left swing
 long idealRightMax;  //ideal max pos of right swing
 long leftMaxOff;     //measurement of how much we overshot on the left
 long rightMaxOff;   //measurement of how much we overshot on the right
 boolean overshotLeft;     // true if  we overshoot on the left
 boolean overshotRight;    // true if  we overshoot on the right
 int rotationInitLeft=0;
 int rotationInitRight=0;
 long deltaPosLeft;
 long deltaPosRight;
 int velLeft;  //average velocity for for swing left
 int velRight;  // average velocity for swing right
 
 long timeVel;   //time of last velocity update
 long timeVelPrev;  //time of veococity update 2 times ago
 long timeDeltaVel;    //current time - time of last Velocity update (timeVel)
 long deltaPosVel;     // = pos-posVel;  //change of positoin since last velcity update 
// long deltaPosVel2;     // = pos-posVelPerv;  //change of positoin since 2 previous velcity updates  
 long posVel=pos; //position of current velocity check
 long deltaTimeVel;    //=millis()-timeVel;  //current time - time of last velocity check
// long deltaTimeVel2;   //millis()-timeVelPrev;     //time passed since 2 velocity checks
 
 
 long idealPos;   //position = amplitude*cos(Time*2pi / Period)
long idealPer  = 4300;  //in milliseconds  //set period   how long (how many counts) it should take for a complete rotation
long idealVel;   //vel = amplitude*cos(Time*2pi / Period + pi/2)   // i think this is right
int idealMotorSpeed;   //idealVel mapped to 0 to 100
float speedMultiplier = .8;   //   used to adjust mSpeed toward ideal curve
float speedMultLeft=.8;    //  adjusts  speed moving left
float speedMultRight=.8;    //  adjusts  speed moving right
int idealAccel=1;  // how much to incriment ideal pos value per cycle
boolean callAdjustSpeed2=0;  // flag to call function adjustSpeed2() to be called outside of the interupt routine
boolean slowLeft=0;   //flag to tell function to slow left
boolean slowRight=0;   //flag  to slow right
boolean increaseLeft=0; //flag  to speed up left
boolean increaseRight=0; //flag to speedup right

 
int dirSense=1;  //var determined by what sensors say about motor direction  1 or -1
boolean knobs= 0;   // if on, knobs are used, if not knobs are not used to automatically change values

int counter1_running=0;
int counter2_running=0;
 
  unsigned long time1=micros();
 unsigned long timePrev=micros();
  unsigned long timePrev2=micros();
 unsigned long time2=micros();
unsigned long timePeriod=0;
unsigned long timeLeft=0;
unsigned long timeRight=0;
unsigned long timeLeft2=0;    //value obtained after slowdown
unsigned long timeRight2=0;    //value obtained after slowdown
 
 
 long deltaPos= pos-posPrev;
 long deltaPos2= pos-posPrev2;
 long deltaTime= time1-timePrev; 
 
long vel=0;           //measured velocity
long velPrev=0;


long accel=0;   // measured aacceleration
//long accel2=0;      //measured accross two time intervals




struct DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    long idealPer;  
    long pos;    //*** make sure to change this on other one
    long center;          
    long time1;
  //  int timeLeft;
  //  int timeRight;
  //  unsigned long pos;
  //  long vel;
  //  long accel;
  //  int maxAccel;
  //  int mAmp;
 //   int mSpeed;
  //  unsigned long timePeriod;
  //  unsigned long interuptCount;
};


//give a name to the group of data
DATA_STRUCTURE dataIn;
DATA_STRUCTURE dataOut;



// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcSerial.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  if (speed < 0)
  {
    smcSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcSerial.write(0x85);  // motor forward command
  }
  smcSerial.write(speed & 0x1F);
  smcSerial.write(speed >> 5);
}


// speed should be a number from -99 to 99
void setMotorSpeedPercent(int motorSpeed)
{
  if(motorSpeed<0)
  {  motorSpeed=abs(motorSpeed);
     mDir=-1;
  }  
  if( motorSpeed>99)
      { motorSpeed=99;}
  
  byte zero= 0x00;
 // smcSerial.write(0xAAD050099);
     smcSerial.write(0xAA); // device 13
    smcSerial.write(13); // device 13
  if (mDir < 0)
  {
       smcSerial.write(0x06);  // motor reverse command   // motorSpeed = -motorSpeed;  // make speed positive
       //motorSpeed= speedLeft;
  }
  else
  {
         smcSerial.write(0x05);  // motor forward command
         //motorSpeed= speedRight;
  }
  smcSerial.write(zero);
  smcSerial.write(motorSpeed);
}

 
void setMotorLimits()
{
  byte zero=0x00;
  byte  maxAccel_b1= maxAccel & 0x7F;  //first byte of int
  byte  maxAccel_b2= maxAccel >> 7;  //second byte of int
  byte  maxDecel_b1= maxDecel & 0x7F;  //first byte of int
  byte  maxDecel_b2= maxDecel >> 7;  //second byte of int
 
  smcSerial.write(0xA2);
  smcSerial.write(5);  //Forward accel id
  smcSerial.write(maxAccel & 0x7F);
  smcSerial.write(maxAccel_b2);
  smcSerial.write(zero); 
  
  smcSerial.write(0xA2);
  smcSerial.write(9);  //Reverse accel id
  smcSerial.write(maxAccel & 0x7F);
  smcSerial.write(maxAccel_b2);
  smcSerial.write(zero); 
  
  smcSerial.write(0xA2);
  smcSerial.write(2); //deceleration id
  smcSerial.write(maxDecel & 0x7F);
  smcSerial.write(maxDecel_b2);
  smcSerial.write(zero); 
  
 /* smcSerial.write(0xAA);
  smcSerial.write(13);  //device number
  smcSerial.write(0x22);
  smcSerial.write(1);    //limit ID  //Max acceleration forward
  smcSerial.write(maxAccel_b1);   //limit Byte 1
  smcSerial.write(maxAccel_b2); //limit Byte 2
//  smcSerial.write(zero);    //Limit Byte 2
  
  
  smcSerial.write(0xAA);
  smcSerial.write(13);  //device number
  smcSerial.write(0x22);
  smcSerial.write(2);    //limit ID  //Max deceleration forward
  smcSerial.write(maxDecel_b1);   //limit Byte 1
  smcSerial.write(maxDecel_b2); //limit Byte 2
//  smcSerial.write(zero);    //Limit Byte 2
  
  
  smcSerial.write(0xAA);
  smcSerial.write(13);  //device number
  smcSerial.write(0x22);
  smcSerial.write(9);    //limit ID  //Max acceleration reverse
  smcSerial.write(maxDecel_b1);   //limit Byte 1
  smcSerial.write(maxDecel_b2); //limit Byte 2

//  smcSerial.write(zero);    //Limit Byte 2
  
  
  smcSerial.write(0xAA);
  smcSerial.write(13);  //device number
  smcSerial.write(0x22);
  smcSerial.write(10);    //limit ID  //Max deceleration forward
  smcSerial.write(maxDecel_b1);   //limit Byte 1
  smcSerial.write(maxDecel_b2); //limit Byte 2

// smcSerial.write(zero);    //Limit Byte 2
*/
}

 
void setup()
{
  
   pinMode( interuptPin1, INPUT_PULLUP);   
   pinMode( interuptPin2, INPUT_PULLUP);
    pinMode(10, OUTPUT);      // sets the digital pin as output
    pinMode(11, OUTPUT);      // sets the digital pin as output
   

     Wire.begin(THIS_ADDRESS);
    //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
    ETin.begin(details(dataIn), &Wire);
     Wire.onReceive(receive);
   
     ETout.begin(details(dataOut), &Wire);


  // initialize software serial object with baud rate of 19.2 kbps
    smcSerial.begin(19200);//115200);   //19200);   //****IMPT***I had to manually change the motor controller settings to a fixed baud rate using the software that came with it
 
    Timer1.initialize(10000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here

 
  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);
 
 Serial.begin(19200);//115200);
  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  smcSerial.write(0xAA);  // send baud-indicator byte
 
  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run
  exitSafeStart();  // clear the safe-start violation and let the motor run
  setMotorLimits();
}


void counter1()  //mSensor1 has changed value
{

}


void counter2()
{ 
  
}


void printValues()
{
    
      Serial.print(F("idealPos = "));
     Serial.print(idealPos-offset);
     Serial.print(F("     pos = "));
     Serial.print(pos-offset);
     Serial.print(F("     time1 = "));
     Serial.print(time1);
     Serial.print(F("      idealCenter = "));
     Serial.print(idealCenter-offset);
     Serial.print(F("      center = "));
     Serial.print(center-offset);
      Serial.print(F("     mSpeed = "));
     Serial.print(mSpeed);
     Serial.print(F("     idealMotorSpeed = "));
     Serial.print(idealMotorSpeed);
     Serial.print(F("      speedMultLeft =     "));
     Serial.print(speedMultLeft);
     Serial.print(F("      speedMultRight =     "));
     Serial.print(speedMultRight);
     Serial.print(F("     mDir = "));
     Serial.print(mDir);
     Serial.print(F("       rotation = "));
     Serial.print(rotation);
     Serial.print(F("        vel = "));
    Serial.print(vel);
    Serial.print(F("         idealVel = "));
    Serial.println(idealVel);
 /*   Serial.print(F("pos = "));    //actual position of pendulum
    Serial.print(pos); 
     Serial.print(F("            vel = "));
    Serial.print(vel);
    Serial.print(F("            rotation = "));
    Serial.print(rotation);


    Serial.print(F("            accel = "));
    Serial.print(accel);
    Serial.print("    deltaTime = ");
    Serial.print(deltaTime);
    Serial.print(F("                  timePeriod = "));    //actual position of pendulum
    Serial.print(timePeriod); 

 /*   Serial.print("            mSpeed= ");    //motor speed      read by pot
    Serial.print(mSpeed);
     Serial.print("          maxAccel= ");    //max accel     read by pot
    Serial.print(maxAccel);
      Serial.print("          maxDecel= ");    //max accel     read by pot
    Serial.print(maxDecel);*/
 /*   Serial.print(F("                time1 = "));
    Serial.print(time1);
    Serial.print(F("                timePrev = "));
    Serial.print(timePrev);
    Serial.print(F("                deltaPos= "));
    Serial.print(deltaPos);
    Serial.print(F("                deltaTime = "));
    Serial.print(deltaTime);
 /*   Serial.print("vel * 10,0000 = ");
    Serial.println(vel);
    Serial.println("");
*/      
 /*   Serial.print("          mAmp = ");    //how high pendulum oscilation is set   read by pot
    Serial.print(mAmp); 
    Serial.print("          center = ");   // center point of pendulum oscilation    read by pot
    Serial.println(center);
    Serial.print("               interuptCount = ");
    Serial.println(interuptCount);  */
    Serial.println("");
}

void printVars2()
{
   
    
    Serial.print(F("pos = "));
    Serial.print(posActual);
    Serial.print(F("    posPrev = "));
    Serial.print(posPrev-offset);
     Serial.print(F("    posPrev2 = "));
    Serial.print(posPrev2-offset);
     Serial.print(F("    deltaPos = "));
    Serial.print(deltaPos);
    Serial.print(F("            vel = "));
    Serial.print(vel);
     Serial.print(F("    velPrev = "));
    Serial.print(velPrev);
    Serial.print(F("            accel = "));
    Serial.print(accel);
    Serial.print(F("    deltaPos = "));
    Serial.print(deltaPos); 
     Serial.print(F("      Rotation = "));
    Serial.print(rotation);   
    Serial.print(F("    deltaTime = "));
    Serial.print(deltaTime);
    Serial.print(F("    interuptCount = "));
    Serial.println(interuptCount);
}    


void printStruct()
{
  Serial.println(F("________PRINT STRUCT_________"));
  Serial.print(F("dataOut.center = ")); 
  Serial.println(dataOut.center);
  /*  Serial.print(F("dataOut.timeLeft = "));
    Serial.println(dataOut.timeLeft);
    Serial.print(F("dataOut.timeRight"));
    Serial.println(dataOut.timeRight);*/
  Serial.print(F("dataIn.center = ")); 
  Serial.println(dataIn.center);
 /*   Serial.print(F("dataIn.timeLeft = "));
    Serial.println(dataIn.timeLeft);
    Serial.print(F("dataIn.timeRight"));
    Serial.println(dataIn.timeRight);*/
}

void updateVel()
/*if more than 10 positions have changed since last velocity update, then update velocity and acceleration values.  */
{
  int x =1;
  if(abs(pos-posVel)>x)   //if more than x positions have changed since last velocity update  then calculate new velocity
  {
      timeDeltaVel=millis()-timeVel;  //current time - time of last Velocity update
      deltaPosVel=pos-posVel;  //change of positoin since last velcity update
      velPrev=vel;      
      posVel= pos;   //position of this velocity check
      deltaTimeVel=millis()-timeVel;  //current time - time of last velocity check
      vel=(deltaPosVel*100)/deltaTimeVel;       //vel = change in position of change in time, scaled by 100 
      timeVelPrev=timeVel;
      timeVel=millis();     //time of velocity update
      
      accel=((vel-velPrev)*100)/deltaTimeVel;
  }   
  /*deltaTime= time1-timePrev;
  deltaTime2= time1-timePrev2;   //time of 2 intevervals
  vel=(deltaPos*1000)/deltaTime;
  vel2=(deltaPos2*1000)/deltaTime2;     //velocity calculated accross two time intervals
  accel=((vel-velPrev)*1000)/deltaTime;
  accel2=((vel-velPrev2)*1000)/deltaTime2;
  */
  
}

void updatePos()
{
  int temp;     
      posPrev2=posPrev;   //position from 2 reads ago
      posPrev=pos;
      //read position sensor*/
      temp=analogRead(posPin);   
      pos_mapped=map(temp, 0, 1023, 0, 359 ); // maps from 0 to 359 degrees for a full rotation
      pos=map(temp, 41, 986, 0, 1000);
      
      testRotationBorder();       //tests to see if the the position has crossed the border values of the sensor  ALso UPDATES POS
      pos=pos+(rotation*1000); 
      rotation=pos/1000;
      deltaPos= pos-posPrev;
      deltaPos2= pos-posPrev2;   //change in pos accross 2 time intervals
      posActual= pos-offset;
      //     timePrev=time1;
//      time1=micros();
}



void testRotationBorder()  //tests to see if the the position has crossed the border values of the sensor
{
      int temp2=posPrev%1000;    //   because posPrev is potentially > than 1000  while pos is <1000    thus temp2  adjusts for this in comparison
      int temp1=pos-temp2;    //posPrev
      if(abs(temp1)>500)    //then the threshhold has been crossed
      {
          if((temp2>700) && (pos<400))
            { rotation++;
           //  mDir=mDir*(-1);
           //   setMotorSpeed(mSpeed);
              }
          else if((temp2<400) && (pos>700))
            {   rotation--;
              }     
              
      }
      
}
  

void updateStructs()
{
    ETin.receiveData(); 
    dataOut.center = center;
 //   dataOut.timeLeft = timeLeft;
 //   dataOut.timeRight = timeRight;
     dataOut.idealPer=idealPer;  
     dataOut.pos=pos;
     dataOut.time1=time1;
   /* myData.vel=vel;
     myData.accel=accel;
     myData.maxAccel=maxAccel;
     myData.mAmp=mAmp;
    myData.center=center;
    myData.mSpeed=mSpeed;
    myData.timePeriod=timePeriod;
    myData.interuptCount=interuptCount;
*/
}

   
void updateVars()
{
    timePrev2=timePrev;
    timePrev=time1;
    time1=millis();
    updatePos();
    updateVel();
    updateStructs();
      // ETout.sendData(ADDRESS);            //send data to ADDRESS

      idealPos=long(mAmp*cos(time1*2*pi/idealPer)+idealCenter);// + idealCenter)); //position = amplitude*sin(Time*2*pi/Period)+idealCenter //Ideal Center is currently shifting the whole sine function up
      idealVel= long(mAmp*cos(time1*2*pi/idealPer+(pi/2)));    //vel = amplitude*sin(Time*2pi / Period)     //i bellieve this is right
      idealMotorSpeed=map(abs(idealVel),0,mAmp,0,99);       // mapped idealVel to motorSpeedPercent
      idealLeftMax=  idealCenter-mAmp;
      idealRightMax= idealCenter+mAmp;
      if(idealVel<0)   // ideal is moving left
      {   
          if(mDir==1)  //then this is the height on right side(before decel).  Time to change directions to match ideal.
          {
             mDir=-1;
             rightMaxPos=pos;
             centerPrev=center;
             center=(rightMaxPos+leftMaxPos)/2;    //the measured center
             centerOff= center-idealCenter; 
             rightMaxOff= rightMaxPos-idealRightMax;   //measures how far we overshot or undershot on the right 
             callAdjustSpeed2=1;   //sets flag for adjustSpeed2 to be called outside of this interupt routine;
             //adjustSpeed2();  
          }
          if(pos<idealLeftMax)  
          {  mSpeed=0;
             leftMaxPos=pos;  // overshootLeft=1;   //flag that tells us we are overshooting on the left
          }
          else  
            mSpeed=int(idealMotorSpeed*speedMultLeft);
          
      }
      else if(idealVel>0)   //moving right
      {
          if(mDir==-1)  //then this is the height on left side (before decel).  Time to change directions to match ideal.
          {
             mDir=1;
             leftMaxPos=pos;
             center=(rightMaxPos+leftMaxPos)/2;    //the measured center
             centerOff= center-idealCenter;  
            //adjustSpeed2(); 
            leftMaxOff= leftMaxPos-idealLeftMax;   //measures how far we overshot or undershot on the Left 
            callAdjustSpeed2=1;     //sets flag for adjustSpeed2 to be called outside of this interupt routine;
     
          }
          if(pos>idealRightMax)  
          {  mSpeed=0;
             //overShootRight=1;
          }
          else  
            mSpeed=idealMotorSpeed*speedMultRight;
            
      } 
      if((vel>0)&&(velPrev<0))  // then pendulum Actual changed directions, from right to left
      {
          rightMaxPos=pos; 
      }
      //Next part is redundant
      /*if((vel>0)&&(velPrev<0))  // then pendulum Actual changed directions, from right to left
      {
          rightMaxPos=pos; 
      }*/
      else if ((vel<0)&&(velPrev>0))  // then pendulum Actual changed directions, from left to right
      {
          leftMaxPos=pos;
      }   
      
    //end markout 

     /*  if((vel<0)&&(velPrev>0))   ||   ((vel>0)&&(velPrev<0))
         {  // then reset oscillation timer
           setMotorSpeedPercent(mSpeed);
      }*/
    
      int temp;  
      if(knobs==1)
      {     
          temp=analogRead(mSpeedPin);   
          mSpeed=map(temp, 0, 1023, 0, 100 ); // maps from 0 to 10 full rotation
          temp=analogRead(mAmpPin);   
          mAmp=map(temp, 0, 1023, 0, 1023 ); // maps from 0 to 10 full rotation
      //    idealCenter=map(temp,0, 1023, 41,986); //map(temp, 0, 1023, 0, 359 ); // maps from 0 to 10 full rotation
          temp=analogRead(accelPin);   
          maxAccel= map(temp, 0, 1023,10, 100);  // default is 500,  goes max Accel can range from 0 to 3200 
       }
      maxDecel=maxAccel;   
    //  temp=analogRead(decelPin);
     // maxDecel=map(temp, 0, 1023,5, 2500);  // default is 500,  goes max Accel can range from 0 to 3200       
}



void updateDeltaPosRight()  
{
  // pendulum swung left from rightMaxPos to leftMaxPos
  int temp=0;
  int deltaRotations=rotationInitRight-rotationInitLeft;
  if(deltaRotations!=0)  //if yes then zero value was crossed
  {     temp=1000*(deltaRotations-1);  
        deltaPosRight=(1000-leftMaxPos)+rightMaxPos+temp;
        return;
  }
  else    //if yes then zero was not crossed
  {      deltaPosRight=rightMaxPos-leftMaxPos;
         return;
  }
}
  
void gotoPos(long p)
{
  Serial.println(F("*********"));
  Serial.print("gotoPos (p)     p= ");
  Serial.println(p);
      printValues();
    while((p!=pos)&&(p!=(pos+1))&&(p!=(pos-1)))  //gives a tolerance while p!=pos +-1
    {  
         Serial.println(F("IN GOTOPOS LOOP"));
       setMotorSpeedPercent(mSpeed);
       if(pos<p)  // go right
           mDir=1; 
      else if(pos>p)   //go left
           mDir=-1;
    }
      mSpeed=0;
      setMotorSpeedPercent(mSpeed); 
  Serial.println(F("_______________"));
  Serial.print("END gotoPos (p)     pos= ");
   Serial.println(pos);
  return;
}



void goRightToPos(long p)   
{
          long tZero=millis(); 
          long posInit=pos;   //initial pos when function is called
          mDir=1;      //pos increases
          setMotorLimits();
          Serial.println("");
          Serial.print(F("RRRRRRRRRRRRRRRR     Go RIGHT to "));
          Serial.println(p);
         
          while((pos<p)&&(mDir==1))
          { //keep going until we get to p
            setMotorSpeedPercent(mSpeed);
            printValues();
          }              
          
           if(mDir==1)   //then we reached p before ideal,  thus we overshot Right
           {
             // overshotRight=1;
              Serial.println("OverShot Right");
              Serial.print("pos = ");
              Serial.print(pos);
              Serial.print("     idealRightMax = ");
              Serial.println(idealRightMax);
              slowRight=1;
           }
           else //we undershot right
           {   increaseRight=1;
           }
        //   mSpeed=0;
    //      setMotorSpeedPercent(mSpeed);
          timeRight=millis()-tZero;
          
        //  dataIn.timeRight=timeRight;
}


void goLeftToPos(long p)   
{
          long tZero=millis(); 
          long posInit=pos;   //initial pos when function is called
          mDir=-1;      //pos decreases
          setMotorLimits();
          Serial.println("");
          Serial.print(F("LLLLLLLLLLLLLLL     Go LEFT to "));
          Serial.println(p);
         
          while((pos>p)&&(mDir==-1))
          { //keep going until we fet to p
            setMotorSpeedPercent(mSpeed);
            printValues();
          }              
       //   mSpeed=0;

          if(mDir==-1)   //then we reached p before ideal,  thus we overshot Right
           {
              Serial.println("OverShot Left");
              Serial.print("pos = ");
              Serial.print(pos);
              Serial.print("     idealLeftMax = ");
              Serial.println(idealLeftMax);
              slowLeft=1;
           }
         // setMotorSpeedPercent(mSpeed);
          timeLeft=millis()-tZero;
        //  dataIn.timeLeft=timeLeft;         
}

void printTime()
{
   Serial.println(F("______________________________________________________________________________________________________________"));
         Serial.print(F("timeRight = "));
         Serial.print(timeRight);
         Serial.print(F("               timeRight2 = "));
         Serial.println(timeRight2);
         Serial.print("timeLeft = ");
         Serial.print(timeLeft);
         Serial.print(F("             timeLeft2 = "));
         Serial.println(timeLeft2);
         Serial.println("");
         Serial.print("speedLeft = ");
         Serial.print(speedLeft);
         Serial.print(F("             speedRight = "));
         Serial.println(speedRight);
         
          Serial.println("");
         Serial.print("rightMaxPos ="); 
           Serial.println(rightMaxPos); 
           Serial.print("leftMaxPos =");
           Serial.println(leftMaxPos); 
           Serial.print("deltaPosRight ="); 
            Serial.println(deltaPosRight); 
           Serial.print("deltaPosLeft ="); 
           Serial.println(deltaPosLeft); 
         Serial.println(F("______________________________________________________________________________________________________________"));
}

void  equalizeLeftRight()
// adjust speeds to make time left mach time right
{    
           if(timeLeft2<timeRight2)
           {   
               if(speedLeft<97)
               {
                 speedLeft+=3;
               }
               else if(speedRight>=94)      // determine if we need to decriment speedRight,  if not we can incriment SpeedLeft instead
               {
                 speedRight-=2;
               }
               
           }
           else if(timeRight2<timeLeft2)
           {
               if(timeRight2<timeLeft2)
               {   
                   if(speedRight<97)
                   {
                     speedRight+=3;
                   }
                   else if(speedLeft>=94)  // determine if we need to decriment speedLeft,  if not we can incriment SpeedRight instead
                   {
                     speedLeft-=2;
                   }
                   
               }
               setMotorSpeedPercent(mSpeed);
             // printVars2(); 
           }
}

void pattern2()
{
    while(1)
    {
        Serial.println("pattern2");
        long temp= time1/2000;
        idealCenter=  idealCenter+temp;
        Serial.print("temp=");
        Serial.println(temp);
        if(mDir==1)
          {goRightToPos(idealRightMax);
          }
          else if(mDir==-1)
          { goLeftToPos(idealLeftMax);
          }
        //   mSpeed=idealMotorSpeed*speedMultiplier;
              if(callAdjustSpeed2==1)
                 adjustSpeed2();
              setMotorSpeedPercent(mSpeed);
        printValues();
  
    }
    
}


void pattern1()
{
    idealCenter=500+offset;  //0;
    
    
    mAmp=100;
    int p1;  // position to go to
    int p2;    // position to go to
    p1=idealCenter+mAmp;
    p2=idealCenter-mAmp;
   gotoPos(idealCenter); 
//   goLeftToPos(idealCenter);
//   mSpeed=0;
   //delay(200);
  // maxAccel=1000;
   //gotoPos(idealCenter);
   delay(500);
   maxAccel=100;
   mSpeed=100;
   speedLeft=100;
   speedRight=100;
   mDir=1;                     //set direction  TEMP
   setMotorLimits();
   int x=1;
 /*   while(1)   //debug loop
   {
       //printValues();
       printVars2();
       if(deltaPos<0)
       {
         Serial.println("");  
         Serial.println("  DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD             Delta P<0");    
       }
   }
 */      //end debug loop
       
   while(1)
   {
       while(x<10)
       {
         //updateStructs();
         maxAccel=28;
         mSpeed=100;
         setMotorLimits();
         setMotorSpeedPercent(mSpeed);
      //   p1+=(x*5);
        // p2-=(x*5);
                                                                                                           Serial.print(F("*****PATTERN 1******"));
                                                                                                           Serial.print("center = ");
                                                                                                           Serial.print(center);
                                                                                                           Serial.print("p1 = ");
                                                                                                           Serial.print(p1);
                                                                                                           Serial.print("               p2 = ");
                                                                                                           Serial.println(p2);
       //   p1=idealCenter+mAmp;
         goRightToPos(p1);
        // p2=idealCenter-mAmp;
         goLeftToPos(p2);
          printTime();
         x++;
         
          
               Serial.println("ETout.sendData");
               ETout.sendData(ADDRESS);            //send data to ADDRESS
              Serial.println("printStruct");
               printStruct();

       }
       x=1; 
    //   mSpeed=0;
   //    delay(1000);
    //   idealCenter+=250;
       maxAccel=100;
     //  mSpeed=30;
       setMotorLimits();
   //    goRightToPos(idealCenter);
     //  mSpeed=0;
     //  delay(1000);
  
       p1=idealCenter+mAmp; 
       p2=idealCenter-mAmp; 
                                                                                                                           Serial.println(F("end if while(x<10)"));   
                                                                                                                           Serial.print(F("center = "));
                                                                                                                           Serial.print(center);                                                                                                                       Serial.print(F("p1 = "));
                                                                                                                           Serial.print(p1);
                                                                                                                           Serial.print(F("               p2 = "));
                                                                                                                           Serial.println(p2);
   }
       
   
}

void speedDown()            
// if mspeed will not be pushed outside of limits,  
//if not then mspeed is a decrimented by a percent from the ideal motorSpeed, and speedMultiplier is decreased
{
          Serial.println("");
          Serial.println("***speedDown()   ");
          float temp2= speedMultiplier - (idealAccel*.01);
          int temp=idealMotorSpeed*temp2;   
          if(temp>99)  
              mSpeed=99;
          else if(temp<1) 
              mSpeed=0;
          else  
          {  
             mSpeed=temp;             //mspeed is now a lesser percent from the idealmotorspeed
             speedMultiplier= temp2;    //multipler is decrimented
          }
}

void speedUp()
// if mspeed will not be pushed outside of limits,  
//if not then mspeed is a incrimented by a percent from the ideal motorSpeed, and speedMultiplier is increased

{
          Serial.println("");
          Serial.println("***speedUp()   ");
          float temp2= speedMultiplier + (idealAccel*.01);
          int temp= idealMotorSpeed*temp2;
          if(temp<1) mSpeed=0;
          else if(temp>99)  mSpeed=99;
          else 
          {
            mSpeed=temp;
            speedMultiplier=temp2;    
            }
}

void shiftSpeedsLeft()
//increases speedMultLeft,  and decreases speedMultRight
{
    Serial.print(F("Shift Speeds LEFT: "));   
    if(leftMaxPos>idealLeftMax)
    {
      //speedMultLeft= speedMultLeft+ (idealAccel*.01);
      speedMultLeft=2;
      Serial.print(F(" multLeft++"));
    }  
    float temp= speedMultRight- (idealAccel*.01);  // twice as much decel
    if(temp<.1)   //slowest speed
    {     temp=.1;
    }
    speedMultRight=2;
    //speedMultRight=temp;  
    Serial.print(F(" multRight--"));
    Serial.println("");   
}

void shiftSpeedsRight()
////increases speedmultRight,  and decreases speedMultLeft
{
    Serial.print(F("Shift Speeds RIGHT: "));   
    if(rightMaxPos<idealRightMax)
    { 
       //speedMultRight= speedMultRight+(idealAccel*.01);
       speedMultRight=2;
       Serial.print(F(" multLeft++,"));
    }
    float   temp= speedMultLeft-(idealAccel*.01);// speedMultLeft+(idealAccel*.01);
    if(temp<.1) 
    {
         temp=.1;
    }
   // speedMultLeft=temp;
    speedMultLeft=2;
    Serial.print(F(" multLeft--"));
    Serial.println("");

} 

void adjustSpeed2()  
//adjust mSpeed based on center vs idealCenter values.   called once per oscillatoin.
{
  Serial.println(F("____________________________"));
   Serial.println(F("ADJUSTSPEED2()"));
   Serial.print(F("idealRightMax:  "));
   Serial.print(idealRightMax);
   Serial.print(F("          rightMaxPos:  "));
   Serial.print(rightMaxPos);
   Serial.print(F("          idealLeftMax:  "));
   Serial.print(idealLeftMax);
    Serial.print(F("          leftMaxPos:  "));
   Serial.println(leftMaxPos);
    float temp;
    
   /* if(((centerPrev>idealCenter)&&(center<idealCenter))|| ((centerPrev<idealCenter)&&(center>idealCenter))) 
    //then center crossed ideal center line, thus make the speedMult for left and right equal to average between them. 
    {
       speedMultRight=(speedMultRight+speedMultLeft)/2;          
       speedMultLeft=speedMultRight;
    }
    if((rightMaxPos>idealRightMax)||(slowRight==1))   // then we are over shooting right and need to slow speedRight
    {
        temp= speedMultRight- (idealAccel*.01);  // twice as much decel
        if(temp<.1)   //slowest speed
             temp=.1;
        speedMultRight=temp;  
           Serial.println(F("Slow speedRight"));      
    }   
     
    if((leftMaxPos<idealLeftMax)||(slowLeft==1))   // then we are over shooting left should decrease speedLeft;
    {
        temp= speedMultLeft-(idealAccel*.01);// speedMultLeft+(idealAccel*.01);
        if(temp<.1) 
             temp=.1;
        speedMultLeft=temp;
        Serial.println(F("Slow speedLeft"));   
    }     
   //  note:  leftMaxOff= leftMaxPos-idealLeftMax; 
    if(leftMaxPos>idealLeftMax)  //then we undershot left,  increase speed left
    {    speedMultLeft= speedMultLeft+ (idealAccel*.01);
         Serial.println(F("Increase speedLeft"));  
    } 
    if(rightMaxPos<idealRightMax)  //then we undershot right,  increase speed right
    {    speedMultRight= speedMultRight+(idealAccel*.01);
         Serial.println(F("Increase speedRight"));  
    }
  */  
    /* if center is off,  do a second adjustment by shifting speeds left or right*/
    if(center<idealCenter)  //if yes shift speeds right
    {
      shiftSpeedsRight(); 
      if((idealCenter-center)>100)   //if center is more than 75 off then call shift right again
         shiftSpeedsRight();
    }
    else if(center>idealCenter)  // if yes then shift speeds left
    {
       shiftSpeedsLeft();
       if((center-idealCenter)>100)   //if center is more than 75 off then call shift left again
         shiftSpeedsLeft();
    }
    
   callAdjustSpeed2=0;  //resets flag to call this function;
    slowRight=0;        //resets flags
    slowLeft=0;
    increaseRight=0;
    increaseLeft=0;
   // setMotorSpeedPercent(mSpeed);
}
   
void adjustSpeed()  //incriments or decriments mSpeed towards idealPos
//Assumes the zero value of position is not crossed
{
  int temp;
  int temp2;
  if(mDir==1)  //going right
  {
      if(idealPos<pos)   //if yes then make slower to let idealpos catch up
           speedDown();
      else   //idealPos>=pos
           speedUp();
   }
  else   //mdir==-1
  {
      if(idealPos<pos)   // if yes then speed up
           speedUp();   
      else
          speedDown();   
   }
   setMotorSpeedPercent(mSpeed);
}

void timerIsr() //called every 1000 microseconds  or something
{
   updateVars();
   interuptCount++;
}


void loop()
{
      mSpeed=50;
      mDir=-1;
      setMotorSpeedPercent(mSpeed);
      idealCenter= 500+offset;
      
      /*
      Serial.println("goto IdealLeftMax");
      gotoPos(tempIdealLeftMax);
      setMotorSpeedPercent();
      gotoPos(tempIdealLeftMax);
      setMotorSpeedPercent(0);
      delay(2000);
      setMotorSpeedPercent(50);
       Serial.println("goto IdealRightMax");
      gotoPos(idealRightMax);
      setMotorSpeedPercent(20);
      gotoPos(idealRightMax);
      setMotorSpeedPercent(0);
      delay(2000);
      */
      Serial.println("goto IdealCenter");
      setMotorSpeedPercent(50);
//      gotoPos(idealCenter);
//      gotoPos(idealCenter);
      setMotorSpeedPercent(50);
//      gotoPos(idealCenter);
      mSpeed=0;
      setMotorSpeedPercent(mSpeed);
      delay(2000);
      mSpeed=50;  
      knobs=0;     //no knobs attached
      setMotorSpeedPercent(mSpeed);
    //  pattern2();
      while(1)
      {   
          Serial.println("");    
          Serial.println("While(1)");
          if(mDir==1)
          {goRightToPos(idealRightMax);
          }
          else if(mDir==-1)
          { goLeftToPos(idealLeftMax);
          }
        //   mSpeed=idealMotorSpeed*speedMultiplier;
              if(callAdjustSpeed2==1)
                 adjustSpeed2();
              setMotorSpeedPercent(mSpeed);
               ETout.sendData(ADDRESS);            //send data to ADDRESS
               printValues();
       }   
        
}



void receive(int numBytes) {}
