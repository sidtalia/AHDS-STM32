/*
 * STM32
 */

#include "MPU6050.h"
#include "Wire.h"

#define BaseKp (float)5.0
#define BaseKd (float)1.6
#define KD2 (float)0.001
int tune; // tune is for toggling alti-hold
float Kp_pitch,Kp_roll;
#define Ki (float)0.0375  //premultiply the time with Ki 
#define IMAX 3000  //this is not in degrees, this is degrees*400 
#define YawKp (float)10   //Kp for yaw rate 
#define minValue 1100  //min throttle value, this is to prevent any motor from stopping mid air.
#define maxValue 2000 //max pwm for any motor 
#define YAW_MAX 300   //max value of yaw input
#define CONSTANT (float)0.00045 
#define DCONSTANT (float)0.0000001

#define CLOCK_SPEED 127
#define FL TIMER4_BASE->CCR3//b8
#define FR TIMER4_BASE->CCR4//b9
#define BL TIMER1_BASE->CCR1//a8
#define BR TIMER1_BASE->CCR4//a11

//-----ACCEL-GYRO STUFF BEGINS---------------------
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 pin high for when you have more than one accelerometers(not sure why you'd want that if you want a fast code but okay).
int16_t a[3];  //accelerations from mpu6050
int16_t g[3];  //gyration rates from mpu6050

float A[3],G[3],lastA[3]={0,0,0},lastG[3]={0,0,0},offsetA[3],offsetG[3],T[2]; //x=0,y=1,z=2, T=tilt.
float sigma[2]={0.0,0.0};  //variable for the integral part
int i,j;
bool connection;

//-----------ACCEL-GYRO STUFF ENDS----------------

//------variables for reading pwm-------
int32_t input_start[4],input[32],delT[4],reset_timer[4],tick[4]={0,8,16,24};
//---------------------------

void setup()
{  
    setup_esc_control();
    setup_receiver_channels();
  //==============done==============================
    Serial.begin(38400);
  //===========ACCELGYRO SETUP BEGINS===============   
    Wire.begin();   
    Wire.setClock(400000);
    accelgyro.initialize();  //do the whole initial setup thingy using this function.
    accelgyro.testConnection()==1 ? connection=1 : connection=0 ; 
   //1598.24||-659.35||16344.98||-4.15||-17.29||-72.81||
    offsetA[0]= 1600;        //these offsets were calculated beforehand
    offsetA[1]= -660; 
    offsetA[2]= -287;
    offsetG[0]= -4.3;
    offsetG[1]= -17.3;
    offsetG[2]= -72.8;
    
    for(j=0;j<2000;j++)   //taking 2000 samples for finding initial orientation,takes about 0.8 seconds  
    {
      accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
      for(i=0;i<2;i++)                                      
      {
        A[i]=a[i];         //transfer value
        A[i]-=offsetA[i]; //subtracting offset
        A[i]*=0.0006103;    //convert to real world value
        lastA[i]+=A[i];    //store in lastA[i]
      }
    }
    A[0]=lastA[0]*0.0005;   //take average of 2000 readings
    A[1]=lastA[1]*0.0005;
    lastA[0]=0;  //lastA[i] was used here only as a place holder for a "sum" variable. it's purpose as a sum variable
    lastA[1]=0;  // has been fullfilled, therefore it will now be restored to 0 so that it can be used for it's origianl purpose
   
    T[0]=(+57.3*asin(A[1]*0.102));  //initial orientation 
    T[1]=(-57.3*asin(A[0]*0.102));
//------------ACCEL-GYRO SETUP ENDS-------------
}



int throttle=1000,dummy; //initializing throttle with default value
float rollsetp=0.0,yawsetp=0.0,pitchsetp=0.0;
float pError,rError,lastPError=0,lastRError =0,dPError=0,dRError=0; //initializing setpoints at 0
float d2PError=0,d2RError=0,lastDPError=0,lastDRError=0;
int p,r,y;    // initializing PID outputs for pitch, yaw, roll
long lastTime;  //timing variable for failsafe 
float tanSqTheta,cosSqTheta,cosTheta,Av=0,lastAv=0,Vv=0,terror;
float sterror=0;
float dist,lastDist=0.2,lastV,delV=0;
volatile bool ultra=false;
#define HOVERTHROTTLE 1500
#define throttleKp (float)120.0
#define throttleKi (float)0.1
#define throttleKd (float)10.0
#define dt (float)0.0025  //cycle time in seconds
//#define my_sqrt(a) (0.25 + a*(1-(0.25*a))) // 30us, DO NOT USE IT FOR numbers too far away from 1, its an approximation for g's sake.

long failsafe=0;  //initializing failsafe variable 
volatile bool servoWrite=0; //initializing servoWrite variable as false(no signal received)
bool arm=0;  //initializing arming variable as false(initially not armed)
bool state=0; //initializing state as false so that the motor is not sent any signals
bool lowbatt = false;



void readMPU()   //function for reading MPU values. its about 80us faster than getMotion6() and hey every us counts!
{
  Wire.beginTransmission(0x68);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(0x68,14); //request 14 bytes from mpu
  //108us for all data to be received.  
  a[0]=Wire.read()<<8|Wire.read();  
  a[1]=Wire.read()<<8|Wire.read(); 
  a[2]=Wire.read()<<8|Wire.read(); 
  g[0]=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  g[0]=Wire.read()<<8|Wire.read();  
  g[1]=Wire.read()<<8|Wire.read();
  g[2]=Wire.read()<<8|Wire.read();
}// this function takes 132us with normal hardware, takes 122 if you use 4.7k pullup resistors. 

int deadBand(int input)//the receiever signals vary a little bit (set value +/- 8us). This function removes that jitter 
{
  if(input>=1496&&input<=1504)  //if i m trying to send 1500 and the value received is between these 2, make it 1500 
  {
    return 1500;
  }
  return input;
}//0 time function

float ExpKp(float k,float error)
{
 return k*(1 + CONSTANT*error*error);
}//2.5us function


inline int limiter(int input)
{
  if(input>maxValue)
  {
    return maxValue;
  }
  if(input<minValue)
  {
    return minValue;
  }
  return input;
}//0 time function

float dead_Zone(float input)
{
  if(input<3)
  {
    return 0;
  }
  if(input>-3)
  {
    return 0;
  }
  return input;
}//0 time function

void correction()
{
   TIMER1_BASE->CNT = 2500; // reset the counter of each timer to their max value so that the auto
   TIMER4_BASE->CNT = 2500; // reload register is set to 0 and all the pwm pins are set to HIGH state
                            //happens in ~0 time
   if(throttle>1800)
   {
      throttle=1800;    //cap the max throttle value
   }
   //takes ~0 time
   if(throttle>minValue)   //if throttle is above minimum value 
   {
     FL = limiter(throttle+p+r+y);//add the values of throttle, pitch, roll and yaw
     FR = limiter(throttle+p-r-y);//the values are already in int format.
     BL = limiter(throttle-p+r-y);//FL,FR,BL,BR are actually aliases for TIMERx_BASE->CCRx
     BR = limiter(throttle-p-r+y);// so the pwm values are castes here directly. this entire process of casting pwms
                                  //takes ~6us
   }
   else//if throttle is less than 1100
   {
      FL=(1000); 
      FR=(1000);
      BL=(1000);
      BR=(1000);
   }
   //this entire thing happens in ~6us. we'll call it 10us
} 

float hovercap(float input)
{
  if(input>150)
  {
    return 150;
  }
  if(input<-150)
  {
    return -150;
  }
  return input;
}//~4us

void loop()
{
 if(connection==0)  //in case accelgyro connection fails
 {
    lastTime = micros();     //get time stamp
    TIMER4_BASE->CNT=2500;//I am using 2 timers so that I can use the default pins for the I2C. This also allows me to use other libraries 
    TIMER1_BASE->CNT=2500;//written for arduino that use the Wire library.*INSERT BLACK GUY FEELING SMART MEME*
    FL=(1000);    //send 0% throttle to all motors 
    FR=(1000);
    BL=(1000);
    BR=(1000); 
 }
 else
 {
   lastTime = micros();
   callimu();   //494us since esc_timer,this function calculates orientation (pitch and roll) 
  
   if(yawsetp<(-150)&&dummy<=minValue)   //arming sequence
   {
      arm=1;
   }
   else if(yawsetp>150&&dummy<=minValue)  //disarming sequence 
   {
      arm=0;
   }
   //4us ,498us since esc_timer

   if(servoWrite) //when new pwm signals from receiver are received (see ISR function at the bottom) 
   {
      //transferring inputs from volatile to non-volatile variables 
      throttle = dummy =input[1];
      yawsetp  =(1500-deadBand(input[3]))*0.6;   //this is yaw rate(deg/sec)
      pitchsetp =(1500-deadBand(input[2]))*0.1;   //roll, pitch setp in degrees
      rollsetp=(deadBand(input[0])-1500)*0.1;   
      tune = input[4];
     
      servoWrite=0;     //making servo write false 
      failsafe=millis(); //giving time-stamp to the failsafe variable. failsafe is updated everytime a signal from the receiver is received.  
   }
   

   if((millis()-failsafe>1000)&&arm)//(if connection has been lost but we are mid air)
   {
      tune=2000;//forcefully go into alti-hold mode,
      dummy = 1450;//drop with 10% full speed, should be about 50cm/s
      rollsetp = 0;
      pitchsetp = 0;
      yawsetp=0;
   }

   if(tune>1500)
   {
      terror = 0.01*(deadBand(dummy)-1500) - Vv;
      sterror += terror;
      throttle = hovercap(throttleKp*terror) + hovercap(throttleKi*sterror) - hovercap(throttleKd*Av) + HOVERTHROTTLE;//open loop + closed loop = advanced PID.

      if(input[1]<minValue)
      {
        throttle = minValue;
      }
   }//70us.820us since esc_timer
   
   //PID begins---
   pError = pitchsetp-T[0]; //storing the error value somewhere as it will be
   dPError = (pError - lastPError)*400;
   d2PError = (dPError - lastDPError)*400;
   lastPError = pError;
   lastDPError = dPError;
   
   rError = rollsetp-T[1]; //used repeatedly
   dRError = (rError - lastRError)*400;
   d2RError = (dRError - lastDRError)*400;
   lastRError = rError; 
   lastDRError = dRError;
   
   sigma[0]+= (pError); //incrementing integral of error 
   sigma[1]+= (rError);
   //20us, 518us since esc_timer
   for(i=0;i<2;i++)
   {
      if(sigma[i]>IMAX)
      {
        sigma[i]=IMAX;  //capping max value of integral of error 
      }
      if(sigma[i]<(-IMAX))
      {
        sigma[i]=(-IMAX);
      }
   }//~4us, 522us since esc_timer   
   //PID (funny how colleges spend 1 month trying to explain something that can be written in a single line of code) 
   Kp_pitch = ExpKp(BaseKp,pError);
   Kp_roll = ExpKp(BaseKp,rError);
    
   r = Kp_roll*(rError) + BaseKd*dRError + Ki*sigma[1] + KD2*d2RError;   //reducing time by not creating a function at all for these tiny tasks
   
   p = Kp_pitch*(pError) + BaseKd*dPError + Ki*sigma[0] + KD2*d2PError;
   
   y = YawKp*(yawsetp-G[2]);
   if(y>0&&y>YAW_MAX) //capping max yaw value
   {
    y=YAW_MAX;
   }
   if(y<0&&y<-YAW_MAX)
   {
    y= -YAW_MAX;
   }
   //170us ,692us since esc_timer
    
   arm? state = 1 : state = 0;
   //
   if(state)   
   {
      correction(); //call the correction function to calculate the pwms and send the pwms to the escs    
   }
   else if(!state)  //if receiver does not reconnect within a second or is lost or the drone is dis-armed, kill throttle 
   {
      throttle=1000;
      correction();
      sigma[0]=0;  //reset integral errors to 0
      sigma[1]=0;
      T[0]=(+57.3*asin(A[1]*0.102));  //initial orientation 
      T[1]=(-57.3*asin(A[0]*0.102));
      
      Vv = 0;
      Av=0;
      lastAv = 0;
      sterror = 0;
   }  
   
 }

 while(micros()-lastTime<2500);  //wait for the 2500 us to be over
}

void handler_channel_1(void)
{
  if (0b1 & GPIOA_BASE->IDR  >> 0) //check if pin PA0 is high, the GPIOA_BASE refers to port A, IDR refers to interrupt detect register which contains the pin number
  {                                // of the pin that just went high and >> 0 is us shifting the IDR by 0 bits. if the IDR's first bit is 1, 0b1&1 would give 1, this means
                                   //that there was a rising edge (the CCER is set to detect the rising edge at first)
    delT[0] = TIMER2_BASE->CCR1 - input_start[0];
    if (delT[0]< 0)delT[0]+= 0xFFFF;
    if(reset_timer[0]>2000)
    {
      tick[0] = 0;
      servoWrite = true;
    }
    else
    {
      input[tick[0]++] = delT[0];
    }
    input_start[0] = TIMER2_BASE->CCR1;//the time of the rising edge is stored in input_start[0] variable
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;// the Compare capture enable register (CCER) is set to detect the falling edge on pin PA0 (CC1P-> a binary value 
  }                                       // in which the value corresponding to pin 1(PA0) is set to high (how do i know its set to high? look at the |= ).
  else    //when the interrupt is generated BUT the edge is not the rising edge but rather the falling edge
  {
    reset_timer[0] = TIMER2_BASE->CCR1 - input_start[0];//the pulse time is the falling edge time - rising edge time.
    if (reset_timer[0]< 0)reset_timer[0]+= 0xFFFF; //if for some reason the value of the channel is less than 0(due to timer restart) then just take the complement.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P; //reset the CCER to detect the rising edge by taking bitwise AND with the compliment of CC1P(doesn't touch anything else but resests
  }                                         //the bit we care about.
}

void handler_channel_2(void)
{
  if (0b1 & GPIOA_BASE->IDR >> 1) 
  {                                // of the pin that just went high and >> 0 is us shifting the IDR by 0 bits. if the IDR's first bit is 1, 0b1&1 would give 1, this means                                //that there was a rising edge (the CCER is set to detect the rising edge at first)
    delT[1] = TIMER2_BASE->CCR2 - input_start[1];
    if (delT[1]< 0)delT[1]+= 0xFFFF;
    if(reset_timer[1]>2000)
    {
      tick[1] = 8;
    }
    else
    {
      input[tick[1]++] = delT[1];
    }
    input_start[1] = TIMER2_BASE->CCR2;
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else
  {
    reset_timer[1]= TIMER2_BASE->CCR2 - input_start[1];
    if (reset_timer[1]< 0)reset_timer[1]+= 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3(void)
{
  if (0b1 & GPIOA_BASE->IDR >> 2) 
  {                                // of the pin that just went high and >> 0 is us shifting the IDR by 0 bits. if the IDR's first bit is 1, 0b1&1 would give 1, this means
                                   //that there was a rising edge (the CCER is set to detect the rising edge at first)
    delT[2] = TIMER2_BASE->CCR3 - input_start[2];
    if (delT[2]< 0)delT[2]+= 0xFFFF;
    if(reset_timer[2]>2000)
    {
      tick[2] = 16;
    }
    else
    {
      input[tick[2]++] = delT[2];
    }
    input_start[2] = TIMER2_BASE->CCR3;
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else
  {
    reset_timer[2]= TIMER2_BASE->CCR3 - input_start[2];
    if (reset_timer[2]< 0)reset_timer[2]+= 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4(void)
{
  if (0b1 & GPIOA_BASE->IDR >> 3) 
  {                                // of the pin that just went high and >> 0 is us shifting the IDR by 0 bits. if the IDR's first bit is 1, 0b1&1 would give 1, this means
                                  //that there was a rising edge (the CCER is set to detect the rising edge at first)
    delT[3] = TIMER2_BASE->CCR4 - input_start[3];
    if (delT[3]< 0)delT[3]+= 0xFFFF;
    if(reset_timer[3]>2000)
    {
      tick[3] = 24;
    }
    else
    {
      input[tick[3]++] = delT[3];
    }
    input_start[3] = TIMER2_BASE->CCR4;
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;
  }
  else 
  {
    reset_timer[3]= TIMER2_BASE->CCR4 - input_start[3];
    if (reset_timer[3]< 0)reset_timer[3]+= 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}




