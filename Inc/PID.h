
//Created by Alex Wong for HkUST RM Internal 2017 :D

#ifndef BATTLEBOTS_PID_H
#define BATTLEBOTS_PID_H
#define ABS(x)				( ((x) > 0) ? (x) : (-(x)) )	//return abs value of x
#define UpperLimit(x,y)		( ((x) > (y)) ? (y) : (x) )		//return y if x>y, else return x(SMALL)
#define LowerLimit(x,y)		( ((x) < (y)) ? (y) : (x) )		//return y if x<y, else return x(BIG ONE)

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float LastError;
    float Output;
    float P;
    float I;
    float D;
    float IntegralDecay;
    float OutputLimit;
    float I_Max;
    float Current;
    float Target;
} PID_Handler;
float PID_UpdateValue(PID_Handler*PID,float target_val, float Current_val);
float PID_output (PID_Handler* PID, float Target_val, float current_val);
void PID_Init (PID_Handler* PID, float Kp, float Ki, float Kd, float IntegralDecay, float OutputLimit, float I_Max);
#endif

 //BATTLEBOTS_PID_H
