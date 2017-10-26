#include "main.h"
#include "PID.h"
#include "chassis_motors.h"
void PID_Init (PID_Handler* PID, float Kp, float Ki, float Kd, float IntegralDecay, float OutputLimit, float I_Max) {

    for (int i = 0; i < 4; i++) {
        PID->Kp = Kp;
        PID->Ki = Ki;
        PID->Kd = Kd;
        PID->LastError = 0;
        PID->Output = 0;
        PID->P = 0;
        PID->I = 0;
        PID->D = 0;
        PID->IntegralDecay = IntegralDecay;
        PID->OutputLimit = OutputLimit;
        PID->I_Max = I_Max;
    }


}
float PID_UpdateValue(PID_Handler*PID,float target_val, float current_val)
{
    PID->Current = current_val;
    return PID_output(PID,target_val,current_val);
}

float PID_output (PID_Handler* PID, float Target_val,float Current_val) {
  PID->Target=Target_val;
  PID->P = PID->Current - PID->Target;		 											//Get P
  PID->D = (PID->P - PID->LastError);											//Get D
  PID->I += (PID->P);															//Get I
  PID->I = UpperLimit(PID->I,PID->I_Max);										//Limit I (+ve)
  PID->I = LowerLimit(PID->I,-(PID->I_Max));									//Limit I (-ve)
  PID->Output = (PID->P * PID->Kp) + (PID->I * PID->Ki) + (PID->D * PID->Kd);	//Get PID
  PID->Output = UpperLimit(PID->Output,PID->OutputLimit);						//Limit PID (+ve)
  PID->Output = LowerLimit(PID->Output,-(PID->OutputLimit));					//Limit PID (-ve)
  PID->LastError = PID->P;													//Save last error
  PID->I = PID->I * PID->IntegralDecay;										//Decay I
  return PID->Output;
}





