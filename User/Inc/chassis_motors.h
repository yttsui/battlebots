/**
  *@file test_can.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
#ifndef _TEST__CAN_H
#define _TEST__CAN_H



#include "stm32f4xx_HAL.h"
#include "PID.h"
#define TEST_CAN1_ID    0x201
#define TEST_CAN2_ID    0x202

#define ENCODER_RATE_BUF_SIZE 3

typedef struct {
    int16_t velocity_from_ESC;
    int16_t position_raw_value;
    int16_t position_raw_value_last;
    int32_t position_ecd_value;
    int32_t position_diff;
    int32_t count;
    uint8_t buf_count;  // Filter buffer count
    int32_t position_ecd_bias; // Initial encoder position
    int32_t velocity_ecd_raw;  // velocity from encoder difference
    int32_t velocity_ecd_raw_last;
    int32_t velocity_buff[ENCODER_RATE_BUF_SIZE];
    int32_t round_count;
    int32_t velocity_filtered;

}Encoder;
_TEST__CAN_H extern PID_Handler wheels_speed_pid[4];
_TEST__CAN_H extern int16_t M_wheel_result[4];
//typedef struct {
//    Encoder ecd;
//    int32_t motor_current_set_point;
//}Chassis_Motor_Type;

extern uint8_t can1_rx_data[8];
extern uint8_t can2_rx_data[8];


void CanFilter_Init(CAN_HandleTypeDef* hcan);
void chassis_control_init(void);
void encoderProcess(volatile Encoder*, CanRxMsgTypeDef*);
void Chassis_Set_Speed(int16_t, int16_t, int16_t, int16_t);
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id);
void update_wheel_pid();
void control_car(PID_Handler *PID_Array);
#endif
