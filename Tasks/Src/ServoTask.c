#include "ServoTask.h"
#include "includes.h"

int servo_id = 0, pwm = 500, time = 0;//Ĭ�϶��ID��0 
char InitServoMes[]="#000P0500T0000!";//#000P0500T1000!:0�Ŷ��ת��0500λ��, λ�÷�Χ500-2500. 
																								//#000PMOD3!:���ģʽ, ���Ƕȷ�Χ180, ˳ʱ����ת
char ServoMes[15];
uint8_t servoBuffer[20]={0};
uint8_t p;
void InitServoUart()
{
	HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&InitServoMes, sizeof(InitServoMes), 0xFFFF);
	if(HAL_UART_Receive_DMA(&SERVO_UART, &p, 1) != HAL_OK){
			Error_Handler();
	} 
}

void servoUartRxCpltCallback()
{

}
