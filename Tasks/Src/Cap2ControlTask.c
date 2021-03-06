/**
  ******************************************************************************
  *FileName     : Cap2ControlTask.c                                            *
  *Description  : 超级电容开关控制程序                                        *
  *Author       : 唐欣阳                                                       *
  ******************************************************************************
  *                                                                            *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University               *
  * All rights reserved.                                                       *
  *                                                                            *
  ******************************************************************************
  */

#include <math.h>
#include <string.h>
#include "includes.h"

/*使用之前注意：
1.按照硬件版本配置Cube
2.调用Cap_Init()、Cap_Run()
3.检查与裁判系统的通信，否则电容无法正常工作
*/

//启用电容的版本,请务必与车上的硬件版本相一致
//#define USE_CAP1
//#define USE_CAP2
#define USE_CAPex

//各版本下启用运行模式
#ifdef USE_CAP1
  #define CAP_AUTO_RECHARGE
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
#endif /* USE_CAP1 */

#ifdef USE_CAP2
  //#define CAP_USE_CURR
  //#define CAP_AUTO_RECHARGE
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
#endif /* USE_CAP2 */

#ifdef USE_CAPex
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
  #define CAP_FINAL_TEST
#endif /* USE_CAPex */

//Program Begin!

#define ADC_CHANNALS            (4)
#define ADC_HITS                (50)

#define VREFINT                 (0)
#define ICOUT                   (1)       //PB0 Channel 8
#define ICIN                    (2)       //PB1 Channel 9
#define UCK1                    (3)       //PC0 Channel 10

#ifdef USE_CAP1
  #define RECHARGE_VOLTAGE_MAX    (21.5)
  #define RE_RECHARGE_VOLTAGE     (20.0)
  #define RELEASE_VOLTAGE_MIN     (8.0)//6
  #define VAL__CAP_VOLTAGE        (FUNC__Get_Voltage(UCK1)  * 9.2 * 13.33 / 11.01)
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.152)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 9.2 * 13.33 / 11.01) //Power Voltage
  #define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])
  #endif /* USE_CAP1 */


#ifdef USE_CAP2
  #define PWM_CMP_MAX             (42000-1)
  #define RECHARGE_POWER_MAX      (65)
  #define RELEASE_POWER_MAX       (70)//70
  #define RECHARGE_CURR_MAX       (3.0)
  #define RELEASE_CURR_MAX        (3.0)
  #define RELEASE_POW_RATE        (1.5)//1.3
  #define RECHARGE_POW_RATE       ((PowerHeatData.chassisPower > RECHARGE_POWER_MAX)?\
                                ((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-0.8) + 0.8):\
                                ((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-1.0) + 1.0))
  #define INPUT_PWM_PERCENT_MAX   (100)
  #define OUTPUT_PWM_PERCENT_MAX  (100)
  #define RECHARGE_VOLTAGE_MAX    (21.5)
  #define RELEASE_VOLTAGE_MIN     (8.0)//6
  #define RE_RECHARGE_VOLTAGE     (20.0)
  #define OUT_VOL_PWM_TIM         htim8
  #define OUT_VOL_PWM_CHANNEL     TIM_CHANNEL_2
  #define IN_VOL_PWM_TIM          htim8
  #define IN_VOL_PWM_CHANNEL      TIM_CHANNEL_3

  #define VAL__INPUT_PWM_MAX      (INPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
  #define VAL__OUTPUT_PWM_MAX     (OUTPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
  #define VAL__OUTPUT_PWM_PERCENT (output_pwm_cmp * 100.0 / PWM_CMP_MAX)
  #define VAL__INPUT_PWM_PERCENT  (input_pwm_cmp * 100.0 / PWM_CMP_MAX)
  #define VAL__CAP_VOLTAGE        (FUNC__Get_Voltage(UCK1)  * 9.2 * 13.33 / 11.01)
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.152)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 9.2 * 13.33 / 11.01) //Power Voltage

  #define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])

  #define FUNC__ADD_INPUT_PWM_PERCENT(rate)   (input_pwm_cmp += (rate)*PWM_CMP_MAX/100)
  #define FUNC__ADD_OUTPUT_PWM_PERCENT(rate)  (output_pwm_cmp += (rate)*PWM_CMP_MAX/100)
  #define FUNC__Cap_Set_Output_Percent(rate)  (output_pwm_cmp = PWM_CMP_MAX*rate/100)
  #define FUNC__Cap_Set_Input_Percent(rate)   (input_pwm_cmp  = PWM_CMP_MAX*rate/100)



  #ifdef CAP_USE_CURR
    #define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - PowerHeatData.chassisPowerBuffer) / 2));
    #define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - PowerHeatData.chassisPowerBuffer) / 2));
  #else
    #define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              PowerHeatData.chassisPower + (60 - PowerHeatData.chassisPowerBuffer) / 2));
    #define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              PowerHeatData.chassisPower + (60 - PowerHeatData.chassisPowerBuffer) / 2));
  #endif /* CAP_USE_CURR */

  #define CAL_RELEASE(max, x) (((max)>(x))?(pow((max)-(x), RELEASE_POW_RATE)):(-pow((x)-(max), RELEASE_POW_RATE)))
  #define CAL_RECHARGE(max, x) (((max)>(x))?(pow((max)-(x), RECHARGE_POW_RATE)):(-pow((x)-(max), RECHARGE_POW_RATE)))

#endif /* USE_CAP2 */



#ifdef USE_CAPex
  #define DAC_PER_MAX             (4095)
  #define RECHARGE_VOLTAGE_MAX    (24.0)
  #define RE_RECHARGE_VOLTAGE     (17.0)
  #define RELEASE_VOLTAGE_MIN     (9.0)
  #define VAL__INPUT_DAC_MAX      (4095)
  #define RE_RELASE_VOLTAGE_MIN   (15.0)
  #define VAL__CAP_VOLTAGE        (FUNC_NEW_Get_Voltage() * 35.2 / 2.2)
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.14)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 35.2 / 2.2) //Power Voltage
  #define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])

  #define FUNC__RECAL_INPUT_DAC        ((75 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
  #define FUNC__RECAL_INPUT_DAC_T        ((75 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
  #define FUNC__RECAL_OUTPUT_DAC       4095*0.14*78/VAL__CAP_Power_Voltage/3.3
																							
  #define CAL_RECHARGE(max, x) (((max)>(x))?(pow((max)-(x), RECHARGE_POW_RATE)):(-pow((x)-(max), RECHARGE_POW_RATE)))
  double FUNC_NEW_Get_Voltage(void);
#endif /* USE_CAPex */

static int16_t ADC_hits_val[ADC_HITS][ADC_CHANNALS];
static int32_t ADC_tmp[ADC_CHANNALS];
static int16_t ADC_val[ADC_CHANNALS];
uint8_t rlease_flag = 0;

static cap_state CapState = CAP_STATE_STOP;
CapControl_t Control_SuperCap = { 0,0 };

static void Cap_State(void);
static void Cap_Ctr(void);

#ifdef USE_CAP2
  static int32_t input_pwm_cmp = 0;
  static int32_t output_pwm_cmp = 0;
  static int16_t recharge_cnt = 0;
  static int16_t release_cnt = 0;
  static int32_t output_pwm_cmp_max = 0;
#endif /* USE_CAP2 */

#ifdef USE_CAPex
  static int32_t input_dac_per = 0;
  static int32_t output_dac_per = 0;
#endif /* USE_CAPex */


#ifdef CAP_DEBUG
  #define DEBUG_HITS    (4000)
  #define DEBUG_AMOUNT  (4)
  static uint8_t cnt=0;
  int32_t cps[DEBUG_AMOUNT][DEBUG_HITS];
  static uint32_t ncnt=0;
  #if DEBUG_HITS * DEBUG_AMOUNT * 4 > 65535
    #error Cap debug info bytes must below 65535, try to change the DEBUG_AMOUNT and DEBUG_HITS
  #endif
#endif /* CAP_DEBUG */

void Cap_Init(void) {
	memset(ADC_hits_val, 0, sizeof(ADC_hits_val));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_hits_val, ADC_CHANNALS*ADC_HITS);
	#ifdef USE_CAP2
	  HAL_TIM_PWM_Start(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL);
	  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
	  HAL_TIM_PWM_Start(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL);
	  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	#endif /* USE_CAP2 */
	
	#ifdef USE_CAPex
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	#endif /* USE_CAPex */
}

#ifdef USE_CAPex
  double FUNC_NEW_Get_Voltage(void)
  {
	  static double last_cap_voltage = 0;
	  static double new_cap_voltage = 0;
	  new_cap_voltage = 1.2 * ADC_val[3] / ADC_val[VREFINT]*0.0002 + last_cap_voltage*0.9998;
	  last_cap_voltage = new_cap_voltage;
	  return new_cap_voltage;
  }	
#endif /* USE_CAPex */

void Cap_Run(void) {
	Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();
	#ifdef CAP_LED_SHOW
	  LED_Show_SuperCap_Voltage(1);
	#endif /* CAP_LED_SHOW */
	Cap_Ctr();
	Cap_State();
	user_data.mask = 0xC0 | ((1 << ((int)(((pow(VAL__CAP_VOLTAGE,2) - pow(10,2)) / (pow(23,2) - pow(10,2))) * 6 + 1))) - 1);
}

void Cap_State_Switch(cap_state State) {
	switch (State) {
	case CAP_STATE_STOP:
		#ifdef USE_CAP1
	        CapState = CAP_STATE_STOP;
          HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_STOP, output_pwm_cmp = 0, input_pwm_cmp = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_STOP, input_dac_per = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		break;
	case CAP_STATE_RELEASE:
		#ifdef USE_CAP1
          CapState = CAP_STATE_RELEASE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
	      	HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_RELEASE, input_pwm_cmp = 0,release_cnt = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
	        rlease_flag = 1;
          CapState = CAP_STATE_RELEASE, input_dac_per = 0;
	        #ifdef CAP_FINAL_TEST
	            HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	        #else
	            HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
	        #endif /* CAP_FINAL_TEST */
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAPex */
		
		break;
	case CAP_STATE_RECHARGE:
		#ifdef USE_CAP1
					CapState = CAP_STATE_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_RECHARGE, output_pwm_cmp = 0, recharge_cnt = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		break;
	
	case CAP_STATE_TEMP_RECHARGE:
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_TEMP_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		break;
	}
}

double Cap_Get_Cap_Voltage(void) {
	return VAL__CAP_VOLTAGE;
}

double Cap_Get_Power_Voltage(void){
	return VAL__CAP_Power_Voltage;
}

double Cap_Get_Power_CURR(void){
	return VAL__CAP_Power_CURR;
}

cap_state Cap_Get_Cap_State(void) {
	return CapState;
}

/**
  * @brief  Set the pwm compare or dac output according to the state.
  * @param  None
  * @retval None
  */
void Cap_State() { // called with period of 2 ms
	switch (CapState) {
	case CAP_STATE_STOP:
		#ifdef USE_CAP2
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	  #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);//
	  #endif /* USE_CAPex */
		break;
	case CAP_STATE_RECHARGE:
		#ifdef USE_CAP2 
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, input_pwm_cmp); //
	  #endif /* USE_CAP2 */
	 
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per );//input_dac_per 4095*3*0.02*10/3.3
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	  #endif /* USE_CAPex */
	
		break;
	case CAP_STATE_RELEASE:
		#ifdef USE_CAP2
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, output_pwm_cmp);
		  //__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, output_pwm_cmp_max);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	  #endif /* USE_CAP2 */
	  
	  #ifdef USE_CAPex
	    #ifdef CAP_FINAL_TEST
		      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per);
	    #else
	        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	    #endif /* CAP_FINAL_TEST */
	    if (Cap_Get_Cap_Voltage() > 9){
		    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, output_dac_per); //
			}
			else{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095); 
			}
				
	  #endif /* USE_CAPex */
		break;
			
	case CAP_STATE_TEMP_RECHARGE:
	 
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per );//input_dac_per 4095*3*0.02*10/3.3
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	  #endif /* USE_CAPex */
	
		break;
	}
}

/**
  * @brief  The detailed capacitance control logic.
  * @param  None
  * @retval None
  */
static void Cap_Ctr_STOP() {
  #ifdef CAP_AUTO_RECHARGE
	  if (VAL__CAP_VOLTAGE < RE_RECHARGE_VOLTAGE) {
		  #ifdef USE_CAP1
		    if (VAL__CAP_VOLTAGE < RECHARGE_VOLTAGE_MAX  && fabs(chassis_t->CMFL.RealAngle) < 1000 && fabs(CMFR.RealAngle) < 1000 && \
				  		  fabs(chassis_t->CMBL.RealAngle) < 1000 && fabs(chassis_t->CMBR.RealAngle) < 1000 && PowerHeatData.chassisPowerBuffer > 59.0f) {
		        Cap_State_Switch(CAP_STATE_RECHARGE);
	      }
		
		  #endif /* USE_CAP1 */
		
		  #ifdef USE_CAP2
		    Cap_State_Switch(CAP_STATE_RECHARGE);
		  #endif /* USE_CAP2 */
		
	}
  #else
	  (void)0;
	
	  #ifdef CAP_DEBUG
	
		  Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	    Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	    Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();*/
      if(++cnt >=5 && ncnt < DEBUG_HITS){
        cps[0][ncnt] = PowerHeatData.chassisPower * 100;
        cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
        cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
        cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
        cnt = 0;
      }else if(ncnt >= DEBUG_HITS){
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      }
    #endif /* CAP_DEBUG */
		
  #endif /*CAP_AUTO_RECHARGE */
	return;
		
	
	
}

static void Cap_Ctr_RECHARGE() {
	#ifdef USE_CAP1
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(chassis_t->CMFL.RealAngle) > 1000 || fabs(CMFR.RealAngle) > 1000 || \
	  					  fabs(chassis_t->CMBL.RealAngle) > 1000 || fabs(chassis_t->CMBR.RealAngle) > 1000 || PowerHeatData.chassisPowerBuffer < 59.0f) {
	  	Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else 
	  {
	  	;
	  }
	
	#endif /* USE_CAP1 */
	
	#ifdef USE_CAP2
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {
        #ifdef CAP_DEBUG
	          Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	          Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	          Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();
            if(++cnt >=5 && ncnt < DEBUG_HITS){
               cps[0][ncnt] = PowerHeatData.chassisPower * 100;
               cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
               cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
               cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
               cnt = 0;
            }else if(ncnt >= DEBUG_HITS){
               HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            }
         #endif /* CAP_DEBUG */
		if (recharge_cnt < 250) {
			recharge_cnt++;
			FUNC__RECAL_INPUT_PWM(0.0004f);//0.0005f
		}
		else {
			FUNC__RECAL_INPUT_PWM(0.001f);//0.002f
		}
		if (input_pwm_cmp > VAL__INPUT_PWM_MAX) {
			input_pwm_cmp = VAL__INPUT_PWM_MAX;
		}
		else if (input_pwm_cmp < 0){
			input_pwm_cmp = 0;
		}
	}
  #endif /* USE_CAP2 */
	
	#ifdef USE_CAPex

	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(chassis_t->CMFL.RealAngle - chassis_t->CMFL.TargetAngle) > 300 || fabs(chassis_t->CMFR.RealAngle - chassis_t->CMFR.TargetAngle) > 3000 || \
					  fabs(chassis_t->CMBL.RealAngle - chassis_t->CMBL.TargetAngle) > 300 || fabs(chassis_t->CMBR.RealAngle - chassis_t->CMBR.TargetAngle) > 300 || PowerHeatData.chassisPowerBuffer < 30.0f){
			      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		  	}else{
				    HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	  }
				
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {
		  input_dac_per = FUNC__RECAL_INPUT_DAC;
		  if (input_dac_per > VAL__INPUT_DAC_MAX) {
			  input_dac_per = VAL__INPUT_DAC_MAX;
		  }
		  else if (input_dac_per < 0){
			  input_dac_per = 0;
		  }
	  }
	
	#endif /* USE_CAPex */
}

static void Cap_Ctr_TEMP_RECHARGE() {
	
	#ifdef USE_CAPex
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(chassis_t->CMFL.RealAngle - chassis_t->CMFL.TargetAngle) > 300 || fabs(chassis_t->CMFR.RealAngle - chassis_t->CMFR.TargetAngle) > 3000 || \
					  fabs(chassis_t->CMBL.RealAngle - chassis_t->CMBL.TargetAngle) > 300 || fabs(chassis_t->CMBR.RealAngle - chassis_t->CMBR.TargetAngle) > 300 || PowerHeatData.chassisPowerBuffer < 30.0f){
			      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		  	}else{
				    HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	  }
				
	  if (VAL__CAP_VOLTAGE > 15){
		  Cap_State_Switch(CAP_STATE_RELEASE);
	  }else{
		    input_dac_per = FUNC__RECAL_INPUT_DAC;
		    if (input_dac_per > VAL__INPUT_DAC_MAX) {
			    input_dac_per = VAL__INPUT_DAC_MAX;
		    }
		    else if (input_dac_per < 0){
			    input_dac_per = 0;
		    }
	  }
		
	
	#endif /* USE_CAPex */
}

static void Cap_Ctr_RELEASE() {
  #ifdef USE_CAP1
	  if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
  #endif /* USE_CAP1 */
	
  #ifdef USE_CAP2
	  if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {

      #ifdef CAP_DEBUG
         if(++cnt>=5 && ncnt<DEBUG_HITS){
          cps[0][ncnt] = PowerHeatData.chassisPower * 100;
          cps[1][ncnt] = PowerHeatData.chassisPowerBuffer  * 100;
          cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
          cps[3][ncnt++] = VAL__OUTPUT_PWM_PERCENT * 100;
          cnt = 0;
         }else if(ncnt >= DEBUG_HITS){
          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,  GPIO_PIN_SET);
         }
      #endif /* CAP_DEBUG */
		  if (release_cnt < 50) { 
			  release_cnt++;
			  output_pwm_cmp_max = 0;
			  output_pwm_cmp = 0;
		  }
		  else {
			  output_pwm_cmp_max = PWM_CMP_MAX;
			  FUNC__RECAL_OUTPUT_PWM(0.0004f);//0.002f
		  }
		
		  if (output_pwm_cmp > VAL__OUTPUT_PWM_MAX) {
			  output_pwm_cmp = VAL__OUTPUT_PWM_MAX;
		  }
		  else if (output_pwm_cmp < 0) {
			  output_pwm_cmp = 0;
		  }
	  }
  #endif /* USE_CAP2 */
	
	#ifdef USE_CAPex
		  #ifdef CAP_FINAL_TEST
		    
				if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		      Cap_State_Switch(CAP_STATE_TEMP_RECHARGE);
	      }
	      else {
					if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(chassis_t->CMFL.RealAngle - chassis_t->CMFL.TargetAngle) > 300 || fabs(chassis_t->CMFR.RealAngle - chassis_t->CMFR.TargetAngle) > 3000 || \
					  fabs(chassis_t->CMBL.RealAngle - chassis_t->CMBL.TargetAngle) > 300 || fabs(chassis_t->CMBR.RealAngle - chassis_t->CMBR.TargetAngle) > 300 || PowerHeatData.chassisPowerBuffer < 30.0f){
			        HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		  	  }else{
				      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
			    } 
		      input_dac_per = FUNC__RECAL_INPUT_DAC_T;
		      if (input_dac_per > VAL__INPUT_DAC_MAX) {
			      input_dac_per = VAL__INPUT_DAC_MAX;
		      }
		      else if (input_dac_per < 0){
			      input_dac_per = 0;
		      }
          output_dac_per = FUNC__RECAL_OUTPUT_DAC;
			  }
		  #else
				if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		      Cap_State_Switch(CAP_STATE_STOP);
	      }
	      else {
		    output_dac_per = FUNC__RECAL_OUTPUT_DAC;
			  }
		  #endif /* CAP_FINAL_TEST */
	#endif /* USE_CAPex */
}

/**
  * @brief  Control the release and recharge progress.
  * @param  None
  * @retval None
  */
void Cap_Ctr() { // called with period of 2 ms
	if (RobotState.remainHP < 1 || WorkState == STOP_STATE || VAL__CAP_Power_Voltage < 5.0) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	else {
		switch (CapState) {
		case CAP_STATE_STOP:
			Cap_Ctr_STOP();
			break;
		case CAP_STATE_RECHARGE:
			Cap_Ctr_RECHARGE();
			break;
		case CAP_STATE_RELEASE:
			Cap_Ctr_RELEASE();
			break;
		case CAP_STATE_TEMP_RECHARGE:
			Cap_Ctr_TEMP_RECHARGE();
			break;	
		}
	}
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	int cnt1, cnt2;
	memset(ADC_tmp, 0, sizeof(ADC_tmp));
	for (cnt1 = 0; cnt1 < ADC_CHANNALS; cnt1++) {
		for (cnt2 = 0; cnt2 < ADC_HITS; cnt2++) {
			ADC_tmp[cnt1] += ADC_hits_val[cnt2][cnt1];
		}
		ADC_val[cnt1] = ADC_tmp[cnt1] / ADC_HITS;
	}
}
#ifdef CAP_DEBUG
  uint8_t sendfinish=1;
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(sendfinish){
      sendfinish = 0;
      HAL_UART_Transmit_DMA(&huart8, (uint8_t*)cps, sizeof(cps));
    }
  }
#endif /* CAP_DEBUG */

void LED_Show_SuperCap_Voltage(uint8_t flag)
{
	if (flag == 0)
	{
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		return;
	}
	if (VAL__CAP_VOLTAGE < 8)
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
	else {
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		int unlight = 7 - (int) ( ((pow(VAL__CAP_VOLTAGE,2) - pow(10,2)) / (pow(23,2) - pow(10,2))) * 6);
		if (unlight < 0) unlight = 0;
		HAL_GPIO_WritePin(GPIOG, 0x1fe >> unlight, GPIO_PIN_RESET);
	}
}
