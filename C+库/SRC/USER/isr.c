/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr.c
 * @brief      		�жϺ�����
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#include "isr.h"
#include  "oled.h"
#include "NRF2401.H"
#include "Wireless.h"
extern int Groy_jicheng[10];
extern int CNT;
extern uint8_t WirelessReceive;
extern uint8_t WirelessSend;
extern uint8_t WirelessReceiveStorage[2];
extern uint8 Receive_Flag;
extern uint8_t oled_flag;
extern uint8_t stop_side;
extern void track_judge(void);
extern void Servo_pid_cal(void);
extern void motor_pwm_set(void);
extern void OLED_show(void);
extern void get_error(void);
extern uint8_t UART_ReceiveData(UART_Type *UARTx,uint8_t *ch);
extern uint8_t motor_stop;
uint16_t uwb_Distance=0;
uint8_t Uls_dat[10];
uint8_t Uls_Num=0;
int uwb[3];
int num=0;
int jiluchang[5];
int last_distance[5];
int jishi=0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��A�������жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
//    //����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
//	PORTA->ISFR = 0xffffffff;
//	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����
	
	sonyccd_VSYNC();//���жϴ�����
	PORTA_FLAG_CLR(sonyccd_VSYNC_Port);	//����жϱ�־λ
}

void PORTD_IRQHandler(void)
{
//    //����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
//	PORTA->ISFR = 0xffffffff;
//	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����
	
	sonyccd_HREF();//���жϴ�����
	PORTD_FLAG_CLR(sonyccd_HREF_Port);	//����жϱ�־λ
}
void PORTE_IRQHandler(void)
{
    //����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
	PORTE->ISFR = 0xffffffff;
	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����
	//PORTC_FLAG_CLR(C1);
   VSYNC();
}
void PORTC_IRQHandler(void)
{
		//����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
		PORTC->ISFR = 0xffffffff;
	
		NRF2401_RecData(NRF2401RXBuffer); //��������
		//�Ƚ���У��
		if(NRF2401RXBuffer[0]==36) 
		{
			WirelessReceive=NRF2401RXBuffer[1];
			Receive_Flag=1;
			NRF2401RXBuffer[0]=0;
			NRF2401RXBuffer[1]=0;
		}
//	GPIO_ClearITPendingBit(NRF2401_IRQ_PORT,NRF2401_IRQ_PIN);

}

void DMA0_IRQHandler(void)
{
	//sonyccd_DMA_IRQ();
	  DMA_IRQ_CLEAN(DMA_CH0);
    row_finished();	
}

extern uint8 change_flag,count_flag;
void PIT1_IRQHandler(void)//���ִ�з���
{
	    PIT_FlAG_CLR(pit1);				
	Groy_jicheng[9]=Groy_jicheng[8];
	Groy_jicheng[8]=Groy_jicheng[7];
	Groy_jicheng[7]=Groy_jicheng[6];
	Groy_jicheng[6]=Groy_jicheng[5];
	Groy_jicheng[5]=Groy_jicheng[4];
	Groy_jicheng[4]=Groy_jicheng[3];
	Groy_jicheng[3]=Groy_jicheng[2];
	Groy_jicheng[2]=Groy_jicheng[1];
	Groy_jicheng[1]=Groy_jicheng[0];
	Groy_jicheng[0]=Gyro_Y;
			L3G4200D_XYZ();//�����Ƕ�ֵ
}
//void PIT2_IRQHandler(void)//���ִ�з���
//{
//	    PIT_FlAG_CLR(pit2);	
//      jishi++;	
//}

uint8_t timer_cnt = 0;
void PIT2_IRQHandler(void)//���ִ�з���
{
	    PIT_FlAG_CLR(pit2);	
      timer_cnt++;
}

void UART3_RX_TX_IRQHandler(void)//����ͷ
{
	//OLED_Write_Char(0,0,1);
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //�������ݼĴ�����
    {
        //�û���Ҫ����������

    }
}

/****				UWB��������ж�				****/
uint8_t Uwb_dat[10];//���ն���
uint8_t receive_tmp = 0;
uint16_t check_tmp = 0;
uint8_t check_buffer[4];
#ifdef old_board
void UART0_RX_TX_IRQHandler(void)
#elif new_board
void UART2_RX_TX_IRQHandler(void)
#endif
{

	#ifdef old_board
	UART_ReceiveData(UART0,/*&Uls_dat[Uls_Num]*/&receive_tmp);
	#elif new_board
	UART_ReceiveData(UART2,/*&Uls_dat[Uls_Num]*/&receive_tmp);
	#endif
	//ÿ�ν��շ��ڶ��е����һλ ���յ���һλ������ǰ��
	for(uint8_t i = 0; i < 9; i++)
	{
		Uwb_dat[i] = Uwb_dat[i+1];
	}
	Uwb_dat[9] = receive_tmp;
	//�����Ƿ������ͷ����
	if(Uwb_dat[0] == 0x6D && Uwb_dat[1] == 0xD6)
	{
		//ͨ��У��λ����У�� 2-7
		check_tmp = Uwb_dat[2] + Uwb_dat[3] + Uwb_dat[4] + Uwb_dat[5] + Uwb_dat[6] + Uwb_dat[7];
		check_tmp = ~check_tmp;
		//����8λ��8�Ƚ�
		//ǰ��8λ��9�Ƚ�
		check_buffer[0] = check_tmp & 0x00FF;
		check_buffer[1] = Uwb_dat[8];
		check_buffer[2] = (check_tmp>>8) & 0x00FF;
		check_buffer[3] = Uwb_dat[9];
		if(check_buffer[0] == check_buffer[1] && check_buffer[2] == check_buffer[3])
		{
			//�����¼
			uwb_Distance = (Uwb_dat[7]<<24) | (Uwb_dat[6]<<16) | (Uwb_dat[5]<<8) | (Uwb_dat[4]);
		}
	}
}

                
#define  CHIP_DEBUG         ON		
#if(CHIP_DEBUG==ON)
__asm void wait()
{
  BX lr	
}
#endif

void HardFault_Handler(void)
{
	OLED_Clear();
	ftm_pwm_init(motor_LU_Port,motor_LU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_LD_Port,motor_LD_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RU_Port,motor_RU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RD_Port,motor_RD_Ch,motor_Frequency,0);
	OLED_Write_String(0,0,(uint8_t *)"The memory");
	
	OLED_Write_String(0,2,(uint8_t *)"or Stacks");	
	
	OLED_Write_String(0,4,(uint8_t *)"overflows");
	
	#if(CHIP_DEBUG==ON)
	wait();
	#endif
	while(1);
}

void NMI_Handler(void)
{
	OLED_Clear();
	ftm_pwm_init(motor_LU_Port,motor_LU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_LD_Port,motor_LD_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RU_Port,motor_RU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RD_Port,motor_RD_Ch,motor_Frequency,0);

	OLED_Write_String(0,0,(uint8_t *)"The Chip");
	
	OLED_Write_String(0,2,(uint8_t *)"have big");	
	
	OLED_Write_String(0,4,(uint8_t *)"error");
	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
	while(1);
}

void MemManage_Handler(void)
{
	ftm_pwm_init(motor_LU_Port,motor_LU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_LD_Port,motor_LD_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RU_Port,motor_RU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RD_Port,motor_RD_Ch,motor_Frequency,0);
	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}

void BusFault_Handler(void)
{
	ftm_pwm_init(motor_LU_Port,motor_LU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_LD_Port,motor_LD_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RU_Port,motor_RU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RD_Port,motor_RD_Ch,motor_Frequency,0);

	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}

void UsageFault_Handler(void)
{
	ftm_pwm_init(motor_LU_Port,motor_LU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_LD_Port,motor_LD_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RU_Port,motor_RU_Ch,motor_Frequency,0);
	ftm_pwm_init(motor_RD_Port,motor_RD_Ch,motor_Frequency,0);


	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}


