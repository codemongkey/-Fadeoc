/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr.c
 * @brief      		中断函数库
 * @company	   		成都逐飞科技有限公司
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
//  @brief      PROTA中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当A口启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
//    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
//	PORTA->ISFR = 0xffffffff;
//	//使用我们编写的宏定义清除发生中断的引脚
	
	sonyccd_VSYNC();//场中断处理函数
	PORTA_FLAG_CLR(sonyccd_VSYNC_Port);	//清楚中断标志位
}

void PORTD_IRQHandler(void)
{
//    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
//	PORTA->ISFR = 0xffffffff;
//	//使用我们编写的宏定义清除发生中断的引脚
	
	sonyccd_HREF();//行中断处理函数
	PORTD_FLAG_CLR(sonyccd_HREF_Port);	//清楚中断标志位
}
void PORTE_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTE->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
	//PORTC_FLAG_CLR(C1);
   VSYNC();
}
void PORTC_IRQHandler(void)
{
		//清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
		PORTC->ISFR = 0xffffffff;
	
		NRF2401_RecData(NRF2401RXBuffer); //接收数据
		//先进行校验
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
void PIT1_IRQHandler(void)//电磁执行方案
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
			L3G4200D_XYZ();//陀螺仪读值
}
//void PIT2_IRQHandler(void)//电磁执行方案
//{
//	    PIT_FlAG_CLR(pit2);	
//      jishi++;	
//}

uint8_t timer_cnt = 0;
void PIT2_IRQHandler(void)//电磁执行方案
{
	    PIT_FlAG_CLR(pit2);	
      timer_cnt++;
}

void UART3_RX_TX_IRQHandler(void)//摄像头
{
	//OLED_Write_Char(0,0,1);
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}

/****				UWB距离接收中断				****/
uint8_t Uwb_dat[10];//接收队列
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
	//每次接收放在队列的最后一位 接收到下一位将队列前移
	for(uint8_t i = 0; i < 9; i++)
	{
		Uwb_dat[i] = Uwb_dat[i+1];
	}
	Uwb_dat[9] = receive_tmp;
	//检验是否满足包头条件
	if(Uwb_dat[0] == 0x6D && Uwb_dat[1] == 0xD6)
	{
		//通过校验位进行校验 2-7
		check_tmp = Uwb_dat[2] + Uwb_dat[3] + Uwb_dat[4] + Uwb_dat[5] + Uwb_dat[6] + Uwb_dat[7];
		check_tmp = ~check_tmp;
		//后面8位与8比较
		//前面8位与9比较
		check_buffer[0] = check_tmp & 0x00FF;
		check_buffer[1] = Uwb_dat[8];
		check_buffer[2] = (check_tmp>>8) & 0x00FF;
		check_buffer[3] = Uwb_dat[9];
		if(check_buffer[0] == check_buffer[1] && check_buffer[2] == check_buffer[3])
		{
			//距离记录
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


