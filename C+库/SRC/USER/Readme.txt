/***********************************************************************************************
                        杭州电子科技大学 智能车（光电）K66例程
												                     初次发布时间：2017年11月19日
																	 授权给 杭电智能车专用
须知：1、本例程适用K66FX1M0VLQ18芯片，其他K66系列芯片暂未测试。
	  2、本例程通过逐飞科技K66MDK库更改而成，并借鉴了超核K60库的相关文件。
					
1.1.1版本更新介绍：如果你正在使用1.1版本的固件库，可以直接替换掉flash.c flash.g port_cfg.h三个文件实现更新

更新说明：
版本号				更新时间				修改人				更新内容
 1.0			   2017.11.19			高源辰			创建1.0版本例程，适用光电摄像头
 1.1			   2018.2.26			高源辰			测试ADC模块，并修改时钟分频
														替换了逐飞oled函数库，使用旧库oled函数，优化了时序
														移植旧库flash读取函数，但尚未测试
1.1.1			   2018.2.26			高源辰			跟随逐飞科技更新了flash相关固件库
2.0.0			   2018.2.26			高源辰			重新创建MDK工程，删除无关函数，节省编译时间
2.0.1			   2018.2.27			高源辰			修复gpio模块输入功能，详见gpio.c文件注释
***********************************************************************************************/  