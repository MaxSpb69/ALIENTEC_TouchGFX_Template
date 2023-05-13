#ifndef __GT9147_H
#define __GT9147_H	
#include "main.h"
#include <string.h>
//////////////////////////////////////////////////////////////////////////////////	 
//
//STM32
//GT1151 Driver
//(c)maxspb69@gmail.com
//Date:2021/08/22
//Version V1.0
////////////////////////////////////////////////////////////////////////////////// 

//IO 
#define GT_RST    		PCout(13)	//GT9147 Reset
#define GT_INT    		PCin(1)		//GT9147 Int
   	
 
//I2C
#define GT1151ADDR		0x14 << 1	// Device I2C address

//GT9147 registers
#define GT_CTRL_REG 	0X8040   	//GT1151  Control register
#define GT_CFGS_REG 	0X8050   	//GT1151 Configuration register
//#define GT_CHECK_REG 	0X80FF   	//GT1151  Checksum register
#define GT_PID_REG 		0X8140   	//GT1151 ID register

#define GT_GSTID_REG 	0X814E   	//GT1151  touch coordinate register
#define GT_TP1_REG 		0X814F  	//1st point address
#define GT_TP2_REG 		0X8157		//2nd point address
#define GT_TP3_REG 		0X815F		//3rd point address
#define GT_TP4_REG 		0X8167		//4th point address
#define GT_TP5_REG 		0X816F		//5th point address


#define GTP_DATA_BUFF_LEN(buf_len)  (1 + 8 * (buf_len) + 2)  //STATUS_TEG(1) + TOUCH_DATA(8*GTP_MAX_TOUCH) + KeyValue(1) + CheckSum(1)


#define TP_PRES_DOWN 0x80  // Флаг касания сенсорного экрана
#define TP_CATH_PRES 0x40  // Флаг нажатия кнопки
#define CT_MAX_TOUCH  10    // Максимальное число фиксируемых контроллером касаний
#define GT1151_HW_CONFIG_LEN	239

typedef struct
{
	uint8_t sta;
	uint16_t x[CT_MAX_TOUCH];
	uint16_t y[CT_MAX_TOUCH];
	uint8_t touchtype;

}_m_tp_dev;



extern I2C_HandleTypeDef hi2c1;







uint8_t GT9147_WR_Reg(uint16_t reg,uint8_t *buf,uint16_t len);
void GT9147_RD_Reg(uint16_t reg,uint8_t *buf,uint16_t len);
//uint8_t GT9147_Init(void);
//uint8_t GT9147_Scan(uint8_t mode);
#endif













