#include "gt1151.h"


//////////////////////////////////////////////////////////////////////////////////	 
//STM32
//GT9147 Driver
//Date:2021/08/21
//Version V1.0
////////////////////////////////////////////////////////////////////////////////// 



uint8_t HW_config[GT1151_HW_CONFIG_LEN];

_m_tp_dev tp_dev;



// Запись в выбранный регистр GT9147
// reg: адрес регистра
// buf: указатель на содержимое
// len: длина массива
// Возврат результата: 0 - ОК, 1 - Error
uint8_t gt1151_wr_reg(uint16_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef err;
	uint8_t localbuf[1024];

	localbuf[0] = (reg>>8);
	localbuf[1] = (reg&0XFF);
	memcpy(&localbuf[2], buf, len);


	err = HAL_I2C_Master_Transmit(&hi2c1, GT1151ADDR, localbuf, len + 2, 300);		// Записали адрес регистра и содержимое buf длиной length
	if(err == HAL_OK)
		return 0;
	else
		return 1;
}

// Чтение из выбранного регистра GT9147
// reg: адрес регистра
// buf: указатель на буфер приема
// len: длина читаемого массива
// Возврат результата: 0 - ОК, 1 - Error
void gt1151_rd_reg(uint16_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef err;
	uint8_t localbuf[2];

	localbuf[0] = (reg>>8);
	localbuf[1] = (reg&0XFF);

	HAL_I2C_Master_Transmit(&hi2c1, GT1151ADDR, localbuf, 2, 100);	// Записали в контроллер адрес из которого будем дальше читать
	err = HAL_I2C_Master_Receive(&hi2c1, GT1151ADDR, buf, len, 100);
	if(err == HAL_OK)
		return 0;
	else
		return 1;
}


GPIO_InitTypeDef GPIO_InitStruct = {0};

// Инициализация контроллера GT9147
// Возвращаемое значение
//	0 - OK
//	1 - ERROR
uint8_t GT9147_Init(void)
{
	uint8_t temp[5];
//	GPIO_InitTypeDef GPIO_InitStruct = {0};

	tp_dev.touchtype = 0;


	HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_SET);	// Вывод INT подтягиваем к 1 для задания  адреса
	HAL_GPIO_WritePin(TOUCH_RES_GPIO_Port, TOUCH_RES_Pin, GPIO_PIN_SET);	// Вывод Touch Reset тоже подтягиваем к 1 изначально
	//osDelay(20);
	HAL_Delay(20);
	HAL_GPIO_WritePin(TOUCH_RES_GPIO_Port, TOUCH_RES_Pin, GPIO_PIN_RESET);	// Вывод Touch Reset сбрасываем в 0
	HAL_Delay(10);
	//osDelay(10);
	HAL_GPIO_WritePin(TOUCH_RES_GPIO_Port, TOUCH_RES_Pin, GPIO_PIN_SET);	// устанавливаем обратно в  1
	HAL_Delay(10);
	//osDelay(10);


	GPIO_InitStruct.Pin = TOUCH_INT_Pin;									//Перенастроить TouchInt на ввод
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);


	HAL_Delay(100);
	//osDelay(100);
	gt1151_rd_reg(GT_PID_REG, temp, 4);			// Считываем ID контроллера
	temp[4] = 0;
	if(strcmp((char*)temp, "1158") == 0)  		//Если ID  = 1158
	{
		temp[0] = 0x02;
		gt1151_wr_reg(GT_CTRL_REG, temp, 1);	//Soft reset GT1151
		gt1151_get_hw_config();

	//	sleep_off();

		return 0;
	} 
	return 1;
}


void sleep_off(void)
{

	HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = TOUCH_INT_Pin;									//Перенастроить TouchInt на вывод
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

	HAL_Delay(60);
	HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_RESET);


	GPIO_InitStruct.Pin = TOUCH_INT_Pin;									//Перенастроить TouchInt на ввод
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);



}


void gt1151_get_hw_config(void)
{

	gt1151_rd_reg(GT_CFGS_REG, HW_config, GT1151_HW_CONFIG_LEN);

}


void gt1151_update_hw_config(void)
{
    uint16_t cfg_len = GT1151_HW_CONFIG_LEN;
    uint16_t checksum = 0, i;

    for (i = 0; i < cfg_len - 3; i += 2)
    {
        checksum += ((uint16_t)HW_config[i] << 8) + HW_config[i+1];
    }
    checksum = 0 - checksum;

    HW_config[cfg_len-3] = (uint8_t)(checksum >> 8);
    HW_config[cfg_len-2] = (uint8_t)(checksum & 0xFF);
    HW_config[cfg_len-1] = 0x1;

    gt1151_wr_reg(GT_CFGS_REG, HW_config, GT1151_HW_CONFIG_LEN);
    HAL_Delay(200);
}



const uint16_t GT9147_TPX_TBL[5]={GT_TP1_REG, GT_TP2_REG, GT_TP3_REG, GT_TP4_REG, GT_TP5_REG};







static uint16_t gt1151_touch_event_handler(uint8_t *data, uint8_t data_len, uint8_t* user_data_buf, uint8_t user_touch_num)
{

    uint8_t touch_data[GTP_DATA_BUFF_LEN(CT_MAX_TOUCH)] = {0};
	uint8_t touch_num = 0;

    touch_num = data[0] & 0x0F;
    if (touch_num > CT_MAX_TOUCH)
        return 0;

    memcpy(touch_data, data, data_len);

	/* read the remaining coor data
        * 0x814E(touch status) + 8(every coordinate
		* consist of 8 bytes data) * touch num +
        * keycode + checksum
        */
    if (touch_num > 1)
    	gt1151_rd_reg(GT_GSTID_REG + data_len, &touch_data[data_len], (GTP_DATA_BUFF_LEN(touch_num) - data_len));

    /* calc checksum */
	uint8_t checksum = 0;
	for (uint16_t i = 0; i < GTP_DATA_BUFF_LEN(touch_num); i++)
		checksum += touch_data[i];

    if (checksum)			 /* checksum error, read again */
    	gt1151_rd_reg(GT_GSTID_REG, touch_data, GTP_DATA_BUFF_LEN(touch_num));

    checksum = 0;
    for (int i = 0; i < GTP_DATA_BUFF_LEN(touch_num); i++)
    	checksum += touch_data[i];

    if (checksum)
        return 0;

    if(user_touch_num < 2 && touch_num > 0)
    {
    	user_data_buf[0] = touch_data[2];
    	user_data_buf[1] = touch_data[3];
    	user_data_buf[2] = touch_data[4];
    	user_data_buf[3] = touch_data[5];

    	return 1;
    }
    else
    	return 0;



}








void gt1151_clear_status(void)
{
	uint8_t temp = 0;
	gt1151_wr_reg(GT_GSTID_REG, &temp, 1);
}


// Сканирование сенсорного экрана (используя метод запроса)
// Возвращаемое значение: текущее состояние сенсорного экрана.
// 0 - нет касания; 1 -  есть касание.



uint8_t touch_gt1151_readpoint(uint8_t * data_buf, uint8_t touch_num)
{

    uint8_t point_data[11] = {0};
    uint8_t res = 0;

    // struct rt_touch_data *pdata = (struct rt_touch_data *)data_buf;

    gt1151_rd_reg(GT_GSTID_REG, point_data, sizeof(point_data));	// Читаем 11 байт из регистра координат

    if ((point_data[0]) < 0x80)  // no data ready
        return 0;

    res = gt1151_touch_event_handler(point_data, sizeof(point_data), data_buf, touch_num);

    gt1151_clear_status();
    return res;
}

























