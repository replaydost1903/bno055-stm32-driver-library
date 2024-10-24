/**
  ******************************************************************************
  * @file    BNO055.c
  * @author  Kubilay Közleme
  * @brief   BNO055 9-axis orientation sensor driver
  * @version V1 07_06_2024
  *
  *          In order to receive the data we want by exchanging data with the BNO055 sensor
  *          via the I2C interface, you must perform the following steps respectively.
  *          	(+) declare a variable of data type bno055_init_t in the source file (main.c)
  *          	(+) call bno055_self_test in the source file if you want to try self-testing
  *          	(+) initialise the elements of the variable of data type bno055_init_t
  *               		by making the desired settings according to the definitions in the BNO055.h file.
  * 		 	(+) call the bno055_config function by writing the address of this variable as a parameter.
  *				(+) Create a data structure for the data types defined in BNO055.h and assign the value
  *				 		to this data structure by calling the desired data type function in a while loop.
  *				 		For example :
  *				 						-@- declarition and initialization
  *											EUL_TypeDef_T EUL={0};
  *										-@- while loop
  *											EUL = bno055_Get_Eul();
  @verbatim
 */
#include "BNO055.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
char txbuff[500];

//bno055 self test helper function
__s BNO055_Status_TypeDef_t Power_On_Self_Test(void);
__s BNO055_Status_TypeDef_t Build_In_Self_Test(void);

//bno055 config helper function
__s BNO055_Status_TypeDef_t Power_Mode_Select(uint32_t pow_mode);
__s BNO055_Status_TypeDef_t Operating_Mode_Select(uint32_t opr_mode);
__s void Axis_Map_Select(uint32_t axis_map);
__s void Axis_Sign_Select(uint32_t axis_sign);
__s BNO055_Status_TypeDef_t Acc_Config(bno055_acc acc_con);
__s BNO055_Status_TypeDef_t Gyr_Config(bno055_gyr gyr_con);
__s BNO055_Status_TypeDef_t Mag_Config(bno055_mag mag_con);
__s void Unit_Selection(uint32_t unit_select);

/**
  * @brief bno055 self test (POST and BIST)
  * @param void
  *
  * @retval void
  */

void bno055_self_test(void)
{
	BNO055_Status_TypeDef_t stat_1,stat_2;

	char msg[]="******************BNO055 Start******************";
    sprintf(txbuff,"%s\n",msg);
  	HAL_UART_Transmit(&huart2,(uint8_t*)txbuff,strlen(txbuff),HAL_MAX_DELAY);
  	HAL_Delay(500);

	stat_1 = Power_On_Self_Test();

	stat_2 = Build_In_Self_Test();

	if((stat_1 & stat_2)==1)
	{
		char msg[] = "******************Build In Self Test and Power On Self Test Success******************";
	    sprintf(txbuff,"%s\n",msg);
	  	HAL_UART_Transmit(&huart2,(uint8_t*)txbuff,strlen(txbuff),HAL_MAX_DELAY);
	  	HAL_Delay(500);
	}
	else
	{
		if((stat_1==BNO055_ERROR)&(stat_2==BNO055_OK))
		{
			char msg[] = "******************Power On Self Test Failed******************";
		    sprintf(txbuff,"%s\n",msg);
		  	HAL_UART_Transmit(&huart2,(uint8_t*)txbuff,strlen(txbuff),HAL_MAX_DELAY);
		  	HAL_Delay(500);
		}
		else if((stat_2==BNO055_ERROR)&(stat_1==BNO055_OK))
		{
			char msg[] = "******************Build In Self Test Failed******************";
		    sprintf(txbuff,"%s\n",msg);
		  	HAL_UART_Transmit(&huart2,(uint8_t*)txbuff,strlen(txbuff),HAL_MAX_DELAY);
		  	HAL_Delay(500);
		}
		else if((stat_2==BNO055_ERROR)&(stat_1==BNO055_ERROR))
		{
			char msg[] = "******************Build In Self Test and Power On Self Test Failed******************";
		    sprintf(txbuff,"%s\n",msg);
		  	HAL_UART_Transmit(&huart2,(uint8_t*)txbuff,strlen(txbuff),HAL_MAX_DELAY);
		  	HAL_Delay(500);
		}
	}
}

/**
  * @brief Power On Self Test (POST)
  * @param void
  * @note	During the device startup, a power on self test is executed.This feature checks that
  * the connected sensors and microcontroller are responding / functioning correctly.
  *
  * @retval	BNO055 State
  */

__s BNO055_Status_TypeDef_t Power_On_Self_Test(void)
{
	  //1.Adım Master Cihaz Slave Cihazdan ACK Feedbackini Aldı Mı? (Haberleşme Bağlantısı Kuruldu Mu?)
	  if(HAL_I2C_IsDeviceReady(&hi2c1, BNO055_DEVICE_ADDRESS, 100, HAL_MAX_DELAY)==HAL_OK)
	  {
		  //2.Adım Page Id Bilgisini Al
		  uint8_t page_id[1]={0};
		  HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, PAGE_ID ,1,page_id,sizeof(page_id),HAL_MAX_DELAY);
		  HAL_Delay(10);

		  //3.Adım CHIP_ID , ACC_ID , MAG_ID , GYR_ID Register Adreslerindeki Bilgileri Oku
		  if(page_id[0] == 0x00)
		  {
			  uint8_t id_control[4];
			  HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, CHIP_ID ,1,id_control,sizeof(id_control),HAL_MAX_DELAY);
			  HAL_Delay(10);

			  if(id_control[0]==BNO055_CHIP_ID_VALUE && id_control[1]==BNO055_ACC_ID_VALUE && id_control[2]==BNO055_MAG_ID_VALUE && id_control[3]==BNO055_GYR_ID_VALUE)
			  {
				  //4.Adım Self Testin Sonucunu ST_RESULT Registerındaki Veriyi Okuyarak Kontrol Et
				  uint8_t st_result[1]={0};
				  HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ST_RESULT ,1,st_result,sizeof(st_result),HAL_MAX_DELAY);
				  HAL_Delay(10);

				  if((st_result[0] & 0x0F)==0x0F)
				  {
					  Complete_Signal();
					  return BNO055_OK;
				  }
				  else
				  {
					  Error_Signal();
					  return BNO055_ERROR;
				  }
			  }
			  else
			  {
				  Error_Signal();
				  return BNO055_ERROR;
			  }
		  }
	  }
	  Error_Signal();
	  return BNO055_ERROR;
}

/**
  * @brief Build In Self-Test (BIST)
  * @param void
  * @note  System Test
  *
  * @retval	BNO055 State
  */

__s BNO055_Status_TypeDef_t Build_In_Self_Test(void)
{
	//Wait 650ms
	HAL_Delay(800);
	uint8_t page_id[1]={0};
	page_id[0]=0;
	/*Page ID 0'a geçildi*/
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, page_id, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	/*BIST Triggered*/
	uint8_t self_test[1];
	self_test[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, SYS_TRIGGER, 1, self_test, sizeof(self_test),HAL_MAX_DELAY);
	/*Wait for 400ms*/
	HAL_Delay(400);
	/*SYS_ERROR Read*/
	uint8_t error_value[1]={0x00};
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, SYS_ERR	, 1, error_value, 1, 1000);
	HAL_Delay(10);
	/*Sys_result Read*/
    uint8_t st_result[1]={0};
    HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ST_RESULT ,1,st_result,sizeof(st_result),HAL_MAX_DELAY);
    HAL_Delay(10);

	if((error_value[0] == 0x00) & ((st_result[0] & 0x0F) == 0x0F))
	{
		Complete_Signal();
		return BNO055_OK;
	}
	else
	{
		Error_Signal();
		return BNO055_ERROR;
	}
}

/**
  * @brief Power Mode Select
  * @param @BNO055_POWER_MODE
  * @note The BNO055 support three different power modes:
  *
  *							|  	- Normal mode      |
  *							| 	- Low Power Mode   |
  *							|	- Suspend mode	   |
  * @retval BNO055 State
  */

__s BNO055_Status_TypeDef_t Power_Mode_Select(uint32_t power_mode)
{
	uint8_t page_id[1]={0};
	page_id[0]=0;
	/*Page ID 0'a geçildi*/
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, page_id, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	uint8_t pwr_mode[1]={0x00};
	pwr_mode[0] = power_mode;
    HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PWR_MODE ,1,pwr_mode,sizeof(pwr_mode),HAL_MAX_DELAY);
	HAL_Delay(15);
	uint8_t reg[1]={0};
    HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, PWR_MODE ,1,pwr_mode,sizeof(pwr_mode),HAL_MAX_DELAY);
	HAL_Delay(10);
	if(reg[0]==pwr_mode[0])
	{
		Complete_Signal();
		return BNO055_OK;
	}
	Error_Signal();
	return BNO055_ERROR;
}

/**
  * @brief Operating Mode Select
  * @param @BNO055_OPERATING_MODE
  * @note The BNO055 provides a variety of output signals, which can be chosen by selecting the
  *	appropriate operation mode.
  *
  *
  *			|		Non-Fusion Modes	  |		Fusion Modes			|
  *			|		 - ACCONLY			  |		  - IMU					|
  *			|		 - MAGONLY			  |		  - COMPASS				|
  *	 	 	|	 	 - GYROONLY			  |	 	  - M4G					|
  *	 		|		 - ACCMAG			  |		  - NDOF_FMC_OFF		|
  *			|	     - ACCGYRO			  |		  - NDOF				|
  *			|	  	 - MAGGYRO			  |								|
  *			|	  	 - AMG			      |								|
  *
  *
  * @retval BNO055 State
  */

__s BNO055_Status_TypeDef_t Operating_Mode_Select(uint32_t mode)
{
	uint8_t page_id[1]={0};
	page_id[0]=0;
	/*Page ID 0'a geçildi*/
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, page_id, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	uint8_t tx_buffer[1]={0},rx_buffer[1]={0};
	tx_buffer[0] = mode;
    HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, OPR_MODE ,1,tx_buffer,sizeof(tx_buffer),HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, OPR_MODE ,1,rx_buffer,sizeof(rx_buffer),HAL_MAX_DELAY);
	if(rx_buffer[0] == tx_buffer[0])
	{
		Complete_Signal();
		return BNO055_OK;
	}
	Error_Signal();
	return BNO055_ERROR;
}

/**
  * @brief Axis remap
  * @param The device mounting position should not limit the data output of the BNO055 device.
  *  The axis of the device can be re-configured to the new reference axis.
  *
  * @retval BNO055 Status
  */

__s void Axis_Map_Select(uint32_t axis_map)
{
	/*page 0'a geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=0;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);
	/*Axis mape sensörün konumu yazıldı (P[0:7])*/
	regbuff[0] = axis_map;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, AXIS_MAP_CONFIG, 1, regbuff, 1, HAL_MAX_DELAY);
}

/**
  * @brief Axis Sign
  * @param The device mounting position should not limit the data output of the BNO055 device.
  *  The axis of the device can be re-configured to the new reference axis.
  *
  * @retval BNO055 Status
  */

__s void Axis_Sign_Select(uint32_t axis_sign)
{
	/*page 0'a geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=0;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);
	/*Axis mape sensörün konumu yazıldı (P[0:7])*/
	regbuff[0] = axis_sign;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, AXIS_MAP_SIGN, 1, regbuff, 1, HAL_MAX_DELAY);
}

/**
  * @brief Accelerometer Config
  * @param bno055_acc data type
  *
  * @note	The accelerometer configuration can be changed by writing to the ACC_Config register.
  *
  *		@BNO055_ACC_G_RANGE
  * 	@BNO055_ACC_BANDWITH
  * 	@BNO055_ACC_OPERATION_MODE
  *
  * @retval BNO055 Status
  */

__s BNO055_Status_TypeDef_t Acc_Config(bno055_acc acc_con)
{
	/*page 1'e geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=1;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);

	uint8_t temp[1]={0};
	temp[0] = (acc_con.bandwith|acc_con.g_range|acc_con.opr_mode);
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, ACC_CONFIG, 1, temp, 1, HAL_MAX_DELAY);

	temp[0]=0;
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ACC_CONFIG, 1, temp, 1, HAL_MAX_DELAY);

	if(temp[0] == (acc_con.bandwith|acc_con.g_range|acc_con.opr_mode))
	{
		return BNO055_OK;
	}
	return BNO055_ERROR;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

__s BNO055_Status_TypeDef_t Gyr_Config(bno055_gyr gyr_con)
{
	/*page 1'e geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=1;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);

	uint8_t temp[2]={0};
	temp[0] = (gyr_con.range | gyr_con.bandwith);
	temp[1] = gyr_con.opr_mode;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, GYR_CONFIG_0, 1, temp,sizeof(temp), HAL_MAX_DELAY);
	temp[0]=0;
	temp[1]=0;
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GYR_CONFIG_0, 1, temp,sizeof(temp), HAL_MAX_DELAY);
	if((temp[0]==(gyr_con.range | gyr_con.bandwith))&(temp[1]==gyr_con.opr_mode))
	{
		return BNO055_OK;
	}
	return BNO055_ERROR;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

__s BNO055_Status_TypeDef_t Mag_Config(bno055_mag mag_con)
{
	/*page 1'e geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=1;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);

	uint8_t temp[1]={0};
	temp[0] = (mag_con.data_output_rate | mag_con.opr_mode | mag_con.power_mode);
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, MAG_CONFIG, 1, temp, 1, HAL_MAX_DELAY);

	temp[0]=0;
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, MAG_CONFIG, 1, temp, 1, HAL_MAX_DELAY);

	if(temp[0] == (mag_con.data_output_rate | mag_con.opr_mode | mag_con.power_mode))
	{
		return BNO055_OK;
	}
	return BNO055_ERROR;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

__s void Unit_Selection(uint32_t unit_select)
{
	/*page 0'a geçildi*/
	uint8_t regbuff[1]={0};
	regbuff[0]=0;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, regbuff, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, UNIT_SEL, 1, regbuff, 1, HAL_MAX_DELAY);

	/*Sensörlerden alınacak verilerin birimleri yazıldı*/
	regbuff[0] = unit_select;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, UNIT_SEL, 1, regbuff, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, UNIT_SEL, 1, regbuff, 1, HAL_MAX_DELAY);

}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

void bno055_config(bno055_init_t* pbno055)
{
	Power_Mode_Select(pbno055->power_mode);

	Operating_Mode_Select(pbno055->operating_mode);

	Axis_Map_Select(pbno055->axis_map_config);

	Axis_Sign_Select(pbno055->axis_sign_config);

	Acc_Config(pbno055->acc_config);

	Gyr_Config(pbno055->gyr_config);

	Mag_Config(pbno055->mag_config);

	Unit_Selection(pbno055->unit_config);
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

ACC_TypeDef_T bno055_Get_Acc(void)
{
	ACC_TypeDef_T acc_data={0};

	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ACC_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	acc_data.x = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ACC_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	acc_data.y = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, ACC_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	acc_data.z = ((float)(accel_data[0]))/100.0f;

	return acc_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

GYR_TypeDef_T bno055_Get_Gyr(void)
{
	GYR_TypeDef_T gyr_data={0};

	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GYR_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	gyr_data.x = ((float)(accel_data[0]))/16.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GYR_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	gyr_data.y = ((float)(accel_data[0]))/16.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GYR_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	gyr_data.z = ((float)(accel_data[0]))/16.0f;

	return gyr_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

MAG_TypeDef_T bno055_Get_Mag(void)
{
	MAG_TypeDef_T mag_data={0};

	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, MAG_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	mag_data.x = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, MAG_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	mag_data.y = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, MAG_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	mag_data.z = ((float)(accel_data[0]))/100.0f;

	return mag_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

EUL_TypeDef_T bno055_Get_Eul(void)
{
	EUL_TypeDef_T eul_data={0};

	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, EUL_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	eul_data.heading = ((float)(accel_data[0]))/16.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, EUL_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	eul_data.roll = ((float)(accel_data[0]))/16.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, EUL_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	eul_data.pitch = ((float)(accel_data[0]))/16.0f;

	return eul_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


QUA_TypeDef_T bno055_Get_Qua(void)
{
	QUA_TypeDef_T qua_data={0};

	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, QUA_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	qua_data.x = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, QUA_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	qua_data.y = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, QUA_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	qua_data.z = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, QUA_DATA_W_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	qua_data.w = ((float)(accel_data[0]))/100.0f;

	return qua_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


LIN_TypeDef_T bno055_Get_Lin(void)
{
	LIN_TypeDef_T lin_data={0};
	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, LIA_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	lin_data.x = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, LIA_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	lin_data.y = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, LIA_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	lin_data.z = ((float)(accel_data[0]))/100.0f;

	return lin_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


GRV_TypeDef_T bno055_Get_Grv(void)
{
	GRV_TypeDef_T grv_data={0};
	uint8_t acc_reading[2];int16_t accel_data[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GRV_DATA_X_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	grv_data.x = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GRV_DATA_Y_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	grv_data.y = ((float)(accel_data[0]))/100.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, GRV_DATA_Z_LSB, 1, acc_reading, sizeof(acc_reading), HAL_MAX_DELAY);
	accel_data[0] = (((int16_t)((acc_reading))[1] << 8) | ((acc_reading))[0]);
	grv_data.z = ((float)(accel_data[0]))/100.0f;

	return grv_data;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


int8_t bno055_Get_Temp(void)
{
	int8_t temp_reading[1];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, TEMP, 1, temp_reading, sizeof(temp_reading), HAL_MAX_DELAY);
	return temp_reading[0];
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

void Complete_Signal(void)
{
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_Delay(500);

	  for(uint32_t time=0;time<50;time++)
	  {
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
		  HAL_Delay(time);
	  }
	  for(uint32_t time=50;time>0;time--)
	  {
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
		  HAL_Delay(time);
	  }
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */

void Error_Signal(void)
{
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  HAL_Delay(500);
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


BNO055_SystemStatus_TypeDef_t System_Status(void)
{
	uint8_t page_id[1]={0};
	page_id[0]=0;
	/*Page ID 0'a geçildi*/
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, page_id, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	uint8_t status[1]={0};
    HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, SYS_STATUS ,1,status,sizeof(status),HAL_MAX_DELAY);
	HAL_Delay(10);

    switch(status[0])
    {
    case (0):
    	return BNO055_SYSTEM_IDLE;
    	break;
    case (1):
    	return BNO055_SYSTEM_ERROR;
    	break;
    case (2):
    	return BNO055_INITIALIZING_PERIPHERALS;
    	break;
    case (3):
    	return BNO055_SYSTEM_INITIALIZATION;
    	break;
    case (4):
    	return BNO055_EXECUTING_SELFTEST;
    	break;
    case (5):
    	return BNO055_SENSOR_FUSION_ALGORITHM_RUNNING;
    	break;
    case (6):
    	return BNO055_SYSTEM_RUNNING_NO_FUSION_ALGORITHM;
    	break;
    }
    return BNO055_SYSTEM_ERROR;
}

/**
  * @brief
  * @param
  *
  *
  * @note
  *
  *
  *
  *
  * @retval
  */


/*Kalibrasyon İşlemini BOSCH Videosundaki Gibi Yap*/
BNO055_CalibStatus_TypeDef_t Calibration_Status(void)
{
	/*Local Variables*/
	uint8_t page_id[]={0};
	uint8_t calib_state[1];

	/*Page ID 0'a geçildi*/
	HAL_I2C_Mem_Write(&hi2c1, BNO055_REGISTER_WRITE_ADDRESS, PAGE_ID, 1, page_id, 1, HAL_MAX_DELAY);

	/*Kalibrasyon Durumunu Okumak İçin calib_state değişkenin live variable sekmesinden izle*/
	HAL_I2C_Mem_Read(&hi2c1, BNO055_REGISTER_READ_ADDRESS, CALIB_STAT ,1,calib_state,1,HAL_MAX_DELAY);

	/*Kalibrasyon Durumunun Değerini Döndür*/
	switch(calib_state[0])
	{
	case(3):
			return BNO055_ONLY_MAG_CALIB;
			break;
	case(12):
			return BNO055_ONLY_ACC_CALIB;
			break;
	case(15):
			return BNO055_ACC_MAG_CALIB;
			break;
	case(48):
			return BNO055_ONLY_GYR_CALIB;
			break;
	case(51):
			return BNO055_GYR_MAG_CALIB;
			break;
	case(60):
			return BNO055_GYR_ACC_CALIB;
			break;
	case(63):
			return BNO055_ACC_MAG_GYR_CALIB;
			break;
	case(192):
			return BNO055_ACC_MAG_GYR_CALIB;
			break;
	default:
			return BNO055_ERROR;
			break;
	}
	return BNO055_ERROR;
}



