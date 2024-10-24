/*
 * BNO055.h
 *
 *  Created on: May 2, 2024
 *      Author: Kubilay
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "main.h"

//static define
#define __s		static

/********************************************************************/
/*****************bno055 init data type parameters*******************/
/********************************************************************/
/*
 *	@BNO055_POWER_MODE
 */
#define BNO055_PWR_MODE_NORMAL					(0x00U)
#define BNO055_PWR_MODE_LOW_POWER_MODE			(0x01U)
#define BNO055_PWR_MODE_SUSPEND_MODE			(0x02U)
/*
 *	@BNO055_OPERATING_MODE
 */
/*Non-Fusion Operating Mode*/
#define BNO055_OPR_MODE_CONFIG_MODE				(0x00U)
#define BNO055_OPR_MODE_ACCONLY					(0x01U)
#define BNO055_OPR_MODE_MAGONLY					(0x02U)
#define BNO055_OPR_MODE_GYROONLY				(0x03U)
#define BNO055_OPR_MODE_ACCMAG					(0x04U)
#define BNO055_OPR_MODE_ACCGYRO					(0x05U)
#define BNO055_OPR_MODE_MAGGYRO					(0x06U)
#define BNO055_OPR_MODE_AMG						(0x07U)
/*Fusion Operating Mode*/
#define BNO055_OPR_MODE_IMU						(0x08U)
#define BNO055_OPR_MODE_COMPASS					(0x09U)
#define BNO055_OPR_MODE_M4G						(0x0AU)
#define BNO055_OPR_MODE_NDOF_FMC_OFF			(0x0BU)
#define BNO055_OPR_MODE_NDOF					(0x0CU)
/*
 *	@BNO055_AXIS_MAP_CONFIG
 */
#define BNO055_AXIS_MAP_P0						(0x21U)
#define BNO055_AXIS_MAP_P1						(0x24U)
#define BNO055_AXIS_MAP_P2						(0x24U)
#define BNO055_AXIS_MAP_P3						(0x21U)
#define BNO055_AXIS_MAP_P4						(0x24U)
#define BNO055_AXIS_MAP_P5						(0x21U)
#define BNO055_AXIS_MAP_P6						(0x21U)
#define BNO055_AXIS_MAP_P7						(0x24U)
/*
 *	@BNO055_AXIS_SIGN_CONFIG
 */
#define BNO055_AXIS_SIGN_P0						(0x04U)
#define BNO055_AXIS_SIGN_P1						(0x00U)
#define BNO055_AXIS_SIGN_P2						(0x06U)
#define BNO055_AXIS_SIGN_P3						(0x02U)
#define BNO055_AXIS_SIGN_P4						(0x03U)
#define BNO055_AXIS_SIGN_P5						(0x01U)
#define BNO055_AXIS_SIGN_P6						(0x07U)
#define BNO055_AXIS_SIGN_P7						(0x05U)
/*
 *	@BNO055_ACC_CONFIG_PAR
 */
/* Acceleration G Range Modes ref @BNO055_ACC_G_RANGE */
#define BNO055_ACC_2G							(0x00U)
#define BNO055_ACC_4G							(0x01U)
#define BNO055_ACC_8G							(0x02U)
#define BNO055_ACC_16G							(0x03U)
/* Acceleration Bandwith Modes ref @BNO055_ACC_BANDWITH */
#define BNO055_ACC_BW_7_81_HZ					(0x00U << 2U)
#define BNO055_ACC_BW_15_63_HZ					(0x01U << 2U)
#define BNO055_ACC_BW_31_25_HZ					(0x02U << 2U)
#define BNO055_ACC_BW_62_5_HZ					(0x03U << 2U)
#define BNO055_ACC_BW_125_HZ					(0x04U << 2U)
#define BNO055_ACC_BW_250_HZ					(0x05U << 2U)
#define BNO055_ACC_BW_500_HZ					(0x06U << 2U)
#define BNO055_ACC_BW_1000_HZ					(0x07U << 2U)
/* Acceleration Operation Modes ref @BNO055_ACC_OPERATION_MODE */
#define BNO055_ACC_OPR_NORMAL					(0x00U << 5U)
#define BNO055_ACC_OPR_SUSPEND					(0x01U << 5U)
#define BNO055_ACC_OPR_LOW_POWER_1				(0x02U << 5U)
#define BNO055_ACC_OPR_STANDBY					(0x03U << 5U)
#define BNO055_ACC_OPR_LOW_POWER_2				(0x04U << 5U)
#define BNO055_ACC_OPR_DEEP_SUSPEND				(0x05U << 5U)
/*
 *	@BNO055_GYR_CONFIG_PAR
 */
/* Gyroscope Range Modes ref @BNO055_GYR_RANGE */
#define BNO055_GYR_2000_DPS						(0x00U)
#define BNO055_GYR_1000_DPS						(0x01U)
#define BNO055_GYR_500_DPS						(0x02U)
#define BNO055_GYR_250_DPS						(0x03U)
#define BNO055_GYR_125_DPS						(0x04U)
/* Gyroscope Bandwith Modes ref @BNO055_GYR_BANDWITH */
#define BNO055_GYR_BW_523						(0x00U << 3U)
#define BNO055_GYR_BW_230						(0x01U << 3U)
#define BNO055_GYR_BW_116						(0x02U << 3U)
#define BNO055_GYR_BW_47						(0x03U << 3U)
#define BNO055_GYR_BW_23						(0x04U << 3U)
#define BNO055_GYR_BW_12						(0x05U << 3U)
#define BNO055_GYR_BW_64						(0x06U << 3U)
#define BNO055_GYR_BW_32						(0x07U << 3U)
/* Gyroscope Operation Modes ref @BNO055_GYR_OPERATION_MODE	*/
#define BNO055_GYR_OPR_NORMAL					(0x00U)
#define BNO055_GYR_OPR_FAST_POWER_UP			(0x01U)
#define	BNO055_GYR_OPR_DEEP_SUSPEND				(0x02U)
#define BNO055_GYR_OPR_SUSPEND					(0x03U)
#define BNO055_GYR_OPR_ADVANCED_POWERSAVE		(0x04U)
/*
 *	@BNO055_MAG_CONFIG_PAR
 */
/* Magnetometer Data Output Rate ref @BNO055_MAG_DATA_OUTPUT_RATE */
#define BNO055_MAG_DOR_2						(0x00U)
#define BNO055_MAG_DOR_6						(0x01U)
#define BNO055_MAG_DOR_8						(0x02U)
#define BNO055_MAG_DOR_10						(0x03U)
#define BNO055_MAG_DOR_15						(0x04U)
#define BNO055_MAG_DOR_20						(0x05U)
#define BNO055_MAG_DOR_25						(0x06U)
#define BNO055_MAG_DOR_30						(0x07U)
/* Magnetometer Operation Mode ref @BNO055_MAG_OPERATION_MODE */
#define BNO055_MAG_OPR_LOW_POWER				(0x00U << 3U)
#define BNO055_MAG_OPR_REGULAR					(0x01U << 3U)
#define BNO055_MAG_OPR_ENHANCED_REGULAR			(0x02U << 3U)
#define BNO055_MAG_OPR_HIGH_ACCURACY			(0x03U << 3U)
/* Magnetometer Power Mode ref @BNO055_MAG_POWER_MODE */
#define BNO055_MAG_PWR_NORMAL					(0x00U << 6U)
#define BNO055_MAG_PWR_SLEEP					(0x01U << 6U)
#define BNO055_MAG_PWR_SUSPEND					(0x02U << 6U)
#define BNO055_MAG_PWR_FORCE					(0x03U << 6U)
/*
 *	@BNO055_UNIT_CONFIG
 */
/* Unit Config ORI_Android_Windows */
#define BNO055_UNIT_ORIENTATION_MODE_WINDOWS	(0x00U << 7U)
#define BNO055_UNIT_ORIENTATION_MODE_ANDROID	(0x01U << 7U)
/* Unit Config TEMP_Unit */
#define BNO055_UNIT_TEMP_CELCIUS				(0x00U << 4U)
#define BNO055_UNIT_TEMP_FAHRENHEIT				(0x01U << 4U)
/* Unit Config EUL_Unit */
#define BNO055_UNIT_EUL_DEGREES					(0x00U << 2U)
#define BNO055_UNIT_EUL_RADIAN					(0x01U << 3U)
/* Unit Config GYR_Unit */
#define BNO055_UNIT_GYR_DPS						(0x00U << 1U)
#define BNO055_UNIT_GYR_RPS						(0x01U << 1U)
/* Unit Config EUL_Unit */
#define BNO055_UNIT_ACC_M_S_2					(0x00U << 0U)
#define BNO055_UNIT_ACC_MG						(0x01U << 0U)

/********************************************************************/
/************************bno055 device address***********************/
/********************************************************************/
#define 	BNO055_DEVICE						0x28U
#define 	BNO055_DEVICE_ADDRESS				(BNO055_DEVICE << 0x1U)
#define		BNO055_REGISTER_READ_ADDRESS		(BNO055_DEVICE_ADDRESS | 0x01)
#define		BNO055_REGISTER_WRITE_ADDRESS		(BNO055_DEVICE_ADDRESS | 0x00)
/********************************************************************/
/*****bno055 data type structure returned from function***************/
/********************************************************************/
typedef enum
{
	BNO055_ERROR,
	BNO055_OK
}BNO055_Status_TypeDef_t;
typedef enum
{
	BNO055_SYSTEM_IDLE,
	BNO055_SYSTEM_ERROR,
	BNO055_INITIALIZING_PERIPHERALS,
	BNO055_SYSTEM_INITIALIZATION,
	BNO055_EXECUTING_SELFTEST,
	BNO055_SENSOR_FUSION_ALGORITHM_RUNNING,
	BNO055_SYSTEM_RUNNING_NO_FUSION_ALGORITHM

}BNO055_SystemStatus_TypeDef_t;
typedef enum
{
	BNO055_ONLY_MAG_CALIB=4,
	BNO055_ONLY_ACC_CALIB=1,
	BNO055_ACC_MAG_CALIB=5,
	BNO055_ONLY_GYR_CALIB=2,
	BNO055_GYR_MAG_CALIB=6,
	BNO055_GYR_ACC_CALIB=3,
	BNO055_ACC_MAG_GYR_CALIB=7

}BNO055_CalibStatus_TypeDef_t;
/********************************************************************/
/***********************bno055 init data type************************/
/********************************************************************/
typedef struct
{
	uint32_t g_range;					/*!< ref @BNO055_ACC_G_RANGE			>!*/
	uint32_t bandwith;					/*!< ref @BNO055_ACC_BANDWITH			>!*/
	uint32_t opr_mode;					/*!< ref @BNO055_ACC_OPERATION_MODE		>!*/

}bno055_acc;
typedef struct
{
	uint32_t opr_mode;					/*!< ref @BNO055_GYR_OPERATION_MODE		>!*/
	uint32_t range;						/*!< ref @BNO055_GYR_RANGE				>!*/
	uint32_t bandwith;					/*!< ref @BNO055_GYR_BANDWITH			>!*/

}bno055_gyr;
typedef struct
{
	uint32_t opr_mode;					/*!< ref @BNO055_MAG_OPERATION_MODE		>!*/
	uint32_t data_output_rate;			/*!< ref @BNO055_MAG_DATA_OUTPUT_RATE	>!*/
	uint32_t power_mode;				/*!< ref @BNO055_MAG_POWER_MODE			>!*/

}bno055_mag;
typedef struct
{
	uint32_t power_mode;				/*!< ref @BNO055_POWER_MODE			>!*/
	uint32_t operating_mode;			/*!< ref @BNO055_OPERATING_MODE		>!*/
	uint32_t axis_map_config;			/*!< ref @BNO055_AXIS_MAP_CONFIG	>!*/
	uint32_t axis_sign_config;			/*!< ref @BNO055_AXIS_SIGN_CONFIG	>!*/
	bno055_acc	acc_config;				/*!< ref @BNO055_ACC_CONFIG_PAR		>!*/
	bno055_gyr	gyr_config;				/*!< ref @BNO055_GYR_CONFIG_PAR		>!*/
	bno055_mag	mag_config;				/*!< ref @BNO055_MAG_CONFIG_PAR		>!*/
	uint32_t unit_config;				/*!< ref @BNO055_UNIT_CONFIG		>!*/

}bno055_init_t;
/********************************************************************/
/************************bno055 register map*************************/
/********************************************************************/

/* page0 register addresses */

#define 	CHIP_ID				0x00U
#define 	ACC_ID				0x01U
#define 	MAG_ID				0x02U
#define		GYR_ID				0x03U
#define		SW_RED_ID_LSB		0x04U
#define 	SW_RED_ID_MSB		0x05U
#define 	BL_REV_ID			0x06U
#define 	PAGE_ID				0x07U
#define		ACC_DATA_X_LSB		0x08U
#define 	ACC_DATA_X_MSB		0x09U
#define		ACC_DATA_Y_LSB		0x0AU
#define		ACC_DATA_Y_MSB		0x0BU
#define		ACC_DATA_Z_LSB		0x0CU
#define		ACC_DATA_Z_MSB		0x0DU
#define		MAG_DATA_X_LSB		0x0EU
#define		MAG_DATA_X_MSB		0x0FU
#define		MAG_DATA_Y_LSB		0x10U
#define		MAG_DATA_Y_MSB		0x11U
#define		MAG_DATA_Z_LSB		0x12U
#define		MAG_DATA_Z_MSB		0x13U
#define		GYR_DATA_X_LSB		0x14U
#define		GYR_DATA_X_MSB		0x15U
#define		GYR_DATA_Y_LSB		0x16U
#define		GYR_DATA_Y_MSB		0x17U
#define		GYR_DATA_Z_LSB		0x18U
#define		GYR_DATA_Z_MSB		0x19U
#define		EUL_DATA_X_LSB		0x1AU
#define		EUL_DATA_X_MSB		0x1BU
#define		EUL_DATA_Y_LSB		0x1CU
#define		EUL_DATA_Y_MSB		0x1DU
#define		EUL_DATA_Z_LSB		0x1EU
#define		EUL_DATA_Z_MSB		0x1FU
#define		QUA_DATA_W_LSB		0x20U
#define		QUA_DATA_W_MSB		0x21U
#define		QUA_DATA_X_LSB		0x22U
#define		QUA_DATA_X_MSB		0x23U
#define		QUA_DATA_Y_LSB		0x24U
#define		QUA_DATA_Y_MSB		0x25U
#define		QUA_DATA_Z_LSB		0x26U
#define 	QUA_DATA_Z_MSB		0x27U
#define		LIA_DATA_X_LSB		0x28U
#define		LIA_DATA_X_MSB		0x29U
#define		LIA_DATA_Y_LSB		0X2AU
#define		LIA_DATA_Y_MSB		0x2BU
#define		LIA_DATA_Z_LSB		0x2CU
#define		LIA_DATA_Z_MSB		0x2DU
#define		GRV_DATA_X_LSB		0x2EU
#define		GRV_DATA_X_MSB		0x2FU
#define		GRV_DATA_Y_LSB		0x30U
#define		GRV_DATA_Y_MSB		0x31U
#define 	GRV_DATA_Z_LSB		0x32U
#define		GRV_DATA_Z_MSB		0x33U
#define		TEMP				0x34U
#define		CALIB_STAT			0x35U
#define		ST_RESULT			0x36U
#define		INT_STA				0x37U
#define		SYS_CLK_STATUS		0x38U
#define		SYS_STATUS			0x39U
#define		SYS_ERR				0x3AU
#define		UNIT_SEL			0x3BU
#define		OPR_MODE			0x3DU
#define		PWR_MODE			0x3EU
#define		SYS_TRIGGER			0x3FU
#define		TEMP_SOURCE			0x40U
#define		AXIS_MAP_CONFIG		0x41U
#define		AXIS_MAP_SIGN		0x42U
#define		ACC_OFFSET_X_LSB	0x55U
#define		ACC_OFFSET_X_MSB	0x56U
#define		ACC_OFFSET_Y_LSB	0x57U
#define		ACC_OFFSET_Y_MSB	0x58U
#define		ACC_OFFSET_Z_LSB	0x59U
#define		ACC_OFFSET_Z_MSB	0x5AU
#define		MAG_OFFSET_X_LSB	0x5BU
#define		MAG_OFFSET_X_MSB	0x5CU
#define		MAG_OFFSET_Y_LSB	0x5DU
#define		MAG_OFFSET_Y_MSB	0x5EU
#define		MAG_OFFSET_Z_LSB	0x5FU
#define		MAG_OFFSET_Z_MSB	0x60U
#define		GYR_OFFSET_X_LSB	0x61U
#define		GYR_OFFSET_X_MSB	0x62U
#define		GYR_OFFSET_Y_LSB	0x63U
#define		GYR_OFFSET_Y_MSB	0x64U
#define		GYR_OFFSET_Z_LSB	0x65U
#define		GYR_OFFSET_Z_MSB	0x66U
#define		ACC_RADIUS_LSB		0x67U
#define		ACC_RADIUS_MSB		0x68U
#define		MAG_RADIUS_LSB		0x69U
#define		MAG_RADIUS_MSB		0x6AU

/* page1 register addresses */
#define		ACC_CONFIG			0x08U
#define		MAG_CONFIG			0x09U
#define		GYR_CONFIG_0		0x0AU
#define		GYR_CONFIG_1		0x0BU
#define		ACC_SLEEP_CONFIG	0x0CU
#define		GYR_SLEEP_CONFIG	0x0DU
#define		INT_MSK				0x0FU
#define		INT_EN				0x10U
#define		ACC_AM_THRES		0x11U
#define		ACC_INT_SETTINGS	0x12U
#define		ACC_HG_DURATION		0x13U
#define		ACC_HG_THRES		0x14U
#define		ACC_NM_THRESH		0x15U
#define		ACC_NM_SET			0x16U
#define		GYR_INT_SETTING		0x17U
#define		GYR_HR_X_SET		0x18U
#define		GYR_DUR_X			0x19U
#define		GYR_HR_Y_SET		0x1AU
#define		GYR_DUR_Y			0x1BU
#define		GYR_HR_Z_SET		0x1CU
#define		GYR_DUR_Z			0x1DU
#define		GYR_AM_THRES		0x1EU
#define		GYR_AM_SET			0x1FU


/********************************************************************/
/******bno055 register description bit definition*******************/
/********************************************************************/

/*
 * CHIP_ID read-only fixed value 0xA0
 */
#define BNO055_CHIP_ID_VALUE	(0xA0U)

/*
 * ACC_ID read-only fixed value 0xFB
 */
#define BNO055_ACC_ID_VALUE		(0xFBU)

/*
 * MAG_ID read-only fixed value 0x32
 */
#define BNO055_MAG_ID_VALUE		(0x32U)

/*
 * GYR_ID read-only fixed value 0x0F
 */
#define BNO055_GYR_ID_VALUE		(0x0FU)

/*
 * SYS_TRIGGER bit definition [0x3F] - only write
 */
#define BNO055_SYS_TRIGGER_RST_SYS_Pos			(5U)
#define BNO055_SYS_TRIGGER_RST_SYS_Msk			(0x1U<<BNO055_SYS_TRIGGER_RST_SYS_Pos)
#define BNO055_SYS_TRIGGER_RST_SYS				(BNO055_SYS_TRIGGER_RST_SYS_Msk)

#define BNO055_SYS_TRIGGER_CLK_SEL_HSE_Pos		(7U)
#define BNO055_SYS_TRIGGER_CLK_SEL_HSE_Msk		(0x1U<<BNO055_SYS_TRIGGER_CLK_SEL_HSE_Pos)
#define BNO055_SYS_TRIGGER_CLK_SEL_HSE			(BNO055_SYS_TRIGGER_CLK_SEL_HSE_Msk)

#define BNO055_SYS_TRIGGER_CLK_SEL_HSI_Pos		(7U)
#define BNO055_SYS_TRIGGER_CLK_SEL_HSI_Msk		(0x0U<<BNO055_SYS_TRIGGER_CLK_SEL_HSI_Pos)
#define BNO055_SYS_TRIGGER_CLK_SEL_HSI			(BNO055_SYS_TRIGGER_CLK_SEL_HSI_Msk)

#define BNO055_SYS_TRIGGER_SELF_TEST_Pos		(0U)
#define BNO055_SYS_TRIGGER_SELF_TEST_Msk		(0x1U<<BNO055_SYS_TRIGGER_SELF_TEST_Pos)
#define BNO055_SYS_TRIGGER_SELF_TEST			(BNO055_SYS_TRIGGER_SELF_TEST_Msk)

/********************************************************************/
/************************bno055 data structure***********************/
/********************************************************************/
/*bno055 get acceleration data*/
typedef struct
{
	float x;
	float y;
	float z;

}ACC_TypeDef_T;
/*bno055 get gyroscope data*/
typedef struct
{
	float x;
	float y;
	float z;

}GYR_TypeDef_T;
/*bno055 get magnetometer data*/
typedef struct
{
	float x;
	float y;
	float z;

}MAG_TypeDef_T;
/*bno055 get euler angle data*/
typedef struct
{
	float heading;
	float roll;
	float pitch;

}EUL_TypeDef_T;
/*bno055 get quaternion angle data*/
typedef struct
{
	float x;
	float y;
	float z;
	float w;

}QUA_TypeDef_T;
/*bno055 get linear acceleration data*/
typedef struct
{
	float x;
	float y;
	float z;

}LIN_TypeDef_T;
/*bno055 get gravity acceleration data*/
typedef struct
{
	float x;
	float y;
	float z;

}GRV_TypeDef_T;

/********************************************************************/
/************************bno055 functions****************************/
/********************************************************************/

/*bno055 self test function*/
void bno055_self_test(void);

/*bno055 config function*/
void bno055_config(bno055_init_t* pbno055);

/*bno055 get data functions*/
ACC_TypeDef_T bno055_Get_Acc(void);
GYR_TypeDef_T bno055_Get_Gyr(void);
MAG_TypeDef_T bno055_Get_Mag(void);
EUL_TypeDef_T bno055_Get_Eul(void);
QUA_TypeDef_T bno055_Get_Qua(void);
LIN_TypeDef_T bno055_Get_Lin(void);
GRV_TypeDef_T bno055_Get_Grv(void);
int8_t bno055_Get_Temp(void);

/*bno055 signal functions*/
void Error_Signal(void);
void Complete_Signal(void);

/*bno055 system status and calibration function*/
BNO055_CalibStatus_TypeDef_t Calibration_Status(void);
BNO055_SystemStatus_TypeDef_t System_Status(void);

#endif /* INC_BNO055_H_ */



























