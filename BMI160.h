/*
 * BMI160.h
 *
 *  Created on: 06.09.2016
 *      Author: Marvin
 */

#ifndef BMI160_H_
#define BMI160_H_

/****************************************************/
/**\name	REGISTER DEFINITIONS       */
/***************************************************/
/*******************/
/**\name CHIP ID */
/*******************/
#define BMI160_USER_CHIP_ID_ADDR				(0x00)
/*******************/
/**\name ERROR STATUS */
/*******************/
#define BMI160_USER_ERROR_ADDR					(0X02)
/*******************/
/**\name POWER MODE STATUS */
/*******************/
#define BMI160_USER_PMU_STAT_ADDR				(0X03)
#define BMI160_ACCEL_GYRO_PMU_MASK				(0X3C)
#define BMI160_NORMAL_MODE						(0X14)
/*******************/
/**\name MAG DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_0_ADDR					(0X04)
#define BMI160_USER_DATA_1_ADDR					(0X05)
#define BMI160_USER_DATA_2_ADDR					(0X06)
#define BMI160_USER_DATA_3_ADDR					(0X07)
#define BMI160_USER_DATA_4_ADDR					(0X08)
#define BMI160_USER_DATA_5_ADDR					(0X09)
#define BMI160_USER_DATA_6_ADDR					(0X0A)
#define BMI160_USER_DATA_7_ADDR					(0X0B)
/*******************/
/**\name GYRO DATA REGISTERS */
/*******************/
#define BMI160_GYRO_X_LSB						(0X0C)
#define BMI160_GYRO_X_MSB						(0X0D)
#define BMI160_GYRO_Y_LSB						(0X0E)
#define BMI160_GYRO_Y_MSB						(0X0F)
#define BMI160_GYRO_Z_LSB						(0X10)
#define BMI160_GYRO_Z_MSB						(0X11)
/*******************/
/**\name ACCEL DATA REGISTERS */
/*******************/
#define BMI160_ACC_X_LSB						(0X12)
#define BMI160_ACC_X_MSB						(0X13)
#define BMI160_ACC_Y_LSB						(0X14)
#define BMI160_ACC_Y_MSB						(0X15)
#define BMI160_ACC_Z_LSB						(0X16)
#define BMI160_ACC_Z_MSB						(0X17)
/*******************/
/**\name SENSOR TIME REGISTERS */
/*******************/
#define BMI160_USER_SENSORTIME_0_ADDR			(0X18)
#define BMI160_USER_SENSORTIME_1_ADDR			(0X19)
#define BMI160_USER_SENSORTIME_2_ADDR			(0X1A)
/*******************/
/**\name STATUS REGISTER FOR SENSOR STATUS FLAG */
/*******************/
#define BMI160_USER_STAT_ADDR					(0X1B)
/*******************/
/**\name INTERRUPT STATUS REGISTERS */
/*******************/
#define BMI160_USER_INTR_STAT_0_ADDR			(0X1C)
#define BMI160_USER_INTR_STAT_1_ADDR			(0X1D)
#define BMI160_USER_INTR_STAT_2_ADDR			(0X1E)
#define BMI160_USER_INTR_STAT_3_ADDR			(0X1F)
/*******************/
/**\name TEMPERATURE REGISTERS */
/*******************/
#define BMI160_USER_TEMPERATURE_0_ADDR			(0X20)
#define BMI160_USER_TEMPERATURE_1_ADDR			(0X21)
/*******************/
/**\name FIFO REGISTERS */
/*******************/
#define BMI160_USER_FIFO_LENGTH_0_ADDR			(0X22)
#define BMI160_USER_FIFO_LENGTH_1_ADDR			(0X23)
#define BMI160_USER_FIFO_DATA_ADDR				(0X24)
/***************************************************/
/**\name ACCEL CONFIG REGISTERS  FOR ODR, BANDWIDTH AND UNDERSAMPLING*/
/******************************************************/
#define BMI160_USER_ACCEL_CONFIG_ADDR			(0X40)
/*******************/
/**\name ACCEL RANGE */
/*******************/
#define BMI160_USER_ACCEL_RANGE_ADDR            (0X41)
/***************************************************/
/**\name GYRO CONFIG REGISTERS  FOR ODR AND BANDWIDTH */
/******************************************************/
#define BMI160_USER_GYRO_CONFIG_ADDR            (0X42)
/*******************/
/**\name GYRO RANGE */
/*******************/
#define BMI160_USER_GYRO_RANGE_ADDR             (0X43)
/***************************************************/
/**\name MAG CONFIG REGISTERS  FOR ODR*/
/******************************************************/
#define BMI160_USER_MAG_CONFIG_ADDR				(0X44)
#ifdef FIFO_ENABLE
/***************************************************/
/**\name REGISTER FOR GYRO AND ACCEL DOWNSAMPLING RATES FOR FIFO*/
/******************************************************/
#define BMI160_USER_FIFO_DOWN_ADDR              (0X45)
/***************************************************/
/**\name FIFO CONFIG REGISTERS*/
/******************************************************/
#define BMI160_USER_FIFO_CONFIG_0_ADDR          (0X46)
#define BMI160_USER_FIFO_CONFIG_1_ADDR          (0X47)
#endif

/***************************************************/
/**\name MAG INTERFACE REGISTERS*/
/******************************************************/
#define BMI160_USER_MAG_IF_0_ADDR				(0X4B)
#define BMI160_USER_MAG_IF_1_ADDR				(0X4C)
#define BMI160_USER_MAG_IF_2_ADDR				(0X4D)
#define BMI160_USER_MAG_IF_3_ADDR				(0X4E)
#define BMI160_USER_MAG_IF_4_ADDR				(0X4F)
/***************************************************/
/**\name INTERRUPT ENABLE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_ENABLE_0_ADDR			(0X50)
#define BMI160_USER_INTR_ENABLE_1_ADDR			(0X51)
#define BMI160_USER_INTR_ENABLE_2_ADDR			(0X52)
#define BMI160_USER_INTR_OUT_CTRL_ADDR			(0X53)
/***************************************************/
/**\name LATCH DURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LATCH_ADDR				(0X54)
/***************************************************/
/**\name MAP INTERRUPT 1 and 2 REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_MAP_0_ADDR				(0X55)
#define BMI160_USER_INTR_MAP_1_ADDR				(0X56)
#define BMI160_USER_INTR_MAP_2_ADDR				(0X57)
/***************************************************/
/**\name DATA SOURCE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_DATA_0_ADDR			(0X58)
#define BMI160_USER_INTR_DATA_1_ADDR			(0X59)
/***************************************************/
/**\name
INTERRUPT THRESHOLD, HYSTERESIS, DURATION, MODE CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LOWHIGH_0_ADDR			(0X5A)
#define BMI160_USER_INTR_LOWHIGH_1_ADDR			(0X5B)
#define BMI160_USER_INTR_LOWHIGH_2_ADDR			(0X5C)
#define BMI160_USER_INTR_LOWHIGH_3_ADDR			(0X5D)
#define BMI160_USER_INTR_LOWHIGH_4_ADDR			(0X5E)
#define BMI160_USER_INTR_MOTION_0_ADDR			(0X5F)
#define BMI160_USER_INTR_MOTION_1_ADDR			(0X60)
#define BMI160_USER_INTR_MOTION_2_ADDR			(0X61)
#define BMI160_USER_INTR_MOTION_3_ADDR			(0X62)
#define BMI160_USER_INTR_TAP_0_ADDR				(0X63)
#define BMI160_USER_INTR_TAP_1_ADDR				(0X64)
#define BMI160_USER_INTR_ORIENT_0_ADDR			(0X65)
#define BMI160_USER_INTR_ORIENT_1_ADDR			(0X66)
#define BMI160_USER_INTR_FLAT_0_ADDR			(0X67)
#define BMI160_USER_INTR_FLAT_1_ADDR			(0X68)
/***************************************************/
/**\name FAST OFFSET CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_FOC_CONFIG_ADDR				(0X69)
/***************************************************/
/**\name MISCELLANEOUS CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_CONFIG_ADDR					(0X6A)
/***************************************************/
/**\name SERIAL INTERFACE SETTINGS REGISTER*/
/******************************************************/
#define BMI160_USER_IF_CONFIG_ADDR				(0X6B)
/***************************************************/
/**\name GYRO POWER MODE TRIGGER REGISTER */
/******************************************************/
#define BMI160_USER_PMU_TRIGGER_ADDR			(0X6C)
/***************************************************/
/**\name SELF_TEST REGISTER*/
/******************************************************/
#define BMI160_USER_SELF_TEST_ADDR				(0X6D)
/***************************************************/
/**\name SPI,I2C SELECTION REGISTER*/
/******************************************************/
#define BMI160_USER_NV_CONFIG_ADDR				(0x70)
/***************************************************/
/**\name ACCEL AND GYRO OFFSET REGISTERS*/
/******************************************************/
#define BMI160_USER_OFFSET_0_ADDR				(0X71)
#define BMI160_USER_OFFSET_1_ADDR				(0X72)
#define BMI160_USER_OFFSET_2_ADDR				(0X73)
#define BMI160_USER_OFFSET_3_ADDR				(0X74)
#define BMI160_USER_OFFSET_4_ADDR				(0X75)
#define BMI160_USER_OFFSET_5_ADDR				(0X76)
#define BMI160_USER_OFFSET_6_ADDR				(0X77)
/***************************************************/
/**\name STEP COUNTER INTERRUPT REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_COUNT_0_ADDR			(0X78)
#define BMI160_USER_STEP_COUNT_1_ADDR			(0X79)
/***************************************************/
/**\name STEP COUNTER CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_CONFIG_0_ADDR			(0X7A)
#define BMI160_USER_STEP_CONFIG_1_ADDR			(0X7B)
/***************************************************/
/**\name COMMAND REGISTER*/
/******************************************************/
#define BMI160_CMD_COMMANDS_ADDR				(0X7E)

#define BMI160_AG	CS_0

#endif /* BMI160_H_ */
