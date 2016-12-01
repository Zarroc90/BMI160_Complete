/*
 * main.h
 *
 *  Created on: 14.06.2016
 *      Author: Marvin
 */
//

#ifndef MAIN_H_
#define MAIN_H_

#include <msp430.h>
#include <math.h>
#include <BMI160.h>
#include <MadgwickAHRS.h>

#define rxIDLE 	0x00
#define rxSOF 	0x01
#define rxCMD 	0x02
#define rxLEN 	0x03
#define rxDAT 	0x04
#define rxCRC 	0x05
#define rxPARSE 0x06
#define ACK 	0x07
#define NAK 	0x08
#define TXDATA 	0x0A
#define RXDATA 	0x0B
#define txAX 	0xA0
#define txAY 	0xA1
#define txAZ 	0xA2
#define txGX 	0xA3
#define txGY	0xA4
#define txGZ 	0xA5
#define txMX 	0xA6
#define txMY 	0xA7
#define txMZ 	0xA8
#define txRoll	0xA9
#define txPitch	0xAA
#define txPosX  0xAB
#define txPosY  0xAC
#define txPosZ  0xAD

int test_0,test_1,test_2,test_3;
unsigned char rx_REC = 0, rx_State = rxIDLE, rx_CRC = 0, rx_CMD = 0, rx_LEN = 0, rec_LEN = 0, rxPackArray[255],rx_ACK=0,uart_rx_received=0;
volatile char received_ch;

enum sensor_type{
	MPU9250,
	LSM9DS1,
	BMX055,
	BMI160,
};

enum result_type{
	none,
	ax_type,
	ay_type,
	az_type,
	gx_type,
	gy_type,
	gz_type,
	mx_type,
	my_type,
	mz_type,
	roll_type,
	pitch_type,
	posX_type,
	posY_type,
	posZ_type
};

#define	M_PI		3.14159265358979323846	/* pi */

#define CS_0 	BIT0
#define CS_1	BIT1
#define CS_2	BIT2


int sensor,read;
int accelorameter_raw[3];
int gyroscope_raw[3];
int magnetometer_raw[3];
int whoami;
int MagID;
//float aRes;
//float gRes;
//float mRes;
float ax,ay,az,gx,gy,gz/*,mx,my,mz*/;
float pitch,roll,yaw/*,pax,pay,paz*/;
float temperature;
//unsigned char ax_char[2],ay_char[2],az_char[2],gx_char[2],gy_char[2],gz_char[2],mx_char[2],my_char[2],mz_char[2],roll_char[2],pitch_char[2],posX_char[2],direction,countx,county;
//long sstatex,sstatey,sstatez;
//signed long velocityx[2],velocityy[2],positionX[2],positionY[2];
//signed int accelerationX[2],accelerationY[2];



//void Float_to_Char_array(float value,enum result_type type);
//void String_number_rightify(float number, char *str);
void Init();

void Init_BMI160();
int Read_Temp();
void Read_Accelorameter(int * destination);
void Read_Gyroscope(int * destination);
int Twos_Complement_To_Decimal(int number_of_bits,int number);
char SPI_Transceive(char cs_signal,char reg,char data);
void SPI_Write (char cs_signal,char reg, char data);
char SPI_Read (char cs_signal,char reg);
void Uart_putchar(char c);
void Uart_TransmitTxPack(char cmd,unsigned char* data, unsigned char length);
char PackCRC(unsigned char *s, unsigned char length);



#endif /* MAIN_H_ */
