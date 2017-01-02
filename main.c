#include <main.h>

int main(void) {

	//const float alpha=0.5;

	sensor = BMI160;
	received_ch = 0;
	const float aRes = 4.0 / 32768.0;
	const float gRes = 2000.0 / 32768.0;
	//const float mRes = 10.0 * 4219.0/32760.0;
	//const float alpha = 0.5;
	//const float OneeightyDivPi = 180.0/M_PI;

	Init();

	switch (sensor) {

	case BMI160: {
		Init_BMI160();
		__delay_cycles(1000000);
		Read_Accelorameter(accelorameter_raw);
		ax = accelorameter_raw[0] * aRes;				//*aRes*1000;
		ay = accelorameter_raw[1] * aRes;				//*aRes*1000;
		az = accelorameter_raw[2] * aRes;				//*aRes*1000;
		Read_Gyroscope(gyroscope_raw);
		break;
	}
	default:
		break;
	}

	__enable_interrupt();

	while (1) {
		switch (rx_State) {
		case rxCRC:
			if (uart_rx_received == 1) {
				if (rx_CRC == PackCRC(rxPackArray, rx_LEN)) {
					rx_State = rxPARSE;
					Uart_putchar(ACK);
				} else {
					rx_State = rxIDLE;
					rx_CMD = 0x00;
					Uart_putchar(NAK);
				}
			}
			break;
		case rxPARSE:
			rx_State = rxIDLE;
			switch (rx_CMD) {
			case RXDATA:
				//getSensors();
				break;
			}
			rx_CMD = 0x00;
			break;
		}

		switch (sensor) {

		case BMI160: {
			//whoami=SPI_Read(CS_0,0x00);							//0xD1
			//test_0=SPI_Read(BMI160_AG,BMI160_USER_PMU_STAT_ADDR);
			//test_1=SPI_Read(BMI160_AG,BMI160_USER_INTR_ENABLE_0_ADDR);
			/*if (read == 1) {									//if anymotion
			 Read_Accelorameter(accelorameter_raw);
			 ax = accelorameter_raw[0] * aRes;				//aRes*1000;
			 ay = accelorameter_raw[1] * aRes;				//aRes*1000;
			 az = accelorameter_raw[2] * aRes;				//aRes*1000;
			 }*/
			if (read == 2) {		//if gyro is awake

				P2OUT |= BIT2;
				Read_Accelorameter(accelorameter_raw);
				//ax = accelorameter_raw[0] * aRes * 1000;		//*aRes*1000;
				ay = accelorameter_raw[1] * aRes * 1000;		//*aRes*1000;
				//az = accelorameter_raw[2] * aRes;				//*aRes*1000;
				Read_Gyroscope(gyroscope_raw);
				gz = gyroscope_raw[2];
				g_Rate = (gz - gz_offset) * gRes;
				yaw += g_Rate / 50.0f;

				if (ay>= ax){
					ax=ay;
				}

				if (ay >= 400.0f) {
													yaw = 0;
												}

				if (((gz <= (gz_offset + 1)) && (gz >= (gz_offset - 1)))
						&& timer == 0) {//gz is near offset (in nomotion) and not longer active then timer
					if (nomotion_gyro_counter == 20) {
						SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x14);//Set Gyro in Suspend Mode
						nomotion_gyro_counter = 0;
					} else {
						nomotion_gyro_counter++;
					}

				}

				/*
				 gx = gyroscope_raw[0] * gRes;		//gRes;
				 gy = gyroscope_raw[1] * gRes;		//gRes;
				 gz = gyroscope_raw[2] * gRes;		//gRes;

				 MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);


				 roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);

				 pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));

				 yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
				 */
				Float_to_Char_array(ax, ay_type);
				Uart_TransmitTxPack(txAY, ay_char, 2);
				//Float_to_Char_array(roll, roll_type);
				//Float_to_Char_array(pitch, pitch_type);
				Float_to_Char_array(yaw, yaw_type);
				//Uart_TransmitTxPack(txRoll, roll_char, 2);
				//Uart_TransmitTxPack(txPitch, pitch_char, 2);
				Uart_TransmitTxPack(txYaw, yaw_char, 2);
				P2OUT &= ~BIT2;
				read = 3;
			}

			//temperature = ((float)Read_Temp()/2 + 23.0);
			break;
		}
		default:
			break;
		}

		//SPI_Write(BMX055_G,BMX055_GYRO_LPM1,0x00);
		//Low Pass Filter
		/*pax = ax * alpha + (pax * 0.5);
		 pay = ay * alpha + (pay * 0.5);
		 paz = az * alpha + (paz * 0.5);

		 Float_to_Char_array(pax,ax_type);
		 Float_to_Char_array(pay,ay_type);
		 Float_to_Char_array(paz,az_type);
		 Float_to_Char_array(gx,gx_type);
		 Float_to_Char_array(gy,gy_type);
		 Float_to_Char_array(gz,gz_type);
		 Uart_TransmitTxPack(txAX,ax_char,2);
		 Uart_TransmitTxPack(txAY,ay_char,2);
		 Uart_TransmitTxPack(txAZ,az_char,2);
		 Uart_TransmitTxPack(txGX,gx_char,2);
		 Uart_TransmitTxPack(txGY,gy_char,2);
		 Uart_TransmitTxPack(txGZ,gz_char,2);

		 //Roll & Pitch Equations
		 roll  = atan2f(-pay, paz);
		 roll  = roll*OneeightyDivPi;
		 pitch = atan2f(pax, sqrtf(pay*pay + paz*paz));
		 pitch = pitch*OneeightyDivPi;*/

		//Position();
		_BIS_SR(LPM3_bits + GIE);

	}
}

void Calculate_Rotation_Angle(float gx, float gy, float gz, float ax, float ay,
		float az) {

}

void Init() {
	//--------------Init SPI ----------------------------------------------------------------------

	// Sensor				BMI160
	//Port 2.0	CS			AG
	//Port 2.1	Int2
	//Port 2.2
	//Port 1.3	Int
	//Port 1.5  CLK
	//Port 1.6	MISO
	//Port 1.7	MOSI

	WDTCTL = WDTPW + WDTHOLD; 								// Stop WDT

	/* Use Calibration values for 1MHz Clock DCO*/
	DCOCTL = 0;
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

	P2OUT |= BIT0;							//Port 2.0 as High
	P2DIR |= BIT0 + BIT2; 							//Port 2.0 as Output
	P1SEL = BIT1 | BIT2 | BIT5 | BIT6 | BIT7; //Port1 Bit 5,6,7 as SPI Interface
	P1SEL2 = BIT1 | BIT2 | BIT5 | BIT6 | BIT7; //Port1 Bit 5,6,7 as SPI Interface

	/* Configure UCB0 as SPI Interface*/
	UCB0CTL1 = UCSWRST;
	UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC; 	// 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2; 									// SMCLK
	UCB0BR0 |= 0x20; 										// /2
	UCB0BR1 = 0; 											//
	UCB0CTL1 &= ~UCSWRST; 					// **Initialize USCI state machine**

	/* Configure UCA0 as UART Interface*/
	UCA0CTL1 = UCSWRST;
	UCA0CTL1 |= UCSSEL_2; 									// SMCLK
	UCA0BR0 = 0x82; //8MHZ 0x41									// 1MHz 9600	104
	UCA0BR1 = 0x06; //8MHz 0x03									// 1MHz 9600 0
	UCA0MCTL = UCBRS0; 									// Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 					// **Initialize USCI state machine**

	/* Enable USCI_A0 RX interrupt */
	IE2 |= UCA0RXIE;

	//P1DIR |= BIT6;											//LED2 as OUtput
	//P1OUT &= ~BIT6;											//LED2 as off

	P1IE |= BIT3;										//P1.3 Interrupt enabled
	P2IE |= BIT1;										//P2.1 Interrupt enabled
	P1IES &= ~BIT3;						//Interrupt direction from low to high
	P2IES &= ~BIT1;						//Interrupt direction from low to high
	P1IFG &= ~BIT3;										//P1.3 IFG is cleared
	P2IFG &= ~BIT1;										//P2.1 IFG is cleared

	//----------------Init SPI End----------------------------------------------------------------------

	/*
	 * SPI  INTERFACE
	 *
	 * 		MSB	|	6	|	5	|	4	|	3	|	2	|	1	|	LSB
	 * 	------------------------------------------------------------------
	 * 		R/W	|	A6	|	A5	|	A4	|	A3	|	A2	|	A1	|	A0
	 *
	 *
	 * 		MSB	|	6	|	5	|	4	|	3	|	2	|	1	|	LSB
	 * 	------------------------------------------------------------------
	 * 		D7	|	D6	|	D5	|	D4	|	D3	|	D2	|	D1	|	D0
	 *
	 * 		Read	1
	 * 		Write	0
	 * 		A		Address
	 * 		D		Data
	 * 		*/
}

void Float_to_Char_array(float value, enum result_type type) {

	int transform;
	transform = (int) value;
	switch (type) {
	case ax_type:
		ax_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		ax_char[1] = (unsigned char) (transform & 0xFF);
		break;
	case ay_type:
		ay_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		ay_char[1] = (unsigned char) (transform & 0xFF);
		break;
	case az_type:
		az_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		az_char[1] = (unsigned char) (transform & 0xFF);
		break;
	case roll_type:
		roll_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		roll_char[1] = (unsigned char) (transform & 0xFF);
		break;
	case pitch_type:
		pitch_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		pitch_char[1] = (unsigned char) (transform & 0xFF);
		break;
	case yaw_type:
		yaw_char[0] = (unsigned char) (((transform) >> 8) & 0xFF);
		yaw_char[1] = (unsigned char) (transform & 0xFF);
		break;
		/*case posX_type:
		 posX_char[0]=(unsigned char)(((transform)>>8) & 0xFF);
		 posX_char[1]=(unsigned char)(transform & 0xFF);
		 break;*/
	default:
		break;
	}

}

void String_number_rightify(float number, char str[]) {

	if ((((int) (number)) <= 99999 && ((int) (number)) >= 10000)
			|| (((int) (number)) >= -99999 && ((int) (number)) <= -10000)) {
		if (str[0] == 0x2d) {
			str[6] = 0;
			str[5] = str[5];
			str[4] = str[4];
			str[3] = str[3];
			str[2] = str[2];
			str[1] = str[1];
			str[0] = str[0];
		} else {
			str[6] = 0;
			str[5] = str[4];
			str[4] = str[3];
			str[3] = str[2];
			str[2] = str[1];
			str[1] = str[0];
			str[0] = 0x20;
		}
	} else if ((((int) (number)) <= 9999 && ((int) (number)) >= 1000)
			|| (((int) (number)) >= -9999 && ((int) (number)) <= -1000)) {
		if (str[0] == 0x2d) {
			str[6] = 0;
			str[5] = str[4];
			str[4] = str[3];
			str[3] = str[2];
			str[2] = str[1];
			str[1] = 0x20;
			str[0] = str[0];
		} else {
			str[6] = 0;
			str[5] = str[3];
			str[4] = str[2];
			str[3] = str[1];
			str[2] = str[0];
			str[1] = 0x20;
			str[0] = 0x20;
		}
	} else if ((((int) (number)) <= 999 && ((int) (number)) >= 100)
			|| (((int) (number)) >= -999 && ((int) (number)) <= -100)) {
		if (str[0] == 0x2d) {
			str[6] = 0;
			str[5] = str[3];
			str[4] = str[2];
			str[3] = str[1];
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = str[0];
		} else {
			str[6] = 0;
			str[5] = str[2];
			str[4] = str[1];
			str[3] = str[0];
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = 0x20;
		}
	} else if ((((int) (number)) <= 99 && ((int) (number)) >= 10)
			|| (((int) (number)) >= -99 && ((int) (number)) <= -10)) {
		if (str[0] == 0x2d) {
			str[6] = 0;
			str[5] = str[2];
			str[4] = str[1];
			str[3] = 0x20;
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = str[0];
		} else {
			str[6] = 0;
			str[5] = str[1];
			str[4] = str[0];
			str[3] = 0x20;
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = 0x20;
		}
	} else if ((((int) (number)) <= 9 && ((int) (number)) >= 0)
			|| (((int) (number)) >= -9 && ((int) (number)) <= -1)) {
		if (str[0] == 0x2d) {
			str[6] = 0;
			str[5] = str[1];
			str[4] = 0x20;
			str[3] = 0x20;
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = str[0];
		} else {
			str[6] = 0;
			str[5] = str[0];
			str[4] = 0x20;
			str[3] = 0x20;
			str[2] = 0x20;
			str[1] = 0x20;
			str[0] = 0x20;
		}
	}
}

void Init_BMI160() {

	int i;
	/*SPI_Write(BMI160_AG,BMI160_CMD_COMMANDS_ADDR,0xB6);			//softreset
	 __delay_cycles(200000);
	 SPI_Write(BMI160_AG,BMI160_USER_ACCEL_CONFIG_ADDR,0x23);	//ACC CONFIG: 2 -> normal mode, 3 ODR 3,125Hz
	 SPI_Write(BMI160_AG,BMI160_USER_ACCEL_RANGE_ADDR,0x03);		//ACC Range 0x03 -> 2g, 0x05 -> 4g, 0x08 ->8g
	 SPI_Write(BMI160_AG,BMI160_USER_GYRO_CONFIG_ADDR,0x26);		//GYRO Config 2 -> normal Mode, 6 -> 25HZ output rate
	 SPI_Write(BMI160_AG,BMI160_USER_GYRO_RANGE_ADDR,0x00);		//Gyro Range: 2000°/s
	 SPI_Write(BMI160_AG,BMI160_CMD_COMMANDS_ADDR,0x11);			//Start ACC
	 __delay_cycles(10000);
	 SPI_Write(BMI160_AG,BMI160_CMD_COMMANDS_ADDR,0x15);			//Start Gyro
	 */

	//-----------LOW Power Mode----------------
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB6);			//softreset
	__delay_cycles(3200000);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x11);			//Start ACC
	__delay_cycles(1600000);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x15);			//Start Gyro
	__delay_cycles(1600000);

	SPI_Write(BMI160_AG, BMI160_USER_ACCEL_CONFIG_ADDR, 0x95);//ACC LP Mode US=1, BWP=AVGus =1 , ODR = 12,5Hz
	SPI_Write(BMI160_AG, BMI160_USER_ACCEL_RANGE_ADDR, 0x05);//ACC Range 0x03 -> 2g, 0x05 -> 4g, 0x08 ->8g
	SPI_Write(BMI160_AG, BMI160_USER_GYRO_CONFIG_ADDR, 0x27);//Gyro Config 2 8 -> 50Hz
	SPI_Write(BMI160_AG, BMI160_USER_GYRO_RANGE_ADDR, 0x00);//Gyro Range: 2000°/s

	//Get Offset
	for (i = 0; i < 100; ++i) {
		Read_Gyroscope(gyroscope_raw);
		gz_offset += gyroscope_raw[2];
		__delay_cycles(1600);
	}
	gz_offset = gz_offset / 100;

	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_0_ADDR, 0x01);	//Mini. consec samples over Motion Threshold value
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_1_ADDR, 0x02);	//anymotion Threshold = 3 *grange(7.81mg) sample to sample difference
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_2_ADDR, 0x01);	//nomotion Threshold = 2 *grange(7.81mg) sample to sample difference
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_3_ADDR, 0x01);	//no motion config
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_0_ADDR, 0x07);	//Enable Any Motion Interrupt on all 3 Acc axis
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_1_ADDR, 0x10);	//Enable Data ready interrupt
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_2_ADDR, 0x07);	//Enable noMotion Interrupt on all 3 Acc axis
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MAP_0_ADDR, 0x04);//Interrupt anymotion MAP to Int 1
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MAP_1_ADDR, 0x08);//Interrupt data ready MAP to Int 2
	SPI_Write(BMI160_AG, BMI160_USER_INTR_OUT_CTRL_ADDR, 0xAA);	//Interrupt 1&2 Enable + Active High

	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x12);	//Start ACC LpMode
	__delay_cycles(1600000);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x14);//Start Gyro Suspend Mode
	__delay_cycles(1600000);

	SPI_Write(BMI160_AG, BMI160_USER_PMU_TRIGGER_ADDR, 0x10);//Gyro sleep to suspend, wakeup if anymotion , sleep when nomotion

	//SPI_Write(BMI160_AG,BMI160_USER_INTR_MAP_1_ADDR,0x07);		//Interrupt MAP to Int 1

}

int Read_Temp() {

	int rawData[2];

	switch (sensor) {

	default:
		break;
	}
	rawData[0] = 0;
	rawData[1] = 0;
	return ((rawData[0] << 8) | rawData[1]);
}

void Read_Accelorameter(int * destination) {

	int rawData[6];

	switch (sensor) {

	case BMI160: {
		rawData[0] = (int) SPI_Read(BMI160_AG, BMI160_ACC_X_MSB);
		rawData[1] = (int) SPI_Read(BMI160_AG, BMI160_ACC_X_LSB);
		rawData[2] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Y_MSB);
		rawData[3] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Y_LSB);
		rawData[4] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Z_MSB);
		rawData[5] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Z_LSB);

		destination[0] = (rawData[0] << 8) | rawData[1];
		destination[1] = (rawData[2] << 8) | rawData[3];
		destination[2] = (rawData[4] << 8) | rawData[5];
		break;
	}
	default:
		break;
	}
}

void Read_Gyroscope(int * destination) {

	int rawData[6];

	switch (sensor) {

	case BMI160: {
		rawData[0] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_X_MSB);
		rawData[1] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_X_LSB);
		rawData[2] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Y_MSB);
		rawData[3] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Y_LSB);
		rawData[4] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Z_MSB);
		rawData[5] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Z_LSB);
		break;
	}
	default:
		break;
	}

	destination[0] = (rawData[0] << 8) | rawData[1];
	destination[1] = (rawData[2] << 8) | rawData[3];
	destination[2] = (rawData[4] << 8) | rawData[5];
}

int Twos_Complement_To_Decimal(int number_of_bits, int number) {

	int signed_dec_int;

	if ((number & (1 << (number_of_bits - 1))) != 0)
		signed_dec_int = number | ~((1 << (number_of_bits - 1)) - 1);
	else
		signed_dec_int = number;

	return signed_dec_int;

}

void SPI_Write(char cs_signal, char reg, char data) {

	SPI_Transceive(cs_signal, reg, data);
}

char SPI_Read(char cs_signal, char reg) {

	return (SPI_Transceive(cs_signal, (reg | 0x80), 0x55));
}

char SPI_Transceive(char cs_signal, char reg, char data) {

	P2OUT &= (~cs_signal); 							// Pin LOW

	while (!(IFG2 & UCB0TXIFG))
		; 					// USCI_A0 TX buffer ready?
	UCB0TXBUF = reg; 					// Send variable "reg" over SPI to Slave
	while (!(IFG2 & UCB0RXIFG))
		; 					// USCI_A0 RX Received?
	received_ch = UCB0RXBUF;						// Store received data

	while (!(IFG2 & UCB0TXIFG))
		; 					// USCI_A0 TX buffer ready?
	UCB0TXBUF = data; 				// Send variable "data" over SPI to Slave
	while (!(IFG2 & UCB0RXIFG))
		; 					// USCI_A0 RX Received?
	received_ch = UCB0RXBUF;						// Store received data

	P2OUT |= (cs_signal); 							// Pin High

	_delay_cycles(150);

	return (received_ch);
}

void Uart_putchar(char c) {
	/* Wait for the transmit buffer to be ready */
	while (!(IFG2 & UCA0TXIFG))
		;

	/* Transmit data */
	UCA0TXBUF = (char) c;
	__delay_cycles(1300);

}

void Uart_TransmitTxPack(char cmd, unsigned char* data, unsigned char length) {
	char txPackArray[255];
	char i = 4;
	// 01 01 09 01 00
	//build Buffer
	txPackArray[0] = 0x01;
	txPackArray[1] = PackCRC(data, length);
	txPackArray[2] = cmd;
	txPackArray[3] = length;
	for (; i < length + 4; i++) {
		txPackArray[i] = (char) data[i - 4];
	}

	//Send Buffer Out
	for (i = 0; i < length + 4; i++) {

		Uart_putchar(txPackArray[i]);
	}
	rx_ACK += 1;
	__delay_cycles(1300);
	i = 0;
	do {
		i++;
		if (rx_REC == ACK) {
			rx_ACK = 0;
		}
	} while (i < 4 ^ rx_ACK != 0);

}

char PackCRC(unsigned char *s, unsigned char length) {
	char c = 0, i = 0;

	for (; i < length; i++) {
		c ^= s[i];
	}
	return c;
}

#pragma vector=PORT1_VECTOR	//Interrupt ACC over Threshold
__interrupt void Port_1(void) {

	//MPU9250
	/*P1OUT ^= BIT6;				//Toggle LED
	 P1IFG &= ~BIT3;				//Clear Interrupt Flag
	 SPI_Read(MPU9250_AGM,MPUREG_INT_STATUS);//Clear INterrupt MPU9250*/

	//BMX055 & BMI160
	P1IFG &= ~BIT3;										//Clear Interrupt Flag
	//P1IE &= ~BIT3;											//P1.3 Interrupt disabled
	read = 1;
	LPM3_EXIT;
}

#pragma vector=PORT2_VECTOR		//Gyro
__interrupt void Port_2(void) {

	// BMI160

	int test = 0;
	test = SPI_Read(BMI160_AG, BMI160_USER_STAT_ADDR);
	if ((test & 0x40) == 0x40) {	//Gyro Data Ready

		if (read == 0) {
			//Timer A Config
			CCTL1 = CCIE;                             // CCR1 interrupt enabled
			CCR0 = 16000;
			CCR1 = 16000;
			TACTL = TASSEL_2 + MC_1;                		// SMCLK + Up_Mode
			timer = 1;
		}
		read = 2;											//Gyro measurement
		//P1IE &= ~BIT3;									//disable interrupt anymotion
	}
	if ((SPI_Read(BMI160_AG, BMI160_USER_PMU_STAT_ADDR) & 0x0C) == 0x00) { //Gyro Suspend Mode
		Read_Gyroscope(gyroscope_raw);
		read = 0;
		//P1IE |= BIT3;								//enable interrupt anymotion
	}
	P2IFG &= ~BIT1;				//Clear Interrupt Flag
	//P1IE &= ~BIT3;											//P1.3 Interrupt disabled
	LPM3_EXIT;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
	while (!(IFG2 & UCA0TXIFG))
		; // USCI_A0 TX buffer ready?
	rx_REC = UCA0RXBUF;
	switch (rx_State) {
	case rxIDLE:
		if (rx_REC == 0x01) {
			rx_State = rxSOF;
		}
		break;
	case rxSOF:
		rx_State = rxCMD;
		rx_CRC = rx_REC;
		break;
	case rxCMD:
		rx_State = rxLEN;
		rx_CMD = rx_REC;
		break;
	case rxLEN:
		rx_State = rxDAT;
		rx_LEN = rx_REC;
		break;
	case rxDAT:
		if (rec_LEN < rx_LEN) {
			rxPackArray[rec_LEN] = rx_REC;
			rec_LEN++;
		}
		if (rec_LEN >= rx_LEN) {
			rx_State = rxCRC;
			rec_LEN = 0;
			uart_rx_received = 1;
		}
		break;
	}
	LPM3_EXIT;
}

// Timer_A3 Interrupt Vector (TA0IV) handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
	switch (TA0IV) {
	case 2:                                  // CCR1
	{
		if (timer_count == 1800) {
			timer = 0;
			timer_count = 0;
			TACTL |= TACLR;										//Reset Timer A
			P2IE |= BIT1;								//P2.1 Interrupt enabled

		} else {
			timer_count++;
		}

	}
		break;
	case 4:
		break;                           // CCR2 not used
	case 10:
		break;                           // overflow not used
	}
}
