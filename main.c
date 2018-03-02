#include <main.h>

int main(void) {

	/*------------------------------------------------------------------------------------
	 *
	 * Initialization
	 *
	 *------------------------------------------------------------------------------------
	 */

	received_ch = 0;


	const float alpha = 0.5;
	const float OneeightyDivPi = 180.0/M_PI;

	Init();											//Initialize MSP430G2

	Init_BMI160();									//Initialize BMI160

	__enable_interrupt();

	/*------------------------------------------------------------------------------------
	 *
	 * Begin with Main Loop
	 *
	 *------------------------------------------------------------------------------------
	 */

	while (1) {

		//Uart Status machine
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
		
		//whoami=SPI_Read(CS_0,0x00);							//0xD1
		
		//Window Tracking Code

		//OUTPUT

                                //P2OUT &= ~BIT2;                             // Pin LOW
		                        //Start_UART_Transmission();
		                        //Uart_putchar((int)fabsf(yaw));
		                        //_delay_cycles(30000);
		                        //P2OUT |= BIT2;                          // Pin High
		//printf("Test" + whoami);
		switch (status){
					case FiFo_WM:
						gyro_status=Gyro_active;
						Get_Fifo(12,gyroscope_raw);



						SPI_Write(BMI160_AG, BMI160_USER_ACCEL_CONFIG_ADDR, 0x99);//ACC LP Mode US=1, BWP=AVGus =1 , ODR = 50 Hz
						//Float_to_Char_array(yaw, yaw_type);
						//Uart_TransmitTxPack(txYaw, yaw_char, 2);
						status = FiFo_Empty;
						break;
					case FiFo_Empty:
						//Read_Accelorameter(accelorameter_raw);
						//ax += accelorameter_raw[0] * aRes * 1000 * alpha;		//*aRes*1000;
						//ay += accelorameter_raw[1] * aRes * 1000 * alpha;		//*aRes*1000;
						//az += accelorameter_raw[2] * aRes * 1000 * alpha;		//*aRes*1000;

						//pitch = atan2f(ax, sqrtf(ay*ay + az*az));
						//pitch = pitch*OneeightyDivPi;
						//Float_to_Char_array(pitch, pitch_type);
						//Uart_TransmitTxPack(txPitch, pitch_char, 2);
						break;
					case High_G: //Window closing
						if (fabsf(yaw)<=180) {
								yaw = 0;
							}

						//Testing to read rest of fifo
						Get_Fifo((test_0/6),gyroscope_raw);
						SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);
						SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB0);	//Flush Fifo

						Read_Accelorameter(accelorameter_raw);
						ax = accelorameter_raw[0] * aRes * 1000;		//*aRes*1000;
						ay = accelorameter_raw[1] * aRes * 1000;		//*aRes*1000;
						az = accelorameter_raw[2] * aRes * 1000;		//*aRes*1000;

						pitch = atan2f(ax, sqrtf(ay*ay + az*az));
						pitch = pitch*OneeightyDivPi;



						if (fabsf(yaw)<=8) {
							if (fabsf(pitch)<=3)
							{
							    Start_UART_Transmission();
							    data = 0xF5;
							}
							else {
							    Start_UART_Transmission();
							    data = 0xF0;
							}
							//Uart_putchar((char)fabsf(yaw));
						}
						else{
							if (fabsf(pitch)<=3)
								{
							    Start_UART_Transmission();
							    data = 0xFF;
								}
							else {
							    Start_UART_Transmission();
							    data = 0xF0;
								} //(char)fabsf(yaw)
							//Uart_putchar((char)fabsf(yaw));
						}

						//Uart_putchar((int)fabsf(yaw));
						SPI_Write(BMI160_AG, BMI160_USER_ACCEL_CONFIG_ADDR, 0x97);//ACC LP Mode US=1, BWP=AVGus =1 , ODR = 50 Hz
						/*Float_to_Char_array(yaw, yaw_type);
						Uart_TransmitTxPack(txYaw, yaw_char, 2);
						Float_to_Char_array(pitch, pitch_type);
						Uart_TransmitTxPack(txPitch, pitch_char, 2);*/
						break;
					case No_High_G: //-> Window open
						Read_Accelorameter(accelorameter_raw);
						ax = accelorameter_raw[0] * aRes * 1000;		//*aRes*1000;
						ay = accelorameter_raw[1] * aRes * 1000;		//*aRes*1000;
						az = accelorameter_raw[2] * aRes * 1000;		//*aRes*1000;

						//Testing to read rest of fifo
						Get_Fifo((test_0/6),gyroscope_raw);
						SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);
						SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB0);	//Flush Fifo

						pitch = atan2f(ax, sqrtf(ay*ay + az*az));
						pitch = pitch*OneeightyDivPi;
						/*Float_to_Char_array(yaw, yaw_type);
						Uart_TransmitTxPack(txYaw, yaw_char, 2);
						Float_to_Char_array(pitch, pitch_type);
						Uart_TransmitTxPack(txPitch, pitch_char, 2);*/



						if (fabsf(yaw)>8) {
							if (fabsf(pitch)<=3)
								{
									Start_UART_Transmission();
									data = 0xFF;
								}
							else {
							    Start_UART_Transmission();
                                data = 0xF0;

								}//(char)fabsf(yaw)
							//Uart_putchar((char)fabsf(yaw));
						}
						else{
							if (fabsf(pitch)<=3)
							{
							    Start_UART_Transmission();
                                data = 0xF5;
							}
							else {
							    Start_UART_Transmission();
                                data = 0xF0;
							}
							//Uart_putchar((char)fabsf(yaw));
						}

						//Uart_putchar((int)fabsf(yaw));
						SPI_Write(BMI160_AG, BMI160_USER_ACCEL_CONFIG_ADDR, 0x97);//ACC LP Mode US=1, BWP=AVGus =1 , ODR = 50 Hz
						break;
					case Reset:
						__disable_interrupt();
						yaw=0;
						pitch=0;
						Init_BMI160();
						status = FiFo_Empty;
						__enable_interrupt();
						break;
					default:
						break;
		}




	/*	switch(status){
			case Gyro_Awake:
				gz = gyroscope_raw[2];
				g_Rate = (gz - gz_offset) * gRes;
				yaw += g_Rate / 100.0f;

				if (((gz <= (gz_offset + 1)) && (gz >= (gz_offset - 1))) && timer == 0) {//gz is near offset (in nomotion) and not longer active then timer
					if (nomotion_gyro_counter == 20) {
						SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x14);//Set Gyro in Suspend Mode
						test1=SPI_Read(BMI160_AG, 0x02);
						P2IFG &= ~BIT1;										//P2.1 IFG is cleared
						P2IE &= ~BIT1;
						P2IFG &= ~BIT1;										//P2.1 IFG is cleared
						status=Gyro_Shutdown;
						Read_Accelorameter(accelorameter_raw);
						ax = accelorameter_raw[0] * aRes * 1000 * alpha;		//*aRes*1000;
						ay = accelorameter_raw[1] * aRes * 1000 * alpha;		//*aRes*1000;
						az = accelorameter_raw[2] * aRes * 1000 * alpha;		//*aRes*1000;
						nomotion_gyro_counter = 0;
					} else {
						nomotion_gyro_counter++;

					}

				}
				else {
					status=Gyro_Sleep;
				}
				break;
			case Gyro_Shutdown:
				Read_Accelorameter(accelorameter_raw);
				ax += accelorameter_raw[0] * aRes * 1000 * alpha;		//*aRes*1000;
				ay += accelorameter_raw[1] * aRes * 1000 * alpha;		//*aRes*1000;
				az += accelorameter_raw[2] * aRes * 1000 * alpha;		//*aRes*1000;


				 //Roll & Pitch Equations

				 //roll  = atan2f(-ay, az);
				 //roll  = roll*OneeightyDivPi;

				 //Float_to_Char_array(roll, roll_type);
				 //Uart_TransmitTxPack(txRoll, roll_char, 2);

				break;
			case Gyro_Sleep:
				 //Go to sleep
				_BIS_SR(LPM3_bits + GIE);
				break;
			default:
				//Go to sleep
				_BIS_SR(LPM3_bits + GIE);
				break;
		}

*/
		//P2OUT |= BIT2;

		_BIS_SR(LPM3_bits + GIE);

	}
}

void Start_UART_Transmission(){
    P2OUT &= ~BIT2;                             // Pin LOW
    //_delay_cycles(30000);
    //P2OUT |= BIT2;                            // Pin High
    TA1R = 0;
    TA1CCTL1 |= CCIE;                           // Enable interrupt
    TA1CCTL2 |= CCIE;                           // Enable interrupt
    TA1CCR0 =  1200;
    TA1CCR1 =  1200;                     //  Load value to compare
    TA1CCR2 =  1100;
    TA1CTL = TASSEL_1 + MC_1;                   // ACLK + Up Mode

}

void Get_Fifo (int number_of_samples,int * destination){

int frame[6] ,number_of_unused_samples;

	number_of_unused_samples = number_of_samples-4;



	for (; number_of_samples > 0; --number_of_samples) {

			test_1 =SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);

			P2OUT &= (~BMI160_AG); 							// Pin LOW

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = (BMI160_USER_FIFO_DATA_ADDR | 0x80); 								// Send variable "reg" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			received_ch = UCB0RXBUF;						// Store received data

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[0] = UCB0RXBUF;							// Store received data = X LSB

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[1] = UCB0RXBUF;							// Store received data = X MSB

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[2] = UCB0RXBUF;							// Store received data = Y LSB

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[3] = UCB0RXBUF;							// Store received data = Y MSB

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[4] = UCB0RXBUF;							// Store received data = Z LSB

			while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
			UCB0TXBUF = 0x55; 								// Send variable "data" over SPI to Slave
			while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
			frame[5] = UCB0RXBUF;							// Store received data = Z MSB


			P2OUT |= (BMI160_AG); 							// Pin High

			_delay_cycles(150);


			if ((frame[0]==frame[1] && frame[1]==frame[2]) && (frame[2]==frame[3] && frame[3]==frame[4])) {
				test_1 = test_1 + 1;
				frame[0] = 0;
				frame[1] = 0;
				frame[2] = 0;
				frame[3] = 0;
				frame[4] = 0;
				frame[5] = 0;
				number_of_samples= number_of_samples +1;
			}

			test_2 =SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);


			if (gyro_status==Gyro_active /*|| number_of_samples<=(number_of_unused_samples)*/) {

				destination[0] = (frame[X_Calibrate+1] << 8) | frame[X_Calibrate];	//x
				destination[1] = (frame[Y_Calibrate+1] << 8) | frame[Y_Calibrate];	//y
				destination[2] = (frame[Z_Calibrate+1] << 8) | frame[Z_Calibrate];	//z

				g_Rate = (destination[2] - gz_offset) * gRes;
				yaw += g_Rate / 50.0f;
			}
			else {
				/*if (test_2 == test_1) {
					frame[0] = 0;
					frame[1] = 0;
					frame[2] = 0;
					frame[3] = 0;
					frame[4] = 0;
					frame[5] = 0;
					number_of_samples= number_of_samples +1;
				}

				destination[0] = (frame[X_Calibrate+1] << 8) | frame[X_Calibrate];	//x
				destination[1] = (frame[Y_Calibrate+1] << 8) | frame[Y_Calibrate];	//y
				destination[2] = (frame[Z_Calibrate+1] << 8) | frame[Z_Calibrate];	//z

				g_Rate = (destination[2] - gz_offset) * gRes;
				yaw += g_Rate / 50.0f;*/
			}



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
	//Port 2.2  UART_READY
	//Port 1.3	Int
	//Port 1.5  CLK
	//Port 1.6	MISO
	//Port 1.7	MOSI
	//Port 1.1	UART RX
	//Port 1.2	UART TX

	WDTCTL = WDTPW + WDTHOLD; 								// Stop WDT

	/* Use Calibration values for 1MHz Clock DCO*/
	DCOCTL = 0;
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;
	BCSCTL3 |= LFXT1S_2;

	P2OUT |= BIT0 + BIT2;							//Port 2.0 as High
	P2DIR |= BIT0 + BIT2; 							//Port 2.0 as Output
	P1SEL = BIT1 | BIT2 | BIT5 | BIT6 | BIT7; //Port1 Bit 5,6,7 as SPI Interface
	P1SEL2 = BIT1 | BIT2 | BIT5 | BIT6 | BIT7; //Port1 Bit 5,6,7 as SPI Interface

	/* Configure UCB0 as SPI Interface*/
	UCB0CTL1 = UCSWRST;
	UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC; 	// 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2; 									// SMCLK
	UCB0BR0 |= 100; 										// /2
	UCB0BR1 = 0; 											//
	UCB0CTL1 &= ~UCSWRST; 					// **Initialize USCI state machine**

	/* Configure UCA0 as UART Interface*/
	UCA0CTL1 = UCSWRST;
	UCA0CTL1 |= UCSSEL_2; 									// SMCLK
	UCA0BR0 = 104; //8MHZ 0x41									// 1MHz 9600	104		//16MHz 0x82
	UCA0BR1 = 0; //8MHz 0x03									// 1MHz 9600 0			//16MHZ 0x06
	UCA0MCTL = UCBRS0; 									// Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 					// **Initialize USCI state machine**

	/* Enable USCI_A0 RX interrupt */
	IE2 |= UCA0RXIE;

	//Power Saving

	P1SEL &= ~(BIT4+BIT0);								//P1 as GPIO
	P1DIR &= ~(BIT4+BIT0);								//P1 as Input
	P1REN |= BIT6+BIT4+BIT1+BIT0;						//P1 Pup/Pdown enabled
	P1OUT &= ~(BIT4+BIT0);								//P1 Pdown

	P2SEL &= ~(BIT7+BIT6+BIT5+BIT4+BIT3);			//P2 as GPIO
	P2DIR &= ~(BIT7+BIT6+BIT5+BIT4+BIT3);			//P2 as Input
	P2REN |= BIT7+BIT6+BIT5+BIT4+BIT3;				//P2 Pup/Pdown enabled
	P2OUT &= ~(BIT7+BIT6+BIT5+BIT4+BIT3);			//P2 Pdown

	P3SEL = 0x00;										//P3 as GPIO
	P3DIR = 0x00;										//P3 as Input
	P3REN = 0xff;										//P3 Pup/Pdown enabled
	P3OUT = 0x00;										//P3 Pdown for all



	//P1DIR |= BIT6;											//LED2 as OUtput
	//P1OUT &= ~BIT6;											//LED2 as off

	P1OUT |= BIT3;										//activate pullup
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

	//-----------LOW Power Mode----------------


	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB6);			//softreset
	__delay_cycles(100000);
	//SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x15);			//Start ACC
	SPI_Read(BMI160_AG, BMI160_USER_PMU_STAT_ADDR);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x15);			//Start ACC
	__delay_cycles(100000);
	SPI_Read(BMI160_AG, BMI160_USER_PMU_STAT_ADDR);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x11);			//Start Gyro
	__delay_cycles(100000);
	SPI_Read(BMI160_AG, BMI160_USER_PMU_STAT_ADDR);

	SPI_Write(BMI160_AG, BMI160_USER_ACCEL_CONFIG_ADDR, 0x97);//ACC LP Mode US=1, BWP=AVGus =1 , ODR = 50 Hz
	SPI_Write(BMI160_AG, BMI160_USER_ACCEL_RANGE_ADDR, 0x05);//ACC Range 0x03 -> 2g, 0x05 -> 4g, 0x08 ->8g
	SPI_Write(BMI160_AG, BMI160_USER_GYRO_CONFIG_ADDR, 0x27);//Gyro Config 2 8 -> 100Hz
	SPI_Write(BMI160_AG, BMI160_USER_GYRO_RANGE_ADDR, 0x00);//Gyro Range: 2000°/s

	Initialization();

	//Get Offset
	for (i = 0; i < 100; ++i) {
		Read_Gyroscope(gyroscope_raw);
		gz_offset += gyroscope_raw[2];
		__delay_cycles(1600);
	}
	gz_offset = gz_offset / 100;

	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_0_ADDR, 0x04);	//Mini. consec samples over Motion Threshold value
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_1_ADDR, 0x04);	//anymotion Threshold = value *grange(7.81mg) sample to sample difference
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_2_ADDR, 0x02);	//nomotion Threshold = value *grange(7.81mg) sample to sample difference
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MOTION_3_ADDR, 0x01);	//no motion config
	//SPI_Write(BMI160_AG, BMI160_USER_INTR_LOWHIGH_0_ADDR, 0x04); //High g Val+1 * 2,5ms
	//SPI_Write(BMI160_AG, BMI160_USER_INTR_LOWHIGH_1_ADDR, 0x04); //High g Val+1 * 2,5ms
	//SPI_Write(BMI160_AG, BMI160_USER_INTR_LOWHIGH_2_ADDR, 0x04); //High g Val+1 * 2,5ms
	SPI_Write(BMI160_AG, BMI160_USER_INTR_LOWHIGH_3_ADDR, 0x03); //High g Val+1 * 2,5ms
	SPI_Write(BMI160_AG, BMI160_USER_INTR_LOWHIGH_4_ADDR, 0x40); //High g Threshold 15,63*Val
	SPI_Write(BMI160_AG, BMI160_USER_FIFO_DOWN_ADDR, 0x08); 	//FIFO Framerate GYR 100Hz ACC 50Hz -> Filtered Data
	SPI_Write(BMI160_AG, BMI160_USER_FIFO_CONFIG_0_ADDR, 0x12); //Fifo Watermark -> 240=F0 Samples ^=960byte ^= 160x Gyrowert
	SPI_Write(BMI160_AG, BMI160_USER_FIFO_CONFIG_1_ADDR, 0x80); //FIFO enable Gyro storage //0x80
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_0_ADDR, 0x07);	//Enable Any Motion Interrupt on all 3 Acc axis 07
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_1_ADDR, 0x44);	//Enable Fifo WM + High g on Z axis interrupt	42
	SPI_Write(BMI160_AG, BMI160_USER_INTR_ENABLE_2_ADDR, 0x07);	//Enable noMotion Interrupt on all 3 Acc axis	07
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MAP_0_ADDR, 0x02);//Interrupt High G to Int 1
	SPI_Write(BMI160_AG, BMI160_USER_INTR_MAP_1_ADDR, 0x06);//Interrupt Fifo WM MAP to Int 2
	SPI_Write(BMI160_AG, BMI160_USER_INTR_OUT_CTRL_ADDR, 0xAA);	//Interrupt 1&2 Enable + Active High

	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x12);	//Start ACC LpMode
	__delay_cycles(100000);
	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0x14);//Start Gyro Suspend Mode
	__delay_cycles(100000);

	SPI_Write(BMI160_AG, BMI160_USER_PMU_TRIGGER_ADDR, 0x34);//Gyro sleep to suspend, wakeup if anymotion , sleep when nomotion

	SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB0);//Flush Fifo
	__delay_cycles(100000);
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

		rawData[0] = (int) SPI_Read(BMI160_AG, BMI160_ACC_X_LSB);
		rawData[1] = (int) SPI_Read(BMI160_AG, BMI160_ACC_X_MSB);
		rawData[2] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Y_LSB);
		rawData[3] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Y_MSB);
		rawData[4] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Z_LSB);
		rawData[5] = (int) SPI_Read(BMI160_AG, BMI160_ACC_Z_MSB);

		destination[0] = (rawData[X_Calibrate+1] << 8) | rawData[X_Calibrate];
		destination[1] = (rawData[Y_Calibrate+1] << 8) | rawData[Y_Calibrate];
		destination[2] = (rawData[Z_Calibrate+1] << 8) | rawData[Z_Calibrate];

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
		rawData[0] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_X_LSB);
		rawData[1] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_X_MSB);
		rawData[2] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Y_LSB);
		rawData[3] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Y_MSB);
		rawData[4] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Z_LSB);
		rawData[5] = (int) SPI_Read(BMI160_AG, BMI160_GYRO_Z_MSB);
		break;
	}
	default:
		break;
	}

	destination[0] = (rawData[X_Calibrate+1] << 8) | rawData[X_Calibrate];	//x
	destination[1] = (rawData[Y_Calibrate+1] << 8) | rawData[Y_Calibrate];	//y
	destination[2] = (rawData[Z_Calibrate+1] << 8) | rawData[Z_Calibrate];	//z

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

	while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
	UCB0TXBUF = reg; 								// Send variable "reg" over SPI to Slave
	while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
	received_ch = UCB0RXBUF;						// Store received data

	while (!(IFG2 & UCB0TXIFG)); 					// USCI_A0 TX buffer ready?
	UCB0TXBUF = data; 								// Send variable "data" over SPI to Slave
	while (!(IFG2 & UCB0RXIFG)); 					// USCI_A0 RX Received?
	received_ch = UCB0RXBUF;						// Store received data

	P2OUT |= (cs_signal); 							// Pin High

	_delay_cycles(180);

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

void Initialization(){

	/* Funktion zur Ermittlung der Gravitaionskraft
	 *
	 * -> Zuordnung der internen Achsen zum externen Koordinatensystems
	 *
	 */
	int i;
	for (i = 0; i < 10; ++i) {
		Read_Accelorameter(accelorameter_raw);
		ax += accelorameter_raw[0] * aRes*1000;
		ay += accelorameter_raw[1] * aRes*1000;
		az += accelorameter_raw[2] * aRes*1000;
		__delay_cycles(1600);
	}

	ax = fabsf(ax / 10);
	ay = fabsf(ay / 10);
	az = fabsf(az / 10);

	//X=0	Y=2		Z=4

	if (ax >= ay)
	{	if (ax >= az)
		{ Z_Calibrate = 0;
		  X_Calibrate = 4;}
		else
		{ Z_Calibrate = 4;
		  }
	}
	else
	{	if (ay >= az)
		{Z_Calibrate = 2;
		 Y_Calibrate = 0;
		 X_Calibrate = 4;}
		else
		{Z_Calibrate = 4;}

	}

}

#pragma vector=PORT1_VECTOR	//Interrupt ACC over Threshold or Anymotion detected-> Window close/Broke	PRIO 19
__interrupt void Port_1(void) {

	timer=1;
	P1IFG &= ~BIT3;											//Clear Interrupt Flag

/*
	int var = 0;
	var = SPI_Read(BMI160_AG, BMI160_USER_INTR_STAT_2_ADDR);
	if ((var & 0x07) > 0){										//Anymotion detected
		status = Gyro_Awake;
		}
	else {
		yaw = 0;
	}
*/
	LPM3_EXIT;
}

#pragma vector=PORT2_VECTOR		//FiFo Prio 20
__interrupt void Port_2(void) {

	// BMI160

	/*int var = 0;
	var = SPI_Read(BMI160_AG, BMI160_USER_STAT_ADDR);		//check Data Ready Status register 	7|6|5|4|3|2|1|0
	if ((var & 0x40) == 0x40) {								//Gyro Data Ready					Acc|Gyr|Mag ...

		status = Gyro_Awake;											//Gyro measurement
		Read_Gyroscope(gyroscope_raw);
		//P1IE &= ~BIT3;									//disable interrupt anymotion
	}
	if ((SPI_Read(BMI160_AG, BMI160_USER_PMU_STAT_ADDR) & 0x0C) == 0x00) { //Gyro Suspend Mode
		Read_Gyroscope(gyroscope_raw);
		status = Gyro_Sleep;
		//P1IE |= BIT3;								//enable interrupt anymotion
	}
	P2IFG &= ~BIT1;				//Clear Interrupt Flag
	//P1IE &= ~BIT3;											//P1.3 Interrupt disabled
	 */

	//SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);

	status = FiFo_WM;


	TAR = 0;
	timer_count=0;
	//TACTL |= TACLR;										//Reset Timer A
	//Timer A Config
	CCTL1 = CCIE;                             		// CCR1 interrupt enabled
	CCR0 = 1000;
	CCR1 = 1000;
	TACTL = TASSEL_1 + MC_1;         		       		// ACLK + Up Mode


	P2IFG &= ~BIT1;				//Clear Interrupt Flag FifO

	LPM3_EXIT;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
	while (!(IFG2 & UCA0TXIFG)); // USCI_A0 TX buffer ready?
	rx_REC = UCA0RXBUF;
	if (rx_REC == 0x1A)
		{
			status = Reset;
		}

/*	rx_REC = UCA0RXBUF;
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
	}*/
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
		if (timer_count == 4) {				///1 Count = 100ms
			timer_count = 0;
			//TACTL |= TACLR;										//Reset Timer A
			TAR = 0;
			TACTL &= ~MC_1;
			//get Gyro Data from FiFo
			//Get_Fifo(((int)(SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR))/6),gyroscope_raw);
			//__delay_cycles(100);
			//Get_Fifo(((int)(SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR))/6),gyroscope_raw);

			//If high_g had occured reset yaw if in closing position
			if (timer==1) {
				status = High_G;
			}
			else{
				status = No_High_G;
			}
			timer=0;
			gyro_status=Gyro_sleep;
			test_0 = SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);
			//SPI_Write(BMI160_AG, BMI160_CMD_COMMANDS_ADDR, 0xB0);	//Flush Fifo
			//SPI_Read(BMI160_AG, BMI160_USER_FIFO_LENGTH_0_ADDR);
			LPM3_EXIT;
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

// Timer1_A1 Interrupt Vector (TAIV) handler
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
{
  switch(TA1IV)
  {
    case  0: break;                            // No interrupt
    case  2: TA1CTL &= ~MC_1;                   // ACLK + Up Mode
            TA1R = 0;
            P2OUT |= BIT2;                          // Pin High
             break;   // CCR1
    case  4: //Uart_putchar(0x11);
        Uart_putchar(data);
        TA1CTL &= ~MC_1;                   // ACLK + Up Mode
                    TA1R = 0;
                    P2OUT |= BIT2;                          // Pin High

        break; // CCR2
    case  6: break;                          // reserved
    case  8: break;                          // reserved
    case 10: break;                          // reserved
    case 12: break;                          // reserved
    case 14: break;                          // overflow
    default: break;
  }
}
