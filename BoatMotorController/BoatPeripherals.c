#include <msp430.h>
#include "BoatPeripherals.h"
#ifdef __MSP430F2272__
#include "../BmsMaster/BmsMasterDefines.h" //for SPI pin commands!
#endif


//This function enables the 1.5V internal reference, and samples very slowly.
void InitAdc( U16 active_inputs )
{

#ifdef __MSP430F2272__

	//Master BMS

	ADC10AE0 |= ( active_inputs & 0x00FF );
	ADC10AE1 |= ( ( active_inputs >> 8 ) & 0x00F0 );

	ADC10CTL0 = ADC10SHT_3 + ADC10ON + SREF_1 + REFON; 			//ADC10SHT_3=64 × ADC10CLKs, SREF_1: VR+=VREF+ and VR- = VSS

	ADC10CTL1 = INCH_1 + ADC10SSEL_3 + ADC10DIV_7; 			//INCH_1 is just a place holder here.
															//ADC10SSEL_3=SMCLK, ADC10DIV_7=/8 clock divider.

#else

	//Motor Controller

	ADC10AE0 |= (active_inputs & 0x00FF);				//BIT0 | BIT5 | BIT4;

	ADC10CTL0 = ADC10SHT_3 + ADC10ON + SREF_1 + REFON; 	//ADC10SHT_3=64 × ADC10CLKs, SREF_1=VR+ = VREF+ and VR- = VSS, REFON
	ADC10CTL1 = INCH_1 + ADC10SSEL_3 + ADC10DIV_7; 			//INCH_1 is just a place holder here.
															//ADC10SSEL_3=SMCLK, ADC10DIV_7=/8 clock divider.

#endif

}

U16 ReadAnalogValue( U16 input )
{
	//input = INCH_5;
	U16 value = 0;

	ADC10CTL0 &= ~ADC10IFG; //clear complete interrupt flag

	ADC10CTL0 &= ~ENC; //clear enable conversion?
	//ADC10CTL0 &= ~ADC10ON; //turn off adc (testing)

	ADC10CTL1 = input + ADC10SSEL_3 + ADC10DIV_7;

	//ADC10CTL0 |= ADC10ON; //turn ADC back on.

	ADC10CTL0 |= ENC | ADC10SC;

	while( (ADC10CTL0 & ADC10IFG) == 0 );

	value = ADC10MEM;

	return value;
}

//Init I2C with interrupts enabled.
void InitI2C( U8 slave_address )
{
//I2C not used on BMS master
#ifndef __MSP430F2272__


	//slave_address = 0x60;

	//Peripheral select
	P1SEL |= BIT6 | BIT7;
	P1SEL2 |= BIT6 | BIT7;

	UCB0CTL1 |= UCSWRST;

	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;

	UCB0CTL1 = UCSSEL_2 + UCSWRST;

	//note, this is much slower on cell BMS boards.
	UCB0BR0 = 0x40;                             // fSCL = SMCLK/?
	UCB0BR1 = 0;

	UCB0I2CSA = slave_address;

	UCB0CTL1 &= ~UCSWRST;
	IE2 |= UCB0RXIE;
#endif

}

void WriteByteI2c( U8 device_address, U8 data )
{
	while( ( UCB0CTL1 & UCTXSTP ) != 0 );				//make sure that there is not a pending stop condition before starting the next transaction.

	if( ( UCB0CTL1 & UCTR ) == 0 )							//Check to see if we are already in TX mode
	{
		UCB0CTL1 |= UCSWRST; 							//set the module into reset.
		UCB0CTL1 |= UCTR; 								//set the transmit bit.
		UCB0CTL1 &= ~UCSWRST;							//exit reset mode.
	}
	UCB0I2CSA = device_address;							//set the slave address.
	UCB0CTL1 |= UCTXSTT; 								//trasmit start condition.

	//while( ( IFG2 & UCB0TXIFG ) == 0x0 );				//wait for the start condition to be sent.

	UCB0TXBUF = data;									//begin transmitting byte.

	while( ( UCB0CTL1 & UCTXSTT ) != 0x0 );				//wait for slave to ack its address.

	while( ( IFG2 & UCB0TXIFG ) == 0x0 );				//wait for the tx shift register to load.

	UCB0CTL1 |= UCTXSTP; 								//set stop condition.

}

void WriteMultipleI2c( U8 device_address, U8* data, U8 data_length )
{
	while( ( UCB0CTL1 & UCTXSTP ) != 0 );				//make sure that there is not a pending stop condition before starting the next transaction.

	if( ( UCB0CTL1 & UCTR ) == 0 )							//Check to see if we are already in TX mode
	{
		UCB0CTL1 |= UCSWRST; 							//set the module into reset.
		UCB0CTL1 |= UCTR; 								//set the transmit bit.
		UCB0CTL1 &= ~UCSWRST;							//exit reset mode.
	}
	UCB0I2CSA = device_address;							//set the slave address.
	UCB0CTL1 |= UCTXSTT; 								//trasmit start condition.

	for(; data_length > 0; --data_length )
	{
		while( ( IFG2 & UCB0TXIFG ) == 0x0 );			//wait for the start condition to be sent, or for the tx shift register to load.

		IFG2 &= ~UCB0TXIFG;								//clear the interupt flag so it can be used for the next slave ack.
		UCB0TXBUF = *data;								//begin transmitting byte.
		++data;

		while( ( UCB0CTL1 & UCTXSTT ) != 0x0 );			//wait for slave to ack its address. this should only have to wait once.
	}

	while( ( IFG2 & UCB0TXIFG ) == 0x0 );				//wait for the tx shift register to load.

	UCB0CTL1 |= UCTXSTP; 								//set stop condition.
}

void InitSerial()
{
	//On Motor Controller:
	//TX: P1.2, UCA0TXD
	//RC: P1.1, UCA0RXD

	//on BMS master:
	//TX: P3.4, UCA0TXD
	//RX: P3.5, UCA0RXD


	//disable serial:
	UCA0CTL1 |= UCSWRST;

	//testing disable flags:
	//UCA0CTL0 = UCMSB; //MSB first, clear all other settings. (set normal mode)

	//enable primary periferal for RX and TX pins
#ifndef __MSP430F2272__
	P1SEL |= BIT1 | BIT2;
	P1SEL2 |= BIT1 | BIT2; //P2SEL2 doesn't exist on the BMS master!
#else
	P3SEL |= BIT4 | BIT5;
#endif

	//Motor controller runs at 16MHz, BMS master runs at 8MHz

	//Set clock soruce as SMCLK
	UCA0CTL1 |= UCSSEL_2;
	//baud rate generation (16MHz / 9600 = 1666.6 = 0x0682)
	UCA0BR0 = 0x82; //low byte
	UCA0BR1 = 0x06; //high byte

#ifdef	__MSP430F2272__
	//baud rate generation (8MHz / 9600 = 833.3333 = 0x0341)
	UCA0BR0 = 0x41; //low byte
	UCA0BR1 = 0x03; //high byte
#endif

	UCA0MCTL = UCBRS0;

	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;	// Enable USCI_A0 RX interrupt

}

void WriteByteSerial(U8 data)
{
	while (!(IFG2&UCA0TXIFG));
	UCA0TXBUF = data;
}

void WriteWordSerial(U16 word)
{
	WriteByteSerial( word >> 8 );
	WriteByteSerial( word );
}

void WriteBufferSerial( U8* data, U16 length )
{
	U16 i;

	for(i = 0; i < length; ++i)
	{
		WriteByteSerial(data[i]);
	}
}

#ifdef __MSP430F2272__
void InitSpi()
{
	//SDO/MISO - P3.2, UCB0SOMI
	//SDI/MOSI - p3.1, UCB0SIMO (we might not need to xmit!)
	//SCK - p3.3, UCB0CLK

	//ADCs set up outputs on falling edge, so we read on rising edge!
	//clock is idle high.
	//conversions are started when CS goes high, or when SCK is pulled low. (interesting)
	//I think we can leave SDI high for 1ksps or low for 250sps. (2-wire mode, but not sure since we're still using CS (3wire))
	//lets set CPOL and CPHA. (high inactive, trailing edge)

	P3SEL |= BIT3 | BIT2;
	P1DIR |= ADC_CS_1_PIN | ADC_CS_2_PIN;

	CS_1_ON; //set both CS pins high.
	CS_2_ON;

	//P3SEL |= BIT1;

	UCB0CTL0 |= UCCKPL | UCMSB | UCMST | UCSYNC; //UCCKPL=clock inactive high. UCMSB=MSB first. UCMST=master mode. UCSYNC=synchronous. (UCCKPH is 0, meaning trailing edge?)
	UCB0CTL1 |= UCSSEL_2; //UCSSEL_2=smclk

	//SMCLK is 8 MHz. we want to run at 1 MHZ.
	//LTC2472 can run SPI at a maximum of 2 MHz.
	//lets run it at 1 MHz.
	UCB0BR0 = 0x07; //low byte.
	UCB0BR1 = 0x00;

	UCB0CTL1 &= ~UCSWRST; //clear reset bit!



}
U16 ReadWordSpi( U8 cs_pin )
{
	U8 rx_byte;
	U16 value = 0;
	U16 counter;

	if( cs_pin == 1 )
		CS_1_OFF;
	else if( cs_pin == 2)
		CS_2_OFF;
	else
		return 0xFFF0;

	//just removed this because we need to operate fast now - on the timer ISR.
	/*for( counter = 0; counter < 1000; ++counter)
	{

	}*/

	UCB0TXBUF = 0x00; //write 00 to force start.

	while (!(IFG2 & UCB0RXIFG));

	rx_byte = UCB0RXBUF; //clears the interrupt automatically!

	value = rx_byte << 8;

	UCB0TXBUF = 0x00;

	while (!(IFG2 & UCB0RXIFG));

	rx_byte = UCB0RXBUF;

	value |= rx_byte;

	//we need to wait 12 ms for the reference to stabilize, in case that's the problem.

	//removed this now that we figured out the issue with the ADC and we need to sample fast.
	/*for( counter = 0; counter < 1000; ++counter)
	{

	}*/

	if( cs_pin == 1 )
		CS_1_ON;
	else if( cs_pin == 2)
		CS_2_ON;

	return value;
}
#endif
