
/*
 * BmsMasterDefines.h
 *
 *  Created on: Jun 29, 2013
 *      Author: Mark
 */

#ifndef BMSMASTERDEFINES_H_
#define BMSMASTERDEFINES_H_

#include <msp430.h>

#define CMD_GET_CELL_VOLTAGE  		0x10
#define CMD_GET_CELL_TEMP 			0x11
#define CMD_SET_DISCHARGE_PROPS  	0x12
#define CMD_PING_CELL				0x13
#define CMD_SET_RELAY 				0x14
#define CMD_GET_MASTER_TEMP  		0x15
#define CMD_GET_MASTER_HUMIDITY  	0x16
#define CMD_GET_CURRENT				0x17
#define CMD_GET_PACK_VOLTAGE		0x18
#define CMD_GET_RELAY				0x19

#define PARAM_ON					0x01
#define PARAM_OFF					0x02

#define YELLOW_BIT	BIT5
#define GREEN_BIT 	BIT6
#define RED_BIT 	BIT4

#define YELLOW_PORT P1OUT
#define GREEN_PORT 	P1OUT
#define RED_PORT 	P1OUT

#define YELLOW_ON	YELLOW_PORT &= ~YELLOW_BIT
#define GREEN_ON	GREEN_PORT &= ~GREEN_BIT
#define RED_ON		RED_PORT &= ~RED_BIT

#define YELLOW_OFF	YELLOW_PORT |= YELLOW_BIT
#define GREEN_OFF	GREEN_PORT |= GREEN_BIT
#define RED_OFF		RED_PORT |= RED_BIT

#define YELLOW_TOGGLE	YELLOW_PORT ^= YELLOW_BIT
#define GREEN_TOGGLE	GREEN_PORT ^= GREEN_BIT
#define RED_TOGGLE		RED_PORT ^= RED_BIT

#define PRE_CHARGE_1_BIT 	BIT7
#define PRE_CHARGE_2_BIT 	BIT3
#define PRE_CHARGE_3_BIT 	BIT1
#define PRE_CHARGE_4_BIT 	BIT6
#define PRE_CHARGE_5_BIT 	BIT5

#define PRE_CHARGE_1_PORT 	P4OUT
#define PRE_CHARGE_2_PORT 	P4OUT
#define PRE_CHARGE_3_PORT	P4OUT
#define PRE_CHARGE_4_PORT	P2OUT
#define PRE_CHARGE_5_PORT	P2OUT

#define PRE_CHARGE_1_ANALOG	INCH_15
#define PRE_CHARGE_2_ANALOG	INCH_13
#define PRE_CHARGE_3_ANALOG	INCH_5
#define PRE_CHARGE_4_ANALOG	INCH_2
#define PRE_CHARGE_5_ANALOG	INCH_1

#define RELAY_1				BIT6
#define RELAY_2				BIT5
#define RELAY_3				BIT2
#define RELAY_4				BIT0
#define RELAY_5				BIT7

#define RELAY_1_PORT		P3OUT
#define RELAY_2_PORT		P4OUT
#define RELAY_3_PORT		P4OUT
#define RELAY_4_PORT		P4OUT
#define RELAY_5_PORT		P2OUT



#define ADC_CS_1_PIN		BIT3
#define ADC_CS_1_PORT		P1OUT
#define ADC_CS_2_PIN		BIT2
#define ADC_CS_2_PORT		P1OUT

#define BATT_ANALOG			INCH_0
#define HUMIDITY_ANALOG		INCH_7
#define NTC_ANALOG			INCH_3

#define BATT_BIT			BIT0
#define HUMIDITY_BIT		BIT7
#define NTC_BIT				BIT3



#define RELAY1_ON			RELAY_1_PORT |= RELAY_1
#define RELAY2_ON			RELAY_2_PORT |= RELAY_2
#define RELAY3_ON			RELAY_3_PORT |= RELAY_3
#define RELAY4_ON			RELAY_4_PORT |= RELAY_4
#define RELAY5_ON			RELAY_5_PORT |= RELAY_5

#define RELAY1_OFF			RELAY_1_PORT &= ~RELAY_1
#define RELAY2_OFF			RELAY_2_PORT &= ~RELAY_2
#define RELAY3_OFF			RELAY_3_PORT &= ~RELAY_3
#define RELAY4_OFF			RELAY_4_PORT &= ~RELAY_4
#define RELAY5_OFF			RELAY_5_PORT &= ~RELAY_5

#define PRE_CHARGE_1_ON		PRE_CHARGE_1_PORT |= PRE_CHARGE_1_BIT
#define PRE_CHARGE_2_ON		PRE_CHARGE_2_PORT |= PRE_CHARGE_2_BIT
#define PRE_CHARGE_3_ON		PRE_CHARGE_3_PORT |= PRE_CHARGE_3_BIT
#define PRE_CHARGE_4_ON		PRE_CHARGE_4_PORT |= PRE_CHARGE_4_BIT
#define PRE_CHARGE_5_ON		PRE_CHARGE_5_PORT |= PRE_CHARGE_5_BIT

#define PRE_CHARGE_1_OFF	PRE_CHARGE_1_PORT &= ~PRE_CHARGE_1_BIT
#define PRE_CHARGE_2_OFF	PRE_CHARGE_2_PORT &= ~PRE_CHARGE_2_BIT
#define PRE_CHARGE_3_OFF	PRE_CHARGE_3_PORT &= ~PRE_CHARGE_3_BIT
#define PRE_CHARGE_4_OFF	PRE_CHARGE_4_PORT &= ~PRE_CHARGE_4_BIT
#define PRE_CHARGE_5_OFF	PRE_CHARGE_5_PORT &= ~PRE_CHARGE_5_BIT

#define CS_1_ON				ADC_CS_1_PORT |= ADC_CS_1_PIN
#define CS_2_ON				ADC_CS_2_PORT |= ADC_CS_2_PIN

#define CS_1_OFF			ADC_CS_1_PORT &= ~ADC_CS_1_PIN
#define CS_2_OFF			ADC_CS_2_PORT &= ~ADC_CS_2_PIN

//used only for ADC input enable, not for conversion or anything else.
#define PRE_CHARGE_1_ANALOG_BIT	BITF
#define PRE_CHARGE_2_ANALOG_BIT	BITD
#define PRE_CHARGE_3_ANALOG_BIT	BIT5
#define PRE_CHARGE_4_ANALOG_BIT	BIT2
#define PRE_CHARGE_5_ANALOG_BIT	BIT1
//these are used to set the ADC10AE0/1 registers and are therefore bits, not INCH values.  INCH values are for setting the current input to sample from.
#define ALL_ANALOG_INPUTS	(PRE_CHARGE_1_ANALOG_BIT | PRE_CHARGE_2_ANALOG_BIT | PRE_CHARGE_3_ANALOG_BIT | PRE_CHARGE_4_ANALOG_BIT | PRE_CHARGE_5_ANALOG_BIT | BATT_BIT | HUMIDITY_BIT | NTC_BIT )

#endif /* BMSMASTERDEFINES_H_ */
