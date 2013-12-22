/*
 * BmsSharedDefines.h
 *
 *  Created on: Dec 15, 2013
 *      Author: Mark
 */

#ifndef BMSSHAREDDEFINES_H_
#define BMSSHAREDDEFINES_H_


#define CELL_MIN_VOLTAGE		715		//2.8V to 3.7V
#define CELL_MAX_VOLTAGE		943

#define CELL_MIN_TEMPERATURE	212		//30.11 F
#define CELL_MAX_TEMPERATURE	804		//130.16 F

#define MAX_MASTER_CURRENT	65387			//+200A? (verify)

#define MAX_CHARGE_CURRENT	36029			//+5A? (verify)

#define MASTER_MIN_TEMPERATURE	193		//30.17 F
#define MASTER_MAX_TEMPERATURE	467		//80.08 F

#define MAX_PACK_VOLTAGE	787			//54V
#define MIN_PACK_VOLTAGE	612			//42V

#define BMS_MASTER_MAX_REFRESH_DELAY	35 //seconds
#define CELL_MAX_REFESH_DELAY	35
#define	BMS_PC_SERIAL_TIMEOUT	60


#endif /* BMSSHAREDDEFINES_H_ */
