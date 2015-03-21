/*
 * File:   ecocar.h
 *
 * Global Header File for EcoCar Project
 * Includes global address definitions for sensor nodes, data types, etc.
 * Last edited: Feb. 22, 2013
 */

#include "../Common/J1939.H"

#ifndef ECOCAR_H
#define	ECOCAR_H

#ifndef DBG_LED0
#define DBG_LED0 LATBbits.LATB0
#endif

#ifndef DBG_LED1
#define DBG_LED1 LATBbits.LATB1
#endif

#ifndef DBG_LED2
#define DBG_LED2 LATBbits.LATB4
#endif

#ifndef DBG_LED3
#define DBG_LED3 LATBbits.LATB5
#endif

#ifndef ERR_LED
#define ERR_LED  LATCbits.LATC5
#endif

// Convenience defines for doing I/O
#define PORT_INPUT_HIGH 1
#define PORT_INPUT_LOW  0

#define PORT_OUTPUT_HIGH 1
#define PORT_OUTPUT_LOW  0

#define PORT_DIRECTION_INPUT  1
#define PORT_DIRECTION_OUTPUT 0

// Interrupt defines
#define INTERRUPT_FLAG_SET 1
#define INTERRUPT_FLAG_CLR 0

#define TIMER_ENABLED  1
#define TIMER_DISABLED 0

// General purpose register settings
#define BIT_SET 1
#define BIT_CLR 0

#define ENABLED  1
#define DISABLED 0

// Function prototype declarations for ecocar.c:
void Broadcast_Data(J1939_MESSAGE *MsgPtr, unsigned char DataType, unsigned char MsgData[]);
void Request_Data(J1939_MESSAGE *MsgPtr, unsigned int DestAddr, unsigned int DataType);
void InitEcoCar(void);
void Set_Oscillator(void);
void putUSART(int i);
void putSerialData(char DataType, char DataMSB, char DataLSB);
void SetDebugStatus(unsigned int status);
void SetErrorState(unsigned char err);
void ShowBootupAnimation();

// =====================================
// Sensor data types (value of Msg.GroupExtension)
// When broadcasting these data types, use Msg.PDUFormat = PDU_BROADCAST
// =====================================
#define DATA_FC_POWER			0xA0
#define DATA_FC_TEMP			0xA1
#define DATA_FC_AMBTEMP			0xA2
#define DATA_FC_CURR			0xA3
#define DATA_FC_VSTACK			0xA4
#define DATA_FC_VBATT			0xA5
#define DATA_FC_PTANK			0xA6
#define DATA_FC_STATUS			0xA7

#define DATA_PWR_VBUCK			0xB0
#define DATA_PWR_MOTORON		0xB1
#define DATA_PWR_CURRMOT1		0xB2
#define DATA_PWR_CURRMOT2		0xB3
#define DATA_PWR_SPEED			0xB4
#define DATA_PWR_DIR			0xB5
#define DATA_PWR_CRUISEON		0xB6

#define DATA_TEMPTRUNK			0xC0
#define DATA_TEMPCABIN			0xC1
#define DATA_TEMPOUTSIDE		0xC2
#define DATA_VACCESSORY			0xC3
#define DATA_BKPALARM   		0xC4

#define CTRL_CRUISE_SET                 0xD0
#define CTRL_CRUISE_INC                 0xD1
#define CTRL_CRUISE_DEC                 0xD2
#define CTRL_CRUISE_CLR                 0xD3
#define DATA_MOTOR_PERC                 0xD4

// Notifications:
#define CYCLE_COMPLETE			0xE0
#define ACK_DONE                        0xE1

// System errors:
#define ERR_TIMEOUT			0xF0

// =====================================
// Message types (value of Msg.PDU_FORMAT)
// =====================================
#define PDU_REQUEST			0x01	// Request for data
#define PDU_CONTROL                     0x02    // Control messages
#define PDU_BROADCAST			0xFF	// Broadcast

// =========================
// Node address definitions:
// Used during initiailization to prevent address contention
// Broadcasts should be sent to Msg.DestinationAddress = NODE_BROADCAST)
// =========================
#define NODE_MASTER			1	// Master node

#define NODE_SLAVE_FC		2	// Slave node for Fuel Cell Data
#define NODE_SLAVE_MOTORS	3	// Slave node for Motors Sensors
#define NODE_SLAVE_MISC 	4	// Slave node for misc. measurements

#define NODE_PL_DRIVER		11	// Passive-listener for Driver
#define NODE_PL_WIRELESS	12 	// Passive-listener for Wireless

#define NODE_BROADCAST		255	// Global address designated for broadcasts

#define true  1
#define false 0
typedef unsigned char bool;

#define forever while (true)

#endif	/* ECOCAR_H */

