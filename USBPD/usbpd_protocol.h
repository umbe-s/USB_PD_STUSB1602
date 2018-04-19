/*
 * usbpd_protocol.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Umberto Stefanini, Design Concepts, Inc.
 */

#ifndef USBPD_PROTOCOL_H_
#define USBPD_PROTOCOL_H_

#include "usbpd_phy.h"
#include "STUSB1602_Driver_Conf.h"
#include "STUSB1602_Driver.h"

#define UNINIT_MSGID 8
#define UNINIT_SPEC_REV 0xFF
#define PORT_POWER_ROLE 0 //Sink
#define PORT_DATA_ROLE 0 //UFP
#define DEFAULT_SPEC_REV 0b01

#define REQUEST_CURRENT_MA 3000
//#define CURRENT_OVERRIDE_MA 5000
#define REQUEST_VOLTAGE_MV 12000

#define DEFAULT_MESSAGE_TIMEOUT 10
#define SOURCE_CAP_TIMEOUT 300
#define RX_GOODCRC_TIMEOUT 10
#define PS_RDY_TIMEOUT 300

#define UINT32_TO_BYTE_ARR(d32,arr,start) ({ \
						arr[(start)]   = (uint8_t)((d32) & 0xFF);\
						arr[(start)+1] = (uint8_t)(((d32) >>  8) & 0xFF);\
						arr[(start)+2] = (uint8_t)(((d32) >> 16) & 0xFF);\
						arr[(start)+3] = (uint8_t)(((d32) >> 24) & 0xFF);\
						})

#define CNTRLMSG_TO_STRING(C) (C == CNTRLMSGTYPE_GOODCRC ? "GOODCRC" : \
															C == CNTRLMSGTYPE_GOTOMIN ?  "GOTOMIN": \
															C == CNTRLMSGTYPE_ACCEPT ? "ACCEPT": \
															C == CNTRLMSGTYPE_REJECT ? "REJECT" : \
															C == CNTRLMSGTYPE_PING? "PING" : \
															C == CNTRLMSGTYPE_PS_RDY? "PS_RDY" : \
															C == CNTRLMSGTYPE_GET_SOURCE_CAP? "GET_SOURCE_CAP" : \
															C == CNTRLMSGTYPE_GET_SINK_CAP? "GET_SINK_CAP" : \
															C == CNTRLMSGTYPE_DR_SWAP? "DR_SWAP" : \
															C == CNTRLMSGTYPE_PR_SWAP? "PR_SWAP" : \
															C == CNTRLMSGTYPE_VCONN_SWAP? "VCONN_SWAP" : \
															C == CNTRLMSGTYPE_WAIT? "WAIT" : \
															C == CNTRLMSGTYPE_SOFT_RESET? "SOFT_RESET" : \
															C == CNTRLMSGTYPE_NOT_SUPPORTED? "NOT_SUPPORTED" : \
															C == CNTRLMSGTYPE_GET_SOURCE_CAP_EXT? "GET_SOURCE_CAP_EXT" : \
															C == CNTRLMSGTYPE_GET_STATUS? "GET_STATUS" : \
															C == CNTRLMSGTYPE_FR_SWAP? "FR_SWAP" : \
															C == CNTRLMSGTYPE_GET_PPS_STATUS? "GET_PPS_STATUS" : \
															C == CNTRLMSGTYPE_GET_COUNTRY_CODES? "GET_COUNTRY_CODES" : "RESERVED" )


#define DATAMSG_TO_STRING(D) (D == DATAMSGTYPE_SOURCE_CAPABILITIES ? "SOURCE_CAPABILITIES" : \
															D == DATAMSGTYPE_REQUEST ?  "REQUEST": \
															D == DATAMSGTYPE_BIST ?  "BIST": \
															D == DATAMSGTYPE_SINK_CAPABILITIES ?  "SINK_CAPABILITIES": \
															D == DATAMSGTYPE_BATTERY_STATUS ?  "BATTERY_STATUS": \
															D == DATAMSGTYPE_ALERT ?  "ALERT": \
															D == DATAMSGTYPE_GET_COUNTRY_INFO ?  "GET_COUNTRY_INFO": \
															D == 	DATAMSGTYPE_VENDOR_DEFINED ?  "VENDOR_DEFINED": "RESERVED")


typedef union {
	uint32_t d32;
	struct {
		uint32_t MaxCurrentIn10mAUnits :10;
		uint32_t VoltageIn50mVUnits :10;
		uint32_t PeakCurrent :2;
		uint32_t Reserved :2;
		uint32_t UnchunkedExtendedMessagesSupported :1;
		uint32_t DualRoleData :1;
		uint32_t USBCommunicationsCapable :1;
		uint32_t UnconstrainedPower :1;
		uint32_t USBSuspendSupported :1;
		uint32_t DualRolePower :1;
		uint32_t FixedSupply :2;
	} b;
} FixedSupplyPDOFromSource_TypeDef;

typedef union {
	uint32_t d32;
	struct {
		uint32_t MaxOperatingCurrent10mAunits :10;
		uint32_t OperatingCurrentIn10mAunits :10;
		uint32_t Reserved20_22 :3;
		uint32_t UnchunkedExtendedMessagesSupported :1;
		uint32_t NoUSBSuspend :1;
		uint32_t USBCommunicationsCapable :1;
		uint32_t CapabilityMismatch :1;
		uint32_t GiveBackFlag :1;
		uint32_t ObjectPosition :3;
		uint32_t Reserved31 :1;
	} b;
} USBPD_SNKFixedVariableRDO_TypeDef;

#define MSGTYPE_ANY 0xFF
typedef enum {
	DATAMSGTYPE_RESERVED = 0b00000,
	DATAMSGTYPE_SOURCE_CAPABILITIES = 0b00001,
	DATAMSGTYPE_REQUEST = 0b00010,
	DATAMSGTYPE_BIST = 0b00011,
	DATAMSGTYPE_SINK_CAPABILITIES = 0b00100,
	DATAMSGTYPE_BATTERY_STATUS = 0b00101,
	DATAMSGTYPE_ALERT = 0b00110,
	DATAMSGTYPE_GET_COUNTRY_INFO = 0b00111,
	DATAMSGTYPE_VENDOR_DEFINED = 0b01111
} DataMsgType_TypeDef;

typedef enum {
	CNTRLMSGTYPE_RESERVED = 0b00000,
	CNTRLMSGTYPE_GOODCRC = 0b00001,
	CNTRLMSGTYPE_GOTOMIN = 0b00010,
	CNTRLMSGTYPE_ACCEPT = 0b00011,
	CNTRLMSGTYPE_REJECT = 0b00100,
	CNTRLMSGTYPE_PING = 0b00101,
	CNTRLMSGTYPE_PS_RDY = 0b00110,
	CNTRLMSGTYPE_GET_SOURCE_CAP = 0b00111,
	CNTRLMSGTYPE_GET_SINK_CAP = 0b01000,
	CNTRLMSGTYPE_DR_SWAP = 0b01001,
	CNTRLMSGTYPE_PR_SWAP = 0b01010,
	CNTRLMSGTYPE_VCONN_SWAP = 0b01011,
	CNTRLMSGTYPE_WAIT = 0b01100,
	CNTRLMSGTYPE_SOFT_RESET = 0b01101,
	CNTRLMSGTYPE_NOT_SUPPORTED = 0b10000,
	CNTRLMSGTYPE_GET_SOURCE_CAP_EXT = 0b10001,
	CNTRLMSGTYPE_GET_STATUS = 0b10010,
	CNTRLMSGTYPE_FR_SWAP = 0b10011,
	CNTRLMSGTYPE_GET_PPS_STATUS = 0b10100,
	CNTRLMSGTYPE_GET_COUNTRY_CODES = 0b10101,
} ControlMsgType_TypeDef;

void getUSBPDContract();
void printAllMessages();


#endif /* USBPD_PROTOCOL_H_ */
