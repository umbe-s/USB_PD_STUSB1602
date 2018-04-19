/*
 * usbpd_protocol.c
 *
 *  Created on: Mar 17, 2018
 *      Author: Umberto Stefanini, Design Concepts, Inc.
 */

#include "usbpd_protocol.h"

// Variables
static FixedSupplyPDOFromSource_TypeDef pdoArray[7];
static uint8_t specificationRevision = UNINIT_SPEC_REV;
static uint8_t messageID = UNINIT_MSGID;
static bool gotPDOs = false;
static bool gotGOODCRC = false;
static ControlMsgType_TypeDef rxCntrlMsg = CNTRLMSGTYPE_RESERVED;

// Private Function Prototypes
bool sendDataMessage(DataMsgType_TypeDef msgType, uint32_t *dataObjArr, uint8_t numDataObj);
bool sendControlMessage(ControlMsgType_TypeDef msgType);
void sendGoodCRC(uint8_t msgID);
bool sendRequestDataObject(uint16_t voltage_mV, uint16_t current_mA);
void setSpecRev(uint8_t rev);
bool checkReceivedMessageID(uint8_t rxMsgID);
void goodCrcReceivedCallback(Message_TypeDef *msg);
bool waitForGoodCRC();
void messageReceivedCallback(Message_TypeDef *msg);
void printMessageCallback(Message_TypeDef *msg);

// Functions

// Send a data message of [parameter] message type, with Data Objects appended from the [parameter] array; return success
bool sendDataMessage(DataMsgType_TypeDef msgType, uint32_t *dataObjArr, uint8_t numDataObj){
	Message_TypeDef msg;

	if (messageID == UNINIT_MSGID)
		messageID = 0;

	msg.header.b.Extended = 0;
	msg.header.b.NumberOfDataObjects = numDataObj;
	msg.header.b.MessageID = messageID;
	msg.header.b.PortPowerRole = PORT_POWER_ROLE;
	msg.header.b.SpecificationRevision = specificationRevision;
	msg.header.b.PortDataRole = PORT_DATA_ROLE;
	msg.header.b.MessageType = msgType;
	for (int i = 0; i < numDataObj; ++i){
		UINT32_TO_BYTE_ARR(dataObjArr[i], msg.data, i * 4);
	}

	DEBUG_PRINT("Sending Data Message [ID:%d] - %s", msg.header.b.MessageID,
			DATAMSG_TO_STRING(msg.header.b.MessageType));

	if (sendMessage(msg) == false){
		DEBUG_PRINT("Data message send failed");
		return false;
	} else
		return true;
}

// Send a control message of [parameter] message type; return success
bool sendControlMessage(ControlMsgType_TypeDef msgType){
	Message_TypeDef msg;

	if (messageID == UNINIT_MSGID)
		messageID = 0;

	msg.header.b.Extended = 0;
	msg.header.b.NumberOfDataObjects = 0;
	msg.header.b.MessageID = messageID;
	msg.header.b.PortPowerRole = PORT_POWER_ROLE; //SINK
	msg.header.b.SpecificationRevision = specificationRevision;
	msg.header.b.PortDataRole = PORT_DATA_ROLE; //UFP
	msg.header.b.MessageType = msgType;

	DEBUG_PRINT("Sending Control Message [ID:%d] - %s", msg.header.b.MessageID,
			CNTRLMSG_TO_STRING(msg.header.b.MessageType));

	if (sendMessage(msg) == false){
		DEBUG_PRINT("Control message send failed");
		return false;
	} else
		return true;
}

//Send a GoodCRC message
void sendGoodCRC(uint8_t msgID){

	Message_TypeDef msg;
	msg.header.b.Extended = 0;
	msg.header.b.NumberOfDataObjects = 0;
	msg.header.b.MessageID = msgID;
	msg.header.b.PortPowerRole = PORT_POWER_ROLE;
	msg.header.b.SpecificationRevision = specificationRevision;
	msg.header.b.PortDataRole = PORT_DATA_ROLE;
	msg.header.b.MessageType = CNTRLMSGTYPE_GOODCRC;

	if (sendMessage(msg) == false)
		DEBUG_PRINT("GoodCRC send failed");
	else
		DEBUG_PRINT("Sent Control Message [ID:%d] - GOODCRC", msg.header.b.MessageID);
}

// Send a Request Data Object for the PDO matching the passed voltage and current requirements
bool sendRequestDataObject(uint16_t voltage_mV, uint16_t current_mA){
	int ind;
	USBPD_SNKFixedVariableRDO_TypeDef rdo;

	if (!gotPDOs){
		DEBUG_PRINT("Cannot send RDO before getting PDOs");
		return false;
	}

	DEBUG_PRINT("Looking for %dmV %dmA", voltage_mV, current_mA);
	for (ind = 0; ind < 7; ++ind){
		if ((pdoArray[ind].b.VoltageIn50mVUnits == voltage_mV / 50)
				&& (pdoArray[ind].b.MaxCurrentIn10mAUnits >= current_mA / 10)){
			break;
		}
	}
	if (ind < 7){
		DEBUG_PRINT("Requesting PDO Position %d", ind + 1);
	} else{
		DEBUG_PRINT("Requested PDO not found");
		return false;
	}

	//Populate RDO fields
	rdo.b.ObjectPosition = ind + 1; //0 is reserved
	rdo.b.GiveBackFlag = 0;
	rdo.b.CapabilityMismatch = 0;
	rdo.b.USBCommunicationsCapable = 0;
	rdo.b.NoUSBSuspend = 1;
	rdo.b.UnchunkedExtendedMessagesSupported = 0;
#ifdef CURRENT_OVERRIDE_MA
	rdo.b.OperatingCurrentIn10mAunits = CURRENT_OVERRIDE_MA / 10;
#else
	rdo.b.OperatingCurrentIn10mAunits = current_mA / 10;
#endif
	rdo.b.MaxOperatingCurrent10mAunits = current_mA / 10;

	//Send Data Message
	if (!sendDataMessage(DATAMSGTYPE_REQUEST, &(rdo.d32), 1))
		return false;

	return waitForGoodCRC();
}

// If uninitialized, set the specification revision
void setSpecRev(uint8_t rev){
	if (specificationRevision == UNINIT_SPEC_REV){
//		DEBUG_PRINT("Specification Revision set to %c%c", (rev & 0b10 ? '1' : '0'),
//				(rev & 0b01 ? '1' : '0'));
		specificationRevision = rev;
	}
}

// If uninitialized, set the message ID (for transmission).
// Return true if the message ID received is not the same as the last one.
bool checkReceivedMessageID(uint8_t rxMsgID){
	static uint8_t lastMsgID = UNINIT_MSGID;
	if (messageID == UNINIT_MSGID){
		messageID = rxMsgID;
		DEBUG_PRINT("MessageID initialized with received value: %d", messageID);
	}

	if (lastMsgID != rxMsgID){
		lastMsgID = rxMsgID;
		DEBUG_PRINT("New MessageID received: %d", rxMsgID);
		return true;
	} else{
		DEBUG_PRINT("Old MessageID received: %d", rxMsgID);
		return false;
	}
}

// Callback to be executed when a GoodCRC message is received
void goodCrcReceivedCallback(Message_TypeDef *msg){
	if ((msg->header.b.NumberOfDataObjects == 0)
			&& (msg->header.b.MessageType == CNTRLMSGTYPE_GOODCRC)
			&& (msg->header.b.MessageID == messageID)){
		// This is a control message of type GOODCRC with MessageID set to what was last sent
		gotGOODCRC = true;
		++messageID;
		messageID %= 8;
		DEBUG_PRINT("Rx GoodCRC - MessageID Incremented");
	} else{
		//Not a valid Good CRC message
		DEBUG_PRINT("Rx %s Message[ID:%d] - %s",
				(msg->header.b.NumberOfDataObjects == 0) ? "Control" : "Data", msg->header.b.MessageID,
				(msg->header.b.NumberOfDataObjects == 0) ? CNTRLMSG_TO_STRING(msg->header.b.MessageType) : DATAMSG_TO_STRING(msg->header.b.MessageType));
	}
}

// Waits for next message to be received; returns true if it is a GoodCRC for the last message sent
bool waitForGoodCRC(){
	gotGOODCRC = false;

	//Only one GoodCRC will be sent
	if (!waitForMessage(&goodCrcReceivedCallback, RX_GOODCRC_TIMEOUT))
		DEBUG_PRINT("Good CRC timeout");

	return gotGOODCRC;
}

// Callback to be executed when a message of type other than GoodCRC is received
void messageReceivedCallback(Message_TypeDef *msg){
	uint32_t word32;

	//Send a GoodCRC for this message
	setSpecRev(msg->header.b.SpecificationRevision);
	sendGoodCRC(msg->header.b.MessageID);

	DEBUG_PRINT("Rx %s Message[ID:%d] - %s",
			(msg->header.b.NumberOfDataObjects == 0) ? "Control" : "Data", msg->header.b.MessageID,
			(msg->header.b.NumberOfDataObjects == 0) ? CNTRLMSG_TO_STRING(msg->header.b.MessageType) : DATAMSG_TO_STRING(msg->header.b.MessageType));

	// Check validity of received message ID
	if (!checkReceivedMessageID(msg->header.b.MessageID))
		return;

	//Parse Message
	if (msg->header.b.NumberOfDataObjects == 0){
		//Control Message

		rxCntrlMsg = msg->header.b.MessageType;

	} else{
		//Data Message
		if (msg->header.b.MessageType == DATAMSGTYPE_SOURCE_CAPABILITIES){
			//Parse PDOs
			for (int i = 0; i < msg->header.b.NumberOfDataObjects; ++i){
				word32 = ASSEMBLE_32B_FROM_BYTES(msg->data, i * 4);
				//Parse Fixed PDOs
				if (word32 >> 30 == 0){
					gotPDOs = true;

					pdoArray[i].d32 = word32;
					DEBUG_PRINT("PDO %d: %d V : %d A", i + 1, (pdoArray[i].b.VoltageIn50mVUnits * 50) / 1000,
							(pdoArray[i].b.MaxCurrentIn10mAUnits * 10) / 1000);
				}
			}
		}
	}
}

//Go through contract negotiation
void getUSBPDContract(){
	DEBUG_PRINT(" ----- Start USBPD Negotiation ----- ");

	DEBUG_PRINT("Set Power Mode to Sink");
	if (STUSB1602_Power_Mode_Set(STUSB1602_I2C_ADDR, SNK_without_accessory_supp) != STUSB1602_OK){
		DEBUG_PRINT("STUSB1602 Communication Failure");
	}

	DEBUG_PRINT("Waiting for PDOs");
	for (int i = 0; i < 10; ++i){
		if (!waitForMessage(&messageReceivedCallback, SOURCE_CAP_TIMEOUT)){
			DEBUG_PRINT("Source Capabilities timeout - %dms", SOURCE_CAP_TIMEOUT);

			setSpecRev(DEFAULT_SPEC_REV);
			if (sendControlMessage(CNTRLMSGTYPE_GET_SOURCE_CAP)){
				if (!waitForGoodCRC())
					DEBUG_PRINT("Good CRC not received");
			}
		} else if (gotPDOs){
			break;
		} else if (i == 9)
			errorUSBPD("Did not receive Source Capabilities");
	}

	if (!sendRequestDataObject(REQUEST_VOLTAGE_MV, REQUEST_CURRENT_MA))
		errorUSBPD("12V PDO not found");

	DEBUG_PRINT("Waiting for ACCEPT");
	for (int i = 0; i < 10; ++i){
		rxCntrlMsg = CNTRLMSGTYPE_RESERVED;
		if (!waitForMessage(&messageReceivedCallback, DEFAULT_MESSAGE_TIMEOUT)){
			DEBUG_PRINT("ACCEPT timeout - %dms", DEFAULT_MESSAGE_TIMEOUT);
		} else{
			if (rxCntrlMsg == CNTRLMSGTYPE_ACCEPT)
				break;
			if (rxCntrlMsg == CNTRLMSGTYPE_REJECT)
				errorUSBPD("12V request Rejected by source");

			DEBUG_PRINT("Rx Control Message: %s", CNTRLMSG_TO_STRING(rxCntrlMsg));
		}
		if (i == 9)
			errorUSBPD("Did not receive ACCEPT from Source");
	}

	STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_ADDR, 12000);

	DEBUG_PRINT("Waiting for PS_RDY");
	for (int i = 0; i < 10; ++i){
		rxCntrlMsg = CNTRLMSGTYPE_RESERVED;
		if (!waitForMessage(&messageReceivedCallback, PS_RDY_TIMEOUT)){
			DEBUG_PRINT("PS_RDY timeout - %dms", PS_RDY_TIMEOUT);
		} else{
			if (rxCntrlMsg == CNTRLMSGTYPE_PS_RDY)
				break;
		}
		if (i == 9)
			errorUSBPD("Did not receive PS_RDY from Source");
	}

	DEBUG_PRINT(" ----- Negotiation complete ----- ");
}

// Callback to be executed when all messages are to be printed
void printMessageCallback(Message_TypeDef *msg){
	sendGoodCRC(msg->header.b.MessageID);
	checkReceivedMessageID(msg->header.b.MessageID);
	DEBUG_PRINT("Rx %s Message[ID:%d] - %s",
			(msg->header.b.NumberOfDataObjects == 0) ? "Control" : "Data", msg->header.b.MessageID,
			(msg->header.b.NumberOfDataObjects == 0) ? CNTRLMSG_TO_STRING(msg->header.b.MessageType) : DATAMSG_TO_STRING(msg->header.b.MessageType));
}

//Prints all messages received in the next second; for debugging
void printAllMessages(){
	uint32_t tick = HAL_GetTick();
	while (HAL_GetTick() - tick < 1000){
		waitForMessage(&printMessageCallback, DEFAULT_MESSAGE_TIMEOUT);
	}
}
