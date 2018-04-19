/*
 * usbpd_phy.c
 *
 *  Created on: Mar 16, 2018
 *      Author: Umberto Stefanini, Design Concepts, Inc.
 */

#include "usbpd_phy.h"

//Variables
I2C_HandleTypeDef hi2c_STUSB1602;
SPI_HandleTypeDef hspi_STUSB1602;

const uint8_t nobbleLookUpTable[] = { //bit reversed because of SPI_FIRSTBIT_MSB
				0b01111, //11110 - nibble data 0
				0b10010, //01001 - nibble data 1
				0b00101, //10100 - nibble data 2
				0b10101, //10101 - nibble data 3
				0b01010, //01010 - nibble data 4
				0b11010, //01011 - nibble data 5
				0b01110, //01110 - nibble data 6
				0b11110, //01111 - nibble data 7
				0b01001, //10010 - nibble data 8
				0b11001, //10011 - nibble data 9
				0b01101, //10110 - nibble data A
				0b11101, //10111 - nibble data B
				0b01011, //11010 - nibble data C
				0b11011, //11011 - nibble data D
				0b00111, //11100 - nibble data E
				0b10111  //11101 - nibble data F
		};

static uint8_t receiveBuffer[RECEIVE_BUFFER_LENGTH];
static BitScanner rxBitScanner = { .byte_p = &receiveBuffer[0], .bitOffset = 0 };

volatile bool flag_USBPD_SPI_Active = false;

//Private Function Prototypes
static int crcBits(int data, int dataBitLen, int start_crc);
static int crcWrap(int c);
static bool checkCRC(uint8_t *data, int dataLength, int rxCrc);

static int decodeMessage(uint8_t *outputBuffer, int buffSize);
static bool scanRxToAfterEndPreamble();
static DecodedData decodeByte();
static SOP_Sequence parseSOP(DecodedData* s);
static bool receiveNextMessage();
static HAL_StatusTypeDef MY_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size,
		volatile bool *rxFlag, int timeout);
static uint8_t getNextByte(BitScanner b);
static void setNextNobble(BitScanner *b, uint8_t nobble);
static void scanForwardBits(BitScanner* b, uint8_t bitsForward);
static void resetRxBitScanner();
static void clearReceiveBuffer();
static void printReceiveBuffer();

extern void errorMode(char *msg, bool saveToFlash);

// ---------- CRC Calculation Functions ----------
//Modified from USB-PD specification example code; returns crc of input data given a crc to start with
int crcBits(int data, int dataBitLen, int start_crc){
	const uint32_t poly = 0x04C11DB6; //spec 04C1 1DB7h ???
	int crc, newbit, newword, rl_crc;
	crc = start_crc;
	for (int i = 0; i < dataBitLen; i++){
		newbit = ((crc >> 31) ^ ((data >> i) & 1)) & 1;
		if (newbit)
			newword = poly;
		else
			newword = 0;
		rl_crc = (crc << 1) | newbit;
		crc = rl_crc ^ newword;
	}
	return crc;
}

//Modified from USB-PD specification example code; returns bit reversed input
int crcWrap(int c){
	int ret = 0;
	int j, bit;
	c = ~c;
	for (int i = 0; i < 32; i++){
		j = 31 - i;
		bit = (c >> i) & 1;
		ret |= bit << j;
	}
	return ret;
}

//Calculates CRC for byte array (data) of length (dataLength), compares it to the received crc, and checks the residue
bool checkCRC(uint8_t *data, int dataLength, int rxCrc){
	int calc_crc = 0xFFFFFFFF;
	int chkCrc, residue;

	//Calculate the crc over each byte in the input array
	for (int i = 0; i < dataLength; ++i){
		calc_crc = crcBits(data[i], 8, calc_crc);
	}
	//The bit-reversed version of the crc is used for checking
	chkCrc = crcWrap(calc_crc);

	if (chkCrc == rxCrc){
		//calculation of the residue ensures that the received crc is a valid one
		residue = crcBits(rxCrc, 32, calc_crc);
		if (residue == 0xC704DD7B)
			return true;
	}
	return false;
}
// ---------- End CRC Calculation Functions----------

// Error handler for USBPD
void errorUSBPD(char* str){
	errorMode(str, true);
}

// Initializes necessary hardware for STUSB1602 and verifies I2C connection
void initSTUSB1602_HW(){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

	// ----- Initialize GPIO -----
  HAL_GPIO_WritePin(STUSB1602_RESET_GPIO_Port, STUSB1602_RESET_Pin, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = STUSB1602_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STUSB1602_ALERT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STUSB1602_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STUSB1602_RESET_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STUSB1602_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STUSB1602_TX_EN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STUSB1602_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STUSB1602_NSS_GPIO_Port, &GPIO_InitStruct);

	//Enable interrupt on SPI NSS signal
	HAL_NVIC_SetPriority(STUSB1602_NSS_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(STUSB1602_NSS_EXTI_IRQn);


	// ----- Initialize I2C -----
	hi2c_STUSB1602.Instance = STUSB1602_I2C;
	hi2c_STUSB1602.Init.ClockSpeed = 400000;
	hi2c_STUSB1602.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c_STUSB1602.Init.OwnAddress1 = 0;
	hi2c_STUSB1602.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c_STUSB1602.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c_STUSB1602.Init.OwnAddress2 = 0;
	hi2c_STUSB1602.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c_STUSB1602.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c_STUSB1602) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


	// ----- Initialize SPI -----
  hspi_STUSB1602.Instance = STUSB1602_SPI;
  hspi_STUSB1602.Init.Mode = SPI_MODE_SLAVE;
  hspi_STUSB1602.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_STUSB1602.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_STUSB1602.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_STUSB1602.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_STUSB1602.Init.NSS = SPI_NSS_SOFT;
  hspi_STUSB1602.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_STUSB1602.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_STUSB1602.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_STUSB1602.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi_STUSB1602) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	STUSB1602_Driver_Init(hi2c_STUSB1602);

	DEBUG_PRINT("Check I2C to STUSB1602");
	for (int i = 0; i < 10; ++i) {
		if(HAL_I2C_IsDeviceReady(&hi2c_STUSB1602, (STUSB1602_I2C_ADDR << 1), 10, DEFAULT_TIMEOUT) == HAL_OK)
			break;
		if(i==9)
			errorUSBPD("No I2C Connection to STUSB1602");
	}
	DEBUG_PRINT("STUSB1602 Hardware Initialization Complete");
}

__weak void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle){

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==STUSB1602_I2C)
  {

    /**I2C GPIO Configuration
    */
    GPIO_InitStruct.Pin = STUSB1602_SCL_Pin|STUSB1602_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = STUSB1602_I2C_GPIO_AF;
    HAL_GPIO_Init(STUSB1602_I2C_GPIO_Port, &GPIO_InitStruct);

    /* I2C clock enable */
    STUSB1602_I2C_CLK_ENABLE();
  }
}

__weak void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle){

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==STUSB1602_SPI)
  {
    /* SPI clock enable */
  	STUSB1602_SPI_CLK_ENABLE();

    /**SPI GPIO Configuration
    */
    GPIO_InitStruct.Pin = STUSB1602_SCLK_Pin|STUSB1602_MISO_Pin|STUSB1602_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = STUSB1602_SPI_GPIO_AF;
    HAL_GPIO_Init(STUSB1602_SPI_GPIO_Port, &GPIO_InitStruct);
  }
}

// TBD
void hardReset(){
	DEBUG_PRINT("Hard Reset not yet implemented");
}

//Formats, encodes, and sends parameter message; returns success
bool sendMessage(Message_TypeDef msg){
	uint32_t tick = 0;
	uint32_t crc = 0xFFFFFFFF;

	//Buffer for nibbles that will be encoded into nobbles
	uint8_t txData[MAX_DATA_LENGTH];
	uint16_t txDataLength = 2 + msg.header.b.NumberOfDataObjects * 4;

	//Buffer for nobbles that will be transmitted via SPI -> STUSB1602 -> BMC -> CC lines
	static uint8_t transmitBuffer[TRANSMIT_BUFFER_LENGTH];
	BitScanner txBitScanner = { .byte_p = &transmitBuffer[0], .bitOffset = 0 };
	K_Code sopCodes[4] = { K_SYNC_1, K_SYNC_1, K_SYNC_1, K_SYNC_2 };
	uint8_t hex[2];
	uint16_t bufferedLength;

	memset(transmitBuffer, 0, TRANSMIT_BUFFER_LENGTH);

	//Plan to transmit header first
	txData[0] = (uint8_t) (msg.header.d16 & 0xFF);
	txData[1] = (uint8_t) (msg.header.d16 >> 8);
	//Then the rest of the message data (if any)
	for (int i = 0; i < msg.header.b.NumberOfDataObjects * 4; ++i){
		txData[2 + i] = msg.data[i];
	}

	//Calculate the CRC
	for (int i = 0; i < txDataLength; ++i){
		crc = crcBits(txData[i], 8, crc);
	}
	crc = crcWrap(crc);

	//Append the CRC to the rest of the message data
	for (int i = 0; i < 4; ++i){
		txData[txDataLength + i] = (uint8_t) ((crc >> (8 * i)) & 0xFF);
	}
	txDataLength += 4;

	//Fill the transmitBuffer
	//PREAMBLE
	for (int i = 0; i < 8; ++i){
		transmitBuffer[i] = PREAMBLE_BYTE;
	}
	txBitScanner.byte_p += 8;

	//SOP
	for (int i = 0; i < 4; ++i){
		//write each K-Code into the outputBuffer
		setNextNobble(&txBitScanner, sopCodes[i]);
	}

	//DATA
	//Perform 4b5b encoding (nibbles -> nobbles)
	for (int i = 0; i < txDataLength; ++i){
		//Transmit the LSBs first
		hex[0] = txData[i] & 0xF;
		hex[1] = txData[i] >> 4;

		//write each hex into the outputBuffer
		for (int j = 0; j < 2; ++j){
			setNextNobble(&txBitScanner, nobbleLookUpTable[hex[j]]);
		}
	}

	//EOP
	setNextNobble(&txBitScanner, K_EOP);

	//OUTPUT READY
	bufferedLength = txBitScanner.byte_p - &transmitBuffer[0] + 1;

	//wait for line to be idle
	tick = HAL_GetTick();
	while (flag_USBPD_SPI_Active){
		if (HAL_GetTick() - tick >= SPI_STATE_TRANS_TIMEOUT){
			DEBUG_PRINT("Pre-Transmission Timeout");
			return false;
		}
	}

	//THIS IS A VERY TIME SENSITIVE AREA - GOODCRC must be sent within 150us
	//PRINT STATEMENTS WILL CAUSE USB PD NEGOTIATION TO FAIL

	//Perform Transmission
	HAL_GPIO_WritePin(STUSB1602_TX_EN_GPIO_Port, STUSB1602_TX_EN_Pin, GPIO_PIN_SET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi_STUSB1602, transmitBuffer, bufferedLength,
	SPI_TRANSMIT_TIMEOUT);
	HAL_GPIO_WritePin(STUSB1602_TX_EN_GPIO_Port, STUSB1602_TX_EN_Pin, GPIO_PIN_RESET);


	//wait for the transmission to be complete
	tick = HAL_GetTick();
	while (flag_USBPD_SPI_Active){
		if (HAL_GetTick() - tick >= SPI_STATE_TRANS_TIMEOUT){
			DEBUG_PRINT("Post-Transmission Timeout");
			return false;
		}
	}

	//THIS IS A VERY TIME SENSITIVE AREA - GOODCRC WILL ARRIVE AFTER 150us
	//PRINT STATEMENTS WILL CAUSE USB PD NEGOTIATION TO FAIL

	if (ret != HAL_OK){
		DEBUG_PRINT("HAL return: %s", HALSTATUS_TO_STRING(ret));
	}

	return ret == HAL_OK;
}

//Receive, decode, and check the CRC for the next message. Calls callback parameter with received message. Returns success
bool waitForMessage(void (*callback)(Message_TypeDef *m), int timeout){
	uint8_t data[MAX_DATA_LENGTH];
	int dataLength;
	uint32_t crc;
	Message_TypeDef msg;

	if (callback == NULL)
		errorMode("Null message callback pointer", false);

	//Receive next message into receiveBuffer
	if (!receiveNextMessage(timeout)){
		return false;
	}

	//Decode the message in receiveBuffer
	dataLength = decodeMessage(data, MAX_DATA_LENGTH);
	if (dataLength == -1){
		return false;
	}

	//Assemble and check the CRC
	crc = ASSEMBLE_32B_FROM_BYTES(data, dataLength - 4);
	dataLength -= 4;
	if (checkCRC(data, dataLength, crc)){
		//Assemble header
		msg.header.d16 = ASSEMBLE_16B_FROM_BYTES(data, 0);
		dataLength -= 2;

		//Copy the rest of the message data into the msg structure
		if (msg.header.b.NumberOfDataObjects != 0)
			memcpy(msg.data, &data[2], dataLength);

		//Pass the message to the callback
		(*callback)(&msg);
		//Do cleanup after sending GoodCRC
		clearReceiveBuffer();
		return true;
	} else{
		DEBUG_PRINT("Bad CRC");
		clearReceiveBuffer();
		return false;
	}
}

//Decodes the next message in the input buffer, and copies the output to the array described by outpBuff and buffSize; returns number of bytes output
int decodeMessage(uint8_t *outpBuff, int buffSize){
	DecodedData d;
	DecodedData s[2];
	SOP_Sequence sop;
	int i;

	//Clear the output buffer
	memset(outpBuff, 0, buffSize);

	//move the rxBitScanner to the end of the preamble
	if (!scanRxToAfterEndPreamble()){
		DEBUG_PRINT("Preamble not found");
		printReceiveBuffer(); //to help debugging
		return -1;
	}

	//The first 4 bytes are K-Codes for the SOP
	for (int i = 0; i < 2; ++i){
		s[i] = decodeByte();
	}

	//Parse the SOP
	sop = parseSOP(s);
	if (sop != SOP){
		DEBUG_PRINT("Got SOP Sequence: %s", SOP_SEQ_TO_STRING(sop));
		if(sop == HARD_RESET)
			hardReset();
		return -1;
	}

	//Decode each byte in receiveBuffer, up to the length of the output buffer
	for (i = 0; i < buffSize; ++i){
		d = decodeByte();
		if ((d.K[0] == K_NULL) && (d.K[1] == K_NULL)){
			//Message Data
			*(outpBuff + i) = d.byte;
		} else if (d.K[0] == K_EOP){
			//End of Packet
			return i;
		} else{
			//Error - Invalid K-Code in receiveBuffer
			DEBUG_PRINT("decodeMessage():: (at rxBuff[%d]) got K_Codes: %s, %s",
					rxBitScanner.byte_p - &receiveBuffer[0], K_CODE_TO_STRING(d.K[0]),
					K_CODE_TO_STRING(d.K[1]));
			//Erase everything that has been written to the output buffer
			memset(outpBuff, 0, i);
			return -1;
		}
	}

	//Error
	if(i==buffSize)
		DEBUG_PRINT("Output buffer overflow");
	else
		DEBUG_PRINT("EOP not found");
	memset(outpBuff, 0, buffSize);
	return -1;
}

//Moves receive bit scanner to the bit after the end of the preamble in receive buffer. Returns success.
bool scanRxToAfterEndPreamble(){
	uint8_t inputSample;
	uint8_t bitInc = 1;
	uint16_t preambleBitsFound = 7;

	while (rxBitScanner.byte_p < &receiveBuffer[RECEIVE_BUFFER_LENGTH - 1]){

		inputSample = getNextByte(rxBitScanner);
//		printf(PRINTF_BINARY "\n", BYTE_TO_BINARY(inputSample));
		switch (inputSample) {
			case PREAMBLE_BYTE:
				//this is preamble
				preambleBitsFound += bitInc;
				//After the preamble is found, start shifting forward 2 bits at a time
				bitInc = 2;
				break;
			default:
				if (preambleBitsFound > 15){
					//this is the end of the preamble: inputSample = 0101 01XX
					//shift to the start of the SOP                         ^ Start of SOP
					scanForwardBits(&rxBitScanner, 6);
					return true;
				}
				break;
		}
		// Move forward one increment (1 bit if preamble has not been encountered yet, else 2)
		scanForwardBits(&rxBitScanner, bitInc);
	}
	//Preamble detection failure
	resetRxBitScanner();
	return false;
}

// Decodes and returns the byte corresponding to the next 10bits of the receive buffer
DecodedData decodeByte(){
	DecodedData d = { .K = { K_NULL, K_NULL }, .byte = 0x0000 };
	uint8_t nobble = 0;
	uint8_t nibble;

	// There are two nibbles in a byte, so decode two nobbles
	for (uint8_t k = 0; k < 2; ++k){
		nibble = 0xFF; //0xFF means not yet decoded
		nobble = getNextByte(rxBitScanner) >> 3; // The next nobble is the next 5 MSBs
		scanForwardBits(&rxBitScanner, 5);

		// Look for the nobble in the lookup table
		for (uint8_t i = 0; i < 16; ++i){
			if (nobble == nobbleLookUpTable[i]){
				// Found it: set nibble to the decoded value
				nibble = i;
				break;
			}
		}

		if (nibble == 0xFF){
			// nobble not found in the lookup table: check against non-numeric K-Codes
			if ((nobble == K_SYNC_1) || (nobble == K_SYNC_2) || (nobble == K_RST_1) || (nobble == K_RST_2)
					|| (nobble == K_EOP)){
				d.K[k] = (K_Code) nobble;
			} else
				d.K[k] = K_RESERVED; //Not a valid K-Code
		} else{
			// nobble found in the lookup table: set corresponding nibble of output byte
			d.byte |= nibble << (k == 0 ? 0 : 4); //first k-code received corresponds to 4 LSBs
		}

//		DEBUG_PRINT("Nobble: " PRINTF_BINARY "\t Hex: %X\t Byte: " PRINTF_BINARY "K-Code: %s\n",
//				BYTE_TO_BINARY(nobble), nibble, BYTE_TO_BINARY(d.byte), K_CODE_TO_STRING(d.K[k]));
	}

	return d;
}

//Determines which valid SOP sequence, if any, is in the length-2 array of DecodedData
SOP_Sequence parseSOP(DecodedData* s){
	uint8_t sopNotValid = 0;
	// As long as 3 of the 4 K-Codes are correct, the sequence is valid

	if (s[0].K[0] != K_SYNC_1)
		++sopNotValid;
	if (s[0].K[1] != K_SYNC_1)
		++sopNotValid;
	if (s[1].K[0] != K_SYNC_1)
		++sopNotValid;
	if (s[1].K[1] != K_SYNC_2)
		++sopNotValid;

	if (sopNotValid <= 1)
		return SOP;

	sopNotValid = 0;
	if (s[0].K[0] != K_RST_1)
		++sopNotValid;
	if (s[0].K[1] != K_RST_1)
		++sopNotValid;
	if (s[1].K[0] != K_RST_1)
		++sopNotValid;
	if (s[1].K[1] != K_RST_2)
		++sopNotValid;

	if (sopNotValid <= 1)
		return HARD_RESET;

	return SOP_UNDEFINED;
}

//Fills the receive buffer. Will wait until the flag_USBPD_SPI_Active flag is high,
//then receives the message, and stops receiving when the flag goes low again, returns success
bool receiveNextMessage(int timeout){
	HAL_StatusTypeDef hal_ret;
	uint32_t tick = 0;

	// Wait for line to be active
	tick = HAL_GetTick();
	while (!flag_USBPD_SPI_Active){
		if (HAL_GetTick() - tick >= timeout){
			DEBUG_PRINT("Pre-Reception Timeout");
			return false;
		}
	}

	// Receive the message
	hal_ret = MY_SPI_Receive(&hspi_STUSB1602, &receiveBuffer[0], RECEIVE_BUFFER_LENGTH,
			&flag_USBPD_SPI_Active, timeout);

	if (hal_ret == HAL_TIMEOUT)
		DEBUG_PRINT("SPI Receive Timeout");

	if (hal_ret == HAL_OK){
		return true;
	} else{
		DEBUG_PRINT("Receive Buffer fill error...");
		clearReceiveBuffer();
		return false;
	}
}

// Called on NSS interrupt
// User needs to call this from the  when the NSS_Pin interrupt occurs (see below)
void STUSB1602_EXTI_Callback(){
	flag_USBPD_SPI_Active = !(HAL_GPIO_ReadPin(STUSB1602_NSS_GPIO_Port, STUSB1602_NSS_Pin));
}

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == STUSB1602_NSS_Pin){
		STUSB1602_EXTI_Callback();
	}
}

//copied and modified from HAL library; flag dependent slave SPI receiver; returns error if buffer overflows
HAL_StatusTypeDef MY_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size,
		volatile bool *rxFlag, int timeout){
	HAL_StatusTypeDef errorcode = HAL_ERROR;
	uint32_t tick = 0;

	if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES)){
		errorcode = HAL_ERROR;
		goto error;
	}

	/* Process Locked */
	__HAL_LOCK(hspi);

	if (hspi->State != HAL_SPI_STATE_READY){
		errorcode = HAL_BUSY;
		goto error;
	}

	if ((pData == NULL) || (Size == 0)){
		errorcode = HAL_ERROR;
		goto error;
	}

	/* Set the transaction information */
	hspi->State = HAL_SPI_STATE_BUSY_RX;
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	hspi->pRxBuffPtr = (uint8_t *) pData;
	hspi->RxXferSize = Size;
	hspi->RxXferCount = Size;

	/*Init field not used in handle to zero */
	hspi->pTxBuffPtr = (uint8_t *) NULL;
	hspi->TxXferSize = 0U;
	hspi->TxXferCount = 0U;
	hspi->RxISR = NULL;
	hspi->TxISR = NULL;

	/* Configure communication direction: 1Line */
	if (hspi->Init.Direction == SPI_DIRECTION_1LINE){
		SPI_1LINE_RX(hspi);
	}

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}

	tick = HAL_GetTick();
	/* Receive data in 8 Bit mode */
	if (hspi->Init.DataSize == SPI_DATASIZE_8BIT){
		/* Transfer loop */
		while (hspi->RxXferCount > 0U){
			/* Check the RXNE flag */
			if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)){
				/* read the received data */
				(*(uint8_t *) pData) = *(__IO uint8_t *) &hspi->Instance->DR;
				pData += sizeof(uint8_t);
				hspi->RxXferCount--;
			} else{
				/*Return when flag goes low*/
				if (!*rxFlag){
					errorcode = HAL_OK;
					goto error;
				}
				/* Timeout management */
				if (HAL_GetTick() - tick >= timeout){
					errorcode = HAL_TIMEOUT;
					goto error;
				}
			}
		}
	} else{
		errorcode = HAL_ERROR;
		goto error;
	}

	/* Check the end of the transaction */
	if ((hspi->Init.Mode == SPI_MODE_MASTER)
			&& ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
					|| (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY))){
		/* Disable SPI peripheral */
		__HAL_SPI_DISABLE(hspi);
	}

	if (hspi->ErrorCode != HAL_SPI_ERROR_NONE){
		errorcode = HAL_ERROR;
	}

	error: hspi->State = HAL_SPI_STATE_READY;
	__HAL_UNLOCK(hspi);
	return errorcode;
}

// Returns the next 8 bits that the passed bit scanner is pointing at
uint8_t getNextByte(BitScanner b){
	return (*(b.byte_p) << b.bitOffset) | (*(b.byte_p + 1) >> (8 - b.bitOffset));
}

// Sets the next 5 bits that the passed bit scanner is pointing at
void setNextNobble(BitScanner *b, uint8_t nobble){
	nobble &= 0b00011111;
	if (b->bitOffset <= 3)
		*b->byte_p |= (nobble << (3 - b->bitOffset));
	else{
		*b->byte_p |= (nobble >> (b->bitOffset - 3));
		*(b->byte_p + 1) |= (nobble << (8 + 3 - b->bitOffset));
	}
	scanForwardBits(b, 5);
}

//Shifts the [parameter] bit-scanner forward [parameter] bits
static void scanForwardBits(BitScanner* b, uint8_t bitsForward){
	b->bitOffset += bitsForward;
	b->byte_p += (b->bitOffset) / 8;
	b->bitOffset %= 8;
}

// Resets the receive bit scanner to the beginning of the receive buffer
static void resetRxBitScanner(){
	rxBitScanner.bitOffset = 0;
	rxBitScanner.byte_p = &receiveBuffer[0];
}

//Clears the receive buffer and resets the receive buffer bit-scanner
void clearReceiveBuffer(){
//	DEBUG_PRINT("Receive Buffer cleared\r\n");
	memset(receiveBuffer, 0, RECEIVE_BUFFER_LENGTH);
	resetRxBitScanner();
}

// Prints out the entire receive buffer; for debugging
void printReceiveBuffer(){
	printf("Rx Buffer:\n");
	for (int i = 0; i < RECEIVE_BUFFER_LENGTH; ++i){
		printf(PRINTF_BINARY"\n", BYTE_TO_BINARY(receiveBuffer[i]));
	}
}
