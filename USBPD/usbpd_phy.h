/*
 * usbpd_phy.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Umberto Stefanini, Design Concepts, Inc.
 */

#ifndef USBPD_PHY_H_
#define USBPD_PHY_H_

//Includes
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "STUSB1602_Driver.h"

//Defines
#define RECEIVE_BUFFER_LENGTH 100
#define TRANSMIT_BUFFER_LENGTH 100
#define SPI_STATE_TRANS_TIMEOUT 10
#define SPI_TRANSMIT_TIMEOUT 100
#define MAX_DATA_LENGTH (2+4*7+4)
#define PREAMBLE_BYTE 0b01010101

/* NOTE:
 * This code was written with SPI configured with SPI_FIRSTBIT_MSB,
 * whereas the USB PD specification suggests that the first bit transmitted is the LSB
 *
 * With bits numbered from LSB to MSB, the received data looks like this:
 * Position    							array[0]			array[1] 		array[...
 * Order of bits received 	[0123 4567]		[89AB CDEF]	 [...
 * Nobbles (5bit nibbles)		[0123 4] [567 89] [AB CDE] [F...
 * Decoded Nibble (bits)		[3210]   [7654]   [CBA9]   [D... Note: bit reversal happens in the Nobble -> Nibble mapping
 * Decoded Nibble						[0x0]		 [0x1]		[0x2]		 [...
 * Nibbles -> 16bit words		[0x1 0x0] [0x2 ...
 *
 * */

//Type Definitions
typedef enum {
	K_NULL = (1 << 6),
	K_SYNC_1 = 0b00011,
	K_SYNC_2 = 0b10001,
	K_RST_1 = 0b11100,
	K_RST_2 = 0b10011,
	K_EOP = 0b10110,
	K_RESERVED = 0b00000
} K_Code;

typedef enum {
	SOP_UNDEFINED,
	SOP,
	SOP_PRIME,
	SOP_DOUBLE_PRIME,
	HARD_RESET,
	CABLE_RESET,
	SOP_PRIME_DEBUG,
	SOP_DOUBLE_PRIME_DEBUG
} SOP_Sequence;

typedef struct {
	K_Code K[2];
	uint8_t byte;
} DecodedData;

typedef union {
	uint16_t d16;
	struct {
		uint16_t MessageType :5;
		uint16_t PortDataRole :1;
		uint16_t SpecificationRevision :2;
		uint16_t PortPowerRole :1;
		uint16_t MessageID :3;
		uint16_t NumberOfDataObjects :3;
		uint16_t Extended :1;
	} b;
} Header_TypeDef;

typedef struct {
	Header_TypeDef header;
	uint8_t data[MAX_DATA_LENGTH - 6]; //doesn't include header or crc
} Message_TypeDef;

typedef struct {
	uint8_t* byte_p;
	uint8_t bitOffset; //Number of bits to shift left to align selected bit as MSB (see note above)
} BitScanner;

// Hardware Defines
#define STUSB1602_ALERT_Pin USBPD_ALERT_Pin
#define STUSB1602_ALERT_GPIO_Port USBPD_ALERT_GPIO_Port
#define STUSB1602_RESET_Pin USBPD_RESET_Pin
#define STUSB1602_RESET_GPIO_Port USBPD_RESET_GPIO_Port

#define STUSB1602_NSS_Pin USBPD_INT_Pin
#define STUSB1602_NSS_GPIO_Port USBPD_INT_GPIO_Port
#define STUSB1602_NSS_EXTI_IRQn USBPD_INT_EXTI_IRQn

#define STUSB1602_TX_EN_Pin USBPD_TX_EN_Pin
#define STUSB1602_TX_EN_GPIO_Port USBPD_TX_EN_GPIO_Port

#define STUSB1602_SPI SPI3
#define STUSB1602_SPI_GPIO_AF GPIO_AF6_SPI3
#define STUSB1602_SPI_CLK_ENABLE() __HAL_RCC_SPI3_CLK_ENABLE()
#define STUSB1602_SCLK_Pin USBPD_SCLK_Pin
#define STUSB1602_MISO_Pin USBPD_MISO_Pin
#define STUSB1602_MOSI_Pin USBPD_MOSI_Pin
#define STUSB1602_SPI_GPIO_Port USBPD_SCLK_GPIO_Port

#define STUSB1602_I2C I2C2
#define STUSB1602_I2C_GPIO_AF GPIO_AF4_I2C2
#define STUSB1602_I2C_CLK_ENABLE() __HAL_RCC_I2C2_CLK_ENABLE()
#define STUSB1602_SCL_Pin USBPD_SCL_Pin
#define STUSB1602_SDA_Pin USBPD_SDA_Pin
#define STUSB1602_I2C_GPIO_Port USBPD_SCL_GPIO_Port
#define STUSB1602_I2C_ADDR STUSB1602_I2C_Add_0


//Macros
#if defined(DEBUG)
#define DBG_LINE() printf("D: %s()\tLine %d\r\n",__func__,__LINE__)
#if DEBUG == 0
#define DEBUG_PRINT(fmt, args...) printf("D:" fmt "\r\n", ##args)
#elif DEBUG == 1
#define DEBUG_PRINT(fmt, args...) printf("DBG: %s()\t:: %d:\t" fmt "\r\n", __func__, __LINE__, ##args)
#endif
#else
#define DBG_LINE()
#define DEBUG_PRINT(fmt, args...)
#endif

#define K_CODE_TO_STRING(K) (K == K_NULL ? "K_NULL" : \
															K == K_SYNC_1 ? "K_SYNC_1" : \
															K == K_SYNC_2 ?  "K_SYNC_2": \
															K == K_RST_1 ? "K_RST_1": \
															K == K_RST_2 ? "K_RST_2" : \
															K == K_EOP? "K_EOP" : "K_RESERVED" \
																	)
#define SOP_SEQ_TO_STRING(S) (S == SOP_UNDEFINED? "SOP_UNDEFINED":\
															S == SOP? "SOP":\
															S == SOP_PRIME? "SOP_PRIME":\
															S == SOP_DOUBLE_PRIME? "SOP_DOUBLE_PRIME":\
															S == HARD_RESET? "HARD_RESET":\
															S == CABLE_RESET? "CABLE_RESET":\
															S == SOP_PRIME_DEBUG? "SOP_PRIME_DEBUG": "SOP_DOUBLE_PRIME_DEBUG"\
																	)

#define ASSEMBLE_16B_FROM_BYTES(arr,start) (((uint16_t) arr[start+1] << 8) | (arr[start]))
#define ASSEMBLE_32B_FROM_BYTES(arr,start) ( \
((uint32_t) (arr[start + 3]) << 24) | \
((uint32_t) (arr[start + 2]) << 16) | \
((uint16_t) (arr[start +1]) << 8) | \
((arr[start])) \
)

// Public Functions
void errorUSBPD(char* str);
void initSTUSB1602_HW();
void STUSB1602_EXTI_Callback();
bool sendMessage(Message_TypeDef msg);
bool waitForMessage(void (*callback)(Message_TypeDef *m), int timeout);

// Hardware Initialization Definitions & Functions
extern void _Error_Handler(char *, int);

extern I2C_HandleTypeDef hi2c_STUSB1602;
void MX_I2C2_Init(void);

extern SPI_HandleTypeDef hspi_STUSB1602;
void MX_SPI3_Init(void);

#endif /* USBPD_PHY_H_ */
