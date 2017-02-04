#include "board.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 57600 					// Desired UART Baud Rate

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;
static uint32_t lastPrint;

static CCAN_MSG_OBJ_T msg_obj; 					// Message Object data structure for manipulating CAN messages
static RINGBUFF_T can_rx_buffer;				// Ring Buffer for storing received CAN messages
static CCAN_MSG_OBJ_T _rx_buffer[BUFFER_SIZE]; 	// Underlying array used in ring buffer

static char str[100];							// Used for composing UART messages
static uint8_t uart_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer
static uint8_t uart_tx_buffer[BUFFER_SIZE];

static bool can_error_flag;
static uint32_t can_error_info;



// -------------------------------------------------------------
// Helper Functions

/**
 * Delay the processor for a given number of milliseconds
 * @param ms Number of milliseconds to delay
 */
void _delay(uint32_t ms) {
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < ms);
}

//inline static void car_status(void){}
// -------------------------------------------------------------

// Main Program Loop

int main(void)
{

	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		Board_UART_Println("Failed to Initialize SysTick. ");
		// Unrecoverable Error. Hang.
		while(1);
	}

	//---------------
	// Initialize GPIO and LED as output
	Board_LEDs_Init(2,10);
	LED_On(2,10);

	//Initialize SSP
//	Board_SPI_Init();
//	Board_UART_Println("SPI pins set");
//	Chip_SSP_Init(LPC_SSP);
//	Board_UART_Println("SSP initialized");
//	Chip_SSP_SetBitRate(LPC_SSP, 30000);
	
//	Board_UART_Println("Chip SSP set up");
//	Chip_SSP_SetFormat(LPC_SSP, SSP_DATA_BITS, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
//	Chip_SSP_SetMaster(LPC_SSP, SSP_MODE_TEST);
//	Chip_SSP_Enable(LPC_SSP);
//	Board_UART_Println("SSP Buffer set up");
//	SSP_Buffer_Init();
	
	//---------------
	// Initialize CAN  and CAN Ring Buffer

	CAN_Init(CCAN_BAUD_RATE);
	// For your convenience.
	// typedef struct CCAN_MSG_OBJ {
	// 	uint32_t  mode_id;
	// 	uint32_t  mask;
	// 	uint8_t   data[8];
	// 	uint8_t   dlc;
	// 	uint8_t   msgobj;
	// } CCAN_MSG_OBJ_T;

	/* [Tutorial] How do filters work?

		Incoming ID & Mask == Mode_ID for msgobj to accept message

		Incoming ID : 0xabc
		Mask: 		  0xF0F &
		            -----------
		              0xa0c

		mode_id == 0xa0c for msgobj to accept message

	*/

	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x000;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* [Tutorial] How do I send a CAN Message?

		There are 32 Message Objects in the CAN Peripherals Message RAM.
		We need to pick one that isn't setup for receiving messages and use it to send.

		For this exmaple we'll pick 31

		msg_obj.msgobj = 31;
		msg_obj.mode_id = 0x600; 		// CAN ID of Message to Send
		msg_obj.dlc = 8; 				// Byte length of CAN Message
		msg_obj.data[0] = 0xAA; 		// Fill your bytes here
		msg_obj.data[1] = ..;
		..
		msg_obj.data[7] = 0xBB:

		Now its time to send
		LPC_CCAN_API->can_transmit(&msg_obj);

	*/
	can_error_flag = false;
	can_error_info = 0;
	bool error_flag = false;
	bool send = true;
	uint32_t lastPrint = msTicks;
	
	while (1) {
		if(error_flag){
	//		car_status()
			error_flag = false;
		}
/*		if(lastPrint < msTicks-1000){
			Board_UART_Println("Sending CAN with ID: 0x7F5");
			msg_obj.msgobj = 2;
			msg_obj.mode_id = 0x7F5;
			msg_obj.dlc = 1;
			msg_obj.data_16[0] = 1;
			LPC_CCAN_API->can_transmit(&msg_obj);	
		}
		if(lastPrint < msTicks-1000){
			lastPrint = msTicks;
			car_status();
		}*/
		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			CAN_Receive(temp_msg);
			Board_UART_Print("Received Message ID: 0x");
			Board_UART_PrintNum(temp_msg.mode_id,16,true);
			Board_UART_PrintNum(temp_msg.data[0],16,true);
			Board_UART_PrintNum(temp_msg.data[1],16,true);	
/*			int count = temp_msg.dlc;
			int x = 0;
			while (x<count){
				Board_UART_PrintNum(temp_msg.data[x],16,true);
				x++;
			}*/
				
		}

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			Board_UART_PrintNum(can_error_info,2,true);
		}

		uint8_t count;

		if ((count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			//Board_UART_SendBlocking(uart_rx_buffer, count); // Echo user input
			switch (uart_rx_buffer[0]) {
				case 'p':
					Board_UART_Println("Sending CAN with ID: 0x305");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x305;
					msg_obj.dlc = 5;
					msg_obj.data_16[0] = 0x00;
					msg_obj.data_16[1] = 0x01;
					msg_obj.data_16[2] = 0x00;
					msg_obj.data_16[3] = 0x01;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'm':
					Board_UART_Println("Sending CAN with ID: 0x705");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x705;
					msg_obj.dlc = 7;
					msg_obj.data_16[0] = 0x01;
					msg_obj.data_16[1] = 0x13;
					msg_obj.data_16[2] = 0x0111;
					msg_obj.data_16[3] = 0x65;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'v':
					Board_UART_Println("Sending CAN with ID: 0x301");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x301;
					msg_obj.dlc = 3;
					msg_obj.data_16[0] = 0x31;
					msg_obj.data_16[1] = 0x00;
					msg_obj.data_16[2] = 0x00;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'x':
					Board_UART_Println("Sending CAN with ID: 0x505");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x505;
					msg_obj.dlc = 4;
					msg_obj.data_16[0] = 0x0020;
					msg_obj.data_16[1] = 0x0F00;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'g':
					Board_UART_PrintNum(0xFFF, 16, true);
					break;
				case 's':	//receive from RaspberryPi
					send = !send;
					break;
				default:
					//Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}

