#include "board.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 57600 					// Desired UART Baud Rate

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;

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

// -------------------------------------------------------------
// CAN Driver Callback Functions

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
    a CAN message has been received */
void CAN_rx(uint8_t msg_obj_num) {
	// LED_On();
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1) {
		RingBuffer_Insert(&can_rx_buffer, &msg_obj);
	}
}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {
	msg_obj_num = msg_obj_num;
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occurred on the CAN bus */
void CAN_error(uint32_t error_info) {
	can_error_info = error_info;
	can_error_flag = true;
}

// -------------------------------------------------------------
// Interrupt Service Routines


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
	Board_LEDs_Init();
	Board_LED_On(LED0);

	//Initialize SSP
	Board_SPI_Init();
//	Board_UART_Println("SPI pins set");
	Chip_SSP_Init(LPC_SSP);
//	Board_UART_Println("SSP initialized");
	Chip_SSP_SetBitRate(LPC_SSP, 30000);
	
//	Board_UART_Println("Chip SSP set up");
	Chip_SSP_SetFormat(LPC_SSP, SSP_DATA_BITS, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
	Chip_SSP_SetMaster(LPC_SSP, SSP_MODE_TEST);
	Chip_SSP_Enable(LPC_SSP);
//	Board_UART_Println("SSP Buffer set up");
	SSP_Buffer_Init();
	
	//---------------
	// Initialize CAN  and CAN Ring Buffer

	RingBuffer_Init(&can_rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), BUFFER_SIZE);
	RingBuffer_Flush(&can_rx_buffer);

	Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);

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
	int i;
	


	while(1) {
		uart_tx_buffer = [0x12];
		Board_UART_SendBlocking(uart_tx_buffer, BUFFER_SIZE);


		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&can_rx_buffer, &temp_msg);
			Board_UART_Print("Received Message ID: 0x");
			itoa(temp_msg.mode_id, str, 16);
			Board_UART_Println(str);

			Board_UART_Print("\t0x");
			itoa(temp_msg.data_16[0], str, 16);
			Board_UART_Println(str);

		}	

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Println(str);
		}

		uint8_t count;

		if ((Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			uint8_t count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE);
			Board_UART_SendBlocking(uart_rx_buffer, count); // Echo user input
			switch (uart_rx_buffer[0]) {
				case 'a':
					Board_UART_Println("Sending CAN with ID: 0x600");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x703;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 300;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'b':
					Board_UART_Println("Sending CAN with ID: 0x600");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x703;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 600;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'c':
					Board_UART_Println("Sending CAN with ID: 0x600");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x703;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 0;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'd':
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x700;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 0;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'e':
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x700;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 32000;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'f':
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x700;
					msg_obj.dlc = 2;
					msg_obj.data_16[0] = 65500;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				default:
					Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}
