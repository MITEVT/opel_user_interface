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

uint32_t velocity_1=0;
uint32_t velocity_2=0;
uint32_t acc_val=0;
uint32_t brake_val=0;
char *key_ignition="NONE";
char *drive_status="NONE";
char *shutdown_ok ="NONE";
uint32_t m_current=0;
uint32_t m_rpm=0;
uint32_t m_voltage=0;
uint32_t m_torque=0;
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
void drive_info(CCAN_MSG_OBJ_T msg) {
	if (msg.data_16[0]==0x0020) {
		key_ignition ="RUN";
	}
	if (msg.data_16[0]==0x0040) {
		key_ignition = "START";
	}
	if (msg.data_16[1]==0x0000){
		drive_status = "PARKED";
	}
	if (msg.data_16[1]==0X00F0){
		drive_status = "FORWARD";
	}
	if (msg.data_16[1]==0X0030){
		drive_status = "REVERSE";
	}
	if (msg.data_16[1]==0X0F00){
		drive_status = "SHUTDOWN_IMPENDING";
	}
	if (msg.data_16[1]==0X0300){
		drive_status = "INIT";
	}
	if (msg.data_16[1]==0XF000){
		drive_status = "CHARGE";
	}
	if (msg.data_16[1]==0X3000){
		drive_status= "OFF";
	}
}

void throttle_info(CCAN_MSG_OBJ_T msg) {
	acc_val = msg.data_16[0];
	brake_val = msg.data_16[1];
}	

void motor_info(CCAN_MSG_OBJ_T msg) {
	if (msg.data_16[0]==0) {
		shutdown_ok = "OK";
	}
	if (msg.data_16[0]==1) {
		shutdown_ok = "NOT_OK";
	}
	m_current = msg.data_16[1];
	m_rpm = msg.data_16[2];
	m_voltage = msg.data_16[3];
	m_torque = msg.data_16[4];
}

void car_status(void) {
	Board_UART_Print("Key_Ignition: ");
	Board_UART_Println(key_ignition);
	Board_UART_Print("Drive_Status: ");
	Board_UART_Println(drive_status);
	Board_UART_Print("Throttle_Acc_Val: ");
	Board_UART_PrintNum(acc_val,10,true);
	Board_UART_Print("Throttle_Brake_Val: ");
	Board_UART_PrintNum(brake_val,10,true); 
	Board_UART_Print("Velocity_1: ");
	Board_UART_PrintNum(velocity_1,10,true);
	Board_UART_Print("Velocity_2: ");
	Board_UART_PrintNum(velocity_2,10,true);
	Board_UART_Print("Shutdown: ");
	Board_UART_Println(shutdown_ok);
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
	//Board_UART_Println("Started up");

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
	msg_obj.mode_id = 0x7F5;
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
	int dlen;
	bool send = false;
	msTicks = 1000;
	lastPrint = msTicks;
	
	while (1) {
		if(lastPrint < msTicks-1000){
			lastPrint = msTicks;
			car_status();
		}

		if (send) {
			if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
				CCAN_MSG_OBJ_T temp_msg;
				RingBuffer_Pop(&can_rx_buffer, &temp_msg);
				if (temp_msg.mode_id== 0x505) {		//driver interface
					drive_info(temp_msg);
				}
				else if (temp_msg.mode_id==0x703) {	//velocity1
					velocity_1 = temp_msg.data_16[0];
				}
				else if (temp_msg.mode_id==0x704) {	//velocity2
					velocity_2 = temp_msg.data_16[0];
				}
				else if (temp_msg.mode_id==0x705) {	//motor interface 
					motor_info(temp_msg);
				}
				else if (temp_msg.mode_id==0x301) {	//throttle interface   
					throttle_info(temp_msg);
				}
				else if (temp_msg.mode_id==) {
					  
				}
				else {
					break;
				}
			}	
		}

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Println(str);
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
				case 't':
					Board_UART_Println("Sending CAN with ID: 0x301");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x301;
					msg_obj.dlc = 3;
					msg_obj.data_16[0] = 0x51;
					msg_obj.data_16[1] = 0x00;
					msg_obj.data_16[2] = 0x00;
					LPC_CCAN_API->can_transmit(&msg_obj);
					break;
				case 'd':
					Board_UART_Println("Sending CAN with ID: 0x505");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x505;
					msg_obj.dlc = 4;
					msg_obj.data_16[0] = 0x0020;
					msg_obj.data_16[1] = 0x00F0;
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

