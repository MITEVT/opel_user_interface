#include "board.h"
#include "stepperMotor.h"

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
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		// Unrecoverable Error. Hang.
		while(1);
	}

	//---------------
	// Initialize GPIO and LED as output
	Board_LEDs_Init();
	Board_LED_On(LED0);

	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

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
	msg_obj.mode_id = 0x700;
	msg_obj.mask = 0x7FC;
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

	Board_UART_Print("Initializing\r\n");
	
	STEPPER_MOTOR_T vgauge;
	vgauge.ports[0] = 2;
	vgauge.ports[1] = 3;
	vgauge.ports[2] = 2;
	vgauge.ports[3] = 2;
	vgauge.pins[0] = 2;
	vgauge.pins[1] = 0;
	vgauge.pins[2] = 7;
	vgauge.pins[3] = 8;
	vgauge.step_per_rotation = 640;
	vgauge.step_delay = 2;
	Stepper_Init(&vgauge);
	Stepper_ZeroPosition(&vgauge, msTicks);	

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_10, IOCON_DIGMODE_EN);
	STEPPER_MOTOR_T tgauge;
	tgauge.ports[0] = 2;
	tgauge.ports[1] = 1;
	tgauge.ports[2] = 3;
	tgauge.ports[3] = 1;
	tgauge.pins[0] = 11;
	tgauge.pins[1] = 5;
	tgauge.pins[2] = 2;
	tgauge.pins[3] = 10;
	tgauge.step_per_rotation = 640;
	tgauge.step_delay = 2;
	Stepper_Init(&tgauge);
	Stepper_ZeroPosition(&tgauge, msTicks);
	
	Stepper_Step(&vgauge, msTicks);
	Stepper_Step(&tgauge, msTicks);


	while (1) {
		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&can_rx_buffer, &temp_msg);
			
			if (temp_msg.mode_id==0x703){
				int vel = (temp_msg.data_16[0]*60*22*22)/(7*12*5280); 
				int vpospercent = vel*100/110;
				Stepper_SetPosition(&vgauge, vpospercent, msTicks); 
			}

			if (temp_msg.mode_id==0x700){
				int throt = temp_msg.data_16[0];
				int tpospercent = throt*640/6535;
				Stepper_SetPosition(&tgauge,tpospercent,msTicks);	
			}

		}	

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Println(str);
		}

		Stepper_Step(&vgauge, msTicks);
		
		Stepper_Step(&tgauge, msTicks);
		
	
		}
}	
