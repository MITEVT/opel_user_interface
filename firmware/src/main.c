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

static uint32_t vel1[3]=[0x703,0,0];
static uint32_t vel2[3]=[0x704,0,0];

static uint32_t throt_acc[3]=[0x301,0,0];
static uint32_t throt_break[3]=[0x301,1,0];

static uint32_t drive_key[3]=[0x505,0,0];
static uint32_t drive_status[3]=[0x505,1,0];

static uint32_t motor_shut_ok[3]=[0x705,0,0];
static uint32_t motor_curr[3]=[0x705,1,0];
static uint32_t motor_speed[3]=[0x705,2,0];
static uint32_t motor_volt[3]=[0x705,3,0];
static uint32_t motor_torque[3]=[0x705,4,0];

static uint32_t low_voltage_bus_battery_flag[3]=[0x305,0,0]; 
static uint32_t low_voltage_DC-DC_status[3]=[0x305,1,0]; 
static uint32_t critical_systems_battery_flag[3]=[0x305,2,0];
static uint32_t critical_systems_DC-DC_status[3]=[0x305,3,0];
static uint32_tpdm_status[3]=[0x305,4,0];

static uint32_t contactor_1_error[3]=[0x6F7,0,0];
static uint32_t contactor_2_error[3]=[0x6F7,1,0];
static uint32_t contactor_1_status[3]=[0x6F7,2,0];
static uint32_t contactor_2_status[3]=[0x6F7,3,0];
static uint32_t 21v_contactor_status[3]=[0x6F7,4,0];
static uint32_t contactor_3_error[3]=[0x6F7,5,0];
static uint32_t contactor_3_status[3]=[0x6F7,6,0];
static uint32_t precharge_state[3]=[0x6F7,7,0];

static uint32_t min_cell_temp[3]=[0x6F9,0,0];
static uint32_t max_cell_temp[3]=[0x6F9,1,0];
static uint32_t cmu_with_min_temp[3]=[0x6F9,2,0];
static uint32_t cmu_with_max_temp[3]=[0x6F9,3,0];

static uint32_t min_cell_voltage[3]=[0x6F8,0,0];
static uint32_t max_cell_voltage[3]=[0x6F8,1,0];
static uint32_t cmu_with_min_cell_voltage[3]=[0x6F8,2,0];
static uint32_t cell_with_min_voltage[3]=[0x6F8,3,0];
static uint32_t cmu_with_max_voltage[3]=[0x6F8,4,0];
static uint32_t cell_with_max_voltage[3]=[0x6F8,5,0];

static uint32_t battery_voltage[3]=[0x6FA,0,0];
static uint32_t battery_current[3]=[0x6FA,1,0];

static uint32_t (*info1[])[3]= {&vel1,&vel1,&throt_acc,&throt_break,&motor_curr,&motor_speed, &motor_volt,&motor_torque,&motor_torque,&min_cell_temp,&max_cell_temp,&cmu_with_min_temp, &cmu_with_max_temp,&min_cell_voltage,&max_cell_voltage,&cmu_with_min_voltage,&cell_with_min_voltage, &cmu_with_max_voltage,&cell_with_max_voltage,&battery_voltage,&battery_current};

static uint32_t (*info2[])[3]={&drive_key,&drive_status,&motor_shut_ok};

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

inline static void car_status(int timestamp) {
	for (int n=0,n<21,n++){
		Board_UART_PrintNum(info1[n][0],16,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(info1[n][1],10,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(info1[n][2],10,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(timestamp,10,true);
	}

	for (int m=0,m<3,m++){
		Board_UART_PrintNum(info2[m][0],16,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(info2[m][1],10,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(info2[m][2],16,false);
		Board_UART_Print(",");
		Board_UART_PrintNum(timestamp,10,true);
	}

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
	bool error_flag = false
	bool send = false;
	int msTicks = 1000;
	int lastPrint = msTicks;
	
	while (1) {
		if(error_flag){
			car_status(msTicks)
			error_flag = false
		}
		if(lastPrint < msTicks-6000){
			Board_UART_Println("Sending CAN with ID: 0x7F5");
			msg_obj.msgobj = 2;
			msg_obj.mode_id = 0x7F5;
			msg_obj.dlc = 1;
			msg_obj.data_16[0] = 1;
			LPC_CCAN_API->can_transmit(&msg_obj);	
		}
		if(lastPrint < msTicks-1000){
			lastPrint = msTicks;
			car_status(lastPrint);
		}
		if (send) {
			if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
				CCAN_MSG_OBJ_T temp_msg;
				RingBuffer_Pop(&can_rx_buffer, &temp_msg);
				if (temp_msg.mode_id== drive_key[0]) {		//driver interface
					drive_key[2]=temp_msg.data_16[0];
					drive_status[2]=temp_msg.data_16[1];
				}
				else if (temp_msg.mode_id==vel1[0]) {	//velocity1
					vel1[2] = temp_msg.data_16[0];
				}
				else if (temp_msg.mode_id==vel2[0]) {	//velocity2
					vel2[2] = temp_msg.data_16[0];
				}
				else if (temp_msg.mode_id==motor_shut_ok[0]) {	//motor interface 
					motor_shut_ok[2]= temp_msg.data_16[0];
					motor_curr[2]= temp_msg.data_16[1];
					motor_speed[2]= temp_msg.data_16[2];
					motor_volt[2]= temp_msg.data_16[3];
					motor_torque[2]= temp_msg.data_16[4];
				}
				else if (temp_msg.mode_id==throt_acc[0]) { //throttle interface   
					throt_acc[2]=temp_msg.data[0];
					throt_break[2]=temp_msg.data[1];
				}
				else if (temp_msg.mode_id==pdm_status[0]) { //pdm    
					low_voltage_bus_battery_flag[2]=(temp_msg.data[0] && 00000001); 
					low_voltage_DC-DC_status[2]= (temp_msg.data[0] >> 1) && 00000001; 
					critical_systems_battery_flag[2]=(temp_msg.data[0] >> 2) && 00000001;
					critical_systems_DC-DC_status[2]=(temp_msg.data[0] >> 3) && 00000001;
					pdm_status[2]=(temp_msg.data[0] >> 4) && 0001;
				}
//				else if (temp_msg.mode_id==0x6F5) {	//pack charge misbalance
//					balance_soc[2]=temp_msg.data_16[0];
//					pack_balance_soc[2]=temp_msg.data_16[1];
//				}
				else if (temp_msg.mode_id==contactor_1_error[0]) { //precharge_status
					contactor_1_error[2]=(temp_msg.data_16[0] && 00000001);
					contactor_2_error[2]=(temp_msg.data_16[0] >> 1) && 00000001;
					contactor_1_status[2]=(temp_msg.data_16[0] >> 2) && 00000001;
					contactor_2_status[2]=(temp_msg.data_16[0] >> 3) && 00000001;
					21v_contactor_status[2]=(temp_msg.data_16[0] >> 4) && 0001;
					contactor_3_error[2]=(temp_msg.data_16[0] >> 5) && 0001;
					contactor_3_status[2]=(temp_msg.data_16[0] >> 6) && 0001;
					precharge_state[2]=(temp_msg.data_16[0] >> 7);
				}
				else if (temp_msg.mode_id==min_cell_temp[0]) {	//cell temps
					min_cell_temp[2]= temp_msg.data_16[0];
					max_cell_temp[2]= temp_msg.data_16[1];
					cmu_with_min_temp[2]= temp_msg.data[2];
					cmu_with_max_temp[2]= temp_msg.data[3];
				}
				else if (temp_msg.mode_id==min_cell_voltage[0]) {//cell voltages
					min_cell_voltage[2]= temp_msg.data_16[0];
					max_cell_voltage[2]= temp_msg.data_16[1];
					cmu_with_min_voltage[2]= temp_msg.data_16[2];
					cell_with_min_voltage[2]= temp_msg.data_16[3];
					cmu_with_max_voltage[2]= temp_msg.data[4];
					cell_with_max_voltage[2]= temp_msg.data[5];
				}
				else if (temp_msg.mode_id==throt_acc[0]) { //battery current voltage  
					battery_voltage[2]=temp_msg.data_16[0];
					battery_current[2]=temp_msg.data_16[1];
				}

//				if (error_msg >> 7){ heartbeat?		//temporary --> new error handling framework
//					error_flag = true
//					if (n) { //more than 7 possible errors 
//						error_type = error_msg && 01111111
//					} 
//					else { // 7 or less possible errors
//						if (error_msg && 00000001) {
//							flag1 = true
//						}
//						if ((error_msg && 00000010) >> 1) {
//							flag2 = true
//						}
//						if ((error_msg && 00000100) >> 2) {
//							flag3 = true
//						}
//						if ((error_msg && 00001000) >> 3) {
//							flag4 = true
//						}
//						if ((error_msg && 00010000) >> 4) {
//							flag5 = true
//						}
//						if ((error_msg && 00100000) >> 5) {
//							flag6 = true
//						}
//						if ((error_msg && 01000000) >> 6) {
//							flag7 = true
//						}
//					}
//				}
				
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

