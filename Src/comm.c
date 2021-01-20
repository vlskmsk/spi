/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"
#include "version.h"
#include "init.h"

float iq_limit = 90.0f;
float kd_gain = .5f;
float m_q_offset = 0;
float m_gear_ratio_conv = 0.134497135f;


uint8_t new_uart_packet = 0;


volatile floatsend_t rx_format;
volatile floatsend_t tx_format;
volatile floatsend_t mcur_format;
volatile float spi_cmd_f = 0;

volatile uint8_t r_data[MAX_SPI_BYTES] = {0};
volatile uint8_t t_data[MAX_SPI_BYTES] = {0,0,0,0,0, 0xBD,'m','o','t','o','r','f','i','n','g','e','r'};

uint8_t uart_read_buffer[MAX_UART_BYTES] = {0};

uint8_t enable_pressure_flag = 0;

int num_uart_bytes = BARO_SENSE_SIZE;

static int gl_num_cursense_bytes = 0;

volatile uint8_t control_mode = CMD_CHANGE_IQ;

void uart_print_float(float v)
{
	floatsend_t ft;
	ft.v = v;
	HAL_UART_Transmit(&huart1, ft.d, 4, 10);
}

void print_string(char*str)
{
	int strlen=0;
	for(strlen=0;str[strlen]!=0;strlen++);
	HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen,10);
}
/*
 * Command structure is determined by the highest 4 bits
 */

#define MODE_SEND_ROTOR_POS  	2
#define MODE_SEND_ROTOR_SPEED  	3

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)  //Bird
{	//HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 1);
	tx_format.v = gl_theta_enc - m_q_offset;	//in all cases, send position
	mcur_format.v = gl_iq_meas;

//	if ((r_data[0]==0x78&&r_data[1]==0x78&&r_data[2]==0x78&&r_data[3]==0x78&&r_data[4]==0x78)||
//			(r_data[0]==0xAB&&r_data[1]==0xCD&&r_data[2]==0xEF&&r_data[3]==0x12&&r_data[4]==0x34&&r_data[5]==0x56))
//		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 1);
//	else
//		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 0);
//
	//if (r_data[0]==0xAB)&&r_data[1]==0xCD&&r_data[2]==0xEF&&r_data[3]==0x12&&r_data[4]==0x34&&r_data[5]==0x56)
//	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 0);


//r_data[7]!=0&&r_data[8]!=0
//r_data[6]!=0||r_data[7]!=0||r_data[8]!=0||r_data[9]!=0||r_data[10]!=0||

	if (r_data[8]==0)
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 1);
	else
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 0);

	for(int i=0;i<4;i++)
		rx_format.d[i] = r_data[i+1];

	//control_mode = r_data[0];
	//parse_master_cmd();

}

uint32_t uart_it_ts = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //Bird
{
	uart_it_ts = HAL_GetTick()+4;
	new_uart_packet = 1;
}

void parse_master_cmd()
{
	switch(control_mode)
	{
	case CMD_LED_OFF:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		break;
	case CMD_LED_ON:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		break;
	case CMD_CHANGE_IQ :
	{
		spi_cmd_f = rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = tx_format.d[i];	//guard against writing q in the middle of updating the spi buffer
		for(int i=0; i < gl_num_cursense_bytes; i++)
			t_data[i+5] = mcur_format.d[i];
		break;
	}
	case CMD_CHANGE_POS:
	{
		spi_cmd_f = rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = tx_format.d[i];	//guard against writing q in the middle of updating the spi buffer
		for(int i=0; i < gl_num_cursense_bytes; i++)
			t_data[i+5] = mcur_format.d[i];
		break;
	}
	case CMD_PLAY_TONE:
	{
		if(t_data[0] == MAIN_LOOP_READY)
		{
			TIM1->PSC = (rx_format.d[0] << 8) | rx_format.d[1];
			TIMER_UPDATE_DUTY(490,510,490);
		}
		break;
	}
	case CMD_STOP_TONE:
	{
		TIMER_UPDATE_DUTY(500,500,500);
		TIM1->PSC = 0;
		break;
	}
	case CMD_FORCE_ENCODER:
	{
		break;
	}
	case CMD_CHANGE_KD:
	{
		kd_gain = rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = rx_format.d[i];	//write confirmation to the tx buf until you exit this control mode and re-enter a motor movement-type control mode
		break;
	}
	case CMD_CHANGE_IQ_LIM:
	{
		iq_limit = rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = rx_format.d[i];	//write confirmation to the tx buf until you exit this control mode and re-enter a motor movement-type control mode
		break;
	}
	case CMD_CHANGE_GEAR_CONV:
	{
		m_gear_ratio_conv = rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = rx_format.d[i];	//write confirmation to the tx buf until you exit this control mode and re-enter a motor movement-type control mode
		break;
	}
	case CMD_ZERO_POS:
	{
		m_q_offset = gl_theta_enc;//unwrap(theta_abs_rad(), &mech_theta_prev);
		break;
	}
	case CMD_SET_POS:
	{
		m_q_offset = gl_theta_enc - rx_format.v;
		for(int i=0;i<4;i++)
			t_data[i+1] = rx_format.d[i];	//write confirmation to the tx buf until you exit this control mode and re-enter a motor movement-type control mode
		break;
	}
	case CMD_DRIVER_ENABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
		break;
	case CMD_DRIVER_DISABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
		break;
	/* Pressure Sensor Related Case. */
	case CMD_EN_PRES_BARO:
		enable_pressure_flag = 1;
		gl_num_cursense_bytes = 0;
		num_uart_bytes = BARO_SENSE_SIZE;
		break;
	case CMD_EN_PRES_MAGSENSE:
		enable_pressure_flag = 1;
		gl_num_cursense_bytes = 0;
		num_uart_bytes = MAG_SENSE_SIZE;
		break;
	case CMD_DIS_PRES:  //Bird
		enable_pressure_flag = 0;
		break;
	case CMD_EN_CURSENSE:
		gl_num_cursense_bytes = 4;
		enable_pressure_flag = 0;
		break;
	/* Bootloader Related Case. */
	case CMD_BOOTLOAD:
		asm("NOP");
		NVIC_SystemReset();
		break;
	case CMD_SLEEP:  //Bird
	//		sleep_reset();
		break;
	case CMD_WAKEUP:
		break;
	case CMD_GET_VERSION:
	{
		uint32_fmt_t fmt;
		fmt.v = DRIVER_VERSION_NUMBER;
		for(int i=0;i<4;i++)
			t_data[i+1] = fmt.d[i];
		break;
	}
	default:
		break;
	}
}


static volatile uint8_t fail_state = 0;
static volatile uint8_t busy = 0;
static volatile uint8_t ss = 0;
static volatile uint8_t frame_done = 1;
void handle_comms(void)
{
	/*Handle SPI Interrupt Packets*/
	//HAL_SPI_TransmitReceive_IT(&hspi3, (uint8_t*)t_data, (uint8_t*)r_data, NUM_MOTOR_BYTES+num_uart_bytes);
	start_spi_slave_rxtx_frame(&hspi3, (uint8_t*)t_data, (uint8_t*)r_data, NUM_MOTOR_BYTES+num_uart_bytes);

	if(HAL_GetTick() > uart_it_ts)
		HAL_UART_Receive_IT(&huart1, uart_read_buffer, num_uart_bytes);

	/*TODO: Enable/test UART!!! This should be INTERRUPT based, not DMA based.*/
	if(new_uart_packet == 1)
	{
		if(enable_pressure_flag)
		{
			//first 5 bytes of r_data and t_data are RESERVED for motor control, and must not be overwritten
			for(int i = NUM_MOTOR_BYTES; i < (NUM_MOTOR_BYTES+num_uart_bytes); i++)
				t_data[i] = uart_read_buffer[i-5];
		}
		new_uart_packet = 0;
	}
}



