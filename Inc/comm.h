/*
 * comm.h
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */

#ifndef COMM_H_
#define COMM_H_
#include "init.h"
#include "foc_commutation.h"
#include "sleep.h"
#include "foc-calibration.h"
#include "spi_slave.h"

//#define MAIN_NOT_READY_BASECODE_TLE			0x40
//#define MAIN_LOOP_READY_TLE					0xAB
#define BUSY_FORCE_ENCODER_REGION		0xDE
#define MAIN_LOOP_READY					0xAD

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

extern float m_q_offset;
extern float kd_gain;
extern float iq_limit;
extern float m_gear_ratio_conv;
extern volatile float spi_cmd_f;

extern volatile uint8_t gl_spi_tx_mutex;


#define MAX_SPI_BYTES 32	//equal to 12+5, i.e. the maximum (magsense) number of data bytes and 5 for motor control and mode control
#define MAX_UART_BYTES 12	//3*2 bytes per sensor, 2 sensors. Max for memory allocation
enum {MAG_SENSE_SIZE = 12, BARO_SENSE_SIZE = 9};
extern int num_uart_bytes;
#define NUM_MOTOR_BYTES 5
//#define NUM_PRES_UART_BYTES 	12	//number of uart bytes used for pressure data transmission
//#define NUM_SPI_BYTES 			17	//equal to 5+12, the maximum number of

extern volatile uint8_t control_mode;

uint8_t new_uart_packet;



#define CMD_LED_OFF 			0
#define CMD_LED_ON 				1
#define CMD_CHANGE_IQ			3
/*****************************Commands for distributed position control***********************************/
#define CMD_GET_ENCODER_POS		5	//the flux position of the rotor in ABSOLUTE frame (dh, once unwrap works maybe record offset)
#define CMD_HARD_BRAKE			6
#define CMD_DRIVER_ENABLE		7
#define CMD_DRIVER_DISABLE		8
#define CMD_ROTOR_POS			9
#define CMD_ROTOR_SPEED			10
#define CMD_RESET_T				11	//reset commutation interval estimation
#define CMD_RESET_POS			12
#define CMD_SLEEP				14
#define CMD_PLAY_TONE			15	//play a tone! master data corresponds to


#define CMD_EN_PRES_BARO		19
#define CMD_DIS_PRES			20	//dummy command, useless
#define CMD_READ_PRES			21	//dummy command, useless

#define CMD_NO_ACTION			22
#define CMD_WAKEUP				23
#define CMD_BOOTLOAD        	24

#define CMD_EN_PRES_MAGSENSE	25

#define CMD_CHANGE_IQ_LIM		26

#define CMD_STOP_TONE			27
#define CMD_FORCE_ENCODER		28

#define CMD_CHANGE_POS 			30
#define CMD_CHANGE_KD			31
#define CMD_ZERO_POS 			32
#define CMD_CHANGE_GEAR_CONV	33

#define CMD_EN_CURSENSE			34
#define CMD_SET_POS				35
#define CMD_GET_VERSION			36

uint16_t r_word;
uint16_t t_word;
extern volatile uint8_t r_data[MAX_SPI_BYTES];
extern volatile uint8_t t_data[MAX_SPI_BYTES];
extern volatile floatsend_t rx_format;
extern volatile floatsend_t tx_format;
extern volatile floatsend_t mcur_format;


//uint8_t pres_data[NUM_PRES_UART_BYTES];
extern uint8_t uart_read_buffer[MAX_UART_BYTES];
uint8_t r_flag;
uint8_t t_flag;

int32_t gl_prev_master_duty;

float gl_iq_u;

extern uint32_t uart_it_ts;
extern uint8_t press_data_transmit_flag;
extern uint8_t enable_pressure_flag;

uint8_t sleep_flag;

void uart_print_float(float v);
void print_string(char*str);

void parse_master_cmd();
void execute_master_cmd();

void handle_comms(void);

#endif /* COMM_H_ */
