/*
 * spi_slave.h
 *
 *  Created on: Jan 13, 2021
 *      Author: Ocanath
 */

#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_
#include "init.h"

HAL_StatusTypeDef start_spi_slave_rxtx_frame(SPI_HandleTypeDef *hspi, uint8_t * tx, uint8_t * rx, int Size);
void spi_tx_rx_frame_complete(SPI_HandleTypeDef*hspi);
void reset_rx_frame(void);
void reset_tx_frame(void);

#endif /* SPI_SLAVE_H_ */
