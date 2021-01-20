/*
 * spi_slave.c
 *
 *  Created on: Jan 13, 2021
 *      Author: Ocanath
 */
#include "spi_slave.h"
#include "stm32f3xx_hal_spi.h"

typedef union
{
	uint16_t v;
	uint8_t d[sizeof(uint16_t)/sizeof(uint8_t)];
}uint16_fmt_t;

static volatile uint16_fmt_t tx_fmt = {0};
static volatile uint16_fmt_t rx_fmt = {0};

static volatile int rx_buf_idx = 0;
static volatile int tx_buf_idx = 0;

void reset_rx_frame(void)
{
	rx_buf_idx = 0;
}
void reset_tx_frame(void)
{
	tx_buf_idx = 0;
}

static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
        on both master and slave sides in order to resynchronize the master
        and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)                                            || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

        }

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}
/**
  * @brief Handle SPI FIFO Communication Timeout.
  * @param hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Fifo Fifo to check
  * @param State Fifo state to check
  * @param Timeout Timeout duration
  * @param Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Fifo, uint32_t State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  __IO uint8_t tmpreg;

  while ((hspi->Instance->SR & Fifo) != State)
  {
    if ((Fifo == SPI_SR_FRLVL) && (State == SPI_FRLVL_EMPTY))
    {
    	READ_REG(*((__IO uint8_t *)&hspi->Instance->DR));
    }

    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
  /* Control if the TX fifo is empty */
  if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FTLVL, SPI_FTLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    return HAL_TIMEOUT;
  }

  /* Control the BSY flag */
  if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    return HAL_TIMEOUT;
  }

  /* Control if the RX fifo is empty */
  if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    return HAL_TIMEOUT;
  }
  return HAL_OK;
}


void spi_tx_rx_frame_complete(SPI_HandleTypeDef*hspi)
{
	uint32_t tickstart = 0U;

	/* Init tickstart for timeout managment*/
	tickstart = HAL_GetTick();

	/* Disable ERR interrupt */
	__HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

	/* Check the end of the transaction */
	if (SPI_EndRxTxTransaction(hspi, 100U, tickstart) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}
	if (hspi->ErrorCode == HAL_SPI_ERROR_NONE)
	{
		if (hspi->State == HAL_SPI_STATE_BUSY_RX)
		{
			hspi->State = HAL_SPI_STATE_READY;
			HAL_SPI_RxCpltCallback(hspi);
		}
		else
		{
			hspi->State = HAL_SPI_STATE_READY;
			HAL_SPI_TxRxCpltCallback(hspi);
		}
	}
	else
	{
		hspi->State = HAL_SPI_STATE_READY;
		HAL_SPI_ErrorCallback(hspi);
	}
}

static void spi_slave_rx_ISR(struct __SPI_HandleTypeDef *hspi)
{

//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0)
//	{
//		/* Receive data in packing mode */
//		uint16_fmt_t data;
//		data.v = hspi->Instance->DR;
//		hspi->pRxBuffPtr[rx_buf_idx] = data.d[0];
//		hspi->pRxBuffPtr[rx_buf_idx+1] = data.d[1];
//		rx_buf_idx += 2;
//		if(rx_buf_idx >= hspi->RxXferSize)
//			rx_buf_idx = 0;
//	}

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0)
	{
		if (hspi->RxXferCount > 1U)
		{
			/* Receive data in packing mode */
//			uint16_fmt_t data;
//			data.v = hspi->Instance->DR;
//			hspi->pRxBuffPtr[rx_buf_idx] = data.d[0];
//			hspi->pRxBuffPtr[rx_buf_idx+1] = data.d[1];
			*((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)(hspi->Instance->DR);
			hspi->pRxBuffPtr += sizeof(uint16_t);
			hspi->RxXferCount -= 2U;
			rx_buf_idx += 2U;

			if (hspi->RxXferCount == 1U)
			{
				/* Set RX Fifo threshold according the reception data length: 8bit */
				SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
			}
		}

		else
		{	/*Recieve the left over 8 bit*/
			*hspi->pRxBuffPtr = *((__IO uint8_t *)&hspi->Instance->DR);
		    hspi->pRxBuffPtr++;
		    rx_buf_idx++;
		    hspi->RxXferCount--;
		}
	}

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != 0) || (hspi->RxXferCount == 0U))
	{
		hspi->RxXferCount = 0U;
		hspi->pRxBuffPtr=hspi->pRxBuffPtr-rx_buf_idx*sizeof(uint8_t);
		/* Disable RXNE  and ERR interrupt */
		__HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

		if (hspi->TxXferCount == 0U)
		{
			spi_tx_rx_frame_complete(hspi);
		}
	}
}

static void spi_slave_tx_ISR(struct __SPI_HandleTypeDef *hspi)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0)
	{
		//hspi->Instance->DR = 0xFFAA; //16 bit, should be from buffer
			/* Transmit data in packing Bit mode */
		if (hspi->TxXferCount >= 2U)
		{
			hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
			hspi->pTxBuffPtr += sizeof(uint16_t);
			hspi->TxXferCount -= 2U;
		}
			  /* Transmit data in 8 Bit mode */
		else
		{
			*(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
			hspi->pTxBuffPtr++;
			hspi->TxXferCount--;
		}
	}

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != 0) || (hspi->TxXferCount == 0U))
	{
		hspi->TxXferCount = 0U;
		/* Disable TXE interrupt */
		__HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

		if (hspi->RxXferCount == 0U)
		{
			spi_tx_rx_frame_complete(hspi);
		}
	}
}

HAL_StatusTypeDef start_spi_slave_rxtx_frame(SPI_HandleTypeDef *hspi, uint8_t * tx, uint8_t * rx, int Size)
{
	uint32_t tmp = 0U, tmp1 = 0U;
	HAL_StatusTypeDef errorcode = HAL_OK;


	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	/* Process locked */
	__HAL_LOCK(hspi);

	tmp  = hspi->State;
	tmp1 = hspi->Init.Mode;

	if (!((tmp == HAL_SPI_STATE_READY) ||
			((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX))))
	{
		errorcode = HAL_BUSY;
		goto error;
	}

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State != HAL_SPI_STATE_BUSY_RX)
	{
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/* Set the transaction information */
	hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
	hspi->pTxBuffPtr  = (uint8_t *)tx;
	hspi->TxXferSize  = Size;
	hspi->TxXferCount = Size;
	hspi->pRxBuffPtr  = (uint8_t *)rx;
	hspi->RxXferSize  = Size;
	hspi->RxXferCount = Size;

	/* Set the function for IT treatment */

	hspi->RxISR     = spi_slave_rx_ISR;
	hspi->TxISR     = spi_slave_tx_ISR;

	/* Check if packing mode is enabled and if there is more than 2 data to receive */
	if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT) || (hspi->RxXferCount >= 2U))
	{
		/* Set fiforxthresold according the reception data length: 16 bit */
		CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	}
	else
	{
		/* Set fiforxthresold according the reception data length: 8 bit */
		SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	}

	/* Enable TXE, RXNE and ERR interrupt */
	__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}

	error :
	/* Process Unlocked */
	__HAL_UNLOCK(hspi);
	return errorcode;
}
