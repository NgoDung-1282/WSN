#include "lora.h"
#include <string.h>

extern TIM_HandleTypeDef htim4;

uint8_t packetIndex;
/*read register*/
uint8_t lora_read_reg(lora_t * module, uint8_t addr) {
	uint8_t txByte = addr & 0x7f;
	uint8_t rxByte = 0x00;
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(module->pin->spi, &txByte, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	
	HAL_SPI_Receive(module->pin->spi,&rxByte, 1, 1000);
	while(HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
	return rxByte;
}

/*write register*/
void lora_write_reg(lora_t * module, uint8_t addr, uint8_t cmd){
	uint8_t add = addr | 0x80;
  HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(module->pin->spi, &add, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(module->pin->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
}

uint8_t lora_init(lora_t * module){
	uint8_t ret;
	/*reset chip*/
	HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_SET);
  HAL_Delay(10);
	
	/*check ID version*/
	ret = lora_read_reg(module, REG_VERSION);
	if(ret != 0x12){
		return 1;
	}
	
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_SLEEP));		/*Lora TM sleep mode*/
	lora_set_frequency(module, FREQUENCY[module->frequency]);										/*setting frequency*/
	lora_write_reg(module, REG_SYNC_WORD, 0x34);																/*setting LoRaWAN network*/
	lora_write_reg(module, REG_PA_CONFIG, SX1278_Power[module->power]);					/*Setting output power parameter*/
	lora_write_reg(module, REG_OCP, 0x0B);																			/*RegOcp,Close Ocp*/
	lora_write_reg(module, REG_LNA, 0x23);																			/*RegLNA,High & LNA Enable*/
	
	if (SX1278_SpreadFactor[module->LoRa_SF] == 6) {														/*SFactor=6*/
		//Implicit Enable, CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		lora_write_reg(module, REG_MODEM_CONFIG_1, ((SX1278_LoRaBandwidth[module->LoRa_BW] << 4) + (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x01)); 

		lora_write_reg(module, REG_MODEM_CONFIG_2, ((SX1278_SpreadFactor[module->LoRa_SF] << 4) + (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

		lora_write_reg(module, REG_DETECTION_OPTIMIZE, 0x05);
		lora_write_reg(module, REG_DETECTION_THRESHOLD, 0x0C);
	} else {
		//Explicit Enable, CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		lora_write_reg(module, REG_MODEM_CONFIG_1, ((SX1278_LoRaBandwidth[module->LoRa_BW] << 4) + (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x00));
		//SFactor &  LNA gain set by the internal AGC loop		
		lora_write_reg(module, REG_MODEM_CONFIG_2, ((SX1278_SpreadFactor[module->LoRa_SF] << 4) + (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00)); 
	}
	
	lora_write_reg(module, REG_FIFO_TX_BASE_ADDR, 0);															/*write base address in FIFO data buffer for TX modulator = 0*/
	lora_write_reg(module, REG_FIFO_RX_BASE_ADDR, 0);															/*read base address in FIFO data buffer for RX demodulator = 0*/
	lora_write_reg(module, REG_MODEM_CONFIG_3, 0x04);															/*ENABLE Low Data Rate Optimize*/
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_STDBY));			/*Lora TM standby mode*/
	return 0;
}

/*rx init*/
int lora_rx_init(lora_t * module) {
	int packetLength = 0, irqFlags;
	irqFlags = lora_read_reg(module, REG_IRQ_FLAGS);																						/*get RegIrqFlags*/
	lora_write_reg(module, REG_IRQ_FLAGS, irqFlags);																						/*clear RegIrqFlags*/	
	if((lora_read_reg(module, REG_OP_MODE)) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);
		lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}
	if((irqFlags & IRQ_RX_DONE_MASK)) {																													/*RxDone*/
		lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
		if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0){																					/*no PayloadCrcError*/
		packetLength = lora_read_reg(module, REG_RX_NB_BYTES);
		lora_write_reg(module, REG_FIFO_ADDR_PTR, lora_read_reg(module, REG_FIFO_RX_CURRENT_ADDR));		
		packetIndex = 0;				
		}
		else if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)== IRQ_PAYLOAD_CRC_ERROR_MASK){						/*yes PayloadCrcError*/
			return -1;
		}
	}
	else																																												/*RxTimeout*/
	{
		return -1;
	}	
	return packetLength;
}

/*receive data*/
void lora_receive(lora_t * module, uint8_t * rxbuf){
	uint8_t i=0;
	while(lora_read_reg(module, REG_RX_NB_BYTES) - packetIndex){
		rxbuf[i] = lora_read_reg(module, REG_FIFO);
		packetIndex++;
		i++;
	}
	rxbuf[i] = '\0';
}



/*Tx init*/
uint8_t lora_tx_init(lora_t * module){
	if ((lora_read_reg(module, REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return 1;
  }
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);			/*lora standby mode*/
	lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);					/*dua con tro den vi tri dau cua bo dem fifo*/
  lora_write_reg(module, REG_PAYLOAD_LENGTH, 0);
	return 0;
}


/*write data fifo*/
void lora_write_fifo(lora_t * module, uint8_t * txbuf, uint8_t size){
	int currentLength = lora_read_reg(module, REG_PAYLOAD_LENGTH);
  if ((currentLength + size > MAX_PKT_LENGTH)){
    size = MAX_PKT_LENGTH - currentLength;
  }

  for (int i = 0; i < size; i++) {
    lora_write_reg(module, REG_FIFO, txbuf[i]);
  }
  lora_write_reg(module, REG_PAYLOAD_LENGTH, currentLength + size);
}

/*wait for IRQ TxDone*/
uint8_t lora_tx_end(lora_t * module, uint32_t timeout){
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((lora_read_reg(module,REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
		if(timeout==0){
			return 1;
		}
		timeout--;
		HAL_Delay(1);
  }
  lora_write_reg(module, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);		/*clear TxDone Flag*/
	return 0;
}

/*transmit data*/
void lora_transmit(lora_t * module, uint8_t * txbuf, uint8_t size, uint32_t timeout){
//	lora_tx_init(module);
	lora_write_fifo(module, txbuf, size);
	while(lora_tx_end(module, timeout)){}
}

void lora_set_frequency(lora_t * module, uint64_t freq){
	uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  lora_write_reg(module, REG_FRF_MSB, (uint8_t)(frf >> 16));
  lora_write_reg(module,REG_FRF_MID, (uint8_t)(frf >> 8));
  lora_write_reg(module,REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_setPreamble(lora_t *module, uint16_t preamble)
{
    
    lora_write_reg(module, 0x20, 0);
    lora_write_reg(module, 0x21, 8);
   
}



