#ifndef __LORA_H__
#define __LORA_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"

//#include "delay.h"


#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP									 0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

#define LORA_SS_PIN    NSS_Pin
#define LORA_RESET_PIN RESET_Pin
//#define LORA_DIO0_PIN  GPIO_PIN_10

#define LORA_SS_PORT    NSS_GPIO_Port
#define LORA_RESET_PORT RESET_GPIO_Port
//#define LORA_DIO0_PORT  GPIOB

typedef struct {
	int pin;
	void * port;
} lora_gpio_t;

typedef struct {
	lora_gpio_t reset;
	lora_gpio_t dio0;
	lora_gpio_t nss;
	void * spi;
} lora_pins_t;

typedef struct {
	lora_pins_t * pin;
	uint8_t frequency;
	uint8_t power;
	uint8_t LoRa_SF;
	uint8_t LoRa_BW;
	uint8_t LoRa_CR;
	uint8_t LoRa_CRC_sum;
} lora_t;

/*setting prequency*/
#define FREQ_433MHZ			0
#define FREQ_865MHZ			1
#define FREQ_866MHZ			2
#define FREQ_867MHZ			3

static const uint64_t FREQUENCY[4] = { 433E6, 865E6, 866E6, 867E6}; 

/*setting power*/
#define SX1278_POWER_20DBM		0
#define SX1278_POWER_17DBM		1
#define SX1278_POWER_14DBM		2
#define SX1278_POWER_11DBM		3

static const uint8_t SX1278_Power[4] = { 0xFF, //20dbm
		0xFC, //17dbm
		0xF9, //14dbm
		0xF6, //11dbm
		};

/*setting spread factor*/
#define SX1278_LORA_SF_6		0
#define SX1278_LORA_SF_7		1
#define SX1278_LORA_SF_8		2
#define SX1278_LORA_SF_9		3
#define SX1278_LORA_SF_10		4
#define SX1278_LORA_SF_11		5
#define SX1278_LORA_SF_12		6

static const uint8_t SX1278_SpreadFactor[7] = { 6, 7, 8, 9, 10, 11, 12 };

/*setting bandwidth*/
#define	SX1278_LORA_BW_7_8KHZ			0
#define	SX1278_LORA_BW_10_4KHZ		1
#define	SX1278_LORA_BW_15_6KHZ		2
#define	SX1278_LORA_BW_20_8KHZ		3
#define	SX1278_LORA_BW_31_2KHZ		4
#define	SX1278_LORA_BW_41_7KHZ		5
#define	SX1278_LORA_BW_62_5KHZ		6
#define	SX1278_LORA_BW_125KHZ			7
#define	SX1278_LORA_BW_250KHZ			8
#define	SX1278_LORA_BW_500KHZ			9

static const uint8_t SX1278_LoRaBandwidth[10] = { 0, //   7.8KHz,
		1, //  10.4KHz,
		2, //  15.6KHz,
		3, //  20.8KHz,
		4, //  31.2KHz,
		5, //  41.7KHz,
		6, //  62.5KHz,
		7, // 125.0KHz,
		8, // 250.0KHz,
		9  // 500.0KHz
		};

/*setting Coding rate*/
#define SX1278_LORA_CR_4_5    0
#define SX1278_LORA_CR_4_6    1
#define SX1278_LORA_CR_4_7    2
#define SX1278_LORA_CR_4_8    3

static const uint8_t SX1278_CodingRate[4] = { 0x01, 0x02, 0x03, 0x04 };

/*setting CRC Enable*/
#define SX1278_LORA_CRC_EN              0
#define SX1278_LORA_CRC_DIS             1

static const uint8_t SX1278_CRC_Sum[2] = { 0x01, 0x00 };

uint8_t lora_read_reg(lora_t * module, uint8_t addr);
void lora_write_reg(lora_t * module, uint8_t addr, uint8_t cmd);
uint8_t lora_init(lora_t * module);
int lora_rx_init(lora_t * module);
void lora_receive(lora_t * module, uint8_t * rxbuf);
uint8_t lora_tx_init(lora_t * module);
void lora_write_fifo(lora_t * module, uint8_t * txbuf, uint8_t size);
uint8_t lora_tx_end(lora_t * module, uint32_t timeout);
void lora_transmit(lora_t * module, uint8_t * txbuf, uint8_t size, uint32_t timeout);
void lora_set_frequency(lora_t * module, uint64_t freq);
void lora_setPreamble(lora_t *module, uint16_t preamble);

#endif

