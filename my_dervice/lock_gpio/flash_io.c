
#include "flash_io.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "app_util_platform.h"
#include "sdk_common.h"


#define TEST_STRING "Nordic"
#define  TRANS_LENGTH  4096
static uint8_t       m_tx_buf[TRANS_LENGTH];           /**< TX buffer. */
static uint8_t       m_rx_buf[TRANS_LENGTH];           /**< RX buffer. */
static const uint16_t m_length ;               /**< Transfer length. */
uint8_t MX25_BUFF[4096];

#define  RANDOM_SEED   106
#define  FLASH_TARGET_ADDR  0x000000


#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
//    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.\r\n");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received: \r\n");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}


void init_mx25l16mb_spi(void)
{
	  ret_code_t err_code;

	nrf_drv_spi_config_t spi_config =  
    {  
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
				.miso_pin     = SPI_MISO_PIN,
				.mosi_pin     = SPI_MOSI_PIN,
				.sck_pin      = SPI_SCK_PIN,			
        .irq_priority = APP_IRQ_PRIORITY_LOW,  
        .orc          = 0xCC,  
        .frequency    = SPI_FREQUENCY_FREQUENCY_M1,  
        .mode         = NRF_DRV_SPI_MODE_0,  
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,  
    };
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL));
		nrf_gpio_cfg_output(SPI_SS_PIN);
}

void device_mx25l16mb_init() 
{
	init_mx25l16mb_spi();
	
	
}

void read_mx25l16_id(void)
{
	  m_tx_buf[0] = FLASH_CMD_RDID;
	

		nrf_gpio_pin_clear(SPI_SS_PIN);  
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 1, m_rx_buf, 4));
		nrf_gpio_pin_set(SPI_SS_PIN); 
		memset(m_rx_buf, 0, m_length);
}

uint8_t read_mx25l16_byte(void)
{	  
	
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf, 1));

    return m_rx_buf[0];	
}

void write_mx25l16_byte(uint8_t data)
{

		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &data, 1, NULL, 0));

	  
}

void read_mx25l16_buf(uint8_t *read_buf, uint32_t flash_address, uint16_t byte_length)
{
	uint8_t temp[4] = {0};
	
	{
		temp[0] = FLASH_CMD_READ;
		temp[1] = (flash_address>> 16);
		temp[2] = (flash_address>> 8);
		temp[3] = (flash_address);
		
		nrf_gpio_pin_clear(SPI_SS_PIN); 

		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, temp, 4, NULL, 0));
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, read_buf, 128));
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, (read_buf+128), 128));
		nrf_gpio_pin_set(SPI_SS_PIN);

	}
	
	memset(m_rx_buf, 0, m_length);
}

	uint16_t i;
void write_mx25lxx_page(uint8_t* write_buff, uint32_t flash_address, uint16_t byte_length)
{

	uint8_t temp[4] = {0};
		
	temp[0] = FLASH_CMD_PP ;           //FLASH_CMD_PP;
	temp[1] = (flash_address>> 16);
	temp[2] = (flash_address>> 8);
	temp[3] = (flash_address);
	
	mx25lxx_write_enable();
	nrf_gpio_pin_clear(SPI_SS_PIN);

	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, temp, 4, NULL, 0));

	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, write_buff, 128, NULL, 0));
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (write_buff+128), 128, NULL, 0));
	
	nrf_gpio_pin_set(SPI_SS_PIN);
	mx25lxx_wait_busy();
}

	uint32_t secpos;
	uint16_t secoff;
	uint16_t secrenum;
	uint16_t i;

void write_mx25l16_buf(uint8_t *write_buf, uint32_t flash_address, uint16_t byte_length)
{

	
	secpos = flash_address/4096;  //计算扇区位置，0，1.2.3....
	secoff = flash_address%4096;  //计算写内容在该扇区多出的长度
	secrenum = 4096 - secoff;
	
	if(byte_length <= secrenum)  //判断写的长度是否大于当前剩余空间长度
	{
		secrenum = byte_length;
	}
	
	while(1)
	{
		read_mx25l16_buf(MX25_BUFF,(secpos*4096),256);
		for(i=0; i<secrenum; i++)
		{
			if(MX25_BUFF[secoff+i]!=0xFF)
			{
				break;
			}
		}
		if(i<secrenum)
		{
			mx25lxx_erase_sector(secpos);
			for(i=0; i<secrenum; i++)
			{
				MX25_BUFF[i+secoff]=write_buf[i];
			}
			write_mx25lxx_nocheck(MX25_BUFF,secpos*4096,4096);
		}
		else
		{
			write_mx25lxx_nocheck(write_buf,flash_address,secrenum);
		}
		
		if(byte_length == secrenum)
		{
			break;
		}
		else
		{ 
			secpos++;
			secoff = 0;			
			write_buf     += secrenum;
			flash_address += secrenum;
			byte_length   -= secrenum;
			
			if(byte_length>4096)
			{
				secrenum = 4096;
			}
			else
			{
				secrenum = byte_length;
			}					
		}
	}
} 



void write_mx25lxx_nocheck(uint8_t* write_buff, uint32_t flash_address, uint16_t byte_length)
{
	uint16_t pageremain;
	
	pageremain = 256 - flash_address%256;
	if(byte_length <= pageremain)
	{
		pageremain = byte_length;
	}
	while(1)
	{
		write_mx25lxx_page(write_buff,flash_address,pageremain);
		if(byte_length == pageremain)
		{
			break;
		}
		else
		{
			write_buff += pageremain;
			flash_address += pageremain;	
			byte_length -= pageremain;			  //减去已经写入了的字节数
			if(byte_length > 256)
			{
				pageremain =256;
			}
			else
			{
				pageremain = byte_length;
			}
		}
	}
}





uint8_t mx25lxx_readsr(void)
{
		uint8_t  temp;
	
		nrf_gpio_pin_clear(SPI_SS_PIN);
		write_mx25l16_byte(FLASH_CMD_RDSR);
		temp = read_mx25l16_byte();
		nrf_gpio_pin_set(SPI_SS_PIN);
	  return temp;
}

void mx25lxx_writesr(uint8_t data)
{
		uint8_t  temp[2] = {0};
	
		temp[0] = FLASH_CMD_WRSR;
		temp[1] = data;
		
		nrf_gpio_pin_clear(SPI_SS_PIN); 		
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, temp, 2, NULL, 0));
		nrf_gpio_pin_set(SPI_SS_PIN);

}


void mx25lxx_write_enable(void)
{

	  nrf_gpio_pin_clear(SPI_SS_PIN); 
		write_mx25l16_byte(FLASH_CMD_WREN); 
		nrf_gpio_pin_set(SPI_SS_PIN);
}

void mx25lxx_write_disable(void)
{

	  nrf_gpio_pin_clear(SPI_SS_PIN); 
	 	write_mx25l16_byte(FLASH_CMD_WRDI); 	
		nrf_gpio_pin_set(SPI_SS_PIN);
}

void mx25lxx_powerdown(void)
{

	  nrf_gpio_pin_clear(SPI_SS_PIN);  
	 	write_mx25l16_byte(FLASH_CMD_DP); 
		nrf_gpio_pin_set(SPI_SS_PIN);
}


void mx25lxx_wakeup(void)	
{
	  nrf_gpio_pin_clear(SPI_SS_PIN);
   	write_mx25l16_byte(FLASH_CMD_RDP); 
		nrf_gpio_pin_set(SPI_SS_PIN);
}

void mx25lxx_wait_busy(void)
{
	 while((mx25lxx_readsr()&0x01) == 0x01);
}


void mx25lxx_erase_sector(uint32_t data_addr)	
{
	uint8_t  temp[4] = {0};
	
	temp[0] = FLASH_CMD_SE;
	temp[1] = (data_addr>> 16);
	temp[2] = (data_addr>> 8);
	temp[3] = (data_addr);

	mx25lxx_write_enable();
	mx25lxx_wait_busy();
	nrf_gpio_pin_clear(SPI_SS_PIN); 
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, temp, 4, NULL, 0));
	nrf_gpio_pin_set(SPI_SS_PIN);
	mx25lxx_wait_busy();
}

void mx25lxx_erase_chip(void)
{
	uint8_t temp1;
	
	temp1 = FLASH_CMD_CE;
	mx25lxx_write_enable();
	mx25lxx_wait_busy();
	nrf_gpio_pin_clear(SPI_SS_PIN); 
	write_mx25l16_byte(FLASH_CMD_CE);		
	nrf_gpio_pin_set(SPI_SS_PIN);
	mx25lxx_wait_busy();
}



void read_mx25_test(void)
{

/*************
	0x00-----0xff   1
	0x100----0x1ff   2
	0x200----0x2ff   3
 	0x300----0x3ff   4
	0x400----0x4ff   5
	0x500----0x5ff   6
	0x600----0x6ff   7
	0x700----0x7ff   8
	0x800----0x8ff   9
	0x900----0x9ff   10
	0xa00----0xaff   11
	0xb00----0xbff   12
	0xc00----0xcff   13
	0xd00----0xdff   14
	0xe00----0xeff   15
  0xf00----0xfff   16	
*********/
//	  read_mx25l16_id();

		for(uint16_t i = 0; i < TRANS_LENGTH; i++)
		{
			 m_tx_buf[i] = 0xaa;
		}
//	 write_mx25l16_buf(m_tx_buf,FLASH_TARGET_ADDR+0x800,256);
	  read_mx25l16_buf(m_rx_buf,FLASH_TARGET_ADDR+0x800,256);
	
		memset(m_rx_buf, 0, m_length);
		

		    /* Erase 4K sector of flash memory
       Note: It needs to erase dirty sector before program */
		memset(m_rx_buf, 0, m_length);
}


