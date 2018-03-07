
#ifndef __FLASH_IO_H_
#define __FLASH_IO_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"

/*** MX25 series command hex code definition ***/
//ID comands
#define    FLASH_CMD_RDID      0x9F    //RDID (Read Identification)
#define    FLASH_CMD_RES       0xAB    //RES (Read Electronic ID)
#define    FLASH_CMD_REMS      0x90    //REMS (Read Electronic & Device ID)

//Register comands
#define    FLASH_CMD_WRSR      0x01    //WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      0x05    //RDSR (Read Status Register)
#define    FLASH_CMD_WRSCUR    0x2F    //WRSCUR (Write Security Register)
#define    FLASH_CMD_RDSCUR    0x2B    //RDSCUR (Read Security Register)

//READ comands
#define    FLASH_CMD_READ        0x03    //READ (1 x I/O)
#define    FLASH_CMD_FASTREAD    0x0B    //FAST READ (Fast read data)
#define    FLASH_CMD_DREAD       0x3B    //DREAD (1In/2 Out fast read)
#define    FLASH_CMD_RDSFDP      0x5A    //RDSFDP (Read SFDP)

//Program comands
#define    FLASH_CMD_WREN     0x06    //WREN (Write Enable)
#define    FLASH_CMD_WRDI     0x04    //WRDI (Write Disable)
#define    FLASH_CMD_PP       0x02    //PP (page program)

//Erase comands
#define    FLASH_CMD_SE       0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE       0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       0x60    //CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define    FLASH_CMD_DP       0xB9    //DP (Deep Power Down)
#define    FLASH_CMD_RDP      0xAB    //RDP (Release form Deep Power Down)
#define    FLASH_CMD_ENSO     0xB1    //ENSO (Enter Secured OTP)
#define    FLASH_CMD_EXSO     0xC1    //EXSO  (Exit Secured OTP)
#ifdef   SBL_CMD_0x77
#else
#endif

//Reset comands

//Security comands
#ifdef LCR_CMD_0xDD_0xD5
#else
#endif


#define SPI_SS_PIN         28  //!< Slave SCL pin
#define SPI_MISO_PIN       29//!< Slave SDA pin
#define SPI_MOSI_PIN       25   //irq
#define SPI_SCK_PIN        24

typedef uint8_t boolean;
void device_mx25l16mb_init(void);
void read_mx25_test(void);
void mx25lxx_erase_chip(void);
void mx25lxx_erase_sector(uint32_t data_addr);
void mx25lxx_wakeup(void);
void mx25lxx_powerdown(void);
void mx25lxx_write_disable(void);
void mx25lxx_write_enable(void);
void mx25lxx_writesr(uint8_t data);
uint8_t mx25lxx_readsr(void);
uint8_t cmd_rdsr(uint8_t *StatusReg);
void mx25lxx_wait_busy(void);
void write_mx25l16_buf(uint8_t *write_buf, uint32_t flash_address, uint16_t byte_length);
void read_mx25l16_buf(uint8_t *read_buf, uint32_t flash_address,  uint16_t byte_length);
void write_mx25lxx_page(uint8_t* write_buff, uint32_t flash_address, uint16_t byte_length);
void write_mx25lxx_nocheck(uint8_t* write_buff, uint32_t flash_address, uint16_t byte_length);

















#endif
