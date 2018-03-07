#include "pn532_i2c.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"



uint8_t _irq, _reset;
uint8_t _uid[7];  // ISO14443A uid
uint8_t _uidLen;  // uid len
uint8_t _key[6];  // Mifare Classic key
uint8_t inListedTag; // Tg number of inlisted tag.
	
uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

uint8_t pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define PN532_PACKBUFFSIZ 64

uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];
static size_t m_addr = 0;
//static uint8_t m_rxbuff[1 + EEPROM_SIM_SEQ_WRITE_MAX];
static bool m_error_flag;
//static const nrf_drv_twis_t m_twi = NRF_DRV_TWIS_INSTANCE(EEPROM_SIM_TWIS_INST);


nrf_drv_twi_t gtMpuTwi = NRF_DRV_TWI_INSTANCE(1);
//APP_TIMER_DEF(gtMpuReadTimer);

 void init_pnc523_i2c(void)
{
	nrf_drv_twi_config_t ltMpuTwiCfg = NRF_DRV_TWI_DEFAULT_CONFIG;
	ltMpuTwiCfg.scl = PN532_CONFIG_SCL;
	ltMpuTwiCfg.sda = PN532_CONFIG_SDA;
	ltMpuTwiCfg.frequency = TWI_FREQUENCY_FREQUENCY_K400;

	nrf_drv_twi_init(&gtMpuTwi, &ltMpuTwiCfg, NULL, NULL);
	nrf_drv_twi_enable(&gtMpuTwi);
}

/******************************************************************************* 
 * ???? :   i2c_device_wirte_data                                                                  
 * ??     :   wang                                                      
 *                                                                                
 * ?   ?  :  void                                                                    
 * ?   ?  :  void 
 * ?   ?  :  void                                                             
 * ???? : 20160317                                                                     
 *******************************************************************************/  


void i2c_write_byte(uint8_t address, uint8_t data)
{  
        uint8_t lau8Data[2] = {0};
				lau8Data[0] = address;
				lau8Data[1] = data;
				nrf_drv_twi_tx(&gtMpuTwi, PN532_I2C_ADDRESS, lau8Data, 2, false); 
}  

void i2c_write_buffer(uint8_t address, uint8_t *data,uint8_t len)
{  
		nrf_drv_twi_tx(&gtMpuTwi, PN532_I2C_ADDRESS, data, len, false); 
}  


uint8_t i2c_device_read_byte(uint8_t data)  
{     
			uint8_t lu8Data = {0};
			lu8Data = data;

//			nrf_drv_twi_tx(&gtMpuTwi, PN532_I2C_ADDRESS, &lu8Data, 1, true);
			nrf_drv_twi_rx(&gtMpuTwi, PN532_I2C_ADDRESS, &lu8Data, 1);
			return lu8Data;
} 

void i2c_device_read_buffer(uint8_t address, uint8_t* data, uint8_t data_Len)
{
//	   nrf_drv_twi_tx(&gtMpuTwi, PN532_I2C_ADDRESS, &address, 1, false);
	   nrf_drv_twi_rx(&gtMpuTwi, PN532_I2C_ADDRESS, data, data_Len);
}


void pn532_gpio_init(void)
{
	//			nrf_gpio_cfg_output(ARDUINO_10_PIN);  //reset  24
			    nrf_gpio_cfg_input(PN532_IRQ,GPIO_PIN_CNF_PULL_Disabled); //irq  25
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
void begin() 
{


//	nrf_gpio_pin_set(PN532_RESET);
//	nrf_gpio_pin_clear(PN532_RESET);
//	nrf_delay_us(4000);
//	nrf_gpio_pin_set(PN532_RESET);
}


/**************************************************************************/
/*! 
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t getFirmwareVersion(void) 
{
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
//	printf("1read data packet\r\n");
  if (! sendCommandCheckAck(pn532_packetbuffer, 1,1000)){
//			printf("1read data packet failed\r\n");
      return 0;
	  }
      nrf_delay_ms(2);
			// read data packet
//		 printf("read data packet ok\r\n");
			wirereaddata(pn532_packetbuffer, 12);
//			printf("2read data packet\r\n");
			// check some basic stuff
			if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {

    return 0;
  }
  
  response = pn532_packetbuffer[7];
  response <<= 8;
  response |= pn532_packetbuffer[8];
  response <<= 8;
  response |= pn532_packetbuffer[9];
  response <<= 8;
  response |= pn532_packetbuffer[10];

  return response;
}


/**************************************************************************/
/*! 
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes 
    @param  timeout   timeout before giving up
    
    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
boolean sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) 
{
  uint16_t timer = 0;
  
  // write the command
//	printf("%2x %2x %2x %2x\r\n",cmd[0],cmd[1],cmd[2],cmd[3]);
//	printf("cmdlen = %2x \r\n",cmdlen);
  wiresendcommand(cmd, cmdlen);
//  printf("write the command\r\n");

  // Wait for chip to say its ready!
  while (wirereadstatus() != PN532_I2C_READY) {
    if (timeout != 0) {
					timer+=1;
					nrf_delay_us(100);
					if (timer > timeout)  
						return false;
    }

  }
  // read acknowledgement
//    printf("sendCommandCheckAck ok\r\n");
//   nrf_delay_us(1000);
  if (!readackframe()) {
    
    return false;
  }

  return true; // ack'd command
}


/**************************************************************************/
/*! 
    Writes an 8-bit value that sets the state of the PN532's GPIO pins
    
    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.
    
             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t writeGPIO(uint8_t pinstate) 
{
  uint8_t errorbit;

  // Make sure pinstate does not try to toggle P32 or P34
  pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);
  
  // Fill command buffer
  pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
  pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
  pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by I2C)



  // Send the WRITEGPIO command (0x0E)  
  if (! sendCommandCheckAck(pn532_packetbuffer, 3,1000))
    return 0x0;
  
  // Read response packet (00 00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0F) DATACHECKSUM)
  wirereaddata(pn532_packetbuffer, 8);

    
  
  return  (pn532_packetbuffer[6] == 0x0F);
}


/**************************************************************************/
/*! 
    Reads the state of the PN532's GPIO pins
    
    @returns An 8-bit value containing the pin state where:
    
             pinState[0]  = P30     
             pinState[1]  = P31     
             pinState[2]  = P32     
             pinState[3]  = P33     
             pinState[4]  = P34     
             pinState[5]  = P35     
*/
/**************************************************************************/
uint8_t readGPIO(void) 
{
  pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

  // Send the READGPIO command (0x0C)  
  if (! sendCommandCheckAck(pn532_packetbuffer, 1,1000))
    return 0x0;
  
  // Read response packet (00 00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0D) P3 P7 IO1 DATACHECKSUM)
  wirereaddata(pn532_packetbuffer, 11);


  return pn532_packetbuffer[6];
}

/**************************************************************************/
/*! 
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
uint8_t SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!
//	 printf("SAMConfig\r\n");
  if (! sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
     return false;
  // read data packet
//	printf("SAMConfig read data packet \r\n");
  wirereaddata(pn532_packetbuffer, 8);
  
  return  (pn532_packetbuffer[6] == 0x15);
}

/***** ISO14443B Commands ******/

uint8_t SetRFConfiguration(void)
{

	
	{
			pn532_packetbuffer[0] = PN532_COMMAND_SETPARAMETERS;
			pn532_packetbuffer[1] = 0x14; // normal mode;

			//	 printf("SAMConfig\r\n");
			if (! sendCommandCheckAck(pn532_packetbuffer, 2, 1000))
			return false;
			// read data packet
			//	printf("SAMConfig read data packet \r\n");
			nrf_delay_ms(150);
			wirereaddata(pn532_packetbuffer, 8);
			if(!(pn532_packetbuffer[6] == 0x13))
			return false;
	}

	{
		  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
			pn532_packetbuffer[1] = 0x01; // normal mode;
			pn532_packetbuffer[2] = 0x00; // timeout 50ms * 20 = 1 second
			//	 printf("SAMConfig\r\n");
			if (! sendCommandCheckAck(pn532_packetbuffer, 3, 1000))
			return false;
			// read data packet
			//	printf("SAMConfig read data packet \r\n");
			nrf_delay_ms(150);
			wirereaddata(pn532_packetbuffer, 8);

			if(!(pn532_packetbuffer[6] == 0x33))
			return false;
	}
//    //   my_memset(pn532_packetbuffer,0,64);
	{
		  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
		 	pn532_packetbuffer[1] = 0x01; // normal mode;
			pn532_packetbuffer[2] = 0x01; // timeout 50ms * 20 = 1 second
			//	 printf("SAMConfig\r\n");
			if (! sendCommandCheckAck(pn532_packetbuffer, 3, 1000))
			return false;
			// read data packet
			//	printf("SAMConfig read data packet \r\n");
			nrf_delay_ms(150);
			wirereaddata(pn532_packetbuffer, 8);

			if(!(pn532_packetbuffer[6] == 0x33))
			return false;
	}
//	  //  my_memset(pn532_packetbuffer,0,64);
	{
		  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
			pn532_packetbuffer[1] = 0x05; // normal mode;
			pn532_packetbuffer[2] = 0xff; // timeout 50ms * 20 = 1 second
		  pn532_packetbuffer[3] = 0xff; // normal mode;
			pn532_packetbuffer[4] = 0xff; // timeout 50ms * 20 = 1 second
			//	 printf("SAMConfig\r\n");
			if (! sendCommandCheckAck(pn532_packetbuffer, 5, 1000))
			return false;
			// read data packet
			//	printf("SAMConfig read data packet \r\n");
			nrf_delay_ms(150);
			wirereaddata(pn532_packetbuffer, 8);

			if(!(pn532_packetbuffer[6] == 0x33))
			return false;
	}
	
	
}
uint8_t CategoryBConfig(void)
{
	/*******40****/
	  my_memset(pn532_packetbuffer,0,64);
	  pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
		uint8_t config_b1[16]={0x63 ,0x01 ,0xff ,0x63 ,0x02 ,0x03 ,0x63 ,0x03 ,0x03 ,0x63 ,0x05 ,0x00 ,0x63 ,0x08 ,0x4d, 0x63};
		uint8_t config_b2[16]={0x09 ,0x4d ,0x63 ,0x0d ,0x10 ,0x63 ,0x0e ,0x03 ,0x63 ,0x14 ,0x68 ,0x63 ,0x17 ,0xff ,0x63, 0x18};
		uint8_t config_b3[7]={0x3f ,0x63 ,0x19 ,0x18 ,0x63 ,0x3c ,0x10};
		uint8_t num_add = 1;
		 for(uint8_t i = 0; i<16; i++)
		 {
			 pn532_packetbuffer[num_add] = config_b1[i];
			 num_add++;
		 }
		 
		 for(uint8_t i = 0; i<16; i++)
		 {
			 pn532_packetbuffer[num_add] = config_b2[i];
			 num_add++;
		 }
		 
		 for(uint8_t i = 0; i<9; i++)
		 {
			 pn532_packetbuffer[num_add] = config_b3[i];
			 num_add++;
		 }
		 
		 if (! sendCommandCheckAck(pn532_packetbuffer, 40, 1000))
			 return false;
		 nrf_delay_ms(100);
    wirereaddata(pn532_packetbuffer, 8);
  
    return  (pn532_packetbuffer[6] == 0x09);
}
/*
*
*This command is used to set internal parameters of the PN532, and then to configure its
behavior regarding different cases.
*
*
*/
uint8_t SetParameters(void) 
	{
	my_memset(pn532_packetbuffer,0,64);
  pn532_packetbuffer[0] = PN532_COMMAND_SETPARAMETERS;
  pn532_packetbuffer[1] = 0x04; // normal mode;

//	 printf("SAMConfig\r\n");
  if (! sendCommandCheckAck(pn532_packetbuffer, 2, 1000))
     return false;
  // read data packet
//	printf("SAMConfig read data packet \r\n");
	 nrf_delay_ms(100);
  wirereaddata(pn532_packetbuffer, 8);
  
  return  (pn532_packetbuffer[6] == 0x13);
}
	

uint8_t readTypeBuid(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout) 
{
	my_memset(pn532_packetbuffer,0,64);
	uint8_t read_config1[5] = {0x05, 0x00, 0x00, 0x71, 0xff};
	uint8_t read_config2[11] = {0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01, 0x01, 0x32, 0x8d};
	uint8_t read_config3[7] = {0x00, 0x36, 0x00, 0x00, 0x08, 0x57, 0x44};
	{	
		pn532_packetbuffer[0] = PN532_COMMAND_INCOMMUNICATETHRU;
		for(uint8_t i = 0; i<5; i++)
		{
		pn532_packetbuffer[i+1] = read_config1[i];
		}
		if (! sendCommandCheckAck(pn532_packetbuffer, 6, 1000))
		 return false;
		// read data packet
		nrf_delay_ms(100);
		wirereaddata(pn532_packetbuffer, 20);
		//my_memset(pn532_packetbuffer,0,64);
	
	}
	{
		pn532_packetbuffer[0] = PN532_COMMAND_INCOMMUNICATETHRU;
		for(uint8_t i = 0; i<11; i++)
		{
		pn532_packetbuffer[i+1] = read_config2[i];
		}
		if (! sendCommandCheckAck(pn532_packetbuffer, 12, 1000))
		 return false;
		// read data packet
		nrf_delay_ms(100);
		wirereaddata(pn532_packetbuffer, 20);
  }
	//my_memset(pn532_packetbuffer,0,64);
	{
		pn532_packetbuffer[0] = PN532_COMMAND_INCOMMUNICATETHRU;
		for(uint8_t i = 0; i<7; i++)
		{
			pn532_packetbuffer[i+1] = read_config3[i];
		}
		if (! sendCommandCheckAck(pn532_packetbuffer, 8, 1000))
		 return false;
		// read data packet
		nrf_delay_ms(150);
		wirereaddata(pn532_packetbuffer, 20);
		
		for(uint8_t i = 0; i < 9; i++)
		{
			uid[i] = pn532_packetbuffer[i+8];
		}
		
		*uidLength = 8;
		
		//	printf("1111111111");
  }
	
	
	
}
/**************************************************************************/
/*! 
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register
    
    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t setPassiveActivationRetries(uint8_t maxRetries) 
{
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;
	
  if (! sendCommandCheckAck(pn532_packetbuffer, 5, 1000))
    return 0x0;  // no ACK
  
  return 1;
}


/***** ISO14443A Commands ******/

/**************************************************************************/
/*! 
    Waits for an ISO14443A target to enter the field
    
    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout) 
{
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;
//   printf("readPassiveTargetID\r\n");
  if (! sendCommandCheckAck(pn532_packetbuffer, 3, 1000))
  {
//    printf("no cards read\r\n");
    return 0x0;  // no cards read
  }
//   printf("Wait for a card to enter the field   ");
  // Wait for a card to enter the field
//  uint8_t status = PN532_I2C_BUSY;

  uint16_t timer = 0;
	nrf_delay_ms(50);
  while (wirereadstatus() != PN532_I2C_READY)
  {
    if (timeout != 0) {
      timer+=1;
			nrf_delay_us(100);
      if (timer > timeout) {

//				 printf("wirereadstatus() != PN532_I2C_READY \r\n");
        return 0x0;
      }
    }

  }

  // read data packet
//	 printf("readPassiveTargetID read data packet \r\n");

  wirereaddata(pn532_packetbuffer, 20);  
  // check some basic stuff
  /* ISO14443A card response should be in the following format:
  
    uint8_t            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */
  if (pn532_packetbuffer[7] != 1) 
    return 0;
    
  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];

  
  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];

  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++) 
  {
    uid[i] = pn532_packetbuffer[13+i];

  }


  return 1;
}

/***** Mifare Classic Functions ******/

/**************************************************************************/
/*! 
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}


bool mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*! 
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a uint8_t array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a uint8_t array containing the 6 uint8_t
                          key value
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t mifareclassic_AuthenticateBlock (uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData)
{
  uint8_t len;
  uint8_t i;
  
  // Hang on to the key and uid data
	for(uint8_t i = 0; i < 6; i++)
	{
		 _key[i] =keyData[i];
	}	
	
	for(uint8_t i = 0; i < 4; i++)
	{
		 _uid[i] =uid[i];
	}	
  _uidLen = uidLen;  

  
  
  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;
	
  for(uint8_t i = 0; i < 6; i++)
	{
		 pn532_packetbuffer[4+i] =_key[i];
	}		
	/* Block Number (1K = 0..63, 4K = 0..255 */

  for (i = 0; i < _uidLen; i++)
  {
    pn532_packetbuffer[10+i] = _uid[i];                /* 4 uint8_t card ID */
  }

  if (! sendCommandCheckAck(pn532_packetbuffer, 10+_uidLen, 1000))
    return 0;

  // Read the response packet
//	 printf("Read the response packet\r\n");
	nrf_delay_ms(10);
  wirereaddata(pn532_packetbuffer, 12);
//  printf("Read ok\r\n");
  // Check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically uint8_t 7: 0x14 but anything other and 0x00 is not good
	
  if (pn532_packetbuffer[7] != 0x00)
  {
    
    return 0;
  }  
  
  return 1;
}


/**************************************************************************/
/*! 
    Tries to read an entire 16-uint8_t data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the uint8_t array that will hold the
                          retrieved data (if any)
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t * data)
{
  
  
  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
  {
    
    return 0;
  }

  /* Read the response packet */
	nrf_delay_ms(10);
  wirereaddata(pn532_packetbuffer, 26);

  /* If uint8_t 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    
    return 0;
  }
    
  /* Copy the 16 data bytes to the output buffer        */
  /* Block content starts at uint8_t 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  /* Display data for debug if requested */
  

  return 1;  
}


/**************************************************************************/
/*! 
    Tries to write an entire 16-uint8_t data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The uint8_t array that contains the data to write.
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t * data)
{
  
  
  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 20, 1000))
  {
    
    return 0;
  }  
  nrf_delay_ms(100);
  
  /* Read the response packet */
  wirereaddata(pn532_packetbuffer, 26);

  return 1;  
}


/**************************************************************************/
/*! 
    Formats a Mifare Classic card to store NDEF Records 
    
    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t mifareclassic_FormatNDEF (void)
{
  uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
  // for the MAD sector in NDEF records (sector 0)
  
  // Write block 1 and 2 to the card
  if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
    return 0;
  // Write key A and access rights card
  if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}


/**************************************************************************/
uint8_t mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char * url)
{
  // Figure out how long the string is
  uint8_t len = strlen(url);
  
  // Make sure we're within a 1K limit for the sector number
  if ((sectorNumber < 1) || (sectorNumber > 15))
    return 0;
  
  // Make sure the URI payload is between 1 and 38 chars
  if ((len < 1) || (len > 38))
    return 0;
    
  // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
  // in NDEF records
	
  // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
  uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len+5, 0xD1, 0x01, len+1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (len <= 6)
  {
    // Unlikely we'll get a url this short, but why not ...
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer1[len+9] = 0xFE;
  }
  else if (len == 7)
  {
    // 0xFE needs to be wrapped around to next block
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer2[0] = 0xFE;
  }
  else if ((len > 7) && (len <= 22))
  {
    // Url fits in two blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer2[len-7] = 0xFE;
  }
  else if (len == 23)
  {
    // 0xFE needs to be wrapped around to final block
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer3[0] = 0xFE;
  }
  else
  {
    // Url fits in three blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, 16);
    memcpy (sectorbuffer3, url+23, len-24);
    sectorbuffer3[len-22] = 0xFE;
  }
  
  // Now write all three blocks back to the card
  if (!(mifareclassic_WriteDataBlock (sectorNumber*4, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+1, sectorbuffer2)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+2, sectorbuffer3)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+3, sectorbuffer4)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}



/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*! 
    Tries to read an entire 4-uint8_t page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the uint8_t array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
uint8_t mifareultralight_ReadPage (uint8_t page, uint8_t * buffer)
{
  if (page >= 64)
  {

    return 0;
  }
  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
  {
    return 0;
  }
  
  /* Read the response packet */
	nrf_delay_us(1000);
  wirereaddata(pn532_packetbuffer, 26);
  

  /* If uint8_t 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data bytes to the output buffer         */
    /* Block content starts at uint8_t 9 of a valid response */
    /* Note that the command actually reads 16 uint8_t or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* bytes                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    
    return 0;
  }

  /* Display data for debug if requested */

  // Return OK signal
  return 1;
}


uint8_t readackframe(void) 
{
  uint8_t ackbuff[6];
  
  wirereaddata(ackbuff, 6);
	
    
  return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}


/**************************************************************************/
/*! 
    @brief  Checks the IRQ pin to know if the PN532 is ready
	
	@returns 0 if the PN532 is busy, 1 if it is free
*/
/**************************************************************************/
uint8_t wirereadstatus(void) 
{
  uint8_t x = nrf_gpio_pin_read(PN532_IRQ);    
//  printf("read irq---> %d\r\n",x);

  if (x == 1)
    return PN532_I2C_BUSY;
  else
    return PN532_I2C_READY;
}

/**************************************************************************/
/*! 
    @brief  Reads n bytes of data from the PN532 via I2C

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void wirereaddata(uint8_t* buff, uint8_t n) 
{

		 uint8_t address =PN532_I2C_ADDRESS;
		 uint8_t buffer[64] = {0};

		 i2c_device_read_buffer(address,buffer,n+2);
		 for(uint8_t i = 0; i< n; i++)
		 {
				 buff[i] = buffer[i+1];
				 
		 }
}
/**************************************************************************/
/*! 
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes 
*/
/**************************************************************************/
void wiresendcommand(uint8_t* cmd, uint8_t cmdlen) 
{
  uint8_t checksum = 0;
	uint8_t buffer[64] ={0};//{0x00,0x00,0xff,0x01,0xff,0xd4,0x02,};
  uint8_t num = 0;
	uint8_t address = PN532_I2C_ADDRESS;
	    
	cmdlen++;
	 checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
	{
  
    buffer[num++] = PN532_I2C_ADDRESS; 
		buffer[num++] = PN532_PREAMBLE;   // 00
		buffer[num++] = PN532_STARTCODE1; // 00
		buffer[num++] = PN532_STARTCODE2; // ff
		buffer[num++] = cmdlen;           
		buffer[num++] = (~cmdlen + 1);
		buffer[num++] = PN532_HOSTTOPN532;
		checksum += PN532_HOSTTOPN532;
		for (uint8_t i=0; i<cmdlen-1; i++) {
				
					buffer[num++] = cmd[i];
					checksum += cmd[i];
		 }
		
		 buffer[num++] = (~checksum);
		 buffer[num++] = PN532_POSTAMBLE; 

	  i2c_write_buffer(address,buffer,num);

	 }

        
} 


/**************************************************************************/
/*! 
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
uint8_t waitUntilReady(uint16_t timeout) 
{
  uint16_t timer = 0;
  while(wirereadstatus() != PN532_I2C_READY) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        return false;
      }
    }
    nrf_delay_us(10);
  }
  return true;
}
    
/**************************************************************************/
/*! 
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************/
uint8_t inDataExchange(uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength) 
{
  if (sendLength > PN532_PACKBUFFSIZ -2) {
      return false;
  }
  uint8_t i;
  
  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }
  
  if (!sendCommandCheckAck(pn532_packetbuffer,sendLength+2,1000)) {
    
    return false;
  }

  if (!waitUntilReady(1000)) {
    
    return false;
  }

  wirereaddata(pn532_packetbuffer,sizeof(pn532_packetbuffer));
  
  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INDATAEXCHANGE) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        return false;
      }
      
      length -= 3;
      
      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }
      
      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }
      *responseLength = length;
      
      return true;
    } 
    else {
    
      return false;
    } 
  } 
  else {
  
    return false;
  }
}



/**************************************************************************/
/*! 
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/**************************************************************************/
uint8_t inListPassiveTarget() 
{
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;
 

  if (!sendCommandCheckAck(pn532_packetbuffer,3,1000)) {
   
    return false;
  }

  if (!waitUntilReady(30000)) {
    return false;
  }

  wirereaddata(pn532_packetbuffer,sizeof(pn532_packetbuffer));
  
  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INLISTPASSIVETARGET) {
      if (pn532_packetbuffer[7] != 1) {
        
        return false;
      }
      
      inListedTag = pn532_packetbuffer[8];
      
      
      return true;
    } else {
 
      return false;
    } 
  } 
  else {
    return false;
  }

  return true;
}



uint8_t pn532_wake_up(void)
{
	uint8_t err_code;

    // Wakeup procedure as specified in PN532 User Manual Rev. 02, p. 7.2.11, page 99.
    uint8_t dummy_byte = 0x55;
    err_code = nrf_drv_twi_tx(&gtMpuTwi, PN532_I2C_ADDRESS, &dummy_byte, 1, false);
    if (err_code != NRF_SUCCESS)
    {
        printf("Failed while calling twi tx, err_code = %d\r\n", err_code);
        return err_code;
    }
    // Wait specified time to ensure that the PN532 shield is fully operational
    // (PN532 data sheet, Rev. 3.2, page 209).
    nrf_delay_ms(2);

    return NRF_SUCCESS;
}


uint8_t pn532_power_down(void)
{
    printf("Powering down the PN532\r\n");

    pn532_packetbuffer[0] = PN532_COMMAND_POWERDOWN;
    pn532_packetbuffer[1] = POWERDOWN_WAKEUP_IRQ;

    uint8_t err_code = sendCommandCheckAck(pn532_packetbuffer,
                                                  COMMAND_POWERDOWN_BASE_LENGTH,
                                                  1000);
//    if (err_code != NRF_SUCCESS)
//    {
//        printf("Failed while checking ACK! err_code = %d\r\n", err_code);
//        return err_code;
//    }
    nrf_delay_ms(1);
       wirereaddata(pn532_packetbuffer, REPLY_POWERDOWN_LENGTH);
//    if (err_code != NRF_SUCCESS)
//    {
//        printf("Failed while reading data! err_code = %d\r\n", err_code);
//        return err_code;
//    }

//    if (!(pn532_packetbuffer[PN532_DATA_OFFSET] == PN532_COMMAND_POWERDOWN + 1))
//    {
//        printf("Failed while checking POWERDOWN response, expected 0x%02x, got 0x%02x\r\n",
//                     PN532_COMMAND_POWERDOWN + 1,
//                     pn532_packetbuffer[PN532_DATA_OFFSET]);
//        return NRF_ERROR_NOT_FOUND;
//    }

    // From PN532 user manual: "The PN532 needs approximately 1 ms to get into Power Down mode,
    // after the command response." (Rev. 02, p. 7.2.11, page 98)
    nrf_delay_ms(1);

    return NRF_SUCCESS;
}




void *my_memset(void *s, char c, unsigned int n)
{
	int i;
	char *ss = (char*)s;
	for(i=0; i<n; i++) ss[i] = c;
	return s;
	
}



