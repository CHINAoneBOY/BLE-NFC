#include "ble_nus.h"
#include "pn532_apply.h"
#include "pn532_i2c.h"
#include "nrf_delay.h"
#include "sdk_config.h"
#include "lock_gpio.h"
//#include "adafruit_pn532.h"




uint8_t i2c_device_address;
uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t keya[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
uint8_t uidLength; 
uint8_t ab[18] = {0};


extern  ble_nus_t m_nus;

void device_pn532_init() // ³õÊ¼»¯pn532
{



			init_pnc523_i2c();
      pn532_gpio_init();
	//	begin();
			nrf_delay_ms(100);
			uint32_t versiondata = getFirmwareVersion();
			if (! versiondata) {
					printf("Didn't find PN53x board");	
					while(1); 			
			}
			printf("---->i2c pn532 ok\r\n");
			printf("---->Found chip PN5%02x\r\n",((versiondata>>24)&0xff)); 
			printf("---->Firmware ver.%d",((versiondata>>16)&0xff));
			printf(".%d\r\n",((versiondata>>8)&0xff));

			printf("---->pn532 config ok\r\n");
			printf("---->Waiting for an ISO14443A Card ...\r\n");
			
}



	
void read_data_card(uint8_t block_num, uint8_t excursion_num, uint8_t* read_data)
{
	
		success = readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,1000); 
    if (success) {
//      printf("\r\n---->Found an ISO14443A card\r\n");	
//		  printf("---->UID Length: %d\r\n",uidLength);
		beep_test();
//    printf("UID :");		
//		for(uint8_t i=0; i<uidLength; i++)
//		{
//      printf("  %02x",uid[i]);	
//		}
//		printf("\r\n");	
			
    ble_nus_string_send(&m_nus, uid, uidLength);
  /*  if (uidLength == 4)
    {
					// We probably have a Mifare Classic card ... 
				
//					 printf("Seems to be a Mifare Classic card (4 byte UID)\r\n");
//					// Now we need to try to authenticate it for read/write access
//					// Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
//					printf("Trying to authenticate block 4 with default KEYA value\r\n");

							  uint8_t keya[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

//					key(keya);					
					// Start with block 4 (the first block of sector 1) since sector 0
					// contains the manufacturer data and it's probably better just
					// to leave it alone unless you know what you're doing
					
//					printf("---->keyA : %02x %02x %02x %02x %02x %02x\r\n",keya[0],keya[1],keya[2],keya[3],keya[4],keya[5]);
//          printf("----> : %02x %02x %02x %02x %02x %02x\r\n",block_num,excursion_num,keya[2],keya[3],keya[4],keya[5]);									
          for(uint8_t i =block_num ; i <= (block_num+excursion_num); i++)
					{
										success = mifareclassic_AuthenticateBlock(uid, uidLength, i, 0, keya);
									
										if (success)
										{
//											printf("Sector 1 (Blocks 4..7) has been authenticated\r\n");
											uint8_t data[16];

											// If you want to write something to block 4 to test with, uncomment
											// the following line and this text should be read back in a minute
											//memcpy(data, (const uint8_t[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof data);
											//success = nfc.mifareclassic_WriteDataBlock (4, data);

											
														// Try to read the contents of block 4
														success = mifareclassic_ReadDataBlock(i, data);
												
														if (success)
														{
															// Data seems to have been read ... spit it out
//															 printf("Reading Block %2d:",i);
//															for(uint8_t i = 0; i<16; i++)
//															{
//																printf("%02x ",data[i]);
//															}
//															 printf("\r\n");
															nrf_delay_ms(20);
															// Wait a bit before reading the card again
                              ble_nus_string_send(&m_nus, data, 16);
														}
														else
														{
//															printf("Ooops ... unable to read the requested block.  Try another key?\r\n");
														}
														
										}
										else
										{
					//						printf("Ooops ... authentication failed: Try another key?\r\n");
										}
											
					}
					nrf_delay_ms(500);
    }
    */
    if (uidLength == 7)
    {
//      printf("Seems to be a Mifare Ultralight tag (7 byte UID)\r\n");
//			printf("Reading page 4\r\n");
      uint8_t data[32];
      success = mifareultralight_ReadPage (4, data);
      if (success)
      {
       
//         for(uint8_t i = 0; i<4; i++)
//					{
//						printf("%02x ",data[i]);
//					}
//					 printf("\r\n");
       
        nrf_delay_ms(100);
      }
      else
      {
//         printf("Ooops ... unable to read the requested page!?\r\n");
      }
    }
			
    }
	   my_memset(uid,0,7);

}


void write_data_card(uint8_t block_num,uint8_t excursion_num, uint8_t* write_data)
{
	
		success = readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,1000); 

    if (success) {
//      printf("\r\n---->Found an ISO14443A card\r\n");	
//		  printf("---->UID Length: %d\r\n",uidLength);
		beep_test();
//    printf("UID :");		
//		for(uint8_t i=0; i<uidLength; i++)
//		{
//      printf("  %02x",uid[i]);	
//		}
//		printf("\r\n");	
			
    ble_nus_string_send(&m_nus, uid, uidLength);
    if (uidLength == 4)
    {
					// We probably have a Mifare Classic card ... 
				
//					 printf("Seems to be a Mifare Classic card (4 byte UID)\r\n");
//					// Now we need to try to authenticate it for read/write access
//					// Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
//					printf("Trying to authenticate block 4 with default KEYA value\r\n");

							  uint8_t keya[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

//					key(keya);					
					// Start with block 4 (the first block of sector 1) since sector 0
					// contains the manufacturer data and it's probably better just
					// to leave it alone unless you know what you're doing
					
//					printf("---->keyA : %02x %02x %02x %02x %02x %02x\r\n",keya[0],keya[1],keya[2],keya[3],keya[4],keya[5]);
//          printf("----> : %02x %02x %02x %02x %02x %02x\r\n",block_num,excursion_num,keya[2],keya[3],keya[4],keya[5]);									
          for(uint8_t i =block_num ; i <= (block_num+excursion_num); i++)
					{
										success = mifareclassic_AuthenticateBlock(uid, uidLength, i, 0, keya);
									
										if (success)
										{
//											printf("Sector 1 (Blocks 4..7) has been authenticated\r\n");
											uint8_t data[16];

											// If you want to write something to block 4 to test with, uncomment
											// the following line and this text should be read back in a minute
											//memcpy(data, (const uint8_t[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof data);
											//success = nfc.mifareclassic_WriteDataBlock (4, data);

											
														// Try to read the contents of block 4
														success = mifareclassic_ReadDataBlock(i, data);
												
														if (success)
														{
															// Data seems to have been read ... spit it out
//															 printf("Reading Block %2d:",i);
//															for(uint8_t i = 0; i<16; i++)
//															{
//																printf("%02x ",data[i]);
//															}
															ble_nus_string_send(&m_nus, data, 16);
//															 printf("\r\n");
															// Wait a bit before reading the card again
                             mifareclassic_WriteDataBlock(i,write_data);
															
														}
														
														else
														{
//															printf("Ooops ... unable to read the requested block.  Try another key?\r\n");
														}
														
										}
										else
										{
					//						printf("Ooops ... authentication failed: Try another key?\r\n");
										}
											
					}
					nrf_delay_ms(500);
    }
    
    if (uidLength == 7)
    {
      printf("Seems to be a Mifare Ultralight tag (7 byte UID)\r\n");
			printf("Reading page 4\r\n");
      uint8_t data[32];
      success = mifareultralight_ReadPage (4, data);
      if (success)
      {
       
         for(uint8_t i = 0; i<4; i++)
					{
						printf("%02x ",data[i]);
					}
					 printf("\r\n");
       
        nrf_delay_ms(100);
      }
      else
      {
         printf("Ooops ... unable to read the requested page!?\r\n");
      }
    }
			
    }
	   my_memset(uid,0,7);

}
void pn532_appliction(uint8_t *a)    //Ñ°¿¨,¶Á¿¨,¶ÁÄÚÈÝ
{
	
	  SAMConfig();
		switch(*a)
		{
				case READ_CARD:
					
							read_data_card(*(a+1),*(a+2),(a+3));

							break;

				case WRITE_CARD:
					
							write_data_card(*(a+1),*(a+2),(a+3));
							break;
				case READ_CARD_B:
					
							test_uid();
							break;	
				default:
							 printf("no_card \r\n");
						
		}
}


void power_down_pn532(void)
{
	pn532_power_down();
	nrf_delay_ms(1000);
}

void wake_up_pn532(void)
{
	pn532_wake_up();
}

void test_uid(void)
{
	uint8_t cardbaudrate; 
	uint8_t uid[16]={0}; 
	uint8_t uidLength = 0xff; 
	uint16_t timeout = 1000;
	
	
  SetRFConfiguration();		
	CategoryBConfig();
  SetParameters();
  readTypeBuid(cardbaudrate,uid,&uidLength,timeout);
	ble_nus_string_send(&m_nus, uid, uidLength);

}



void pn532_timeout_handler(void * p_context)
{
		
}



uint8_t data_ctrl()
{

	
	
}
	