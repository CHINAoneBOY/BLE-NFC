
#ifndef __PN532_APPLY_H__
#define __PN532_APPLY_H__

#include <stdint.h>
 enum _num_
{
	READ_CARD = 1,
	WRITE_CARD = 2,
	READ_CARD_B = 3,
};

void device_pn532_init();
void pn532_appliction(uint8_t *);
void power_down_pn532(void);
void wake_up_pn532(void);
void pn532_timeout_handler(void * p_context);
void read_data_card(uint8_t block_num, uint8_t excursion_num ,uint8_t *read_data);
void write_data_card(uint8_t block_num, uint8_t excursion_num, uint8_t *write_data);
void test_uid(void);
#endif