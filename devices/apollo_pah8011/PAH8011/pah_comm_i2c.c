/*==============================================================================
* Edit History
* 
* This section contains comments describing changes made to the module. Notice
* that changes are listed in reverse chronological order. Please use ISO format
* for dates.
* 
* when       who       what, where, why
* ---------- ---       -----------------------------------------------------------
* 2016-04-12 bh        Add license information and revision information
* 2016-04-07 bh        Initial revision.
==============================================================================*/

#include "pah_comm.h"

// platform support
#include "pah_platform_functions.h"
#include "i2c.h"

/*============================================================================
STATIC VARIABLE DEFINITIONS
============================================================================*/

// valid bank range: 0x00 ~ 0x03
static uint8_t _curr_bank = 0xFF;


/*============================================================================
PUBLIC FUNCTION DEFINITIONS
============================================================================*/
bool pah_comm_write(uint8_t addr, uint8_t data)
{
    if (addr == 0x7F)
    {
        if (_curr_bank == data)
            return true;

				pah8011_i2c_write(0x7F, &data,1);
        _curr_bank = data;
        return true;
    }

		if(addr!=0x7F)
        debug_printf("B:%x,R:0x%02x,D:0x%02x\n",_curr_bank,addr,data);
		pah8011_i2c_write (addr, &data,1);
		return true;
}

bool pah_comm_read(uint8_t addr, uint8_t *data)
{
    pah8011_i2c_read (addr, data, 1);
    return true;
}

bool pah_comm_burst_read(uint8_t addr, uint8_t *data, uint8_t num)
{
    pah8011_i2c_read(addr, data, num);
    return true;
}
