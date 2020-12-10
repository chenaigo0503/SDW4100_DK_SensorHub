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

// apollo print log
#include "am_bsp.h"
#include "apollo_tracelog.h"
#include "apollo3_init.h"
#include "apollo3_rtc.h"

#define PAH8011_IOM_MODULE 3
#define PAH8011_ADDR       0x15

static void* g_Pah8011Hanldle;
static uint32_t pahBuf[2];

extern volatile bool        has_interrupt_button;
extern volatile bool        has_interrupt_pah;
extern volatile uint64_t    interrupt_pah_timestamp;

// ISR callback for the PAH8011
static void pah_int1_handler(void)
{
    has_interrupt_pah = true;
    interrupt_pah_timestamp = get_tick_count();
}

static void pah_int2_handler(void)
{
    pr_err("int2\n");
    return;
}
/*============================================================================
STATIC VARIABLE DEFINITIONS
============================================================================*/

// valid bank range: 0x00 ~ 0x03
static uint8_t _curr_bank = 0xFF;


/*============================================================================
APOLLO READ WRITE FUNCTION
============================================================================*/
static bool i2c_write_reg(uint8_t addr, uint8_t data)
{
    am_hal_iom_transfer_t       Transaction;
    
    *(uint8_t*)pahBuf = data;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = addr;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = 1;
    Transaction.pui32TxBuffer   = pahBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = PAH8011_ADDR;
    
    if (am_hal_iom_blocking_transfer(g_Pah8011Hanldle, &Transaction))
        return false;
    else
        return true;
}

static bool i2c_read_reg(uint8_t addr, uint8_t *data)
{
    am_hal_iom_transfer_t       Transaction;
    
    if (data == NULL)
        return false;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = addr;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = 1;
    Transaction.pui32RxBuffer   = pahBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = PAH8011_ADDR;

    if (am_hal_iom_blocking_transfer(g_Pah8011Hanldle, &Transaction))
    {
        return false;
    }
    else
    {
        *data = (uint8_t)pahBuf[0];
        return true;
    }
}

static bool i2c_burst_read_reg(uint8_t addr, uint8_t *data, uint16_t len)
{
    am_hal_iom_transfer_t       Transaction;
    
    if (data == NULL)
        return false;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = addr;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = pahBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = PAH8011_ADDR;

    if (am_hal_iom_blocking_transfer(g_Pah8011Hanldle, &Transaction))
    {
        return false;
    }
    else
    {
        memcpy(data, pahBuf, len);
        return true;
    }
}

void apollo_platform_init(void)
{
    am_hal_iom_config_t m_sIOMI2cConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_1MHZ,
    };

    am_hal_gpio_pincfg_t m_pah8011GpioInt1 = 
    {
        .uFuncSel = AM_HAL_PIN_19_GPIO,
        .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    };

    am_hal_gpio_pincfg_t m_pah8011GpioInt2 = 
    {
        .uFuncSel = AM_HAL_PIN_18_GPIO,
        .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    };

    // init i2c
    if(g_IOMArray[PAH8011_IOM_MODULE] == NULL)
    {
        am_hal_iom_initialize(PAH8011_IOM_MODULE,
                                &g_IOMArray[PAH8011_IOM_MODULE]);
        g_Pah8011Hanldle = g_IOMArray[PAH8011_IOM_MODULE];
        am_hal_iom_power_ctrl(g_Pah8011Hanldle, AM_HAL_SYSCTRL_WAKE, false);

        // Set the required configuration settings for the IOM.
        am_hal_iom_configure(g_Pah8011Hanldle, &m_sIOMI2cConfig);

        // Configure the IOM pins.
        am_bsp_iom_pins_enable(PAH8011_IOM_MODULE, AM_HAL_IOM_I2C_MODE);
    }
    else
    {
        g_Pah8011Hanldle = g_IOMArray[PAH8011_IOM_MODULE];
    }

    // Enable the IOM.
    am_hal_iom_enable(g_Pah8011Hanldle);
    
    // PIN INIT
    am_hal_gpio_state_write(PAH8011_PIN_PDN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_pinconfig(PAH8011_PIN_PDN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pinconfig(PAH8011_PIN_INT1, m_pah8011GpioInt1);
    am_hal_gpio_pinconfig(PAH8011_PIN_INT2, m_pah8011GpioInt2);
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, PAH8011_PIN_INT1));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, PAH8011_PIN_INT2));
    am_hal_gpio_interrupt_register(PAH8011_PIN_INT1, pah_int1_handler);
    am_hal_gpio_interrupt_register(PAH8011_PIN_INT2, pah_int2_handler);
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, PAH8011_PIN_INT1));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, PAH8011_PIN_INT2));
    NVIC_EnableIRQ(GPIO_IRQn);
    
    PR_ERR("irq address: %x", &has_interrupt_button);
}

/*============================================================================
PUBLIC FUNCTION DEFINITIONS
============================================================================*/
bool pah_comm_write(uint8_t addr, uint8_t data)
{
    if (addr == 0x7F)
    {
        if (_curr_bank == data)
            return true;

        if (!i2c_write_reg(0x7F, data))
            return false;

        _curr_bank = data;
        return true;
    }

	if(addr!=0x7F)
        pr_info("B:%x,R:%x,D:%x\n",_curr_bank,addr,data);
	
    return i2c_write_reg(addr, data);
}

bool pah_comm_read(uint8_t addr, uint8_t *data)
{
    return i2c_read_reg(addr, data);
}

bool pah_comm_burst_read(uint8_t addr, uint8_t *data, uint8_t num)
{
    static const uint16_t rx_size_per_read = 8;
    uint16_t read_index = 0;

    while (num)
    {
        uint16_t read_size = rx_size_per_read;
        if (num < rx_size_per_read)
            read_size = num % rx_size_per_read;

        if (!i2c_burst_read_reg(addr, &data[read_index], read_size))
            return false;

        read_index += rx_size_per_read;
        num -= read_size;
    }

    return true;
}
