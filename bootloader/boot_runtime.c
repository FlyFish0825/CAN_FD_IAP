#include "boot_runtime.h"
#include "boot_can_config.h"
#include "boot_crc.h"
#include "stm32g4xx_hal.h"

static void BootRuntime_EnableBackupWrite(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

#if defined(__HAL_RCC_RTCAPB_CLK_ENABLE)
    __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif
}

void BootRuntime_RequestBootloader(void)
{
    BootRuntime_EnableBackupWrite();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    __DSB();
}

uint8_t BootRuntime_ConsumeBootRequest(void)
{
    uint8_t requested;

    BootRuntime_EnableBackupWrite();
    requested = (TAMP->BKP0R == BOOT_REQUEST_MAGIC) ? 1U : 0U;

    if (requested != 0U)
    {
        TAMP->BKP0R = 0U;
        __DSB();
    }

    return requested;
}

uint8_t BootRuntime_VectorTableValid(void)
{
    uint32_t msp = *(const volatile uint32_t *)BOOT_APP_START_ADDR;
    uint32_t reset_handler = *(const volatile uint32_t *)(BOOT_APP_START_ADDR + 4UL);
    uint32_t reset_address = reset_handler & ~1UL;
    uint8_t stack_valid = 0U;

    if ((msp >= BOOT_SRAM_ALIAS_START_ADDR) && (msp <= BOOT_SRAM_ALIAS_END_ADDR))
    {
        stack_valid = 1U;
    }
    else if ((msp >= BOOT_CCM_START_ADDR) && (msp <= BOOT_CCM_END_ADDR))
    {
        stack_valid = 1U;
    }

    if (stack_valid == 0U)
    {
        return 0U;
    }

    if ((reset_handler & 1UL) == 0UL)
    {
        return 0U;
    }

    if ((reset_address < BOOT_APP_START_ADDR) || (reset_address > BOOT_APP_END_ADDR))
    {
        return 0U;
    }

    return 1U;
}

uint8_t BootRuntime_ValidatePersistedApp(Boot_PersistConfig_t *cfg_out)
{
    Boot_PersistConfig_t cfg;
    uint32_t crc;

    if (BootStorage_LoadConfig(&cfg) == 0U)
    {
        return 0U;
    }

    if (cfg.app_valid == 0U)
    {
        return 0U;
    }

    if ((cfg.app_size == 0U) || (cfg.app_size > BOOT_APP_MAX_SIZE))
    {
        return 0U;
    }

    if (BootRuntime_VectorTableValid() == 0U)
    {
        return 0U;
    }

    crc = BootCRC32_Calculate((const uint8_t *)BOOT_APP_START_ADDR, cfg.app_size);
    if (crc != cfg.app_crc32)
    {
        return 0U;
    }

    if (cfg_out != NULL)
    {
        *cfg_out = cfg;
    }

    return 1U;
}

void BootRuntime_JumpToApp(void)
{
    typedef void (*AppEntry_t)(void);

    uint32_t app_msp = *(const volatile uint32_t *)BOOT_APP_START_ADDR;
    uint32_t app_reset = *(const volatile uint32_t *)(BOOT_APP_START_ADDR + 4UL);
    uint32_t i;
    AppEntry_t entry = (AppEntry_t)(uintptr_t)app_reset;

    __disable_irq();

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    for (i = 0U; i < 8U; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = BOOT_APP_START_ADDR;
    __DSB();
    __ISB();

    __set_CONTROL(0U);
    __set_MSP(app_msp);
    __DSB();
    __ISB();

    __enable_irq();
    entry();

    while (1)
    {
        /* Should never return. */
    }
}
