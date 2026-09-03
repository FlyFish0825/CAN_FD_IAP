#include "boot_crc.h"
#include "boot_can_config.h"
#include "stm32g4xx_hal.h"

/*
 * The CRC peripheral is used directly so the Bootloader does not depend on
 * a CubeMX-generated hcrc handle. All calls are made from the main context;
 * do not call these functions concurrently from interrupts.
 */

static void BootCRC_FeedBytes(const uint8_t *data, uint32_t len)
{
    uint32_t i = 0U;
    uint16_t halfword;

    while ((len - i) >= 4U)
    {
        CRC->DR = ((uint32_t)data[i] << 24U) |
                  ((uint32_t)data[i + 1U] << 16U) |
                  ((uint32_t)data[i + 2U] << 8U) |
                  ((uint32_t)data[i + 3U]);
        i += 4U;
    }

    switch (len - i)
    {
    case 3U:
        halfword = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
        *(__IO uint16_t *)(__IO void *)&CRC->DR = halfword;
        *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i + 2U];
        break;

    case 2U:
        halfword = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
        *(__IO uint16_t *)(__IO void *)&CRC->DR = halfword;
        break;

    case 1U:
        *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i];
        break;

    default:
        break;
    }
}

static void BootCRC_Configure(uint32_t polynomial,
                              uint32_t init_value,
                              uint32_t polynomial_size)
{
    __HAL_RCC_CRC_CLK_ENABLE();

    /* No bit reversal on input or output. */
    MODIFY_REG(CRC->CR,
               CRC_CR_POLYSIZE | CRC_CR_REV_IN | CRC_CR_REV_OUT,
               polynomial_size);

    CRC->POL = polynomial;
    CRC->INIT = init_value;
    CRC->CR |= CRC_CR_RESET;
}

uint8_t BootCRC8_Calculate(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len != 0U))
    {
        return 0U;
    }

    BootCRC_Configure((uint32_t)BOOT_CTRL_CRC8_POLY,
                      (uint32_t)BOOT_CTRL_CRC8_INIT,
                      CRC_CR_POLYSIZE_1); /* 8-bit polynomial */

    if (len != 0U)
    {
        BootCRC_FeedBytes(data, len);
    }

    return (uint8_t)(CRC->DR & 0xFFU);
}

uint32_t BootCRC32_Calculate(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len != 0U))
    {
        return 0U;
    }

    BootCRC_Configure(BOOT_CRC32_POLY,
                      BOOT_CRC32_INIT,
                      0U); /* 32-bit polynomial */

    if (len != 0U)
    {
        BootCRC_FeedBytes(data, len);
    }

    return CRC->DR;
}
