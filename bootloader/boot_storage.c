#include "boot_storage.h"
#include "boot_can_config.h"
#include "boot_crc.h"
#include "stm32g4xx_hal.h"

#include <stddef.h>
#include <string.h>

_Static_assert((BOOT_CONFIG_PAGE_SIZE % 8UL) == 0UL, "Config page must be double-word aligned");
_Static_assert((BOOT_APP_START_ADDR % 8UL) == 0UL, "APP start must be 8-byte aligned");
_Static_assert(sizeof(Boot_PersistConfig_t) < BOOT_CONFIG_PAGE_SIZE, "Persistent config too large");
_Static_assert(sizeof(Boot_PersistConfig_t) == 84U, "Unexpected persistent config layout");

typedef union
{
    uint64_t words[BOOT_CONFIG_PAGE_SIZE / 8UL];
    uint8_t  bytes[BOOT_CONFIG_PAGE_SIZE];
} Boot_ConfigPageBuffer_t;

static Boot_ConfigPageBuffer_t g_config_stage;
static uint8_t g_config_stage_active = 0U;

static uint32_t BootStorage_ConfigCRC(const Boot_PersistConfig_t *cfg)
{
    return BootCRC32_Calculate((const uint8_t *)cfg,
                               (uint32_t)offsetof(Boot_PersistConfig_t, crc32));
}

static uint8_t BootStorage_ConfigHeaderValid(const Boot_PersistConfig_t *cfg)
{
    if (cfg == NULL)
    {
        return 0U;
    }

    if (cfg->magic != BOOT_CONFIG_MAGIC)
    {
        return 0U;
    }

    if (cfg->config_version != BOOT_CONFIG_VERSION)
    {
        return 0U;
    }

    if (cfg->length != (uint16_t)sizeof(Boot_PersistConfig_t))
    {
        return 0U;
    }

    if ((cfg->node_id < 1U) || (cfg->node_id > BOOT_MAX_NODE_NUM))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t BootStorage_ErasePage(uint32_t page_index)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0xFFFFFFFFUL;
    HAL_StatusTypeDef st;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = page_index;
    erase.NbPages = 1U;

    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 1U : 0U;
}

static uint8_t BootStorage_ProgramPage(uint32_t address,
                                       const uint64_t *words,
                                       uint32_t word_count)
{
    uint32_t i;
    HAL_StatusTypeDef st = HAL_OK;

    if ((words == NULL) || ((address & 0x7UL) != 0UL))
    {
        return 0U;
    }

    HAL_FLASH_Unlock();

    for (i = 0U; i < word_count; ++i)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               address + i * 8UL,
                               words[i]);
        if (st != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    if (st != HAL_OK)
    {
        return 0U;
    }

    if (memcmp((const void *)(uintptr_t)address, words, word_count * 8UL) != 0)
    {
        return 0U;
    }

    return 1U;
}

void BootStorage_MakeDefaultConfig(Boot_PersistConfig_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = BOOT_CONFIG_MAGIC;
    cfg->config_version = BOOT_CONFIG_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);
    cfg->node_id = BOOT_DEFAULT_NODE_ID;
    cfg->hardware_version = 0U;

    cfg->current_gain_a = 1.0f;
    cfg->current_gain_b = 1.0f;
    cfg->current_gain_c = 1.0f;
    cfg->vbus_gain = 1.0f;

    cfg->app_size = 0U;
    cfg->app_crc32 = 0U;
    cfg->app_valid = 0U;
    cfg->crc32 = BootStorage_ConfigCRC(cfg);
}

uint8_t BootStorage_LoadConfig(Boot_PersistConfig_t *cfg)
{
    uint32_t expected_crc;

    if (cfg == NULL)
    {
        return 0U;
    }

    memcpy(cfg, (const void *)BOOT_CONFIG_PAGE_ADDR, sizeof(*cfg));

    if (BootStorage_ConfigHeaderValid(cfg) == 0U)
    {
        return 0U;
    }

    expected_crc = BootStorage_ConfigCRC(cfg);
    if (expected_crc != cfg->crc32)
    {
        return 0U;
    }

    return 1U;
}

uint8_t BootStorage_SaveConfig(const Boot_PersistConfig_t *cfg)
{
    Boot_PersistConfig_t tmp;

    if (cfg == NULL)
    {
        return 0U;
    }

    /* Preserve every byte in the last Flash page that is outside our struct. */
    memcpy(g_config_stage.bytes,
           (const void *)BOOT_CONFIG_PAGE_ADDR,
           BOOT_CONFIG_PAGE_SIZE);

    tmp = *cfg;
    tmp.magic = BOOT_CONFIG_MAGIC;
    tmp.config_version = BOOT_CONFIG_VERSION;
    tmp.length = (uint16_t)sizeof(tmp);

    if ((tmp.node_id < 1U) || (tmp.node_id > BOOT_MAX_NODE_NUM))
    {
        tmp.node_id = BOOT_DEFAULT_NODE_ID;
    }

    tmp.crc32 = BootStorage_ConfigCRC(&tmp);
    memcpy(g_config_stage.bytes, &tmp, sizeof(tmp));

    if (BootStorage_ErasePage(BOOT_CONFIG_PAGE_INDEX) == 0U)
    {
        return 0U;
    }

    return BootStorage_ProgramPage(BOOT_CONFIG_PAGE_ADDR,
                                   g_config_stage.words,
                                   (uint32_t)(BOOT_CONFIG_PAGE_SIZE / 8UL));
}

uint8_t BootStorage_InvalidateApp(uint8_t fallback_node_id)
{
    Boot_PersistConfig_t cfg;

    if (BootStorage_LoadConfig(&cfg) == 0U)
    {
        BootStorage_MakeDefaultConfig(&cfg);
        if ((fallback_node_id >= 1U) && (fallback_node_id <= BOOT_MAX_NODE_NUM))
        {
            cfg.node_id = fallback_node_id;
        }
    }

    cfg.app_valid = 0U;
    return BootStorage_SaveConfig(&cfg);
}

uint8_t BootStorage_SaveAppMetadata(uint32_t app_size, uint32_t app_crc32, uint8_t app_valid)
{
    Boot_PersistConfig_t cfg;

    if (BootStorage_LoadConfig(&cfg) == 0U)
    {
        BootStorage_MakeDefaultConfig(&cfg);
    }

    cfg.app_size = app_size;
    cfg.app_crc32 = app_crc32;
    cfg.app_valid = (app_valid != 0U) ? 1U : 0U;

    return BootStorage_SaveConfig(&cfg);
}

uint8_t BootStorage_EraseApp(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0xFFFFFFFFUL;
    HAL_StatusTypeDef st;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (uint32_t)BOOT_APP_FIRST_PAGE;
    erase.NbPages = (uint32_t)BOOT_APP_PAGE_COUNT;

    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 1U : 0U;
}

uint8_t BootStorage_ProgramApp(uint32_t offset, const uint8_t *data, uint32_t valid_len)
{
    uint32_t address;
    uint32_t consumed = 0U;
    uint8_t block[8];
    uint32_t chunk;
    uint64_t value;
    HAL_StatusTypeDef st = HAL_OK;

    if ((data == NULL) || (valid_len == 0U))
    {
        return 0U;
    }

    if (offset >= BOOT_APP_MAX_SIZE)
    {
        return 0U;
    }

    if (valid_len > (BOOT_APP_MAX_SIZE - offset))
    {
        return 0U;
    }

    address = BOOT_APP_START_ADDR + offset;
    if ((address & 0x7UL) != 0UL)
    {
        return 0U;
    }

    HAL_FLASH_Unlock();

    while (consumed < valid_len)
    {
        memset(block, 0xFF, sizeof(block));
        chunk = valid_len - consumed;
        if (chunk > 8U)
        {
            chunk = 8U;
        }

        memcpy(block, &data[consumed], chunk);
        memcpy(&value, block, sizeof(value));

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               address + consumed,
                               value);
        if (st != HAL_OK)
        {
            break;
        }

        if (memcmp((const void *)(uintptr_t)(address + consumed), block, sizeof(block)) != 0)
        {
            st = HAL_ERROR;
            break;
        }

        consumed += chunk;
    }

    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 1U : 0U;
}

uint8_t BootStorage_IsFlashRangeValid(uint32_t address, uint32_t length)
{
    const uint32_t flash_limit = BOOT_FLASH_END_ADDR + 1UL;

    if (length == 0U)
    {
        return 0U;
    }

    if ((address < BOOT_FLASH_BASE_ADDR) || (address >= flash_limit))
    {
        return 0U;
    }

    if (length > (flash_limit - address))
    {
        return 0U;
    }

    return 1U;
}

uint8_t BootStorage_ReadFlash(uint32_t address, uint8_t *dst, uint32_t length)
{
    if ((dst == NULL) || (BootStorage_IsFlashRangeValid(address, length) == 0U))
    {
        return 0U;
    }

    memcpy(dst, (const void *)(uintptr_t)address, length);
    return 1U;
}

uint8_t BootStorage_ConfigStageBegin(void)
{
    Boot_PersistConfig_t cfg;

    if (BootStorage_LoadConfig(&cfg) != 0U)
    {
        memcpy(g_config_stage.bytes,
               (const void *)BOOT_CONFIG_PAGE_ADDR,
               BOOT_CONFIG_PAGE_SIZE);
    }
    else
    {
        memset(g_config_stage.bytes, 0xFF, BOOT_CONFIG_PAGE_SIZE);
        BootStorage_MakeDefaultConfig(&cfg);
        memcpy(g_config_stage.bytes, &cfg, sizeof(cfg));
    }

    g_config_stage_active = 1U;
    return 1U;
}

uint8_t BootStorage_ConfigStageWrite(uint32_t offset, const uint8_t *data, uint32_t length)
{
    if ((g_config_stage_active == 0U) || (data == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (offset >= BOOT_CONFIG_PAGE_SIZE)
    {
        return 0U;
    }

    if (length > (BOOT_CONFIG_PAGE_SIZE - offset))
    {
        return 0U;
    }

    memcpy(&g_config_stage.bytes[offset], data, length);
    return 1U;
}

uint8_t BootStorage_ConfigStageCommit(void)
{
    Boot_PersistConfig_t *cfg;

    if (g_config_stage_active == 0U)
    {
        return 0U;
    }

    cfg = (Boot_PersistConfig_t *)(void *)g_config_stage.bytes;
    cfg->magic = BOOT_CONFIG_MAGIC;
    cfg->config_version = BOOT_CONFIG_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);

    if ((cfg->node_id < 1U) || (cfg->node_id > BOOT_MAX_NODE_NUM))
    {
        cfg->node_id = BOOT_DEFAULT_NODE_ID;
    }

    cfg->crc32 = BootStorage_ConfigCRC(cfg);

    if (BootStorage_ErasePage(BOOT_CONFIG_PAGE_INDEX) == 0U)
    {
        g_config_stage_active = 0U;
        return 0U;
    }

    if (BootStorage_ProgramPage(BOOT_CONFIG_PAGE_ADDR,
                                g_config_stage.words,
                                (uint32_t)(BOOT_CONFIG_PAGE_SIZE / 8UL)) == 0U)
    {
        g_config_stage_active = 0U;
        return 0U;
    }

    g_config_stage_active = 0U;
    return 1U;
}

void BootStorage_ConfigStageAbort(void)
{
    g_config_stage_active = 0U;
}
