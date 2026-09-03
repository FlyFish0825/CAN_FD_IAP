#ifndef __BOOT_STORAGE_H
#define __BOOT_STORAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t magic;
    uint16_t config_version;
    uint16_t length;

    uint8_t  node_id;
    uint8_t  hardware_version;
    uint16_t reserved0;

    uint16_t current_offset_a;
    uint16_t current_offset_b;
    uint16_t current_offset_c;
    uint16_t vbus_offset;

    float current_gain_a;
    float current_gain_b;
    float current_gain_c;
    float vbus_gain;

    uint32_t app_size;
    uint32_t app_crc32;
    uint8_t  app_valid;
    uint8_t  reserved1[3];

    uint32_t reserved[8];

    uint32_t crc32;
} Boot_PersistConfig_t;

void    BootStorage_MakeDefaultConfig(Boot_PersistConfig_t *cfg);
uint8_t BootStorage_LoadConfig(Boot_PersistConfig_t *cfg);
uint8_t BootStorage_SaveConfig(const Boot_PersistConfig_t *cfg);
uint8_t BootStorage_InvalidateApp(uint8_t fallback_node_id);
uint8_t BootStorage_SaveAppMetadata(uint32_t app_size, uint32_t app_crc32, uint8_t app_valid);

uint8_t BootStorage_EraseApp(void);
uint8_t BootStorage_ProgramApp(uint32_t offset, const uint8_t *data, uint32_t valid_len);

uint8_t BootStorage_IsFlashRangeValid(uint32_t address, uint32_t length);
uint8_t BootStorage_ReadFlash(uint32_t address, uint8_t *dst, uint32_t length);

/* Config write staging: preserve the whole 2 KiB page in RAM, modify a prefix,
 * then erase+rewrite the page once at WRITE_END.
 */
uint8_t BootStorage_ConfigStageBegin(void);
uint8_t BootStorage_ConfigStageWrite(uint32_t offset, const uint8_t *data, uint32_t length);
uint8_t BootStorage_ConfigStageCommit(void);
void    BootStorage_ConfigStageAbort(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_STORAGE_H */
