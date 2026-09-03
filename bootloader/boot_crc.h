#ifndef __BOOT_CRC_H
#define __BOOT_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  BootCRC8_Calculate(const uint8_t *data, uint32_t len);
uint32_t BootCRC32_Calculate(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CRC_H */
