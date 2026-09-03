#ifndef __BOOT_RUNTIME_H
#define __BOOT_RUNTIME_H

#include <stdint.h>
#include "boot_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

void    BootRuntime_RequestBootloader(void);
uint8_t BootRuntime_ConsumeBootRequest(void);

uint8_t BootRuntime_VectorTableValid(void);
uint8_t BootRuntime_ValidatePersistedApp(Boot_PersistConfig_t *cfg_out);

void BootRuntime_JumpToApp(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_RUNTIME_H */
