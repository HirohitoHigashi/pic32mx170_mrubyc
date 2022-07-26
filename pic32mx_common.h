#include <stdint.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void __delay_us(uint32_t us);
void __delay_ms(uint32_t ms);
void system_register_lock(void);
void system_register_unlock(void);
void system_reset(void);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif
