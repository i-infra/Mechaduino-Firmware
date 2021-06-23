// Declarations for 187kHz PWM implementation.
// Stock analogWrite is much slower and is very audible!

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void analogFastWrite( uint32_t ulPin, uint32_t ulValue ) ;

#ifdef __cplusplus
}
#endif