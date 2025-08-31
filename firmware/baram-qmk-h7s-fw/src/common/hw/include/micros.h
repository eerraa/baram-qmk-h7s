#ifndef MICROS_H_
#define MICROS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_MICROS


bool microsInit(void);
uint32_t micros(void);

// [V1.5.0] 오버플로우-안전 64비트 마이크로초 함수 및 콜백 함수 추가
uint64_t micros64(void);
void microsSetCallback(void (*p_func)(void));


#endif


#ifdef __cplusplus
}
#endif


#endif