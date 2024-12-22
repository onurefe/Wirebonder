/**
 * @author     Onur Efe
 */

#ifndef GENERIC_H
#define GENERIC_H

#include <stdint.h>

#define MAX_UINT8 0xFF
#define MAX_INT8 0x7F
#define MIN_INT8 (-MAX_INT8 - 1)
#define MAX_UINT12 4095
#define MAX_UINT16 65535
#define MAX_INT16 32767
#define MIN_INT16 (-MAX_INT16 - 1)
#define MAX_INT32 2147483647
#define MIN_INT32 (-MAX_INT32 - 1)
#define MAX_UINT32 4294967295
#define FALSE (0)
#define TRUE (1)

/* Exported macros ---------------------------------------------------------*/
#define SET_BIT(variable, bit) ((variable) |= (1 << (bit)))
#define CLEAR_BIT(variable, bit) ((variable) &= ~(1 << (bit)))
#define READ_BIT(variable, bit) (((variable) & (1 << (bit))) >> (bit))

#define SET_FLAG(flagsRegister, flagBitPosition) ((flagsRegister) |= (1 << (flagBitPosition)))
#define CLEAR_FLAG(flagsRegister, flagBitPosition) ((flagsRegister) &= ~(1 << (flagBitPosition)))
#define IS_FLAG_SET(flagsRegister, flagBitPosition) (((flagsRegister) & (1 << (flagBitPosition))) == 0 ? FALSE : TRUE)

/* Exported types ----------------------------------------------------------*/
enum
{
  OPERATION_RESULT_FAILURE = 0x00,
  OPERATION_RESULT_SUCCESS = 0x01
};
typedef uint8_t OperationResult_t;

enum
{
  STATE_UNINIT = 0x00,
  STATE_READY = 0x01,
  STATE_OPERATING = 0x02,
  STATE_ERROR = 0x03
};
typedef uint8_t State_t;

typedef uint8_t Bool_t;

typedef uint64_t quint48_16_t;
typedef uint64_t quint56_8_t;
typedef int64_t qint47_16_t;
typedef int64_t qint55_8_t;
typedef int64_t qint31_32_t;
typedef int32_t qint15_16_t;
typedef uint32_t quint16_16_t;
typedef uint32_t quint24_8_t;
typedef int32_t qint23_8_t;
typedef int32_t qint16_15_t;
typedef int16_t qint7_8_t;
typedef uint16_t quint8_8_t;
typedef int32_t qint19_12_t;

#endif