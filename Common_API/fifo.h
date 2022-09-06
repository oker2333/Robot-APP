/**
* @file
* @author Steve Karg
* @date 2004
*/
#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

/**
* FIFO data structure
*
* @{
*/
struct fifo_buffer_t {
    /** first byte of data */
    unsigned head;
    /** last byte of data */
    unsigned tail;
    /** block of memory or array of data */
    uint8_t *buffer;
    /** length of the data */
    unsigned buffer_len;
};
typedef struct fifo_buffer_t FIFO_BUFFER;
/** @} */

#define fifoENTER_CRITICAL_FROM_ISR() uint32_t ulReturn = taskENTER_CRITICAL_FROM_ISR()
#define fifoEXIT_CRITICAL_FROM_ISR() taskEXIT_CRITICAL_FROM_ISR(ulReturn)

typedef void (*txCallback_t)(uint8_t *buffer,uint16_t len);
typedef uint16_t (*rxCallback_t)(uint8_t *buffer);

void FIFO_Callback_Init(txCallback_t tx_cb,rxCallback_t rx_cb);
void FIFO_Tansmit(FIFO_BUFFER *b);
void FIFO_Recv(FIFO_BUFFER *b);

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

    unsigned FIFO_Count(
        FIFO_BUFFER const *b);

    bool FIFO_Full(
        FIFO_BUFFER const *b);

    bool FIFO_Available(
        FIFO_BUFFER const *b,
        unsigned count);

    bool FIFO_Empty(
        FIFO_BUFFER const *b);

    uint8_t FIFO_Peek(
        FIFO_BUFFER const *b);

    uint8_t FIFO_Get(
        FIFO_BUFFER * b);

    unsigned FIFO_Pull(
        FIFO_BUFFER * b,
        uint8_t * data_bytes,
        unsigned length);

    bool FIFO_Put(
        FIFO_BUFFER * b,
        uint8_t data_byte);

    bool FIFO_Add(
        FIFO_BUFFER * b,
        uint8_t * data_bytes,
        unsigned count);

    void FIFO_Flush(
        FIFO_BUFFER * b);

/* note: buffer_len must be a power of two */
    void FIFO_Init(
        FIFO_BUFFER * b,
        uint8_t * buffer,
        unsigned buffer_len);

#ifdef TEST
#include "ctest.h"
    void testFIFOBuffer(
        Test * pTest);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif
