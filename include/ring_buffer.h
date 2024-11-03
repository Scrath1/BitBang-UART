#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#ifndef __STATIC_INLINE
    #define __STATIC_INLINE static inline
#endif // __STATIC_INLINE

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "return_codes.h"

/**
* Usage:
* 1. Create a ring_buffer_t (ringbuf) object with a fixed size
* 2. Call RING_BUFFER_DEF(ringbuf,ringbufSize)
* 3. Use ring_buffer_put(&ringbuf, input)
*/

#define RING_BUFFER_DEF(x, sz) \
    uint8_t x##_data[sz];      \
    ring_buffer_t x = {        \
        .buffer = x##_data,    \
        .head = 0,             \
        .tail = 0,             \
        .len = sz}

typedef struct ring_buffer
{
    uint8_t *buffer;
    volatile uint32_t head;
    volatile uint32_t tail;
    const uint32_t len;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *const rb);
RC_t ring_buffer_put(ring_buffer_t *const rb, uint8_t c);
RC_t ring_buffer_get(ring_buffer_t *const rb, uint8_t *const c);

__STATIC_INLINE bool ring_buffer_is_empty(ring_buffer_t *const rb)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    return (head == tail);
}

__STATIC_INLINE bool ring_buffer_is_full(ring_buffer_t *const rb)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    return (((head + 1) % rb->len) == tail);
}

/**
 * Returns number of items currently stored in ringbuffer 
 */
__STATIC_INLINE uint32_t ring_buffer_avail(ring_buffer_t *const rb)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    return (head - tail) % rb->len;
}
#ifdef __cplusplus
}
#endif
#endif
