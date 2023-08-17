//
// Created by XIAOSENLUO on 2023/2/27.
//

#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "cmsis_gcc.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    uint32_t full;
    uint32_t head;
    uint32_t tail;
    const uint32_t size;
    void* *buffer;
}queue_t;

#define QUEUE_ITEM_DIM(q)       (sizeof(q) / sizeof(q[0]))

void *queue_read(queue_t *queue);
int queue_write(queue_t *queue, void *data);

static inline bool queue_is_empty(queue_t *queue){
    return (queue->full == 0) && (queue->tail == queue->head);
}
static inline bool queue_is_full(queue_t *queue){
    return queue->full;
}
static inline uint32_t queue_size(queue_t *queue){
    return (queue->tail - queue->head) > 0 ? queue->tail - queue->head : queue->size + queue->tail - queue->head;
}

static inline void queue_clear(queue_t *queue){
    queue->head = queue->tail;
    queue->full = 0;
}


struct byte_queue_type {
    struct {
        struct {
            uint8_t front;
            uint8_t full;
        };
        uint16_t volatile tail;
        uint16_t volatile head;
        uint16_t volatile end;
    };
    struct {
        uint16_t volatile nfree;
        uint16_t nmin;
    };
    uint8_t *buffer;
};

typedef struct byte_queue_type byte_queue_t;
typedef void * b_handle;

#define CRIT_SEC_E_     __disable_irq()
#define CRIT_SEC_X_     __enable_irq()

b_handle bq_initialize(byte_queue_t * const bq, uint8_t * const buffer, const uint16_t bsize);

const uint16_t bq_post(b_handle * const bqh, uint8_t const * const data, const uint16_t dsize);

const uint16_t bq_post_lifo(b_handle * const bqh, uint8_t const *const data, const uint16_t dsize);

const uint16_t bq_get(b_handle * const bqh, uint8_t * const odata, const uint16_t dsize);

const uint8_t bq_get_byte(b_handle * const bqh, uint8_t * const ok);

static inline const uint16_t bq_get_free(b_handle * const bqh){
    CRIT_SEC_E_;
    const uint16_t nfree = ((byte_queue_t *)(*bqh))->nfree;
    CRIT_SEC_X_;
    return nfree;
}

static inline const uint16_t bq_full_size(b_handle * const bqh){
    CRIT_SEC_E_;
    const uint16_t end = ((byte_queue_t *)(*bqh))->end;
    CRIT_SEC_X_;
    return end;
}

static inline const uint16_t  bq_get_min(b_handle const *bqh){
    CRIT_SEC_E_;
    const uint16_t nmin = ((byte_queue_t *)(*bqh))->nmin;
    CRIT_SEC_X_;
    return nmin;
}

static inline const uint8_t bq_is_empty(b_handle * const bqh){
    CRIT_SEC_E_;
    uint8_t empty = 0;
    if(((byte_queue_t *)(*bqh))->nfree == ((byte_queue_t *)(*bqh))->end) empty = 1;
    CRIT_SEC_X_;
    return empty;
}

static inline const uint8_t bq_is_full(b_handle * const bqh){
    return bq_get_free(bqh) ? 0 : 1;
}

static inline const uint16_t bq_length(b_handle * const bqh){
    CRIT_SEC_E_;
    const uint16_t available = ((byte_queue_t *)(*bqh))->end - ((byte_queue_t *)(*bqh))->nfree;
    CRIT_SEC_X_;
    return available;
}

static inline const uint16_t bq_size(b_handle * const bqh){
    return bq_length(bqh);
}

static inline void bq_clear(b_handle * const bqh){
    CRIT_SEC_E_;
    ((byte_queue_t *)(*bqh))->nfree = ((byte_queue_t *)(*bqh))->end;
    ((byte_queue_t *)(*bqh))->head = 0;
    ((byte_queue_t *)(*bqh))->tail = 0;
    CRIT_SEC_X_;
}

#ifdef __cplusplus
}
#endif

#endif //__QUEUE_H
