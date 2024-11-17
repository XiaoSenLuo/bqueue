//
// Created by XIAOSENLUO on 2023/2/27.
//

#include "bqueue/bqueue.h"


#ifdef __cplusplus
extern "C" {
#endif

void *queue_read(queue_t *queue){
    if(queue_is_empty(queue)) return NULL;
    void *data = queue->buffer[queue->tail];
    queue->tail = (queue->tail + 1) % queue->size;
    if(queue->tail == queue->head) queue->full = 0;
    return data;
}

int queue_write(queue_t *queue, void *data){
    if(queue_is_full(queue)) return -1;
    queue->buffer[queue->head] = data;
    queue->head = (queue->head + 1) % queue->size;
    if(queue->head == queue->tail) queue->full = 1;
    return queue->head;
}

static void default_lock(void){

}

static void default_unlock(void){

}

b_handle bq_initialize(byte_queue_t * const bq, uint8_t * const buffer, const uint16_t bsize,
                       void (*lock)(void), void (*unlock)(void)){
    byte_queue_t * const bhandle = (byte_queue_t *)(bq);
    bhandle->buffer = buffer;
    bhandle->end = bsize;
    bhandle->front = 0;
    bhandle->full = 0;
    if(bsize != 0){
        bhandle->head = 0;
        bhandle->tail = 0;
    }
    bhandle->nfree = bsize;
    bhandle->nmin = bhandle->nfree;
    bhandle->lock = default_lock;
    bhandle->unlock = default_unlock;

    if(lock) bhandle->lock = lock;
    if(unlock) bhandle->unlock = unlock;

    return (b_handle)bq;
}


uint16_t bq_post(b_handle bqh, uint8_t const *data, const uint16_t dsize){
    byte_queue_t * const bhandle = (byte_queue_t *)(bqh);
    CRIT_SEC_E_;
    volatile uint16_t nfree = bhandle->nfree;
    // uint32_t *dest = (uint32_t *)&bhandle->buffer[bhandle->head];
    // uint32_t *src = (uint32_t *)data;
    uint16_t bc = 0, len = 0;

    if((nfree == 0) || (dsize == 0)){
        CRIT_SEC_X_;
        return 0;
    }
    len =  (nfree < dsize) ? nfree : dsize;
    nfree -= len;
    bhandle->nfree = nfree;

    if(bhandle->nmin > nfree){
        bhandle->nmin = nfree;
    }

//    bc = len >> 2;
//    while(bc > 0){
//        *dest++ = *src++;
//        bc--;
//        bhandle->head = (bhandle->head + 4) % bhandle->end;
//    }

    bc = len;

    while(bc > 0){
        bhandle->buffer[bhandle->head] = data[len - bc];
        --bc;
        ++bhandle->head;
        if(bhandle->head == bhandle->end){
            bhandle->head = 0;
        }
    }
    CRIT_SEC_X_;
    return len;
}

uint16_t bq_post_lifo(b_handle bqh, uint8_t const *data, const uint16_t dsize){
    byte_queue_t * const bhandle = (byte_queue_t *)(bqh);

    CRIT_SEC_E_;

    volatile uint16_t nfree = bhandle->nfree;
    uint32_t *dest = (uint32_t *)&bhandle->buffer[bhandle->head];
    uint32_t *src = (uint32_t *)data;
    uint16_t bc = 0, len = 0;

    if((nfree == 0) || (dsize == 0)){
        CRIT_SEC_X_;
        return 0;
    }
    len =  (nfree < dsize) ? nfree : dsize;
    nfree -= len;
    bhandle->nfree = nfree;

    if(bhandle->nmin > nfree){
        bhandle->nmin = nfree;
    }

    bc = len;

    while(bc > 0){
        if(bhandle->tail == 0){
            bhandle->tail = bhandle->end;
        }
        --bhandle->tail;
        bhandle->buffer[bhandle->tail] = data[len - bc];
        --bc;
    }

    CRIT_SEC_X_;
    return len;
}

uint16_t bq_get(b_handle bqh, uint8_t * const odata, const uint16_t dsize){
    byte_queue_t * const bhandle = (byte_queue_t *)(bqh);
    CRIT_SEC_E_;
    const uint16_t available = bhandle->end - bhandle->nfree;
    uint16_t bc = 0;

    if(available == 0){
        CRIT_SEC_X_;
        return 0;
    }

    bc = dsize > (available) ? available : dsize;

    bhandle->nfree += bc;

    for(uint16_t i = 0; i < bc; i++){
        odata[i] = bhandle->buffer[bhandle->tail];
        ++bhandle->tail;
        if(bhandle->tail == bhandle->end){
            bhandle->tail = 0;
        }
    }

    CRIT_SEC_X_;
    return bc;
}

uint8_t bq_get_byte(b_handle bqh, uint8_t * const ok){
    uint16_t nr = 0;
    uint8_t rd = 0;
    nr = bq_get(bqh, &rd, 1);
    if(ok){
        *ok = (nr) ? 1 : 0;
    }
    return rd;
}

#ifdef __cplusplus
}
#endif

