#pragma once
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    _Atomic uint32_t head;
    _Atomic uint32_t tail;
    uint32_t mask;
    uint32_t elem;
    uint8_t *buf;
} ring_t;

static inline bool ring_init(ring_t *r, void *buf, uint32_t size, uint32_t elem) {
    if (size == 0 || (size & (size - 1)) != 0) return false;
    atomic_init(&r->head, 0);
    atomic_init(&r->tail, 0);
    r->mask = size - 1;
    r->elem = elem;
    r->buf = buf;
    return true;
}

static inline bool ring_push(ring_t *r, const void *item) {
    uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&r->tail, memory_order_acquire);
    if (h - t > r->mask) return false;
    memcpy(r->buf + (h & r->mask) * r->elem, item, r->elem);
    atomic_store_explicit(&r->head, h + 1, memory_order_release);
    return true;
}

static inline bool ring_pop(ring_t *r, void *item) {
    uint32_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
    uint32_t h = atomic_load_explicit(&r->head, memory_order_acquire);
    if (h == t) return false;
    memcpy(item, r->buf + (t & r->mask) * r->elem, r->elem);
    atomic_store_explicit(&r->tail, t + 1, memory_order_release);
    return true;
}

static inline uint32_t ring_count(ring_t *r) {
    return atomic_load_explicit(&r->head, memory_order_acquire) -
           atomic_load_explicit(&r->tail, memory_order_acquire);
}
