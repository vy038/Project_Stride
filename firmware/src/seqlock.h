#pragma once
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    _Atomic uint32_t seq;
} seqlock_t;

static inline void seqlock_write(seqlock_t *s, void *dst, const void *src, size_t n) {
    uint32_t v = atomic_load_explicit(&s->seq, memory_order_relaxed);
    atomic_store_explicit(&s->seq, v + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    memcpy(dst, src, n);
    atomic_store_explicit(&s->seq, v + 2, memory_order_release);
}

static inline void seqlock_read(seqlock_t *s, void *dst, const void *src, size_t n) {
    uint32_t v;
    do {
        v = atomic_load_explicit(&s->seq, memory_order_acquire);
        memcpy(dst, src, n);
        atomic_thread_fence(memory_order_acquire);
    } while ((v & 1u) || atomic_load_explicit(&s->seq, memory_order_relaxed) != v);
}
