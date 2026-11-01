#include "bldc/foc.h"
#include "bldc/pid.h"
#include "ring.h"
#include "seqlock.h"
#include <math.h>
#include <threads.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

// ---------- pid ----------
static void test_pid_step(void) {
    pid_ctrl_t p = {.kp = 2.0f, .ki = 10.0f, .i_limit = 100.0f, .out_limit = 100.0f};
    pid_reset(&p);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f + 0.05f, pid_step(&p, 1.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f + 0.15f, pid_step(&p, 1.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f + 0.25f, pid_step(&p, 1.0f, 0.01f));
}

static void test_pid_clamp(void) {
    pid_ctrl_t p = {.kp = 1.0f, .ki = 100.0f, .i_limit = 0.5f, .out_limit = 1.2f};
    pid_reset(&p);
    float out = 0.0f;
    for (int i = 0; i < 1000; i++) out = pid_step(&p, 3.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, p.integral);
    TEST_ASSERT_EQUAL_FLOAT(1.2f, out);
    for (int i = 0; i < 1000; i++) out = pid_step(&p, -3.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, p.integral);
    TEST_ASSERT_EQUAL_FLOAT(-1.2f, out);
}

static void test_pid_ramp(void) {
    pid_ctrl_t p = {.kp = 10.0f, .out_limit = 100.0f, .ramp = 100.0f};
    pid_reset(&p);
    for (int i = 1; i <= 10; i++) TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)i, pid_step(&p, 1.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, pid_step(&p, 1.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.0f, pid_step(&p, 0.0f, 0.01f));
}

// ---------- foc ----------
static void test_foc_zero_uq(void) {
    float d[3];
    for (float th = -7.0f; th < 7.0f; th += 0.1f) {
        foc_svpwm(0.0f, th, 22.0f, d);
        for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL_FLOAT(0.5f, d[i]);
    }
    foc_svpwm(3.0f, 1.0f, 0.0f, d);
    for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL_FLOAT(0.5f, d[i]);
}

static void test_foc_range_and_vector(void) {
    const float vbus = 22.0f, sqrt3 = 1.7320508f;
    float d[3];
    for (float uq = -30.0f; uq <= 30.0f; uq += 1.5f)
        for (float th = -7.0f; th < 7.0f; th += 0.05f) {
            foc_svpwm(uq, th, vbus, d);
            for (int i = 0; i < 3; i++) {
                TEST_ASSERT_TRUE(d[i] >= 0.0f);
                TEST_ASSERT_TRUE(d[i] <= 1.0f);
            }
            if (fabsf(uq) > vbus / sqrt3) continue;
            float alpha = (2.0f / 3.0f) * (d[0] - 0.5f * (d[1] + d[2])) * vbus;
            float beta = (d[1] - d[2]) / sqrt3 * vbus;
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, -uq * sinf(th), alpha);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, uq * cosf(th), beta);
        }
}

// ---------- ring ----------
static void test_ring_full_empty(void) {
    uint32_t buf[4], x;
    ring_t r;
    TEST_ASSERT_FALSE(ring_init(&r, buf, 3, sizeof x));
    TEST_ASSERT_TRUE(ring_init(&r, buf, 4, sizeof x));
    TEST_ASSERT_FALSE(ring_pop(&r, &x));
    for (x = 0; x < 4; x++) TEST_ASSERT_TRUE(ring_push(&r, &x));
    x = 99;
    TEST_ASSERT_FALSE(ring_push(&r, &x));
    TEST_ASSERT_EQUAL_UINT32(4, ring_count(&r));
    for (uint32_t i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(ring_pop(&r, &x));
        TEST_ASSERT_EQUAL_UINT32(i, x);
    }
    TEST_ASSERT_FALSE(ring_pop(&r, &x));
}

static void test_ring_wraparound(void) {
    uint32_t buf[4], x, next_in = 0, next_out = 0;
    ring_t r;
    ring_init(&r, buf, 4, sizeof x);
    atomic_store(&r.head, UINT32_MAX - 5);
    atomic_store(&r.tail, UINT32_MAX - 5);
    for (int round = 0; round < 50; round++) {
        for (int k = 0; k < 3; k++, next_in++) TEST_ASSERT_TRUE(ring_push(&r, &next_in));
        for (int k = 0; k < 3; k++, next_out++) {
            TEST_ASSERT_TRUE(ring_pop(&r, &x));
            TEST_ASSERT_EQUAL_UINT32(next_out, x);
        }
        TEST_ASSERT_EQUAL_UINT32(0, ring_count(&r));
    }
}

// ---------- seqlock ----------
typedef struct {
    uint32_t a, b, c;
} telem_t;

static seqlock_t sl;
static telem_t shared;
static atomic_bool writer_done;

static int writer(void *arg) {
    (void)arg;
    for (uint32_t i = 1; i <= 2000000; i++) {
        telem_t t = {i, ~i, i * 3};
        seqlock_write(&sl, &shared, &t, sizeof t);
    }
    atomic_store(&writer_done, true);
    return 0;
}

static void test_seqlock_roundtrip(void) {
    telem_t in = {7, ~7u, 21}, out;
    seqlock_write(&sl, &shared, &in, sizeof in);
    seqlock_read(&sl, &out, &shared, sizeof out);
    TEST_ASSERT_EQUAL_MEMORY(&in, &out, sizeof in);
    TEST_ASSERT_EQUAL_UINT32(0, atomic_load(&sl.seq) & 1u);
}

static void test_seqlock_no_torn_reads(void) {
    thrd_t th;
    atomic_store(&writer_done, false);
    TEST_ASSERT_EQUAL(thrd_success, thrd_create(&th, writer, NULL));
    uint32_t reads = 0, torn = 0;
    while (!atomic_load(&writer_done)) {
        telem_t t;
        seqlock_read(&sl, &t, &shared, sizeof t);
        if (t.b != ~t.a || t.c != t.a * 3) torn++;
        reads++;
    }
    thrd_join(th, NULL);
    TEST_ASSERT_EQUAL_UINT32(0, torn);
    TEST_ASSERT_TRUE(reads > 0);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_step);
    RUN_TEST(test_pid_clamp);
    RUN_TEST(test_pid_ramp);
    RUN_TEST(test_foc_zero_uq);
    RUN_TEST(test_foc_range_and_vector);
    RUN_TEST(test_ring_full_empty);
    RUN_TEST(test_ring_wraparound);
    RUN_TEST(test_seqlock_roundtrip);
    RUN_TEST(test_seqlock_no_torn_reads);
    return UNITY_END();
}
