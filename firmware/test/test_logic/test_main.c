#include "bldc/calib.h"
#include "bldc/foc.h"
#include "bldc/joint.h"
#include "can/can_dispatch.h"
#include "can/can_proto.h"
#include "bldc/pid.h"
#include "bldc/vel.h"
#include "ring.h"
#include "seqlock.h"
#include <errno.h>
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

// ---------- vel ----------
// quantised sine through the estimator, compared to the true velocity delayed by the expected lag
static void test_vel_tracks_quantised_sine(void) {
    const float q = 6.2831853f / 4096, amp = 0.5f, w = 6.2831853f; // 1 Hz, peak 3.1 rad/s
    const float lag = VEL_WINDOW_N * VEL_DT / 2 + VEL_TF_S;
    vel_t v = {0};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_step(&v, 1.0f + amp * sinf(0.0f)));
    v = (vel_t){0};
    float worst = 0.0f;
    for (int k = 0; k < 2 * CFC_CTRL_HZ; k++) {
        float t = k * VEL_DT;
        float est = vel_step(&v, roundf((1.0f + amp * sinf(w * t)) / q) * q);
        if (t < 0.1f) continue; // settling
        float err = fabsf(est - amp * w * cosf(w * (t - lag)));
        if (err > worst) worst = err;
    }
    TEST_ASSERT_TRUE(worst < 0.15f); // one window quantum
}

// ---------- calib ----------
// fake motor: rotor electrical angle is pp*m + off, the encoder reports joint angle s*m + c in 12-bit steps.
// the field comes back out of the real foc_svpwm duties, so a sign slip in either module shows up
typedef struct { float m, w, off, s, c, stuck; } plant_t;

static float plant_angle(const plant_t *p) {
    const float q = 6.2831853f / 4096;
    return roundf((p->s * p->m + p->c) / q) * q;
}

static void plant_run(plant_t *p, float uq, float theta_e, int ticks) { // ticks of 200 us
    float d[3];
    foc_svpwm(uq, theta_e, SUPPLY_V, d);
    float alpha = SUPPLY_V * (2 * d[0] - d[1] - d[2]) / 3, beta = SUPPLY_V * (d[1] - d[2]) / 1.7320508f;
    for (int k = 0; k < ticks; k++) {
        float pe = POLE_PAIRS * p->m + p->off;
        float torque = beta * cosf(pe) - alpha * sinf(pe); // q-axis part of the field
        p->w += (40.0f * torque - 20.0f * p->w) * 0.0002f;
        p->m += p->w * 0.0002f * (1.0f - p->stuck);
    }
}

static int run_calib(plant_t *p, calib_t *c) {
    calib_start(c);
    for (int i = 0; i < 10 * CFC_CTRL_HZ; i++) {
        float uq, th;
        int r = calib_step(c, plant_angle(p), &uq, &th);
        if (r != CALIB_RUNNING) return r;
        plant_run(p, uq, th, FOC_HZ / CFC_CTRL_HZ);
    }
    return -ETIMEDOUT;
}

static void test_calib_fake_plant(void) {
    const float offs[] = {0.3f, 2.0f, 4.5f}, cs[] = {-2.0f, 0.0f, 1.0f};
    for (int si = 0; si < 2; si++)
        for (int oi = 0; oi < 3; oi++)
            for (int ci = 0; ci < 3; ci++) {
                plant_t p = {.m = 0.1f * oi, .off = offs[oi], .s = si ? -1.0f : 1.0f, .c = cs[ci]};
                calib_t c;
                TEST_ASSERT_EQUAL_INT(CALIB_DONE, run_calib(&p, &c));
                TEST_ASSERT_EQUAL_FLOAT(p.s, c.dir);
                float want = POLE_PAIRS * p.s * p.c - p.off; // rotor electrical angle is 0 there
                float e = remainderf(c.zero_elec - want, 6.2831853f);
                TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, e);
                // commutate: +uq must raise the joint angle, -uq lower it, whatever the wiring
                for (float uq = -1.0f; uq <= 1.0f; uq += 2.0f) {
                    float a0 = plant_angle(&p);
                    for (int t = 0; t < FOC_HZ / 2; t++) plant_run(&p, uq, calib_theta_e(&c, plant_angle(&p)), 1);
                    TEST_ASSERT_TRUE(uq * (plant_angle(&p) - a0) > 0.5f);
                }
            }
}

static void test_calib_stuck_rotor_fails(void) {
    plant_t p = {.off = 1.0f, .s = 1.0f, .stuck = 1.0f};
    calib_t c;
    TEST_ASSERT_EQUAL_INT(-EIO, run_calib(&p, &c));
}

// ---------- joint ----------
// enabled joint in CLOSED_PID at angle a, calibration skipped
static joint_t joint_closed(float a) {
    joint_t j;
    joint_init(&j, true, true);
    j.calibrated = true;
    joint_step(&j, a, 0.0f);
    TEST_ASSERT_EQUAL_INT(0, joint_enable(&j));
    TEST_ASSERT_EQUAL_INT(JOINT_CLOSED_PID, j.state);
    return j;
}

static void test_joint_enable_guards(void) {
    joint_t j;
    joint_init(&j, true, true);
    TEST_ASSERT_EQUAL_INT(-ERANGE, joint_enable(&j)); // no angle yet
    joint_step(&j, TRAVEL_RAD + 0.1f, 0.0f);
    TEST_ASSERT_EQUAL_INT(-ERANGE, joint_enable(&j));
    joint_step(&j, -TRAVEL_RAD - 0.1f, 0.0f);
    TEST_ASSERT_EQUAL_INT(-ERANGE, joint_enable(&j));
    TEST_ASSERT_EQUAL_INT(JOINT_IDLE, j.state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, j.uq);

    joint_init(&j, true, false); // boot encoder check failed
    joint_step(&j, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(-ENODEV, joint_enable(&j));

    joint_init(&j, true, true); // first enable calibrates
    joint_step(&j, 0.1f, 0.0f);
    TEST_ASSERT_EQUAL_INT(0, joint_enable(&j));
    TEST_ASSERT_EQUAL_INT(JOINT_CALIBRATING, j.state);
}

static void test_joint_bumpless_enable(void) {
    joint_t j = joint_closed(0.3f);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, j.target); // holds where it is
    joint_step(&j, 0.3f, 0.0f);
    // the feedforward holds gravity from the first step, the PID adds nothing at zero error
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, GRAV_K * sinf(0.3f + GRAV_PHI), j.uq);
    TEST_ASSERT_TRUE(GRAV_K != 0.0f); // native env sets it, or this proves nothing
}

static void test_joint_target_and_vlimit_clamp(void) {
    joint_t j = joint_closed(0.0f);
    TEST_ASSERT_EQUAL_INT(0, joint_set_target(&j, 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(TRAVEL_RAD, j.target);
    TEST_ASSERT_EQUAL_INT(-EINVAL, joint_set_target(&j, NAN));
    TEST_ASSERT_EQUAL_FLOAT(TRAVEL_RAD, j.target);

    TEST_ASSERT_EQUAL_INT(0, joint_set_vlimit(&j, 100.0f));
    TEST_ASSERT_EQUAL_FLOAT(CLOSEDLOOP_V, j.uq_limit);
    TEST_ASSERT_EQUAL_INT(0, joint_set_vlimit(&j, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(VLIMIT_MIN_V, j.uq_limit);
    TEST_ASSERT_EQUAL_INT(-EINVAL, joint_set_vlimit(&j, INFINITY));
    for (int i = 0; i < 1000; i++) {
        joint_step(&j, -TRAVEL_RAD, 0.0f); // big error held: integral and output pinned
        TEST_ASSERT_TRUE(fabsf(j.uq) <= VLIMIT_MIN_V);
    }
}

// the PI is clamped at PID_VEL_LIMIT like the bring-up; only the feedforward goes past it
static void test_joint_pi_limit(void) {
    joint_t j = joint_closed(0.0f);
    joint_set_target(&j, TRAVEL_RAD);
    for (int i = 0; i < 1000; i++) {
        joint_step(&j, -TRAVEL_RAD, 0.0f);
        TEST_ASSERT_TRUE(fabsf(j.uq - GRAV_K * sinf(-TRAVEL_RAD + GRAV_PHI)) <= PID_VEL_LIMIT + 1e-5f);
    }
    TEST_ASSERT_EQUAL_FLOAT(PID_VEL_LIMIT, j.pid_vel.integral);
}

static void test_joint_fault_zeroes_uq(void) {
    joint_t j = joint_closed(0.0f);
    joint_set_target(&j, 0.4f);
    for (int i = 0; i < 50; i++) joint_step(&j, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(j.uq > 0.1f);
    joint_step(&j, NAN, 0.0f);
    TEST_ASSERT_EQUAL_INT(JOINT_FAULT, j.state);
    TEST_ASSERT_TRUE(j.faults & FAULT_ENCODER_LOST);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, j.uq);
    joint_step(&j, 0.0f, 0.0f); // stays off with a good angle again
    TEST_ASSERT_EQUAL_FLOAT(0.0f, j.uq);

    j = joint_closed(0.0f);
    joint_step(&j, 0.0f, INFINITY);
    TEST_ASSERT_EQUAL_INT(JOINT_FAULT, j.state);
}

static void test_joint_cfc_fallback_bumpless(void) {
    joint_t j = joint_closed(0.0f);
    TEST_ASSERT_EQUAL_INT(0, joint_set_cfc(&j, true));
    TEST_ASSERT_EQUAL_INT(JOINT_CLOSED_CFC, j.state);
    joint_step(&j, 0.1f, 0.0f);
    float uq_cfc = j.uq;
    // error past CFC_MAX_ERR, velocity already at what the angle loop asks: the PID must carry on from
    // the CfC's uq, not jump
    float err = CFC_MAX_ERR + 0.05f;
    float vel = fminf(PID_ANGLE_P * err, VEL_LIMIT);
    joint_step(&j, j.target - err, vel);
    TEST_ASSERT_EQUAL_INT(JOINT_CLOSED_PID, j.state);
    TEST_ASSERT_EQUAL_UINT32(1, j.cfc_fallbacks);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, uq_cfc, j.uq);

    joint_t k;
    joint_init(&k, false, true);
    k.calibrated = true;
    joint_step(&k, 0.0f, 0.0f);
    joint_enable(&k);
    TEST_ASSERT_EQUAL_INT(-ENOTSUP, joint_set_cfc(&k, true)); // golden check failed
}

// ---------- can dispatch ----------
static can_frame_t frame(uint16_t id, uint8_t dlc, float v) {
    can_frame_t f = {.id = id, .dlc = dlc};
    can_put_f32(f.data, v);
    return f;
}

static void test_can_dispatch_commands(void) {
    const uint8_t me = 1;
    can_frame_t f = frame(can_make_id(me, CMD_SET_TARGET), 4, 0.1f);
    can_action_t a = can_dispatch(&f, me);
    TEST_ASSERT_TRUE(a.from_host && a.has_cmd);
    TEST_ASSERT_EQUAL_INT(JCMD_TARGET, a.cmd_out.op);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, a.cmd_out.arg);
    TEST_ASSERT_EQUAL_UINT8(0, a.set_faults);

    f = frame(can_make_id(me, CMD_ENABLE), 0, 0);
    a = can_dispatch(&f, me);
    TEST_ASSERT_TRUE(a.has_cmd && a.clear_faults);
    TEST_ASSERT_EQUAL_INT(JCMD_ENABLE, a.cmd_out.op);
    TEST_ASSERT_EQUAL_INT8(1, a.can_control);

    f = frame(can_make_id(me, CMD_DISABLE), 0, 0);
    a = can_dispatch(&f, me);
    TEST_ASSERT_EQUAL_INT(JCMD_DISABLE, a.cmd_out.op);
    TEST_ASSERT_EQUAL_INT8(-1, a.can_control);

    f = frame(can_make_id(me, CMD_HEARTBEAT), 0, 0); // refreshes the host timer, does nothing else
    a = can_dispatch(&f, me);
    TEST_ASSERT_TRUE(a.from_host && !a.has_cmd && !a.set_faults && !a.can_control);
    TEST_ASSERT_EQUAL_UINT8(CMD_HEARTBEAT, a.cmd);

    f = frame(can_make_id(2, CMD_ENABLE), 0, 0); // the hip's: not ours, not even a host frame
    a = can_dispatch(&f, me);
    TEST_ASSERT_TRUE(!a.from_host && !a.has_cmd && !a.set_faults);
}

static void test_can_dispatch_bad_frames(void) {
    const uint8_t me = 1;
    const can_frame_t bad[] = {
        frame(can_make_id(me, CMD_SET_TARGET), 4, NAN),
        frame(can_make_id(me, CMD_SET_TARGET), 4, INFINITY),
        frame(can_make_id(me, CMD_SET_TARGET), 3, 0.1f), // short
        frame(can_make_id(me, CMD_SET_VLIMIT), 8, 1.0f), // long
        frame(can_make_id(me, 0x1F), 0, 0),              // unknown
        frame(can_make_id(me, MSG_STATUS), 8, 0),        // a leg->host message sent to us
    };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        can_action_t a = can_dispatch(&bad[i], me);
        TEST_ASSERT_TRUE(a.from_host); // still proves the host is alive
        TEST_ASSERT_FALSE(a.has_cmd);  // never reaches the joint
        TEST_ASSERT_EQUAL_UINT8(FAULT_BAD_FRAME, a.set_faults);
    }
}

static void test_can_dispatch_estop(void) {
    for (uint8_t dlc = 0; dlc <= 8; dlc += 8) {
        can_frame_t f = frame(CAN_ID_ESTOP, dlc, 0);
        can_action_t a = can_dispatch(&f, 1);
        TEST_ASSERT_TRUE(a.has_cmd);
        TEST_ASSERT_EQUAL_INT(JCMD_DISABLE, a.cmd_out.op);
        TEST_ASSERT_EQUAL_UINT8(FAULT_ESTOP, a.set_faults);
        TEST_ASSERT_EQUAL_INT8(-1, a.can_control);
        TEST_ASSERT_FALSE(a.from_host); // an E-STOP can't keep a CAN-enabled leg alive
    }
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
    RUN_TEST(test_vel_tracks_quantised_sine);
    RUN_TEST(test_calib_fake_plant);
    RUN_TEST(test_calib_stuck_rotor_fails);
    RUN_TEST(test_joint_enable_guards);
    RUN_TEST(test_joint_bumpless_enable);
    RUN_TEST(test_joint_target_and_vlimit_clamp);
    RUN_TEST(test_joint_pi_limit);
    RUN_TEST(test_joint_fault_zeroes_uq);
    RUN_TEST(test_joint_cfc_fallback_bumpless);
    RUN_TEST(test_can_dispatch_commands);
    RUN_TEST(test_can_dispatch_bad_frames);
    RUN_TEST(test_can_dispatch_estop);
    return UNITY_END();
}
