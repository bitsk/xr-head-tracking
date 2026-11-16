// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// Volatile single-writer real-gyro history. Atomic words avoid torn reads when
// a renderer samples concurrently with the sole IIO reader. No DSP pose is
// manufactured here; consumers integrate only after a valid measured anchor.
#define A081_IMU_HISTORY_MAGIC UINT64_C(0x314d554938314141)
#define A081_IMU_HISTORY_COUNT 256u
struct a081_imu_entry { uint64_t guard, mono_ns, gyro_bits[3]; };
struct a081_imu_history {
    uint64_t magic, published;
    struct a081_imu_entry entries[A081_IMU_HISTORY_COUNT];
};
struct a081_imu_sample { uint64_t mono_ns; double gyro[3]; };
static inline uint64_t a081_imu_load(const uint64_t *v) { return __atomic_load_n(v, __ATOMIC_SEQ_CST); }
static inline void a081_imu_store(uint64_t *v, uint64_t n) { __atomic_store_n(v, n, __ATOMIC_SEQ_CST); }
static inline void a081_imu_append(struct a081_imu_history *h, uint64_t mono_ns, const float gyro[3])
{
    uint64_t sequence = a081_imu_load(&h->published) + 1;
    struct a081_imu_entry *e = &h->entries[sequence % A081_IMU_HISTORY_COUNT];
    a081_imu_store(&e->guard, sequence * 2 - 1);
    a081_imu_store(&e->mono_ns, mono_ns);
    for (unsigned j = 0; j < 3; ++j) {
        double v = gyro[j]; uint64_t bits; memcpy(&bits, &v, sizeof(bits));
        a081_imu_store(&e->gyro_bits[j], bits);
    }
    a081_imu_store(&e->guard, sequence * 2);
    a081_imu_store(&h->published, sequence);
}
static inline bool a081_imu_read(const struct a081_imu_history *h, uint64_t sequence, struct a081_imu_sample *out)
{
    const struct a081_imu_entry *e = &h->entries[sequence % A081_IMU_HISTORY_COUNT];
    if (a081_imu_load(&e->guard) != sequence * 2) return false;
    out->mono_ns = a081_imu_load(&e->mono_ns);
    for (unsigned j = 0; j < 3; ++j) {
        uint64_t bits = a081_imu_load(&e->gyro_bits[j]); memcpy(&out->gyro[j], &bits, sizeof(bits));
        if (!isfinite(out->gyro[j]) || fabs(out->gyro[j]) > 35.) return false;
    }
    return a081_imu_load(&e->guard) == sequence * 2;
}
// Right-multiply by the integral of body angular velocity (xyzw convention).
static inline void a081_imu_rotate(double q[4], const double w[3], double dt)
{
    double speed = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
    double scale = speed > 1e-12 ? sin(speed*dt*.5)/speed : dt*.5;
    double x=w[0]*scale, y=w[1]*scale, z=w[2]*scale, r=cos(speed*dt*.5);
    double next[4] = {q[3]*x+q[0]*r+q[1]*z-q[2]*y, q[3]*y-q[0]*z+q[1]*r+q[2]*x,
                      q[3]*z+q[0]*y-q[1]*x+q[2]*r, q[3]*r-q[0]*x-q[1]*y-q[2]*z};
    memcpy(q,next,sizeof(next));
}
// q is in XR basis, samples are Android portrait. Integrate only observed IMU
// times, never beyond latest. Caller keeps the original pose if history is
// stale, incomplete, overwritten, or from a different clock epoch.
static inline bool a081_imu_advance(const struct a081_imu_history *h, uint64_t anchor, uint64_t now,
                                   double q[4], uint64_t *latest, double body_velocity[3])
{
    if (!anchor || now < anchor || now-anchor > UINT64_C(100000000) ||
        a081_imu_load(&h->magic) != A081_IMU_HISTORY_MAGIC) return false;
    uint64_t end=a081_imu_load(&h->published);
    if (!end) return false;
    struct a081_imu_sample tail;
    if (!a081_imu_read(h,end,&tail) || tail.mono_ns<=anchor || tail.mono_ns>now ||
        now-tail.mono_ns>UINT64_C(10000000)) return false;
    uint64_t begin=end>A081_IMU_HISTORY_COUNT-1 ? end-(A081_IMU_HISTORY_COUNT-1) : 1;
    // Find the sample bracketing this anchor first. A gap before a new OEM
    // anchor is already covered by that anchor and must not disable live IMU
    // advancement until the entire ring turns over. Still reject every gap
    // within the interval that we actually integrate.
    struct a081_imu_sample previous={0};
    uint64_t first=end;
    for (;;) {
        if (!a081_imu_read(h,first,&previous)) return false;
        if (previous.mono_ns<=anchor) break;
        if (first==begin) return false;
        --first;
    }
    double result[4]; memcpy(result,q,sizeof(result));
    uint64_t cursor=anchor;
    for (uint64_t i=first+1;i<=end;++i) {
        struct a081_imu_sample sample;
        if (!a081_imu_read(h,i,&sample)) return false;
        if (sample.mono_ns<=previous.mono_ns || sample.mono_ns-previous.mono_ns>UINT64_C(5000000)) return false;
        double fraction=(double)(cursor-previous.mono_ns)/(double)(sample.mono_ns-previous.mono_ns);
        double avg[3];
        for (unsigned j=0;j<3;++j) avg[j]=.5*(previous.gyro[j]+fraction*(sample.gyro[j]-previous.gyro[j])+sample.gyro[j]);
        double body[3]={-avg[1],avg[0],avg[2]};
        a081_imu_rotate(result,body,(double)(sample.mono_ns-cursor)/1e9);
        cursor=sample.mono_ns; previous=sample;
    }
    if (cursor!=tail.mono_ns) return false;
    memcpy(q,result,sizeof(result)); *latest=cursor;
    body_velocity[0]=-tail.gyro[1]; body_velocity[1]=tail.gyro[0]; body_velocity[2]=tail.gyro[2];
    return true;
}
