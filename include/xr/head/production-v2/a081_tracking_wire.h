// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <stddef.h>
#include <stdint.h>

// Local AF_UNIX/SOCK_SEQPACKET ABI, little endian on A081 ARM32/ARM64.
// A packet contains real input and optional raw vendor output, never Xr flags.
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "A081 tracking wire format requires a little-endian target"
#endif
#define A081_TRACKING_MAGIC UINT32_C(0x54523841)
#define A081_TRACKING_VERSION 2u
#define A081_TRACKING_IMU 1u
#define A081_TRACKING_CLOCK 2u
#define A081_TRACKING_RAW_POSE 4u
#define A081_TRACKING_CAMERA_SUBMITTED 8u
#define A081_TRACKING_KNOWN_FLAGS 15u

// Original QVR pose state is a bit mask, separate from the tracker enum.
enum a081_oem_tracking_state {
    A081_OEM_UNINITIALIZED = 0,
    A081_OEM_RELOCATING = 1,
    A081_OEM_SUSPENDED = 2,
    A081_OEM_TRACKING = 4,
    A081_OEM_FATAL_ERROR = 8,
};

enum a081_pose_read_status {
	A081_POSE_NOT_READ = 0,
	A081_POSE_COPIED = 1,
	A081_POSE_BAD_INDEX = 2,
	A081_POSE_CHANGED_DURING_COPY = 3,
	A081_POSE_TIMESTAMP_MISMATCH = 4,
};

struct a081_tracking_packet {
	uint32_t magic, version, bytes, flags;
	uint64_t session_id, sequence;
	uint64_t gyro_boottime_ns, accel_boottime_ns;
	uint64_t sample_monotonic_ns, send_monotonic_ns;
	uint64_t pose_boottime_ns, last_camera_boottime_ns;
	float gyro[3], accel[3]; // rad/s and m/s^2, original native Android-portrait axes.
	float rotation_xyzw[4], translation[3], pose_quality;
	uint32_t tracking_state, warning_flags, pose_index, clock_bracket_ns;
	uint32_t pose_read_status, reserved[3];
	// OEM prediction coefficients from the same coherent 192-byte pose record.
	float angular_s[3], angular_b[3], angular_bdt[3], angular_bdt2[3];
	float translation_s[3], translation_b[3];
};

#ifdef __cplusplus
static_assert(sizeof(a081_tracking_packet) == 240);
static_assert(offsetof(a081_tracking_packet, gyro) == 80);
static_assert(offsetof(a081_tracking_packet, tracking_state) == 136);
#else
_Static_assert(sizeof(struct a081_tracking_packet) == 240, "tracking packet size");
_Static_assert(offsetof(struct a081_tracking_packet, gyro) == 80, "tracking input offset");
_Static_assert(offsetof(struct a081_tracking_packet, tracking_state) == 136, "tracking state offset");
#endif
