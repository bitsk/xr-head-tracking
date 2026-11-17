// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "a081_tracking_wire.h"
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include <math.h>
#include <string.h>

// Explicit experimental SDK Portrait profile. The raw QVR pose is converted
// to a device pose, not copied from the SXR consumer's view quaternion.
// QVR: x up, y left, z toward the user. XR: x right, y up, z toward the user.
// C = Rz(+90 degrees), p_xr = C p_raw, q_xr = C q_raw C^-1.
// Scale 1 follows the original metric calibration and SDK arithmetic. Physical
// accuracy and the tracker-to-eye origin remain uncalibrated on this device.
static inline bool
a081_qvr_pose(const struct a081_tracking_packet *p, uint64_t now, struct xrt_space_relation *out)
{
	memset(out, 0, sizeof(*out));
	out->pose.orientation.w = 1.0f;
	if (p->flags != A081_TRACKING_KNOWN_FLAGS || p->pose_read_status != A081_POSE_COPIED ||
	    !isfinite(p->pose_quality) || p->pose_quality < 0.0f || p->pose_quality > 1.0f ||
	    p->tracking_state != 4 || p->pose_index >= 80 ||
	    p->pose_boottime_ns != p->gyro_boottime_ns || !p->last_camera_boottime_ns ||
	    p->last_camera_boottime_ns > p->gyro_boottime_ns ||
	    p->gyro_boottime_ns - p->last_camera_boottime_ns >= A081_TRACKING_MAX_AGE_NS ||
	    !p->sample_monotonic_ns || p->sample_monotonic_ns > p->send_monotonic_ns ||
	    p->send_monotonic_ns > now || now - p->sample_monotonic_ns >= A081_TRACKING_MAX_AGE_NS ||
	    p->clock_bracket_ns > A081_TRACKING_MAX_BRACKET_NS) return false;
	float norm = 0.0f;
	for (unsigned i = 0; i < 4; ++i) {
		if (!isfinite(p->rotation_xyzw[i])) return false;
		norm += p->rotation_xyzw[i] * p->rotation_xyzw[i];
	}
	for (unsigned i = 0; i < 3; ++i) {
		if (!isfinite(p->translation[i]) || !isfinite(p->accel[i]) || !isfinite(p->gyro[i]) ||
		    !isfinite(p->angular_s[i]) || !isfinite(p->angular_b[i]) ||
		    !isfinite(p->angular_bdt[i]) || !isfinite(p->angular_bdt2[i]) ||
		    !isfinite(p->translation_s[i]) || !isfinite(p->translation_b[i])) return false;
	}
	if (!isfinite(norm) || fabsf(norm - 1.0f) > 0.01f) return false;
	out->pose.orientation = (struct xrt_quat){-p->rotation_xyzw[1], p->rotation_xyzw[0],
	                                        p->rotation_xyzw[2], p->rotation_xyzw[3]};
	// SXR's _predict_translation uses translation_s + translation_b * dt / 1.021.
	// Publish that native state at its measured timestamp; SteamVR predicts once.
	out->pose.position = (struct xrt_vec3){-p->translation_s[1], p->translation_s[0], p->translation_s[2]};
	out->linear_velocity = (struct xrt_vec3){-p->translation_b[1] / 1.021f,
	                                         p->translation_b[0] / 1.021f, p->translation_b[2] / 1.021f};
	// SXR _predict_motion right-multiplies the pose by angular_s (device rad/s).
	// Monado requires world velocity; its OpenVR adapter converts it back.
	const struct xrt_vec3 body = {-p->angular_s[1], p->angular_s[0], p->angular_s[2]};
	math_quat_rotate_vec3(&out->pose.orientation, &body, &out->angular_velocity);
	// SXR L_CheckPoseQuality only removes kTrackingPosition when quality is
	// low. A coherent OEM rotation is still used by L_TrackingToReturn;
	// replacing it with a separate IMU filter changes the estimator mid-turn.
	// MIX's shipped svrapi_config.txt sets gMinPoseQuality = 0.2.
	out->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                      XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                      XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	if (p->pose_quality >= 0.2f) out->relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	return true;
}
