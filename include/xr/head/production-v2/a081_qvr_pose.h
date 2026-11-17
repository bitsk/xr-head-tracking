// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "a081_tracking_wire.h"
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include <math.h>
#include <string.h>

// QVR: x up, y left, z toward the user. XR: x right, y up, z toward the user.
// The one metric body state is converted by C=Rz(+90deg), q_xr=C*q_raw*C^-1.
// Future prediction and the rotating factory body-to-eye offset belong to the
// runtime. Confidence never selects another estimator.
static inline bool a081_qvr_finite3(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static inline bool
a081_qvr_pose(const struct a081_tracking_packet *p, uint64_t now, struct xrt_space_relation *out)
{
	memset(out, 0, sizeof(*out));
	out->pose.orientation.w = 1.0f;
	// Identity, epoch and representability checks, not motion-quality gates.
	// CAMERA_SUBMITTED and wall-clock age do not determine OEM pose usability.
	const unsigned required = A081_TRACKING_RAW_POSE | A081_TRACKING_CLOCK;
	if ((p->flags & required) != required || p->pose_read_status != A081_POSE_COPIED ||
	    p->pose_index >= 80 || !p->pose_boottime_ns || p->pose_boottime_ns != p->gyro_boottime_ns ||
	    !p->sample_monotonic_ns || p->sample_monotonic_ns > p->send_monotonic_ns ||
	    p->send_monotonic_ns > now) return false;
	double norm2 = 0.0;
	for (unsigned i = 0; i < 4; ++i) {
		if (!isfinite(p->rotation_xyzw[i])) return false;
		double v = p->rotation_xyzw[i];
		norm2 += v * v;
	}
	// The shipped SDK rejects an all-zero quaternion. Normalize every finite
	// nonzero representation for XRT; no arbitrary norm-error cutoff.
	if (!(norm2 > 0.0)) return false;
	double invnorm = 1.0 / sqrt(norm2);
	out->pose.orientation = (struct xrt_quat){-p->rotation_xyzw[1] * invnorm,
	                                        p->rotation_xyzw[0] * invnorm,
	                                        p->rotation_xyzw[2] * invnorm,
	                                        p->rotation_xyzw[3] * invnorm};
	out->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	// QVRTypes state bits: relocating=1, suspended=2, tracking=4, fatal=8.
	// SDK pose availability and tracking confidence are distinct. Suspended or
	// fatal state must not be advertised as tracked merely because q is usable.
	bool tracked = (p->tracking_state & A081_OEM_TRACKING) && !(p->tracking_state & (A081_OEM_SUSPENDED | A081_OEM_FATAL_ERROR));
	if (tracked) out->relation_flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	// SXR nonzero-ahead translation uses translation_s + translation_b*dt/1.021.
	// Export this same prediction anchor and derivative; do not extrapolate here.
	if (a081_qvr_finite3(p->translation_s)) {
		out->pose.position = (struct xrt_vec3){-p->translation_s[1], p->translation_s[0], p->translation_s[2]};
		out->relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
		// MIX gMinPoseQuality=0.2 only removes positional tracking confidence.
		if (tracked && isfinite(p->pose_quality) && p->pose_quality >= 0.2f)
			out->relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	if (a081_qvr_finite3(p->translation_b)) {
		out->linear_velocity = (struct xrt_vec3){-p->translation_b[1] / 1.021f,
		                                         p->translation_b[0] / 1.021f, p->translation_b[2] / 1.021f};
		out->relation_flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
	}
	// Only angular_s is consumed by this runtime's constant-velocity interface.
	// Unused higher-order coefficients cannot veto an otherwise usable rotation.
	if (a081_qvr_finite3(p->angular_s)) {
		const struct xrt_vec3 body = {-p->angular_s[1], p->angular_s[0], p->angular_s[2]};
		math_quat_rotate_vec3(&out->pose.orientation, &body, &out->angular_velocity);
		if (isfinite(out->angular_velocity.x) && isfinite(out->angular_velocity.y) && isfinite(out->angular_velocity.z))
			out->relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
		else
			out->angular_velocity = (struct xrt_vec3){0};
	}
	return true;
}
