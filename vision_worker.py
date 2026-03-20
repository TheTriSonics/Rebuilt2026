"""Vision processing worker — runs as a separate OS process.

Reads PhotonVision camera results, estimates and fuses poses, and publishes
the fused result to NetworkTables for the main robot process to consume.

Launched from robot.py via subprocess.Popen with stdin piped so the worker
dies automatically when the parent exits.

IMPORTANT: This file must NOT import wpilib or hal — doing so on the
roboRIO will kill the main robot process.
"""

import math
import os
import sys
import threading
import time

import ntcore
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation3d,
)
from wpimath import units
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator


def _watch_parent():
    """Block until the parent process closes our stdin pipe, then exit."""
    try:
        sys.stdin.read()
    except Exception:
        pass
    os._exit(0)


def _fuse_estimates(estimates):
    """Inverse-variance weighted fusion of multiple pose estimates.

    Each element: (Pose2d, timestamp, (std_x, std_y, std_rot))
    Returns (fused_pose, newest_ts, (std_x, std_y, std_rot)).
    """
    if len(estimates) == 1:
        return estimates[0]

    estimates.sort(key=lambda e: e[1])
    newest_ts = estimates[-1][1]

    # Skip time-alignment across cameras — inter-camera timestamp deltas
    # are sub-millisecond and the velocity correction is negligible.

    sum_inv_var_x = 0.0
    sum_inv_var_y = 0.0
    sum_inv_var_rot = 0.0
    weighted_x = 0.0
    weighted_y = 0.0
    weighted_sin = 0.0
    weighted_cos = 0.0

    for pose, _ts, stds in estimates:
        inv_var_x = 1.0 / (stds[0] ** 2) if stds[0] > 0 else 1e6
        inv_var_y = 1.0 / (stds[1] ** 2) if stds[1] > 0 else 1e6
        inv_var_rot = 1.0 / (stds[2] ** 2) if stds[2] > 0 else 1e6

        sum_inv_var_x += inv_var_x
        sum_inv_var_y += inv_var_y
        sum_inv_var_rot += inv_var_rot

        weighted_x += pose.x * inv_var_x
        weighted_y += pose.y * inv_var_y
        rot = pose.rotation().radians()
        weighted_sin += math.sin(rot) * inv_var_rot
        weighted_cos += math.cos(rot) * inv_var_rot

    fused_x = weighted_x / sum_inv_var_x
    fused_y = weighted_y / sum_inv_var_y
    fused_rot = math.atan2(weighted_sin, weighted_cos)

    fused_std_x = math.sqrt(1.0 / sum_inv_var_x)
    fused_std_y = math.sqrt(1.0 / sum_inv_var_y)
    fused_std_rot = math.sqrt(1.0 / sum_inv_var_rot)

    fused_pose = Pose2d(fused_x, fused_y, Rotation2d(fused_rot))
    return (fused_pose, newest_ts, (fused_std_x, fused_std_y, fused_std_rot))


def main():
    # Die when parent process exits (stdin pipe closes)
    threading.Thread(target=_watch_parent, daemon=True).start()

    # --- NetworkTables (client mode, connects to robot process) ---
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.setServer("127.0.0.1")
    nt.startClient4("vision_worker")

    print("[vision_worker] Waiting for NetworkTables connection...")
    deadline = time.monotonic() + 10.0
    while not nt.isConnected() and time.monotonic() < deadline:
        time.sleep(0.1)
    if nt.isConnected():
        print("[vision_worker] Connected to NetworkTables")
    else:
        print("[vision_worker] WARNING: NT connection timeout, continuing anyway")

    # --- Cameras (same configuration as VisionComponent) ---
    camera_rr = PhotonCamera("Rear_Right")
    camera_rl = PhotonCamera("Rear_Left")
    camera_back = PhotonCamera("Rear")

    camera_rr_offset = Transform3d(
        Translation3d(
            units.inchesToMeters(-12.0),
            units.inchesToMeters(-10.5),
            units.inchesToMeters(8.5),
        ),
        Rotation3d.fromDegrees(0.0, -22.0, -85.0),
    )
    camera_rl_offset = Transform3d(
        Translation3d(
            units.inchesToMeters(-12.0),
            units.inchesToMeters(10.5),
            units.inchesToMeters(8.5),
        ),
        Rotation3d.fromDegrees(0.0, -22.0, 85.0),
    )
    camera_back_offset = Transform3d(
        Translation3d(
            units.inchesToMeters(-10.0),
            units.inchesToMeters(0.0),
            units.inchesToMeters(7.5),
        ),
        Rotation3d.fromDegrees(0.0, -10.0, 180.0),
    )

    field = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    pose_estimator_rr = PhotonPoseEstimator(field, camera_rr_offset)
    pose_estimator_rl = PhotonPoseEstimator(field, camera_rl_offset)
    pose_estimator_back = PhotonPoseEstimator(field, camera_back_offset)

    cameras = [camera_rr, camera_rl, camera_back]
    pose_estimators = [pose_estimator_rr, pose_estimator_rl, pose_estimator_back]

    # --- Configuration ---
    linear_baseline_std = 0.10
    angular_baseline_std = math.radians(10)

    # --- NT publishers (output to robot process) ---
    worker_table = nt.getTable("/vision_worker")

    # Fused estimate: [x, y, rot_rad, timestamp, std_x, std_y, std_rot]
    fused_pub = worker_table.getDoubleArrayTopic("fused_estimate").publish()

    # Per-camera debug poses (same topics the old VisionComponent used)
    cam_publishers = [
        nt.getStructTopic("/components/vision/pose_rr", Pose2d).publish(),
        nt.getStructTopic("/components/vision/pose_rl", Pose2d).publish(),
        nt.getStructTopic("/components/vision/pose_back", Pose2d).publish(),
    ]

    # --- NT subscribers (input from robot process) ---
    enabled_sub = worker_table.getBooleanTopic("robot_enabled").subscribe(False)
    robot_pose_sub = worker_table.getDoubleArrayTopic("robot_pose").subscribe(
        [0.0, 0.0, 0.0]
    )

    # --- Main loop ---
    print("[vision_worker] Starting processing loop")
    while True:
        robot_enabled = enabled_sub.get()
        rp = robot_pose_sub.get()
        robot_pose = Pose2d(rp[0], rp[1], Rotation2d(rp[2]))

        valid_estimates = []

        for cam, pose_est, pub in zip(cameras, pose_estimators, cam_publishers):
            try:
                res = cam.getLatestResult()
            except Exception:
                continue

            if not res:
                continue

            targets = res.getTargets()
            if not targets:
                continue

            best_target = res.getBestTarget()
            if best_target and best_target.poseAmbiguity > 0.2:
                continue

            pupdate = pose_est.estimateCoprocMultiTagPose(res)
            if pupdate is None:
                continue

            ts = pupdate.timestampSeconds
            pose3d = pupdate.estimatedPose
            twod_pose = pose3d.toPose2d()
            pub.set(twod_pose)

            # Z-height rejection
            if abs(pose3d.Z()) > 0.2:
                continue

            # Distance from current pose rejection (only when enabled)
            if robot_enabled:
                dist = robot_pose.relativeTo(twod_pose).translation().norm()
                if dist > 1.0:
                    continue

            # Distance-based std devs
            tag_count = len(targets)
            total_dist = sum(
                t.getBestCameraToTarget().translation().norm() for t in targets
            )
            avg_dist = total_dist / tag_count
            if avg_dist > 2.0 and robot_enabled:
                continue

            std_factor = (avg_dist ** 2) / tag_count
            std_xy = linear_baseline_std * std_factor
            std_rot = angular_baseline_std * std_factor
            if robot_enabled:
                std_xy = max(std_xy, 0.05)
                std_rot = max(std_rot, 0.5)

            valid_estimates.append((twod_pose, ts, (std_xy, std_xy, std_rot)))

        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, ts, stds = valid_estimates[0]
            else:
                pose, ts, stds = _fuse_estimates(valid_estimates)

            fused_pub.set([
                pose.x, pose.y, pose.rotation().radians(),
                ts,
                stds[0], stds[1], stds[2],
            ])
            nt.flush()

        time.sleep(0.005)  # ~200Hz polling


if __name__ == "__main__":
    main()
