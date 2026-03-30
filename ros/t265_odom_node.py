#!/usr/bin/env python3
import math
import queue

import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


G = 9.80665


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(q, qv), quaternion_conjugate(q))
    return rotated[:3]


def vector_norm(v):
    return math.sqrt(sum(x * x for x in v))


class ImuPosVelKalman:
    def __init__(self, process_accel_noise, position_measurement_noise, velocity_measurement_noise):
        self.x = np.zeros((6, 1), dtype=float)
        self.p = np.eye(6, dtype=float) * 1e-2
        self.initialized = False
        self.q_accel = float(process_accel_noise)
        self.r = np.diag(
            [
                position_measurement_noise,
                position_measurement_noise,
                position_measurement_noise,
                velocity_measurement_noise,
                velocity_measurement_noise,
                velocity_measurement_noise,
            ]
        ).astype(float)
        self.h = np.eye(6, dtype=float)
        self.h_vel = np.array(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def initialize(self, position, velocity):
        self.x[0:3, 0] = position
        self.x[3:6, 0] = velocity
        self.initialized = True

    def predict_with_accel(self, dt, acceleration):
        if not self.initialized:
            return
        dt = max(float(dt), 1e-4)
        ax, ay, az = acceleration
        f = np.eye(6, dtype=float)
        f[0, 3] = dt
        f[1, 4] = dt
        f[2, 5] = dt
        b = np.array(
            [
                [0.5 * dt * dt, 0.0, 0.0],
                [0.0, 0.5 * dt * dt, 0.0],
                [0.0, 0.0, 0.5 * dt * dt],
                [dt, 0.0, 0.0],
                [0.0, dt, 0.0],
                [0.0, 0.0, dt],
            ],
            dtype=float,
        )
        u = np.array([[ax], [ay], [az]], dtype=float)

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        q_block = np.array([[dt4 / 4.0, dt3 / 2.0], [dt3 / 2.0, dt2]], dtype=float) * self.q_accel
        q = np.zeros((6, 6), dtype=float)
        q[np.ix_([0, 3], [0, 3])] = q_block
        q[np.ix_([1, 4], [1, 4])] = q_block
        q[np.ix_([2, 5], [2, 5])] = q_block

        self.x = f @ self.x + b @ u
        self.p = f @ self.p @ f.T + q

    def update(self, position, velocity):
        z = np.array(
            [[position[0]], [position[1]], [position[2]], [velocity[0]], [velocity[1]], [velocity[2]]],
            dtype=float,
        )
        if not self.initialized:
            self.initialize(position, velocity)
            return

        y = z - self.h @ self.x
        s = self.h @ self.p @ self.h.T + self.r
        k = self.p @ self.h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.p = (np.eye(6, dtype=float) - k @ self.h) @ self.p

    def update_velocity_only(self, velocity, measurement_noise):
        if not self.initialized:
            return
        z = np.array([[velocity[0]], [velocity[1]], [velocity[2]]], dtype=float)
        r = np.eye(3, dtype=float) * measurement_noise
        y = z - self.h_vel @ self.x
        s = self.h_vel @ self.p @ self.h_vel.T + r
        k = self.p @ self.h_vel.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.p = (np.eye(6, dtype=float) - k @ self.h_vel) @ self.p

    def clamp_state(self, position=None, velocity=None):
        if position is not None:
            self.x[0:3, 0] = np.array(position, dtype=float)
        if velocity is not None:
            self.x[3:6, 0] = np.array(velocity, dtype=float)

    def current_position(self):
        return tuple(float(v) for v in self.x[0:3, 0])

    def current_velocity(self):
        return tuple(float(v) for v in self.x[3:6, 0])


def find_tracking_sensor():
    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        raise RuntimeError("No RealSense device detected")

    dev = devices[0]
    rospy.loginfo(
        "T265 device: %s serial=%s fw=%s",
        dev.get_info(rs.camera_info.name),
        dev.get_info(rs.camera_info.serial_number),
        dev.get_info(rs.camera_info.firmware_version),
    )

    for sensor in dev.query_sensors():
        if sensor.get_info(rs.camera_info.name) == "Tracking Module":
            return sensor

    raise RuntimeError("Tracking Module sensor not found")


def find_tracking_profiles(sensor):
    selected = {}
    for profile in sensor.get_stream_profiles():
        stream_type = profile.stream_type()
        fmt = profile.format()
        if stream_type == rs.stream.pose:
            selected["pose"] = profile
        elif stream_type == rs.stream.accel and fmt == rs.format.motion_xyz32f:
            selected["accel"] = profile
        elif stream_type == rs.stream.gyro and fmt == rs.format.motion_xyz32f:
            selected["gyro"] = profile

    missing = [name for name in ("pose", "accel", "gyro") if name not in selected]
    if missing:
        raise RuntimeError("Missing tracking streams: %s" % ", ".join(missing))
    return selected


def build_covariances(data, scale_boost=1.0):
    tracker_confidence = float(getattr(data, "tracker_confidence", 0))
    mapper_confidence = float(getattr(data, "mapper_confidence", 0))
    conf_scale = max(0.0, 3.0 - min(tracker_confidence, mapper_confidence)) * scale_boost

    pos_cov = 1e-3 + conf_scale * 1e-1
    rot_cov = 1e-3 + conf_scale * 5e-2
    vel_cov = 1e-2 + conf_scale * 1e-1
    ang_cov = 1e-2 + conf_scale * 5e-2

    pose_covariance = [0.0] * 36
    pose_covariance[0] = pos_cov
    pose_covariance[7] = pos_cov
    pose_covariance[14] = pos_cov
    pose_covariance[21] = rot_cov
    pose_covariance[28] = rot_cov
    pose_covariance[35] = rot_cov

    twist_covariance = [0.0] * 36
    twist_covariance[0] = vel_cov
    twist_covariance[7] = vel_cov
    twist_covariance[14] = vel_cov
    twist_covariance[21] = ang_cov
    twist_covariance[28] = ang_cov
    twist_covariance[35] = ang_cov
    return pose_covariance, twist_covariance


def build_odometry_message(frame_id, child_frame_id, stamp, position, orientation, linear_velocity, angular_velocity, pose_covariance, twist_covariance):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose = Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3],
        ),
    )
    msg.twist.twist = Twist(
        linear=Vector3(
            x=linear_velocity[0],
            y=linear_velocity[1],
            z=linear_velocity[2],
        ),
        angular=Vector3(
            x=angular_velocity[0],
            y=angular_velocity[1],
            z=angular_velocity[2],
        ),
    )
    msg.pose.covariance = pose_covariance
    msg.twist.covariance = twist_covariance
    return msg


def main():
    rospy.init_node("t265_odom_node")

    odom_topic = rospy.get_param("~odom_topic", "/t265/odom/filtered")
    raw_odom_topic = rospy.get_param("~raw_odom_topic", "/t265/odom/raw")
    frame_id = rospy.get_param("~frame_id", "odom")
    child_frame_id = rospy.get_param("~child_frame_id", "t265_pose_frame")
    reset_origin = rospy.get_param("~reset_origin", True)
    publish_rate_hz = float(rospy.get_param("~publish_rate", 30.0))
    frame_queue_depth = int(rospy.get_param("~frame_queue_depth", 1000))
    process_accel_noise = float(rospy.get_param("~process_accel_noise", 1.2))
    position_measurement_noise = float(rospy.get_param("~position_measurement_noise", 3e-4))
    velocity_measurement_noise = float(rospy.get_param("~velocity_measurement_noise", 1.5e-3))
    static_linear_threshold = float(rospy.get_param("~static_linear_threshold", 0.008))
    static_angular_threshold = float(rospy.get_param("~static_angular_threshold", 0.008))
    static_accel_threshold = float(rospy.get_param("~static_accel_threshold", 0.12))
    static_position_deadband = float(rospy.get_param("~static_position_deadband", 2.5e-4))
    static_lock_frames = int(rospy.get_param("~static_lock_frames", 6))
    gravity_alpha = float(rospy.get_param("~gravity_alpha", 0.02))
    zupt_velocity_noise = float(rospy.get_param("~zupt_velocity_noise", 1e-5))
    min_confidence_to_update = float(rospy.get_param("~min_confidence_to_update", 0.0))
    accel_gain_x = float(rospy.get_param("~accel_gain_x", 0.35))
    accel_gain_y = float(rospy.get_param("~accel_gain_y", 0.35))
    accel_gain_z = float(rospy.get_param("~accel_gain_z", 0.01))

    raw_publisher = rospy.Publisher(raw_odom_topic, Odometry, queue_size=50)
    filtered_publisher = rospy.Publisher(odom_topic, Odometry, queue_size=50)

    sensor = find_tracking_sensor()
    profiles = find_tracking_profiles(sensor)
    frames = queue.Queue(maxsize=max(frame_queue_depth, 1))

    def callback(frame):
        try:
            stream_type = frame.profile.stream_type()
            timestamp = frame.get_timestamp() * 1e-3
            if stream_type == rs.stream.pose:
                item = ("pose", frame.as_pose_frame(), timestamp)
            elif stream_type == rs.stream.accel:
                item = ("accel", frame.as_motion_frame().get_motion_data(), timestamp)
            elif stream_type == rs.stream.gyro:
                item = ("gyro", frame.as_motion_frame().get_motion_data(), timestamp)
            else:
                return
            frames.put_nowait(item)
        except queue.Full:
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
            try:
                frames.put_nowait(item)
            except Exception:
                pass

    origin_t = None
    origin_q = None
    current_rel_q = None
    latest_linear_velocity = (0.0, 0.0, 0.0)
    latest_angular_velocity = (0.0, 0.0, 0.0)
    latest_position = (0.0, 0.0, 0.0)
    latest_pose_data = None
    latest_raw_position = (0.0, 0.0, 0.0)
    latest_raw_orientation = (0.0, 0.0, 0.0, 1.0)
    latest_raw_linear_velocity = (0.0, 0.0, 0.0)
    latest_raw_angular_velocity = (0.0, 0.0, 0.0)
    filtered_position = (0.0, 0.0, 0.0)
    gravity_origin = None
    latest_accel_residual_norm = 0.0
    latest_min_confidence = 3.0
    static_frame_count = 0
    last_filter_time = None

    kalman = ImuPosVelKalman(process_accel_noise, position_measurement_noise, velocity_measurement_noise)

    sensor.open([profiles["pose"], profiles["accel"], profiles["gyro"]])
    sensor.start(callback)
    rospy.loginfo("T265 tracking started, publishing raw odometry to %s", raw_odom_topic)
    rospy.loginfo("T265 tracking started, publishing filtered odometry to %s", odom_topic)

    try:
        rate = rospy.Rate(max(publish_rate_hz, 1.0))
        while not rospy.is_shutdown():
            processed = False
            while True:
                try:
                    item_type, payload, sensor_time = frames.get_nowait()
                except queue.Empty:
                    break

                processed = True

                if item_type == "gyro":
                    raw_gyro = (payload.x, payload.y, payload.z)
                    if reset_origin and current_rel_q is not None:
                        latest_angular_velocity = rotate_vector(current_rel_q, raw_gyro)
                    else:
                        latest_angular_velocity = raw_gyro
                    continue

                if item_type == "accel":
                    if current_rel_q is None:
                        continue

                    raw_accel = (payload.x, payload.y, payload.z)
                    if reset_origin:
                        accel_origin = rotate_vector(current_rel_q, raw_accel)
                    else:
                        accel_origin = raw_accel

                    if gravity_origin is None:
                        gravity_origin = accel_origin

                    is_quasi_static = vector_norm(latest_linear_velocity) < static_linear_threshold and vector_norm(latest_angular_velocity) < static_angular_threshold
                    if is_quasi_static:
                        gravity_origin = tuple(
                            (1.0 - gravity_alpha) * a + gravity_alpha * b
                            for a, b in zip(gravity_origin, accel_origin)
                        )

                    linear_accel = tuple(a - g for a, g in zip(accel_origin, gravity_origin))
                    latest_accel_residual_norm = vector_norm(linear_accel)
                    linear_accel = (
                        linear_accel[0] * accel_gain_x,
                        linear_accel[1] * accel_gain_y,
                        linear_accel[2] * accel_gain_z,
                    )

                    if last_filter_time is None:
                        last_filter_time = sensor_time
                        continue

                    dt = max(sensor_time - last_filter_time, 1e-3)
                    last_filter_time = sensor_time

                    if is_quasi_static:
                        kalman.update_velocity_only((0.0, 0.0, 0.0), zupt_velocity_noise)
                        filtered_position = kalman.current_position()
                        latest_linear_velocity = (0.0, 0.0, 0.0)
                        continue

                    kalman.predict_with_accel(dt, linear_accel)
                    filtered_position = kalman.current_position()
                    latest_linear_velocity = kalman.current_velocity()
                    continue

                pose_frame = payload
                data = pose_frame.get_pose_data()
                latest_pose_data = data

                t = (data.translation.x, data.translation.y, data.translation.z)
                q = (data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w)
                raw_linear_vel = (data.velocity.x, data.velocity.y, data.velocity.z)
                raw_angular_vel = (
                    data.angular_velocity.x,
                    data.angular_velocity.y,
                    data.angular_velocity.z,
                )

                if origin_t is None:
                    origin_t = t
                    origin_q = q
                    rospy.loginfo("T265 origin fixed from first valid pose frame")

                if reset_origin:
                    delta_world = (
                        t[0] - origin_t[0],
                        t[1] - origin_t[1],
                        t[2] - origin_t[2],
                    )
                    rel_t = rotate_vector(quaternion_conjugate(origin_q), delta_world)
                    rel_q = quaternion_multiply(quaternion_conjugate(origin_q), q)
                    rel_linear_vel = rotate_vector(quaternion_conjugate(origin_q), raw_linear_vel)
                    rel_angular_vel = rotate_vector(quaternion_conjugate(origin_q), raw_angular_vel)
                else:
                    rel_t = t
                    rel_q = q
                    rel_linear_vel = raw_linear_vel
                    rel_angular_vel = raw_angular_vel

                rel_q = normalize_quaternion(rel_q)
                current_rel_q = rel_q
                latest_position = rel_t
                latest_raw_position = rel_t
                latest_raw_orientation = rel_q
                latest_raw_linear_velocity = rel_linear_vel
                latest_raw_angular_velocity = rel_angular_vel
                latest_angular_velocity = rel_angular_vel

                tracker_confidence = float(getattr(data, "tracker_confidence", 0))
                mapper_confidence = float(getattr(data, "mapper_confidence", 0))
                min_confidence = min(tracker_confidence, mapper_confidence)
                latest_min_confidence = min_confidence
                if not kalman.initialized:
                    kalman.initialize(rel_t, rel_linear_vel)
                elif min_confidence >= min_confidence_to_update:
                    kalman.update(rel_t, rel_linear_vel)
                filtered_position = kalman.current_position()
                latest_linear_velocity = kalman.current_velocity()

                if (
                    vector_norm(latest_linear_velocity) < static_linear_threshold
                    and vector_norm(latest_angular_velocity) < static_angular_threshold
                    and latest_accel_residual_norm < static_accel_threshold
                ):
                    static_frame_count += 1
                else:
                    static_frame_count = 0

                if static_frame_count >= static_lock_frames:
                    kalman.update_velocity_only((0.0, 0.0, 0.0), zupt_velocity_noise)
                    kalman.clamp_state(velocity=(0.0, 0.0, 0.0))
                    filtered_position = kalman.current_position()
                    latest_linear_velocity = kalman.current_velocity()
                    latest_linear_velocity = (0.0, 0.0, 0.0)

            if latest_pose_data is not None and current_rel_q is not None:
                stamp = rospy.Time.now()
                raw_pose_covariance, raw_twist_covariance = build_covariances(latest_pose_data, scale_boost=1.0)
                filtered_pose_covariance, filtered_twist_covariance = build_covariances(latest_pose_data, scale_boost=0.6)

                raw_msg = build_odometry_message(
                    frame_id,
                    child_frame_id,
                    stamp,
                    latest_raw_position,
                    latest_raw_orientation,
                    latest_raw_linear_velocity,
                    latest_raw_angular_velocity,
                    raw_pose_covariance,
                    raw_twist_covariance,
                )
                filtered_msg = build_odometry_message(
                    frame_id,
                    child_frame_id,
                    stamp,
                    filtered_position,
                    current_rel_q,
                    latest_linear_velocity,
                    latest_angular_velocity,
                    filtered_pose_covariance,
                    filtered_twist_covariance,
                )

                raw_publisher.publish(raw_msg)
                filtered_publisher.publish(filtered_msg)

            rate.sleep()
    finally:
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            sensor.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
