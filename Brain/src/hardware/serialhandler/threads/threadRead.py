#!/usr/bin/env python3
# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import logging
import threading
import re
import os
import math
from datetime import datetime, timedelta

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    BatteryLvl,
    ImuData,
    ImuAck,
    InstantConsumption,
    EnableButton,
    ResourceMonitor,
    CurrentSpeed,
    CurrentSteer,
    ShutDownSignal,
    SerialConnectionState,
    CalibPWMData,
    CalibRunDone,
    SteeringLimits,
    AliveSignal
)
from src.utils.messages.messageHandlerSender import messageHandlerSender

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    from geometry_msgs.msg import Vector3Stamped
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from sensor_msgs.msg import Imu
except Exception:  # allow running without ROS2 deps
    rclpy = None
    Node = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    Vector3Stamped = None
    TwistWithCovarianceStamped = None
    Imu = None


class threadRead(ThreadWithStop):
    """This thread reads the data that NUCLEO sends to Raspberry PI."""

    # ===================================== INIT =========================================
    def __init__(self, process, logFile, queueList, logger, debugger=False):
        super(threadRead, self).__init__(pause=0.01)
        self.process = process
        self.logFile = logFile
        self.buffer = ""
        self.queuesList = queueList
        self.logger = logger
        self.debugger = debugger
        self.debug_encoder = os.getenv("SERIAL_DEBUG_ENCODER", "").lower() in ("1", "true", "yes", "y")
        self.event = threading.Event()
        self._init_senders()
        self._init_ros_state()

        self.expectedValues = {
            "kl": "0, 15 or 30", "instant": "1 or 0", "battery": "1 or 0",
            "resourceMonitor": "1 or 0", "imu": "1 or 0", "steer": "between -25 and 25",
            "speed": "between -500 and 500", "break": "between -250 and 250"
        }

        self.warningPattern = r'^(-?[0-9]+)H(-?[0-5]?[0-9])M(-?[0-5]?[0-9])S$'
        self.resourceMonitorPattern = r'Heap \((\d+\.\d+)\);Stack \((\d+\.\d+)\)'

        # error rate limiting
        self.last_error_time = None
        self.error_cooldown = timedelta(seconds=3)

        self.queue_sending()

    def _init_ros_state(self):
        self._ros_node = None

        # IMU pub (기존 유지)
        self._imu_pub = None
        self._imu_topic = "/Imu"
        self._imu_frame = "base_link"
        self._imu_angle_unit = os.getenv("IMU_ANGLE_UNIT", "deg").lower()

        self._imu_apply_vehicle_frame = os.getenv("IMU_APPLY_VEHICLE_FRAME", "1").lower() in ("1", "true", "yes", "y")

        # CCW yaw => + (ROS FLU right-hand rule). keep OFF by default
        self._imu_yaw_invert = os.getenv("IMU_YAW_INVERT", "0").lower() in ("1", "true", "yes", "y")
        self._imu_pitch_invert = os.getenv("IMU_PITCH_INVERT", "1").lower() in ("1", "true", "yes", "y")

        self._imu_to_base_quat = self._quat_normalize((1.0, 0.0, 0.0, 0.0))

        # IMU covariance defaults (spec-based rough)
        accel_noise_density = 190e-6 * 9.80665  # m/s^2/√Hz
        accel_bw_hz = 62.5
        gyro_noise_density = 0.014 * (math.pi / 180.0)  # rad/s/√Hz
        gyro_bw_hz = 32.0
        heading_accuracy_deg = 2.5

        default_orientation_cov = (heading_accuracy_deg * (math.pi / 180.0)) ** 2
        default_ang_vel_cov = (gyro_noise_density ** 2) * gyro_bw_hz
        default_lin_acc_cov = (accel_noise_density ** 2) * accel_bw_hz

        self._imu_orientation_cov = self._read_float_env("IMU_ORIENTATION_COV", default_orientation_cov)
        self._imu_ang_vel_cov = self._read_float_env("IMU_ANGULAR_VELOCITY_COV", default_ang_vel_cov)
        self._imu_lin_acc_cov = self._read_float_env("IMU_LINEAR_ACCELERATION_COV", default_lin_acc_cov)

        # Existing wheel encoder pub (compat 유지)
        self._wheel_pub = None
        self._wheel_topic = "/wheel_encoder"  # Vector3Stamped x=rpm y=vel z=dist
        self._wheel_frame = "base_link"

        # NEW: wheel twist for robot_localization twist0
        self._wheel_twist_pub = None
        self._wheel_twist_topic = os.getenv("WHEEL_TWIST_TOPIC", "/wheel_twist")
        self._wheel_twist_frame = os.getenv("WHEEL_TWIST_FRAME", "base_link")

        # Wheel covariance tuning (velocity only)
        # sigma_v [m/s]. 기본 0.05 (상황 따라 0.03~0.10 조절)
        self._wheel_sigma_v = self._read_float_env("WHEEL_SIGMA_V", 0.05)

        # For imuenc time
        self._imuenc_time_base_us = None
        self._imuenc_time_base_ros_ns = None

        self._ros_import_warned = False
        self._ros_init_attempted = False

    def _init_ros(self):
        if self._ros_node is not None and self._imu_pub is not None:
            return True

        if rclpy is None or Imu is None:
            if not self._ros_import_warned:
                print("[SerialHandler] ROS2 publish disabled (missing rclpy/sensor_msgs/geometry_msgs).")
                self._ros_import_warned = True
            return False

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self._ros_node = Node("imu_serial_bridge")
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )

            # IMU publisher
            self._imu_pub = self._ros_node.create_publisher(Imu, self._imu_topic, qos)

            # Existing wheel encoder publisher (compat)
            if Vector3Stamped is not None:
                self._wheel_pub = self._ros_node.create_publisher(Vector3Stamped, self._wheel_topic, qos)

            # NEW: wheel twist publisher
            if TwistWithCovarianceStamped is not None:
                self._wheel_twist_pub = self._ros_node.create_publisher(
                    TwistWithCovarianceStamped, self._wheel_twist_topic, qos
                )

            return True
        except Exception as exc:
            print(f"[SerialHandler] ROS2 init failed: {exc}")
            self._ros_node = None
            self._imu_pub = None
            self._wheel_pub = None
            self._wheel_twist_pub = None
            return False

    def _get_ros_now(self):
        if not self._init_ros():
            return None
        try:
            return self._ros_node.get_clock().now()
        except Exception:
            return None

    def _apply_stamp(self, msg, stamp):
        if stamp is None:
            return
        try:
            msg.header.stamp = stamp.to_msg()
            return
        except Exception:
            pass
        try:
            msg.header.stamp = stamp
        except Exception:
            pass

    def _stamp_from_us(self, ts_us):
        """Map MCU microsecond timestamp to ROS time using first sample as anchor."""
        if ts_us is None:
            return self._get_ros_now()
        if rclpy is None:
            return None
        ros_now = self._get_ros_now()
        if ros_now is None:
            return None
        try:
            ts_us = int(ts_us)
        except Exception:
            return ros_now

        if self._imuenc_time_base_us is None or ts_us < self._imuenc_time_base_us:
            self._imuenc_time_base_us = ts_us
            try:
                self._imuenc_time_base_ros_ns = ros_now.nanoseconds
            except Exception:
                self._imuenc_time_base_ros_ns = None

        if self._imuenc_time_base_ros_ns is None:
            return ros_now

        delta_us = ts_us - self._imuenc_time_base_us
        stamp_ns = self._imuenc_time_base_ros_ns + (delta_us * 1000)
        try:
            return rclpy.time.Time(nanoseconds=stamp_ns)
        except Exception:
            return ros_now

    def _shutdown_ros(self):
        if self._ros_node is None:
            return
        try:
            self._ros_node.destroy_node()
        except Exception:
            pass
        self._ros_node = None
        self._imu_pub = None
        self._wheel_pub = None
        self._wheel_twist_pub = None
        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ---------------- Parsing ----------------
    def _parse_imu_values(self, value):
        parts = [p.strip() for p in value.split(";") if p.strip() != ""]
        if len(parts) < 12:
            return None
        try:
            return [float(parts[i]) for i in range(12)]
        except ValueError:
            return None

    def _parse_imuenc_values(self, value):
        parts = [p.strip() for p in value.split(";") if p.strip() != ""]
        if len(parts) < 16:
            return None
        try:
            ts_us = int(float(parts[0]))
            roll = float(parts[1])
            pitch = float(parts[2])
            yaw = float(parts[3])
            gyrox = float(parts[4])
            gyroy = float(parts[5])
            gyroz = float(parts[6])
            accelx = float(parts[7])
            accely = float(parts[8])
            accelz = float(parts[9])
            velx = float(parts[10])
            vely = float(parts[11])
            velz = float(parts[12])
            rpm = float(parts[13])
            velocity = float(parts[14])
            distance = float(parts[15])

            magx = magy = magz = None
            quat = None
            cov = None
            if len(parts) >= 32:
                magx = float(parts[16]); magy = float(parts[17]); magz = float(parts[18])
                qx = float(parts[19]); qy = float(parts[20]); qz = float(parts[21]); qw = float(parts[22])
                quat = (qx, qy, qz, qw)
                cov = [float(v) for v in parts[23:32]]

            return {
                "ts_us": ts_us,
                "rpy": (roll, pitch, yaw),
                "gyro": (gyrox, gyroy, gyroz),
                "accel": (accelx, accely, accelz),
                "vel": (velx, vely, velz),
                "encoder": (rpm, velocity, distance),
                "mag": (magx, magy, magz),
                "quat": quat,
                "cov": cov,
            }
        except ValueError:
            return None

    def _parse_encoder_values(self, value):
        parts = [p.strip() for p in value.split(";") if p.strip() != ""]
        if len(parts) < 3:
            return None
        try:
            return [float(parts[0]), float(parts[1]), float(parts[2])]
        except ValueError:
            return None

    # ---------------- Quaternion helpers ----------------
    def _rpy_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    def _quat_to_rpy(self, q):
        w, x, y, z = q
        sinr_cosp = 2.0 * ((w * x) + (y * z))
        cosr_cosp = 1.0 - 2.0 * ((x * x) + (y * y))
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * ((w * y) - (z * x))
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _quat_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
            (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
            (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
            (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2),
        )

    def _quat_normalize(self, q):
        w, x, y, z = q
        norm = math.sqrt((w * w) + (x * x) + (y * y) + (z * z))
        if norm == 0:
            return (1.0, 0.0, 0.0, 0.0)
        return (w / norm, x / norm, y / norm, z / norm)

    def _imu_vector_to_base(self, x, y, z):
        return (x, y, z)

    def _read_float_env(self, name, default):
        try:
            return float(os.getenv(name, default))
        except Exception:
            return float(default)

    # ---------------- Covariance fill ----------------
    def _fill_imu_covariance(self, msg, has_angular_velocity, has_linear_acceleration,
                             orientation_cov=None, angular_cov=None, linear_cov=None):
        if orientation_cov is not None and len(orientation_cov) == 9:
            msg.orientation_covariance = list(orientation_cov)
        else:
            msg.orientation_covariance = [0.0] * 9
            for idx in (0, 4, 8):
                msg.orientation_covariance[idx] = self._imu_orientation_cov

        if has_angular_velocity:
            if angular_cov is not None and len(angular_cov) == 9:
                msg.angular_velocity_covariance = list(angular_cov)
            else:
                msg.angular_velocity_covariance = [0.0] * 9
                for idx in (0, 4, 8):
                    msg.angular_velocity_covariance[idx] = self._imu_ang_vel_cov
        else:
            msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if has_linear_acceleration:
            if linear_cov is not None and len(linear_cov) == 9:
                msg.linear_acceleration_covariance = list(linear_cov)
            else:
                msg.linear_acceleration_covariance = [0.0] * 9
                for idx in (0, 4, 8):
                    msg.linear_acceleration_covariance[idx] = self._imu_lin_acc_cov
        else:
            msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _wheel_twist_cov36(self):
        """TwistWithCovarianceStamped covariance(36) with only linear.x variance set."""
        sigma_v = float(self._wheel_sigma_v)
        var_v = sigma_v * sigma_v
        cov = [0.0] * 36
        cov[0] = var_v  # linear.x
        return cov

    # ---------------- Publishers ----------------
    def _publish_imu(self, roll, pitch, yaw, accelx, accely, accelz, gyrox, gyroy, gyroz,
                     stamp=None, quat=None, orientation_cov=None, angular_cov=None, linear_cov=None):
        if not self._init_ros():
            return

        q = None
        if quat is not None:
            qx, qy, qz, qw = quat
            q = (qw, qx, qy, qz)
            if self._imu_apply_vehicle_frame and (self._imu_yaw_invert or self._imu_pitch_invert):
                r, p, y = self._quat_to_rpy(q)
                if self._imu_yaw_invert:
                    y = -y
                if self._imu_pitch_invert:
                    p = -p
                qx, qy, qz, qw = self._rpy_to_quaternion(r, p, y)
                q = (qw, qx, qy, qz)
        else:
            if self._imu_angle_unit in ("deg", "degree", "degrees"):
                roll = math.radians(roll)
                pitch = math.radians(pitch)
                yaw = math.radians(yaw)

            if self._imu_apply_vehicle_frame:
                if self._imu_yaw_invert:
                    yaw = -yaw
                if self._imu_pitch_invert:
                    pitch = -pitch

            qx, qy, qz, qw = self._rpy_to_quaternion(roll, pitch, yaw)
            q = (qw, qx, qy, qz)

        if self._imu_apply_vehicle_frame:
            q = self._quat_multiply(q, self._imu_to_base_quat)
            if gyrox is not None and gyroy is not None and gyroz is not None:
                gyrox, gyroy, gyroz = self._imu_vector_to_base(gyrox, gyroy, gyroz)
            if accelx is not None and accely is not None and accelz is not None:
                accelx, accely, accelz = self._imu_vector_to_base(accelx, accely, accelz)

        qw, qx, qy, qz = self._quat_normalize(q)

        msg = Imu()
        if stamp is None:
            stamp = self._get_ros_now()
        self._apply_stamp(msg, stamp)
        msg.header.frame_id = self._imu_frame

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        has_linear_acceleration = accelx is not None and accely is not None and accelz is not None
        if has_linear_acceleration:
            msg.linear_acceleration.x = accelx
            msg.linear_acceleration.y = accely
            msg.linear_acceleration.z = accelz

        has_angular_velocity = gyrox is not None and gyroy is not None and gyroz is not None
        if has_angular_velocity:
            msg.angular_velocity.x = gyrox
            msg.angular_velocity.y = gyroy
            msg.angular_velocity.z = gyroz

        self._fill_imu_covariance(
            msg,
            has_angular_velocity,
            has_linear_acceleration,
            orientation_cov=orientation_cov,
            angular_cov=angular_cov,
            linear_cov=linear_cov,
        )

        try:
            self._imu_pub.publish(msg)
        except Exception as exc:
            print(f"[SerialHandler] ROS2 IMU publish failed: {exc}")

    def _publish_wheel_encoder_and_twist(self, values, stamp=None):
        """
        Publish:
          - /wheel_encoder (Vector3Stamped) for backward compat
          - /wheel_twist  (TwistWithCovarianceStamped) for robot_localization twist0
        """
        if not self._init_ros():
            return

        rpm, velocity, distance = values
        if stamp is None:
            stamp = self._get_ros_now()

        # 1) 기존 /wheel_encoder 유지
        if self._wheel_pub is not None and Vector3Stamped is not None:
            msg = Vector3Stamped()
            self._apply_stamp(msg, stamp)
            msg.header.frame_id = self._wheel_frame
            msg.vector.x = float(rpm)
            msg.vector.y = float(velocity)
            msg.vector.z = float(distance)
            try:
                self._wheel_pub.publish(msg)
            except Exception as exc:
                print(f"[SerialHandler] ROS2 wheel_encoder publish failed: {exc}")

        # 2) NEW /wheel_twist 추가
        if self._wheel_twist_pub is not None and TwistWithCovarianceStamped is not None:
            tmsg = TwistWithCovarianceStamped()
            self._apply_stamp(tmsg, stamp)
            tmsg.header.frame_id = self._wheel_twist_frame  # base_link
            tmsg.twist.twist.linear.x = float(velocity)
            tmsg.twist.twist.linear.y = 0.0
            tmsg.twist.twist.linear.z = 0.0
            tmsg.twist.twist.angular.x = 0.0
            tmsg.twist.twist.angular.y = 0.0
            tmsg.twist.twist.angular.z = 0.0
            tmsg.twist.covariance = self._wheel_twist_cov36()
            try:
                self._wheel_twist_pub.publish(tmsg)
            except Exception as exc:
                print(f"[SerialHandler] ROS2 wheel_twist publish failed: {exc}")

    # ---------------- Message handlers ----------------
    def _handle_imu_sample(self, roll, pitch, yaw, accelx, accely, accelz, gyrox, gyroy, gyroz,
                           stamp=None, quat=None, orientation_cov=None):
        data = {"roll": str(roll), "pitch": str(pitch), "yaw": str(yaw)}
        self.imuDataSender.send(str(data))
        self._publish_imu(
            roll, pitch, yaw,
            accelx, accely, accelz,
            gyrox, gyroy, gyroz,
            stamp,
            quat=quat,
            orientation_cov=orientation_cov,
        )

    def _handle_encoder_sample(self, rpm, velocity, distance, stamp=None):
        self._publish_wheel_encoder_and_twist([rpm, velocity, distance], stamp)

    def _init_senders(self):
        self.enableButtonSender = messageHandlerSender(self.queuesList, EnableButton)
        self.batteryLvlSender = messageHandlerSender(self.queuesList, BatteryLvl)
        self.instantConsumptionSender = messageHandlerSender(self.queuesList, InstantConsumption)
        self.imuDataSender = messageHandlerSender(self.queuesList, ImuData)
        self.imuAckSender = messageHandlerSender(self.queuesList, ImuAck)
        self.resourceMonitorSender = messageHandlerSender(self.queuesList, ResourceMonitor)
        self.currentSpeedSender = messageHandlerSender(self.queuesList, CurrentSpeed)
        self.currentSteerSender = messageHandlerSender(self.queuesList, CurrentSteer)
        self.warningSender = messageHandlerSender(self.queuesList, ShutDownSignal)
        self.serialConnectionStateSender = messageHandlerSender(self.queuesList, SerialConnectionState)
        self.calibPWMDataSender = messageHandlerSender(self.queuesList, CalibPWMData)
        self.calibRunDoneSender = messageHandlerSender(self.queuesList, CalibRunDone)
        self.steeringLimitsSender = messageHandlerSender(self.queuesList, SteeringLimits)
        self.aliveSignalSender = messageHandlerSender(self.queuesList, AliveSignal)

    # ====================================== RUN ==========================================
    def thread_work(self):
        try:
            if not self._ros_init_attempted:
                self._ros_init_attempted = True
                self._init_ros()

            with self.process.serialLock:
                serial_con = self.process.serialCon
                if serial_con is None or not self.process.serialConnected or not serial_con.is_open:
                    return

                if serial_con.in_waiting > 0:
                    try:
                        data = serial_con.read(serial_con.in_waiting).decode("ascii")
                        self.buffer += data
                    except Exception as e:
                        if self._should_send_error():
                            self.serialConnectionStateSender.send(False)
                            print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;91mERROR\033[0m - Reading from serial ({e})")
                        return

            while ";;" in self.buffer:
                msg, self.buffer = self.buffer.split(";;", 1)
                if msg.strip():
                    try:
                        self.send_queue(msg.strip())
                    except Exception as e:
                        print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;91mERROR\033[0m - Processing message \033[94m{msg.strip()}\033[0m ({e})")
        except Exception as e:
            if self._should_send_error():
                self.serialConnectionStateSender.send(False)
                print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;91mERROR\033[0m - Thread run method ({e})")

    # ==================================== SENDING =======================================
    def queue_sending(self):
        self.enableButtonSender.send(True)
        threading.Timer(1, self.queue_sending).start()

    def send_queue(self, buff):
        if '@' in buff and ':' in buff:
            action, value = buff.split(":", 1)
            action = action[1:]
            action_lower = action.lower()

            if self.debugger:
                self.logger.info(buff)

            if action_lower == "imuenc":
                parsed = self._parse_imuenc_values(value)
                if parsed is not None:
                    ts_us = parsed["ts_us"]
                    roll, pitch, yaw = parsed["rpy"]
                    gyrox, gyroy, gyroz = parsed["gyro"]
                    accelx, accely, accelz = parsed["accel"]
                    rpm, velocity, distance = parsed["encoder"]
                    quat = parsed["quat"]
                    cov = parsed["cov"]

                    stamp = self._stamp_from_us(ts_us)

                    self._handle_imu_sample(
                        roll, pitch, yaw,
                        accelx, accely, accelz,
                        gyrox, gyroy, gyroz,
                        stamp,
                        quat=quat,
                        orientation_cov=cov,
                    )
                    # wheel encoder도 동일 stamp로 (EKF sync 좋아짐)
                    self._handle_encoder_sample(rpm, velocity, distance, stamp)
                elif self.debugger:
                    try:
                        self.logger.warning(f"[SerialHandler] IMUENC parse failed: {value}")
                    except Exception:
                        pass
                return

            if action_lower in ("encoder", "enc") or action_lower.startswith("enc"):
                self._log_encoder(buff, value)
                parsed = self._parse_encoder_values(value)
                if parsed is not None:
                    rpm, velocity, distance = parsed
                    stamp = self._get_ros_now()
                    self._handle_encoder_sample(rpm, velocity, distance, stamp)

            if action == "imu":
                if len(buff) > 20:
                    parts = [p.strip() for p in value.split(";") if p.strip() != ""]
                    if len(parts) >= 12:
                        data = {"roll": parts[0], "pitch": parts[1], "yaw": parts[2]}
                        self.imuDataSender.send(str(data))

                    imu_values = self._parse_imu_values(value)
                    if imu_values is not None:
                        roll, pitch, yaw, gyrox, gyroy, gyroz, accelx, accely, accelz, velx, vely, velz = imu_values
                        stamp = self._get_ros_now()
                        self._handle_imu_sample(
                            roll, pitch, yaw,
                            accelx, accely, accelz,
                            gyrox, gyroy, gyroz,
                            stamp
                        )
                else:
                    splittedValue = value.split(";")
                    self.imuAckSender.send(splittedValue[0])

            elif action == "brake":
                self.currentSpeedSender.send(0.0)
                self.currentSteerSender.send(0.0)

            elif action == "speed":
                speed = value.split(",")[0]
                if self.is_float(speed):
                    self.currentSpeedSender.send(float(speed))

            elif action == "steer":
                steer = value.split(",")[0]
                if self.is_float(steer):
                    self.currentSteerSender.send(float(steer))

            elif action == "vcdCalib":
                splittedValue = value.split(";")
                speedPWM = splittedValue[0]
                steerPWM = splittedValue[1]
                if speedPWM == "0" and steerPWM == "0":
                    self.calibRunDoneSender.send(True)
                else:
                    self.calibPWMDataSender.send({"speedPWM": speedPWM, "steerPWM": steerPWM})

            elif action == "alive":
                self.aliveSignalSender.send(True)

            elif action == "steerLimits":
                splittedValue = value.split(";")
                lowerLimit = splittedValue[0]
                upperLimit = splittedValue[1]
                self.steeringLimitsSender.send({"lowerLimit": lowerLimit, "upperLimit": upperLimit})

            elif action == "instant":
                if self.check_valid_value(action, value):
                    self.instantConsumptionSender.send(float(value))

            elif action == "battery":
                if self.check_valid_value(action, value):
                    percentage = (int(value) - 7000) / 14
                    percentage = max(0, min(100, round(percentage)))
                    self.batteryLvlSender.send(percentage)

            elif action == "resourceMonitor":
                if self.check_valid_value(action, value):
                    data = re.match(self.resourceMonitorPattern, value)
                    if data:
                        message = {"heap": data.group(1), "stack": data.group(2)}
                        self.resourceMonitorSender.send(message)

            elif action == "warning":
                data = re.match(self.warningPattern, value)
                if data:
                    print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;93mWARNING\033[0m - Shutdown in \033[94m{data.group(1)}h {data.group(2)}m {data.group(3)}s\033[0m")
                    self.warningSender.send(data)

            elif action == "shutdown":
                print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;93mWARNING\033[0m - \033[94mShutting down now!\033[0m")
                self.event.wait(3)
                os.system("sudo shutdown -h now")

    def _log_encoder(self, raw_msg, value):
        if not (self.debug_encoder or self.debugger):
            return
        msg = f"[ENCODER] raw={raw_msg} value={value}"
        try:
            print(msg)
            if self.logger:
                self.logger.info(msg)
        except Exception:
            pass

    def check_valid_value(self, action, message):
        if message == "syntax error":
            print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;93mWARNING\033[0m - Invalid \033[94m{action.upper()}\033[0m value (expected {self.expectedValues[action]})")
            return False
        if message == "kl 15/30 is required!!":
            print(f"\033[1;97m[ Serial Handler ] :\033[0m \033[1;93mWARNING\033[0m - KL 15/30 required for \033[94m{action.upper()}\033[0m")
            return False
        if message == "ack":
            return False
        return True

    def is_float(self, string):
        try:
            float(string)
        except ValueError:
            return False
        return True

    def _should_send_error(self):
        now = datetime.now()
        if self.last_error_time is None or (now - self.last_error_time) >= self.error_cooldown:
            self.last_error_time = now
            return True
        return False

    def stop(self):
        self._shutdown_ros()
        super(threadRead, self).stop()