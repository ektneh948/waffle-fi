from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

import numpy as np

import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


@dataclass
class RosCoverageConfig:
    # LiDAR -> 관측 다운샘플 개수
    lidar_bins: int = 36
    max_lidar_range: float = 3.5

    # step()에서 action 유지 시간(초)
    step_dt: float = 0.15

    max_steps: int = 600
    warmup_steps: int = 8

    # 충돌 판정
    collision_dist: float = 0.3

    # coverage (odom 기반 방문 셀)
    cell_size: float = 0.25  # 25cm 그리드
    reward_new_cell: float = 1.0
    reward_revisit: float = 0.0
    step_penalty: float = -0.01
    collision_penalty: float = -10.0

    # 목표: 방문한 셀 개수(간단 버전)
    target_visited_cells: int = 250

    # cmd_vel 매핑
    v_forward: float = 0.18
    w_turn: float = 0.9

    # 토픽/서비스
    scan_topic: str = "/scan"
    odom_topic: str = "/odom"
    cmd_vel_topic: str = "/cmd_vel"
    reset_services: Tuple[str, ...] = ("/reset_simulation", "/reset_world")


class RosCoverageEnv(gym.Env):
    """
    ROS2 + Gazebo에서 커버리지용 Gym Env (cmd_vel 직접 제어)

    Action (Discrete 4):
      0: forward, 1: left, 2: right, 3: stop

    Observation (벡터):
      [lidar_bins(0~1), x_tanh, y_tanh, sin(yaw), cos(yaw), coverage_tanh, step_frac]
    """

    def __init__(self, cfg: RosCoverageConfig = RosCoverageConfig()):
        super().__init__()
        self.cfg = cfg
        self._warmup_left = 0

        # obs 차원 = lidar_bins + 6
        obs_dim = self.cfg.lidar_bins + 6
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Discrete(4)

        # ROS init
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("coverage_ros_env")

        self.cmd_pub = self.node.create_publisher(Twist, self.cfg.cmd_vel_topic, 10)
        self.scan_sub = self.node.create_subscription(LaserScan, self.cfg.scan_topic, self._on_scan, 10)
        self.odom_sub = self.node.create_subscription(Odometry, self.cfg.odom_topic, self._on_odom, 10)

        # reset 서비스는 환경마다 이름이 달라서 둘 다 시도
        self.reset_clients = [
            self.node.create_client(Empty, name) for name in self.cfg.reset_services
        ]

        # 센서 캐시 + 이벤트
        self._scan: Optional[LaserScan] = None
        self._odom: Optional[Odometry] = None
        self._scan_event = threading.Event()
        self._odom_event = threading.Event()

        # 에피소드 상태
        self.step_count = 0
        self.visited_cells = set()

        # 마지막으로 보낸 cmd_vel (디버그)
        self._last_action = 3

    def close(self):
        # 로봇 멈추고 종료
        try:
            self._publish_twist(0.0, 0.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # ---------------- ROS callbacks ----------------
    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_event.set()

    def _on_odom(self, msg: Odometry):
        self._odom = msg
        self._odom_event.set()

    # ---------------- Gym API ----------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        self.step_count = 0
        self._warmup_left = self.cfg.warmup_steps
        self.visited_cells.clear()

        # 1) gazebo reset 시도
        self._try_reset_sim()

        # 2) 센서 안정화
        self._wait_sensors(timeout_sec=3.0)

        # 3) 첫 visited 마킹
        x, y, _ = self._get_pose()
        self._mark_visited(x, y)

        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def step(self, action: int):
        self.step_count += 1
        self._last_action = int(action)

        # 1) action -> cmd_vel
        if action == 0:      # forward
            v, w = self.cfg.v_forward, 0.0
        elif action == 1:    # left
            v, w = 0.05, self.cfg.w_turn
        elif action == 2:    # right
            v, w = 0.05, -self.cfg.w_turn
        elif action == 3:    # stop
            v, w = 0.0, 0.0
        else:
            raise ValueError(f"Invalid action: {action}")

        # 2) dt 동안 주기적으로 publish + spin
        self._hold_action(v, w, self.cfg.step_dt)

        # 3) 보상/종료 계산
        reward = self.cfg.step_penalty
        done = False

        # 충돌 체크
        if self._warmup_left > 0:
            self._warmup_left -= 1
        else:
            # min_dist = self._min_lidar_dist()
            min_dist = self._min_lidar_dist_filtered()
            if min_dist is not None and min_dist < self.cfg.collision_dist:
                reward += self.cfg.collision_penalty
                done = True

        # coverage 체크 (새 셀 방문 보상)
        x, y, _ = self._get_pose()
        new_cell = self._mark_visited(x, y)
        if new_cell:
            reward += self.cfg.reward_new_cell
        else:
            reward += self.cfg.reward_revisit

        # 목표 방문 셀 달성
        if len(self.visited_cells) >= self.cfg.target_visited_cells:
            done = True

        # 최대 step
        if self.step_count >= self.cfg.max_steps:
            done = True

        obs = self._get_obs()
        info = self._get_info()

        terminated = done
        truncated = False

        if "gymnasium" in gym.__name__:
            return obs, float(reward), terminated, truncated, info
        else:
            return obs, float(reward), done, info

    # ---------------- helpers ----------------
    def _try_reset_sim(self):
        for cli in self.reset_clients:
            try:
                if cli.wait_for_service(timeout_sec=0.6):
                    future = cli.call_async(Empty.Request())
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                    return
            except Exception:
                continue

    def _wait_sensors(self, timeout_sec: float):
        self._scan_event.clear()
        self._odom_event.clear()
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._scan is not None and self._odom is not None:
                return

    def _hold_action(self, v: float, w: float, duration: float):
        # Nav2 등 다른 퍼블리셔가 있어도 우리가 이기도록 짧은 주기로 계속 쏴줌
        t_end = time.time() + duration
        while time.time() < t_end:
            self._publish_twist(v, w)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def _publish_twist(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _get_pose(self) -> Tuple[float, float, float]:
        if self._odom is None:
            return 0.0, 0.0, 0.0
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation

        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(p.x), float(p.y), float(yaw)

    def _lidar_downsample(self) -> np.ndarray:
        if self._scan is None or self._scan.ranges is None:
            return np.zeros(self.cfg.lidar_bins, dtype=np.float32)

        ranges = np.array(self._scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges,
            nan=self.cfg.max_lidar_range,
            posinf=self.cfg.max_lidar_range,
            neginf=0.0
        )
        ranges = np.clip(ranges, 0.0, self.cfg.max_lidar_range)

        n = len(ranges)
        bins = self.cfg.lidar_bins
        idx = np.linspace(0, n - 1, bins).astype(np.int32)
        ds = ranges[idx] / self.cfg.max_lidar_range  # 0~1
        return ds.astype(np.float32)

    def _min_lidar_dist(self) -> Optional[float]:
        if self._scan is None or self._scan.ranges is None:
            return None
        r = np.array(self._scan.ranges, dtype=np.float32)
        r = np.nan_to_num(r, nan=np.inf, posinf=np.inf, neginf=np.inf)
        return float(np.min(r))
    
    def _min_lidar_dist_filtered(self) -> Optional[float]:
        if self._scan is None or self._scan.ranges is None:
            return None

        r = np.array(self._scan.ranges, dtype=np.float32)

        # NaN/Inf 제거
        r = np.nan_to_num(r, nan=np.inf, posinf=np.inf, neginf=np.inf)

        # LaserScan.range_min/range_max 활용
        rmin = float(getattr(self._scan, "range_min", 0.0))
        rmax = float(getattr(self._scan, "range_max", self.cfg.max_lidar_range))

        # 너무 작은 값(튐) 제거
        r[(r < max(rmin, 0.02))] = np.inf
        r[(r > rmax)] = np.inf

        m = float(np.min(r))
        if not np.isfinite(m):
            return None
        return m

    def _cell_index(self, x: float, y: float) -> Tuple[int, int]:
        cx = int(np.floor(x / self.cfg.cell_size))
        cy = int(np.floor(y / self.cfg.cell_size))
        return cx, cy

    def _mark_visited(self, x: float, y: float) -> bool:
        cell = self._cell_index(x, y)
        if cell in self.visited_cells:
            return False
        self.visited_cells.add(cell)
        return True

    def _coverage_ratio_like(self) -> float:
        # SLAM 맵 없이 “비율”을 정의하려면 분모가 애매하니까
        # 초보 단계에서는 tanh로 스케일된 방문량을 coverage처럼 사용
        return float(np.tanh(len(self.visited_cells) / 300.0))

    def _get_obs(self) -> np.ndarray:
        import math
        lidar = self._lidar_downsample()  # 0~1

        x, y, yaw = self._get_pose()
        x_s = np.tanh(x / 5.0)
        y_s = np.tanh(y / 5.0)
        siny = math.sin(yaw)
        cosy = math.cos(yaw)

        cov = self._coverage_ratio_like()
        step_frac = self.step_count / float(max(1, self.cfg.max_steps))

        extra = np.array([x_s, y_s, siny, cosy, cov, step_frac], dtype=np.float32)
        obs = np.concatenate([lidar, extra], axis=0).astype(np.float32)

        return obs

    def _get_info(self) -> Dict[str, Any]:
        x, y, yaw = self._get_pose()
        return {
            "step": self.step_count,
            "x": x, "y": y, "yaw": yaw,
            "visited_cells": len(self.visited_cells),
            "coverage_like": self._coverage_ratio_like(),
            "last_action": self._last_action,
        }
