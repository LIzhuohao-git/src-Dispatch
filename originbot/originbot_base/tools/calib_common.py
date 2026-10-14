#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定工具共享模块：自动行驶 + 雷达真值估计

【用途】
  给 odom_calib.py / imu_check.py / lidar_crop_check.py 共用：
    1. MotionDriver —— 闭环驱动小车（行驶指定距离 / 原地转到指定角度）
    2. 雷达真值估计 —— 比对运动前后的 /scan，算出小车实际走了多远、转了多少
    3. 活参数读取 —— 从运行中的 /originbot_base 读 correct_factor_*

【为什么旋转不能简单地"把距离序列转一下再互相关"】
  雷达装在 base_link 前方 mount_x（出厂 0.15 m）处。小车绕 base_link 原点原地
  旋转时，传感器本身在半径 0.15 m 的圆上摆动，转 180° 时等效平移达
  2×0.15×sin(90°) = 0.21 m，在 2 m 处约造成 6° 的虚假方位偏移 —— 远大于我们要
  的精度。所以必须用下面的精确刚体模型，让"传感器偏移"成为模型的一部分。

  设小车在初始车体系 B0 下的运动为 (dx, dy, dφ)（dφ 为偏航角变化，逆时针为正），
  传感器偏移 m = (mount_x, mount_y)，mount_yaw = 0，则同一物理点在
  "运动后雷达系" 与 "运动前雷达系" 的坐标满足：

      p_ref = R(dφ)·p_cur + (R(dφ) − I)·m + (dx, dy)

  旋转测试时 dx = dy = 0（只有 1 个自由度）；平移测试是 3 个自由度。
  这样解出的 (dx, dy, dφ) 就是车体在 base_link 下的运动，与 /odom 直接可比。

【为什么用"点到距离廓线"而不是"点到点"做匹配】
  最初用的是最近邻点对点代价，实测**即使在无噪声合成场景下**内点残差 RMS 也有
  26~33 mm、位移误差 4~12 mm。原因不是噪声而是**角度离散化**：雷达角分辨率
  0.7156°，在 3 m 处相邻点间隔 37 mm，两个稀疏点云之间"最近点距离"本身就被这个
  采样间隔撑住了 —— 量的是自己的采样，不是运动。

  改为：把参考帧的距离数据当作一条**距离廓线** r_ref(方位角)（同一方位角单值），
  每个当前点按候选变换投到参考雷达系后，用它的方位角在廓线上**插值**取期望距离，
  残差 = 实测距离 − 插值距离。这样量的是"点是否落在参考表面上"，与采样间隔无关，
  且每次评价只需 O(K) 次查表（对点对点是 O(K²)），于是可以用满分辨率扫描。

【依赖】
  只用 numpy（本机没有 scipy）。rclpy 缺失时本模块仍可 import，纯数学部分与
  --selftest 可离线运行。

【自测】
  python3 calib_common.py --selftest      # 合成场景验证估算器，不需要硬件
  ros2 run originbot_base odom_calib.py --rotate

【注意】
  本模块不是可执行工具，故意不注册到 CMakeLists 的 _tool_scripts 列表；
  它靠同级脚本的 sys.path[0] 被 import。若把某个工具脚本单独拷走，请连同本文件
  一起拷 —— 拷走后 import 失败时工具会自行回落到人工测量，不会崩。
"""

import dataclasses
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# ROS 可选导入：缺失时纯数学部分与 --selftest 仍可用
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node as _RclpyNode
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    HAVE_ROS = True
except ImportError:                                     # pragma: no cover
    HAVE_ROS = False

    class _RclpyNode:                                   # 占位，保证类定义不报错
        def __init__(self, *args, **kwargs):
            raise RuntimeError("rclpy 不可用：本模块的 ROS 功能需要 ROS 环境")

    Twist = Odometry = LaserScan = None


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
R_MIN = 0.16        # 有效回波下限：清掉 0.0 与 0.1m 盲区（驱动已把裁剪/越界置 NaN）
R_MAX = 12.0        # 有效回波上限。STL06N 实测有效距离约 12m（robot_params.py:73）；
                    # 注意消息里 range_max 报的是 30.0，这里刻意取更小的经验值
D_MAX = 0.30        # 残差截断上限，米。同时用作"算不算内点"的门限
MIN_PTS = 80        # 内点少于此数直接判不可靠
MIN_FRAMES = 6      # 采集一帧所需的最少 /scan 消息数
DISC_MAX = 0.20     # 廓线插值安全门限：相邻样点距离差超过它就不插值（是条边，不是连续面）
MAX_PTS = 2000      # 当前帧点云预算。点到廓线代价是 O(K)，K 可以给得很大

# 可靠判据阈值。**这些是按噪声模型推的初始值，未实测**，上机后应按真实采集调整。
SIGMA_D_MAX = 0.010         # 位移 1σ 上限，米。取 10mm 是为了与工具里
                            # "sigma/distance > 1% 就告警" 的口径一致（1m × 1% = 10mm）
SIGMA_DTH_MAX = math.radians(0.5)   # 转角 1σ 上限
RMS_INLIER_MAX = 0.05       # 内点残差均方根上限，米
OVERLAP_MIN = 0.55          # 参考廓线被覆盖的比例下限
RIVAL_RATIO_MIN = 1.30      # 对手峰代价 / 最优代价 的下限
CONFIDENCE_MIN = 0.5

# sigma 的保守修正系数。_sigma() 由代价曲率推出的不确定度是**偏乐观的下界**：
# 它的公式假设 n 个残差互相独立，但相邻 beam 打在同一面墙上、残差是强相关的，
# 真正独立的信息量远少于点数。合成蒙特卡洛（见 --selftest 的 C9/C12）实测：
#   平移：真实离散 1.78mm vs 公式 0.47mm → 偏乐观 3.8 倍
#   转角：真实离散 0.030° vs 公式 0.011° → 偏乐观 2.8 倍
# 这里统一乘 3 做保守修正（宁可多回退到人工测量，也不要"自信地答错"）。
# 修正后的数值才是打印给用户的 1σ，可靠判据也基于它。
SIGMA_SAFETY = 3.0

# 系统性偏差的上限（不是随机量，无法由曲率推出）：合成蒙特卡洛实测约
# 平移 +2mm / 旋转 -0.08°，主要来自参考廓线的网格插值与远场离散采样。
# 1m 行程上 2mm 即 0.2%，远小于 2% 验收带，故可接受；但用户需知道真实
# 精度 ≈ sqrt(bias² + (3σ)²)，而不是仅看 σ。

# Web 端参数配置页生成的配置目录（参数真源，launch 启动时读取）
GENERATED_DIR = Path.home() / ".originbot_delivery" / "config" / "generated"

# 粗搜索步长，同时也是"曲率取样尺度"（见 _curvature_coarse）
COARSE_D = 0.05             # 平移粗网格步长，米
COARSE_DTH = math.radians(2.0)   # 平移搜索里残余转角的粗步长
COARSE_DTH_ROT = math.radians(1.0)   # 旋转搜索的粗步长，也是曲率尺度


# ---------------------------------------------------------------------------
# 刚体变换
# ---------------------------------------------------------------------------
def _apply(pts, dx, dy, dth, mount_xy, mount_yaw=0.0):
    """把"运动后雷达系"的点变换到"运动前雷达系"。

    见模块头部的推导：p_ref = R(dth)·p_cur + R(−ψ)·[(R(dth) − I)·m + d]，
    其中 ψ = mount_yaw（雷达相对车体的安装偏航，出厂 0）。

    注意 mount_yaw 对**标定用到的两个量**没有影响：dth 始终等于车体转角变化，
    |d| 也不变（只是方向被旋转到雷达系）。但若忽略它，平移项会整体偏转，拟合
    会变差甚至给错解 —— 而配置页明确允许把 mount_yaw 设成 3.1416 来对应倒装，
    所以这里必须带上。
    """
    c, s = math.cos(dth), math.sin(dth)
    rot = np.array([[c, -s], [s, c]])
    m = np.asarray(mount_xy, dtype=np.float64)
    offset = (rot - np.eye(2)) @ m + np.array([dx, dy])
    if mount_yaw:
        cy, sy = math.cos(-mount_yaw), math.sin(-mount_yaw)
        offset = np.array([[cy, -sy], [sy, cy]]) @ offset
    return pts @ rot.T + offset


def load_lidar_mount(fallback=(0.15, 0.0, 0.0)):
    """从 Web 端生成的 lidar.json 读安装位姿，返回 (mount_x, mount_y, mount_yaw)。

    雷达相对 base_link 的偏移直接决定旋转测试的模型（见模块头部），装错位置会让
    原地旋转等效成一个很大的平移，所以这里必须用真值而不是硬编码默认值。
    """
    values = list(fallback)
    try:
        with open(GENERATED_DIR / "lidar.json", "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return tuple(values)
    for i, key in enumerate(("mount_x", "mount_y", "mount_yaw")):
        if key in stored:
            try:
                values[i] = float(stored[key])
            except (TypeError, ValueError):
                pass
    return tuple(values)


# ---------------------------------------------------------------------------
# 参考距离廓线与代价函数
# ---------------------------------------------------------------------------
class RefProfile:
    """参考帧的距离廓线 r(方位角)，同一方位角单值。

    ranges 是逐 beam 的距离数组（无效处为 NaN）。smooth[i] 表示第 i 个样点与
    下一个样点之间可以安全插值（两者都有效、且距离差不超过 DISC_MAX —— 超过
    说明那里是遮挡边缘，插值会跨过一条边）。r_min/r_max 随廓线一起保存，这样
    调用方可以用 --r-max 收紧有效距离而不必改全局常量。
    """

    def __init__(self, ranges, angle_min, angle_increment, *, r_min=R_MIN,
                 r_max=R_MAX):
        self.ranges = np.asarray(ranges, dtype=np.float64)
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_increment)
        self.r_min = float(r_min)
        self.r_max = float(r_max)
        nxt = np.roll(self.ranges, -1)
        nxt_valid = np.isfinite(nxt)
        valid = np.isfinite(self.ranges)
        self.valid = valid
        self.smooth = (valid & nxt_valid
                       & (np.abs(nxt - self.ranges) <= DISC_MAX)
                       & (self.ranges >= self.r_min)
                       & (self.ranges <= self.r_max))
        # 分母：既有效又可插值的样点才算"应该被看到的面"
        self.usable_count = int(self.smooth.sum())

    def __len__(self):
        return int(self.valid.sum())


@dataclasses.dataclass
class ScanFrame:
    """一帧 /scan 的测距数据 + 它自己的角度参数。

    为什么必须逐帧带着角度参数：实测底盘上 /scan 的 beam 数在 503~506 之间跳变，
    而驱动总是把 [0, 2π] 均分（angle_increment = 2π/(N−1)），所以**每帧的角间距
    都不一样**。用第一帧的 increment 去解释所有帧会错位最多半个 beam。
    """
    ranges: np.ndarray
    angle_min: float
    angle_increment: float

    def angles(self):
        return self.angle_min + np.arange(self.ranges.size) * self.angle_increment


# 参考廓线用的统一角度网格：按标称 beam 数均分 [0, 2π)
GRID_N = 504
GRID_INC = 2.0 * math.pi / GRID_N


def _resample_to_grid(frame, grid_n=GRID_N):
    """把一帧插值到统一角度网格上。

    因为每帧的角间距不同（见 ScanFrame），直接拼接会崩、按最小长度截断又会让
    帧间产生方位错位，所以统一插值到固定网格。
    """
    dst = np.arange(grid_n) * (2.0 * math.pi / grid_n)
    src = frame.angles()
    valid = np.isfinite(frame.ranges)
    if valid.sum() < 2:
        return np.full(grid_n, np.nan)
    vx = src[valid]
    vy = frame.ranges[valid]
    out = np.interp(dst, vx, vy)
    # 标出"离最近有效样点太远"的网格点：跨过 NaN 空洞、或超出有效角度范围。
    # 不标掉的话 np.interp 会把空洞两侧直接线性连起来，凭空造出一段假墙面。
    pos = np.searchsorted(vx, dst)
    left = np.clip(pos - 1, 0, vx.size - 1)
    right = np.clip(pos, 0, vx.size - 1)
    gap = np.minimum(np.abs(dst - vx[left]), np.abs(vx[right] - dst))
    out[gap > 1.5 * max(frame.angle_increment, 2.0 * math.pi / grid_n)] = np.nan
    return out


def profile_from_frames(frames, *, r_min=R_MIN, r_max=R_MAX):
    """多帧 /scan 插值到统一网格后按样点求均值 → 参考廓线（多帧平均降噪）。"""
    if not frames:
        return RefProfile(np.zeros(0), 0.0, GRID_INC, r_min=r_min, r_max=r_max)
    stack = np.vstack([_resample_to_grid(f) for f in frames])
    # 手写均值而不用 np.nanmean：某个网格点若在所有帧里都无效，nanmean 会往
    # stderr 抛 "Mean of empty slice" 的 RuntimeWarning，干扰工具输出。
    valid = np.isfinite(stack)
    count = valid.sum(axis=0)
    total = np.where(valid, stack, 0.0).sum(axis=0)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(count > 0, total / np.maximum(count, 1), np.nan)
    return RefProfile(mean, 0.0, GRID_INC, r_min=r_min, r_max=r_max)


def prepare_points(frames, *, r_min=R_MIN, r_max=R_MAX, max_points=MAX_PTS):
    """多帧 /scan → 雷达系 (K,2) 点云。

    不做分箱：分箱会把方位角量化，反而丢掉精度。这里逐帧用**该帧自己的**
    angle_increment 还原每个 beam 的方位角（不能拿第一帧的去套，见 ScanFrame），
    拼接多帧原始回波后按固定步长抽取 —— 既保留角度分辨率又保留多帧降噪效果，
    且抽取是确定性的（保持纯函数性质）。
    """
    if not frames:
        return np.zeros((0, 2), dtype=np.float64)
    ranges = np.concatenate([f.ranges for f in frames])
    angles = np.concatenate([f.angles() for f in frames])
    # 裁剪与越界读数驱动已置 NaN（ldlidar main.cpp:265-274），isfinite 已足够；
    # 再叠加 r >= r_min 只是防御（例如别的驱动写 0.0）。
    ok = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
    ranges, angles = ranges[ok], angles[ok]
    if ranges.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    if ranges.size > max_points:
        step = int(math.ceil(ranges.size / max_points))
        ranges, angles = ranges[::step], angles[::step]
    return np.stack([ranges * np.cos(angles), ranges * np.sin(angles)], axis=1)


def _robust(res2, cap2):
    """Geman-McClure 稳健损失：小残差≈res²，大残差饱和到 cap²。

    为什么不用"硬截断的平方"：硬截断会让"哪些点落在上限内"在参数微动时跳变，
    代价面因此出现细锯齿。抛物线拟合会把锯齿当成曲率，于是走廊这类本来平坦
    （不可观测）的方向会解出一个虚假的小 sigma —— 也就是"自信地答错"，这是
    标定工具最危险的失败模式。Geman-McClure 处处光滑（C∞），曲率是真实曲率。
    """
    return res2 / (1.0 + res2 / cap2)


def _profile_cost(prof, cur_pts, dx, dy, dth, mount_xy, mount_yaw=0.0):
    """点到廓线代价（m²）：当前点投到参考系后，与插值廓线的距离残差。

    用稳健损失（见 _robust）求和取均值，使代价面对参数光滑连续 —— 这是后面
    抛物线插值能给出有意义曲率、进而 sigma 可信的前提。
    """
    t = _apply(cur_pts, dx, dy, dth, mount_xy, mount_yaw)
    r = np.hypot(t[:, 0], t[:, 1])
    bearing = np.arctan2(t[:, 1], t[:, 0]) % (2.0 * math.pi)
    f = (bearing - prof.angle_min) / prof.angle_increment
    n = prof.ranges.size
    i0 = np.floor(f).astype(np.int64) % n
    frac = f - np.floor(f)
    i1 = (i0 + 1) % n

    ok = (prof.smooth[i0] & prof.smooth[i1]
          & (r > prof.r_min) & (r < prof.r_max))
    if not ok.any():
        return float(D_MAX * D_MAX)          # 没有任何可用对应 → 最差代价
    r0 = prof.ranges[i0[ok]]
    r1 = prof.ranges[i1[ok]]
    expected = r0 + frac[ok] * (r1 - r0)
    residual = r[ok] - expected
    return float(_robust(residual * residual, D_MAX * D_MAX).mean())


def _curvature_coarse(cost_at, value, delta):
    """在两个粗尺度上估计代价面曲率，取较小者（更保守 → sigma 更大）。

    为什么要用粗尺度而不是细网格步长：代价面里"哪些点算可用"（落在 R_MAX 内、
    落在可插值样点之间）会随参数微动而跳变，这是代价面上的一层细锯齿。在很小的
    尺度上拟合抛物线，量到的是锯齿而不是真实曲率，会给出虚假的大曲率 —— 表现为
    sigma 很小但答案很错（走廊场景曾给出 2mm 的 sigma 配 200mm 的误差）。

    放大到粗网格尺度（旋转 1°、平移 0.05m）后，真实曲率远大于锯齿；同时这个
    尺度仍处于损失的二次区间内（残差 ~2m×1° = 35mm，远小于 D_MAX=0.3m 的
    饱和点），不会因为饱和而低估曲率。再取 δ 与 2δ 两尺度中较小的曲率作为
    保守估计：宁可多回退到人工测量，也不要给出"自信地答错"的建议值。
    """
    kappas = []
    for d in (delta, 2.0 * delta):
        denom = cost_at(value - d) - 2.0 * cost_at(value) + cost_at(value + d)
        kappas.append(denom / (d * d) if denom > 1e-15 else 0.0)
    return min(kappas)


def _profile_stats(prof, cur_pts, dx, dy, dth, mount_xy, mount_yaw=0.0):
    """返回 (overlap, rms_inlier, n_inlier)。

    overlap = 参考廓线里"被当前帧重新看到"的样点比例 —— 这是真正能抓住
    "环境变了 / 新出现遮挡"的指标（例如有人走进视野挡住了原来的墙面）。
    rms_inlier = 内点的距离残差均方根，是直接可解释的精度指标。
    """
    t = _apply(cur_pts, dx, dy, dth, mount_xy, mount_yaw)
    r = np.hypot(t[:, 0], t[:, 1])
    bearing = np.arctan2(t[:, 1], t[:, 0]) % (2.0 * math.pi)
    n = prof.ranges.size
    f = (bearing - prof.angle_min) / prof.angle_increment
    i0 = np.floor(f).astype(np.int64) % n
    frac = f - np.floor(f)
    i1 = (i0 + 1) % n

    ok = (prof.smooth[i0] & prof.smooth[i1]
          & (r > prof.r_min) & (r < prof.r_max))
    if not ok.any() or prof.usable_count == 0:
        return 0.0, float("inf"), 0
    expected = (prof.ranges[i0[ok]]
                + frac[ok] * (prof.ranges[i1[ok]] - prof.ranges[i0[ok]]))
    residual = r[ok] - expected
    inlier = np.abs(residual) <= D_MAX
    n_inlier = int(inlier.sum())
    if n_inlier:
        core = residual[inlier]
        rms_raw = math.sqrt(float((core ** 2).mean()))
        rms = trimmed_rms(core)
        # 野值比例：残差偏大但还没到 D_MAX 截断线的点（边缘混合像素等）
        outlier_frac = float((np.abs(core) > RMS_INLIER_MAX).mean())
    else:
        rms_raw = rms = float("inf")
        outlier_frac = 1.0

    # 参考样点被覆盖：统计有多少个可插值样点收到了至少一个内点
    covered = np.bincount(i0[ok][inlier], minlength=n) > 0
    usable = prof.smooth
    overlap = float(covered[usable].sum()) / max(int(usable.sum()), 1)
    return overlap, rms, n_inlier, rms_raw, outlier_frac


def _parabola(cost_m, cost_0, cost_p, h):
    """三点抛物线顶点插值 → (顶点相对偏移, 曲率 kappa)。

    曲率 kappa = (c(−h) − 2c(0) + c(+h)) / h²，用于 sigma 传播。
    曲率非正说明是边界极值而非谷底，返回 kappa=0 让上层判不可靠。
    """
    denom = cost_m - 2.0 * cost_0 + cost_p
    if denom <= 1e-15:
        return 0.0, 0.0
    return 0.5 * (cost_m - cost_p) / denom * h, denom / (h * h)


def _refine_axis(cost_of, params, axis, coarse_step, bounds, rounds=2,
                 fine_div=20, fine_half=10):
    """对单个参数轴做细扫 + 抛物线插值，返回 (新值, kappa)。

    细步长 = 粗步长/fine_div，扫 ±fine_half 个细步（= ±0.5 个粗步），保证覆盖
    粗网格最优点与真值之间最多半个粗步的间隔。
    """
    h = coarse_step / fine_div
    kappa = 0.0
    for _ in range(rounds):
        lo, hi = bounds
        offsets = np.arange(-fine_half, fine_half + 1) * h
        vals = np.clip(params[axis] + offsets, lo, hi)
        costs = np.array([cost_of(params, axis, v) for v in vals])
        j = int(np.argmin(costs))
        params[axis] = float(vals[j])
        if 0 < j < len(vals) - 1:
            step_m = vals[j] - vals[j - 1]
            step_p = vals[j + 1] - vals[j]
            if step_m > 0 and step_p > 0 and abs(step_m - step_p) < 1e-12:
                delta, kappa = _parabola(costs[j - 1], costs[j], costs[j + 1],
                                         step_m)
                params[axis] = float(np.clip(vals[j] + delta, lo, hi))
            else:
                kappa = 0.0
    return params[axis], kappa


def trimmed_rms(residual, trim=0.05):
    """截尾 RMS：去掉 |残差| 最大的一端 trim 比例后求均方根。

    为什么不用普通 RMS：真实雷达在物体边缘会产生**混合像素**（一束光同时打到
    两个面，回波距离落在两者之间）以及偶发野值。实测静止自检里普通 RMS 会从
    11mm 偶发跳到 56mm，把一个本来很准的结果（估计位移误差 1.9mm）判成不可靠，
    而 1m 行程上真正需要的是毫米级精度。

    这个统计量只用于**质量判据**，不参与代价函数，所以用稳健估计是安全的；
    代价函数那边必须保持处处光滑（见 _robust 的说明），不能靠排序。
    """
    if residual.size == 0:
        return float("inf")
    ordered = np.sort(np.abs(residual))
    keep = max(int(ordered.size * (1.0 - trim)), 1)
    return float(np.sqrt((ordered[:keep] ** 2).mean()))


def _sigma(kappa, rms_inlier, n_inlier):
    """由代价曲率与内点残差传播出的不确定度（已按 SIGMA_SAFETY 做保守修正）。

    这是本模块最有价值的指标：代价面平坦（环境无特征、走廊里沿墙平移）会自动
    产生很大的 sigma，不需要为这些退化情形单独特判；点数变少时也会自然退化。

    ⚠ 它只反映**随机**不确定度，不含系统性误差（参考廓线的网格插值、停车俯仰、
    雷达外参、旋转中心假设等）。真实精度约有 bias ⊕ 3σ 的量级，务必用钢尺
    交叉验证一次。
    """
    if kappa <= 1e-15 or n_inlier <= 0:
        return float("inf")
    raw = math.sqrt(2.0 * rms_inlier * rms_inlier / n_inlier / kappa)
    return SIGMA_SAFETY * raw


# ---------------------------------------------------------------------------
# 估计结果
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class MotionEstimate:
    """一次运动的雷达真值估计。长度单位为米，角度单位为弧度。"""
    dx: float = 0.0
    dy: float = 0.0
    dth: float = 0.0
    distance: float = 0.0
    sigma_d: float = float("inf")
    sigma_dth: float = float("inf")
    overlap: float = 0.0
    rms_inlier: float = float("inf")
    rms_raw: float = float("inf")       # 未截尾的 RMS（诊断用，含边缘野值）
    outlier_frac: float = 0.0           # |残差| > RMS_INLIER_MAX 的内点比例
    n_inlier: int = 0
    rival_ratio: float = float("inf")
    rival_dth: float | None = None
    residual_dx: float = 0.0        # 旋转模型固定 dx=dy=0 后解出的残余平移（诊断）
    residual_dy: float = 0.0
    lidar_frame_xy: tuple = (0.0, 0.0)   # 换算前、雷达系里的平移解
    confidence: float = 0.0
    reliable: bool = False
    reasons: list = dataclasses.field(default_factory=list)

    def describe(self, indent="  "):
        """多行中文摘要，供工具直接打印。"""
        lines = [
            f"{indent}雷达实测位移 = {self.distance:.4f} m "
            f"(dx={self.dx:+.4f}, dy={self.dy:+.4f})",
            f"{indent}雷达实测转角 = {math.degrees(self.dth):+.3f}°",
            f"{indent}1σ 不确定度 : 位移 {self.sigma_d * 1000:.2f} mm，"
            f"转角 {math.degrees(self.sigma_dth):.3f}°"
            f"   （仅随机分量，不含系统性误差）",
            f"{indent}内点         : {self.n_inlier} 点，"
            f"参考面覆盖 {self.overlap * 100:.1f}%，"
            f"残差 RMS {self.rms_inlier * 1000:.2f} mm"
            f"（截尾 5% 后的值；原始 {self.rms_raw * 1000:.1f} mm，"
            f"野值占比 {self.outlier_frac * 100:.1f}%）",
        ]
        if self.rival_dth is not None:
            lines.append(
                f"{indent}对手峰       : {math.degrees(self.rival_dth):+.1f}° 处代价比 "
                f"{self.rival_ratio:.2f}（≈1.0 表示环境有旋转对称，真值有歧义）")
        lines.append(f"{indent}置信度       : {self.confidence:.2f}"
                     f"    判定：{'可靠' if self.reliable else '不可靠'}")
        for reason in self.reasons:
            lines.append(f"{indent}  · {reason}")
        return "\n".join(lines)


def _boundary_hits(values, bounds, names, *, frac=0.02):
    """找出解落在搜索边界上的参数轴。

    解被顶在边界上说明代价面在该方向单调（没有内部极小），常见于沿墙/走廊这类
    退化场景。这比 kappa 更早、更明确地暴露问题，所以单独报出来。
    """
    hits = []
    for value, (lo, hi), name in zip(values, bounds, names):
        span = max(hi - lo, 1e-12)
        if min(abs(value - lo), abs(value - hi)) < frac * span:
            hits.append(name)
    return hits


def _finish(est, stats, kappa_d, kappa_dth, *, tol_d, tol_dth, need_rival,
            boundary_hits=()):
    """填充 sigma / confidence / reliable。

    tol_d / tol_dth 为 None 表示该轴不参与判据：旋转测试只关心转角（dx/dy 是
    旋转模型从传感器偏移推出来的量），平移测试不关心那个很小的残余转角。
    """
    est.overlap, est.rms_inlier, est.n_inlier, est.rms_raw, est.outlier_frac = stats
    est.sigma_d = _sigma(kappa_d, est.rms_inlier, est.n_inlier) \
        if tol_d is not None else float("inf")
    est.sigma_dth = _sigma(kappa_dth, est.rms_inlier, est.n_inlier) \
        if tol_dth is not None else float("inf")

    reasons = []
    if boundary_hits:
        reasons.append(
            f"解被顶在搜索边界上（{'、'.join(boundary_hits)}）—— 说明代价面在这个"
            f"方向上没有内部极小，实际运动超出了预测附近的搜索范围。")
    if est.n_inlier < MIN_PTS:
        reasons.append(f"内点过少（{est.n_inlier} 点，要求 ≥{MIN_PTS}）")
    if est.overlap < OVERLAP_MIN:
        reasons.append(f"参考面只被重新看到 {est.overlap * 100:.0f}%，低于 "
                       f"{OVERLAP_MIN * 100:.0f}% —— 环境在两帧之间变了"
                       f"（有人走动？被挡住？）或模型不符")
    if est.rms_inlier > RMS_INLIER_MAX:
        reasons.append(f"内点残差 {est.rms_inlier * 1000:.0f} mm（已截尾 5%）大于 "
                       f"{RMS_INLIER_MAX * 1000:.0f} mm，解不干净")

    parts = [est.overlap / 0.80,
             (RMS_INLIER_MAX + 0.01 - est.rms_inlier) / RMS_INLIER_MAX
             if math.isfinite(est.rms_inlier) else 0.0]

    if tol_d is not None:
        if est.sigma_d > tol_d:
            reasons.append(f"位移不确定度 {est.sigma_d * 1000:.2f} mm 大于 "
                           f"{tol_d * 1000:.0f} mm（代价面太平坦：环境缺特征，"
                           f"或沿墙/走廊这类退化场景）")
        parts.append(tol_d / est.sigma_d if est.sigma_d > 0 else 0.0)

    if tol_dth is not None:
        if est.sigma_dth > tol_dth:
            reasons.append(f"转角不确定度 {math.degrees(est.sigma_dth):.3f}° 大于 "
                           f"{math.degrees(tol_dth):.3f}°（代价面太平坦？）")
        parts.append(tol_dth / est.sigma_dth if est.sigma_dth > 0 else 0.0)

    if need_rival:
        if est.rival_ratio < RIVAL_RATIO_MIN:
            reasons.append(
                f"环境中存在几乎等价的另一解（{math.degrees(est.rival_dth):+.1f}° "
                f"处，代价比 {est.rival_ratio:.2f}）—— 房间可能是矩形/正方形等"
                f"对称形状，雷达无法区分这两者，请人工确认实际转角")
        parts.append((est.rival_ratio - 1.0) / 0.50)

    est.confidence = float(np.clip(min(parts), 0.0, 1.0))
    if est.confidence < CONFIDENCE_MIN and not reasons:
        reasons.append(f"置信度 {est.confidence:.2f} 低于 {CONFIDENCE_MIN}")
    est.reasons = reasons
    est.reliable = (not reasons) and est.confidence >= CONFIDENCE_MIN
    return est


def estimate_rotation(prof, cur_pts, *, predicted_dth, mount_xy=(0.15, 0.0),
                      mount_yaw=0.0):
    """原地旋转的雷达真值。返回 MotionEstimate（dth 为车体转角变化，弧度）。

    predicted_dth 是里程计给出的预测转角（弧度，可超过 2π，即"多圈"），
    用于把搜索窗限制在预测附近 —— **这一点是设计的关键**：

      代价函数对 dth 是 360° 周期的，720° 与 0° 是同一个解。只有把窗口收窄，
      才能唯一确定"转了几圈"。窗口上界取 80°，严格小于 90° —— 即小于房间可能
      出现的最小旋转对称周期（正方形），于是矩形房间的 180° 镜像解与正方形房间
      的 90° 镜像解都落在窗口外，不会静默锁定到错误的多圈解。
    """
    est = MotionEstimate()
    if len(prof) < MIN_PTS or len(cur_pts) < MIN_PTS:
        est.reasons = [f"有效点/样点过少（参考 {len(prof)}，当前 {len(cur_pts)}）"]
        return est

    def cost_of(dth, dx=0.0, dy=0.0):
        return _profile_cost(prof, cur_pts, dx, dy, dth, mount_xy, mount_yaw)

    window = float(np.clip(0.10 * abs(predicted_dth),
                           math.radians(8.0), math.radians(80.0)))

    # ---- 粗扫 ----
    coarse = np.arange(predicted_dth - window, predicted_dth + window + 1e-12,
                       COARSE_DTH_ROT)
    best = float(coarse[int(np.argmin([cost_of(v) for v in coarse]))])

    # ---- 细扫（±1 粗步，细步 0.02°）----
    fine_h = math.radians(0.02)
    fine = np.arange(best - math.radians(1.0), best + math.radians(1.0) + 1e-12,
                     fine_h)
    fine_costs = np.array([cost_of(v) for v in fine])
    fj = int(np.argmin(fine_costs))
    best = float(fine[fj])
    if 0 < fj < len(fine) - 1:
        # 抛物线只用来把最优点插值到网格以下（精度），不用它取曲率
        best += _parabola(fine_costs[fj - 1], fine_costs[fj],
                          fine_costs[fj + 1], fine_h)[0]
    est.dth = best
    kappa = _curvature_coarse(cost_of, best, COARSE_DTH_ROT)

    # ---- 对手峰扫描：故意扫到 ±180°（比搜索窗宽）----
    # 矩形房间的 180° 镜像、正方形房间的 90° 镜像都会在这里现形；窗口内的候选
    # 被排除，因为它们属于同一个解的邻域。
    rival_cost, rival_at = float("inf"), None
    for k in range(-60, 61):
        cand = best + math.radians(3.0) * k
        if abs(cand - best) <= window + 1e-9:
            continue
        c = cost_of(cand)
        if c < rival_cost:
            rival_cost, rival_at = c, cand
    c_best = cost_of(best)
    est.rival_ratio = (rival_cost / c_best) if c_best > 1e-15 else float("inf")
    est.rival_dth = rival_at

    # ---- 残余平移（诊断：base_link 是否真的在旋转中心上）----
    # 模型里旋转测试固定 dx=dy=0。若实际旋转中心与 base_link 原点不符，这里会解出
    # 非零的残余平移；多轮运行中若持续 >3cm，说明旋转中心假设有问题，
    # 前面 0.1~0.3° 的精度声明就不成立（见计划风险 #1）。
    def cost_xy(params, axis, value):
        p = list(params)
        p[axis] = value
        return cost_of(best, dx=p[0], dy=p[1])

    xy = [0.0, 0.0]
    for axis in (0, 1):
        xy[axis], _ = _refine_axis(cost_xy, xy, axis, 0.20, (-0.30, 0.30),
                                   rounds=2, fine_div=10, fine_half=5)
    # 注意：优化变量 (dx,dy) 本身就在车体系里 —— 它们位于 _apply 里
    # R(−ψ)[(R(dth)−I)m + d] 那个括号内，与真实变换逐项对应，所以不能再做一次
    # "雷达系→车体系"的换算，否则会重复扣除传感器偏移（90° 旋转会凭空多出
    # |(R(90°)−I)m| = 0.21m 的假残余）。
    est.dx, est.dy = float(xy[0]), float(xy[1])
    est.residual_dx, est.residual_dy = est.dx, est.dy
    est.distance = math.hypot(est.dx, est.dy)
    est.lidar_frame_xy = (float(xy[0]), float(xy[1]))

    stats = _profile_stats(prof, cur_pts, xy[0], xy[1], est.dth, mount_xy,
                           mount_yaw)
    hits = _boundary_hits((est.dx, est.dy), ((-0.30, 0.30), (-0.30, 0.30)),
                          ("残余 dx", "残余 dy"))
    hits += _boundary_hits((est.dth,),
                           ((predicted_dth - window, predicted_dth + window),),
                           ("转角搜索窗",))
    return _finish(est, stats, float("inf"), kappa, tol_d=None,
                   tol_dth=SIGMA_DTH_MAX, need_rival=True,
                   boundary_hits=hits)


def estimate_translation(prof, cur_pts, *, predicted_dx, mount_xy=(0.15, 0.0),
                         mount_yaw=0.0):
    """直行的雷达真值。返回 MotionEstimate（distance 为净位移，米）。

    predicted_dx 是里程计报告的行程（米），只用来把搜索窗限制在预测附近 ——
    估计器从不负责"找到"运动，只负责在预测附近**精修**。
    """
    est = MotionEstimate()
    if len(prof) < MIN_PTS or len(cur_pts) < MIN_PTS:
        est.reasons = [f"有效点/样点过少（参考 {len(prof)}，当前 {len(cur_pts)}）"]
        return est

    def cost_of(p):
        return _profile_cost(prof, cur_pts, p[0], p[1], p[2], mount_xy,
                             mount_yaw)

    # ---- 粗网格 ----
    dths = np.arange(-math.radians(6.0), math.radians(6.0) + 1e-12,
                     math.radians(2.0))
    dxs = np.arange(predicted_dx - 0.20, predicted_dx + 0.20 + 1e-12, 0.05)
    dys = np.arange(-0.15, 0.15 + 1e-12, 0.05)
    best_cost, best = float("inf"), [float(predicted_dx), 0.0, 0.0]
    for dth in dths:
        for dx in dxs:
            for dy in dys:
                c = cost_of([dx, dy, dth])
                if c < best_cost:
                    best_cost, best = c, [float(dx), float(dy), float(dth)]

    # ---- 细扫：逐轴坐标下降两轮 ----
    # 代价远离分离，但最优解附近近似可分离，逐轴抛物线插值成立；全网格精搜是
    # 61³≈22.7 万次评价，没必要。
    bounds = ((predicted_dx - 0.30, predicted_dx + 0.30), (-0.25, 0.25),
              (-math.radians(8.0), math.radians(8.0)))
    steps = (COARSE_D, COARSE_D, COARSE_DTH)

    def cost_axis(params, axis, value):
        p = list(params)
        p[axis] = value
        return cost_of(p)

    for _ in range(2):
        for axis in range(3):
            # 这里只用 _refine_axis 定位最优点；曲率另外在粗尺度上取
            best[axis], _ = _refine_axis(cost_axis, best, axis, steps[axis],
                                         bounds[axis])

    est.dth = best[2]
    # 同上：优化变量本身就在车体系，不做二次换算
    est.dx, est.dy = float(best[0]), float(best[1])
    est.lidar_frame_xy = (float(best[0]), float(best[1]))
    est.distance = math.hypot(est.dx, est.dy)
    # 平移不做对手峰扫描：走廊/沿墙这类退化表现为代价面平坦，由 sigma 捕获
    est.rival_ratio = float("inf")
    est.rival_dth = None

    stats = _profile_stats(prof, cur_pts, best[0], best[1], best[2], mount_xy,
                           mount_yaw)
    # 曲率在粗网格尺度上取（见 _curvature_coarse），不用细扫步长
    kappa_dx = _curvature_coarse(
        lambda v: cost_of([v, best[1], best[2]]), best[0], COARSE_D)
    kappa_dth = _curvature_coarse(
        lambda v: cost_of([best[0], best[1], v]), best[2], COARSE_DTH)
    coarse_bounds = ((float(dxs[0]), float(dxs[-1])),
                     (float(dys[0]), float(dys[-1])),
                     (float(dths[0]), float(dths[-1])))
    hits = _boundary_hits(best, coarse_bounds, ("dx", "dy", "dθ"))
    return _finish(est, stats, kappa_dx, kappa_dth, tol_d=SIGMA_D_MAX,
                   tol_dth=math.radians(5.0), need_rival=False,
                   boundary_hits=hits)


# ---------------------------------------------------------------------------
# 驱动判据（抽成纯函数，便于离线自测）
# ---------------------------------------------------------------------------
def drive_step_action(progress, target, elapsed, max_time, since_window,
                      window_progress, stall_window, stall_eps):
    """闭环驱动的单步决策。返回 "done" / "timeout" / "stall" / "drive"。"""
    if abs(progress) >= abs(target):
        return "done"
    if elapsed >= max_time:
        return "timeout"
    if since_window >= stall_window and abs(progress - window_progress) < stall_eps:
        return "stall"
    return "drive"


@dataclasses.dataclass
class DriveResult:
    requested: float = 0.0
    reached: float = 0.0
    ok: bool = False
    reason: str = "ok"
    elapsed: float = 0.0


# ---------------------------------------------------------------------------
# ROS 侧：采集、驱动、活参数
# ---------------------------------------------------------------------------
def _scan_qos():
    """与 lidar_crop_check.py 相同的 QoS —— 已验证能收到数据。"""
    return QoSProfile(depth=10,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


class ScanGrabber:
    """订阅 /scan 并缓存若干帧。作为独立小节点，便于独立初始化。"""

    def __init__(self, node, topic="/scan"):
        self.node = node
        self.frames = []
        self.meta = None
        self.node.create_subscription(LaserScan, topic, self._on_scan,
                                      _scan_qos())

    def _on_scan(self, msg):
        if self.meta is None:
            self.meta = (msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max)
        # 逐帧记录角度参数：实测 beam 数会跳变，每帧的 increment 都不同
        self.frames.append(ScanFrame(np.asarray(msg.ranges, dtype=np.float64),
                                     float(msg.angle_min),
                                     float(msg.angle_increment)))

    def clear(self):
        self.frames = []


def capture_frames(node, grabber, *, seconds=1.0, min_frames=MIN_FRAMES,
                   zero_pub=None):
    """采集一批 /scan 帧。返回 (frames, meta)；失败时 frames 为 None 并附原因。

    采集期间可传 zero_pub 持续发零速——零速不会重置底盘看门狗（重置要求
    |vx|>1e-4），故它只用于抵抗其它遥控发布者，不会干扰停车。
    """
    grabber.clear()
    deadline = time.monotonic() + seconds
    while HAVE_ROS and rclpy.ok() and time.monotonic() < deadline:
        if zero_pub is not None:
            zero_pub()
        rclpy.spin_once(node, timeout_sec=0.05)

    if grabber.meta is None or len(grabber.frames) < min_frames:
        return None, {"reason": f"未收到足够的 /scan（需要 ≥{min_frames} 帧，"
                                f"实得 {len(grabber.frames)} 帧）"}
    # 逐帧检查角度模型（不能只看第一帧：beam 数在实机上会跳变）
    for frame in grabber.frames:
        if abs(frame.angle_min) > 1e-6 or frame.angle_increment <= 0.0:
            return None, {"reason": f"/scan 的角度模型不符："
                                    f"angle_min={frame.angle_min:.4f}, "
                                    f"angle_increment={frame.angle_increment:.5f}；"
                                    f"本模块假设 angle_min=0 且 increment>0"}
    beams = sorted({int(f.ranges.size) for f in grabber.frames})
    return grabber.frames, {"frames": len(grabber.frames),
                            "angle_min": grabber.meta[0],
                            "angle_increment": grabber.meta[1],
                            "beams": beams}


def capture_reference(node, grabber, *, r_min=R_MIN, r_max=R_MAX, **kw):
    """采集参考廓线。返回 (RefProfile, meta) 或 (None, {"reason": ...})。"""
    frames, meta = capture_frames(node, grabber, **kw)
    if frames is None:
        return None, meta
    prof = profile_from_frames(frames, r_min=r_min, r_max=r_max)
    if len(prof) < MIN_PTS:
        return None, {"reason": f"参考廓线有效样点仅 {len(prof)}，不足 {MIN_PTS}"}
    return prof, meta


def capture_points(node, grabber, *, r_min=R_MIN, r_max=R_MAX, **kw):
    """采集当前帧点云。返回 (points, meta) 或 (None, {"reason": ...})。"""
    frames, meta = capture_frames(node, grabber, **kw)
    if frames is None:
        return None, meta
    pts = prepare_points(frames, r_min=r_min, r_max=r_max)
    if len(pts) < MIN_PTS:
        return None, {"reason": f"有效点数仅 {len(pts)}，不足 {MIN_PTS}"}
    return pts, meta


class MotionDriver(_RclpyNode):
    """里程计跟踪 + 闭环驱动。是 odom_calib.py 里 OdomTracker 的超集，
    原有属性 (distance / yaw_total / speed / yaw_rate / samples) 语义不变。
    """

    def __init__(self, node_name, *, cmd_topic="/cmd_vel", odom_topic="/odom",
                 publish_hz=20.0):
        super().__init__(node_name)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.publish_period = 1.0 / publish_hz
        self.distance = 0.0
        self.yaw_total = 0.0
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.samples = 0

    # ---- 里程计累计 ----
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last_x is not None:
            self.distance += math.hypot(x - self.last_x, y - self.last_y)
            # 相邻帧角度差先折算到 (-π,π] 再累加，避免 ±π 跳变
            self.yaw_total += math.atan2(math.sin(yaw - self.last_yaw),
                                         math.cos(yaw - self.last_yaw))
        self.last_x, self.last_y, self.last_yaw = x, y, yaw
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        self.samples += 1

    def reset_accumulators(self):
        self.distance = 0.0
        self.yaw_total = 0.0

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout=8.0):
        """等到收到第一帧 /odom，返回是否成功。

        不要用"固定 spin 2 秒然后看有没有数据"：ROS 图发现在实机上是变长的
        （机器上还跑着 rosbridge / delivery 等服务时更慢），实测会偶发地误报
        "未收到 /odom"。这里改成轮询等待，拿到数据就立刻返回。
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.samples == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.samples > 0

    def publish_zero(self, seconds=1.0):
        """持续发零速。零速不重置看门狗，仅防止其它发布者接管。"""
        stop = Twist()
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.cmd_pub.publish(stop)
            except Exception:
                return
            rclpy.spin_once(self, timeout_sec=self.publish_period)

    def wait_until_stopped(self, timeout=8.0):
        """等小车停稳（连续 0.5s 速度低于阈值）。"""
        deadline = time.monotonic() + timeout
        still_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            quiet = abs(self.speed) < 0.005 and abs(self.yaw_rate) < 0.01
            if quiet:
                if still_since is None:
                    still_since = time.monotonic()
                elif time.monotonic() - still_since > 0.5:
                    return True
            else:
                still_since = None
        return False

    def stop_robot(self):
        """发 2s 零速并等停稳。吞掉异常，保证在 rclpy 已关闭时也能安全调用。"""
        try:
            self.publish_zero(seconds=2.0)
            self.wait_until_stopped()
        except Exception:
            pass

    # ---- 闭环驱动 ----
    def _drive_loop(self, twist, target, progress_of, max_time, stall_window,
                    stall_eps):
        """按固定 Twist 驱动到 |progress| >= |target|，随后停车并等停稳。

        目标是"软目标"：最终标定用的是"实测/里程计"比值而非"实测/指令值"，
        所以过冲无害，不需要减速补偿 —— 这也是这段代码能这么简单的原因。
        """
        start = time.monotonic()
        window_start = start
        window_progress = progress_of()
        progress = window_progress
        try:
            while rclpy.ok():
                progress = progress_of()
                elapsed = time.monotonic() - start
                since_window = time.monotonic() - window_start
                action = drive_step_action(progress, target, elapsed, max_time,
                                           since_window, window_progress,
                                           stall_window, stall_eps)
                if action != "drive":
                    if action == "timeout":
                        self.get_logger().warning("驱动超时：未达到目标即中止")
                    elif action == "stall":
                        self.get_logger().warning(
                            "检测到停滞：里程计 1s 内几乎无增量，可能打滑或"
                            "轮子悬空，已中止")
                    return DriveResult(requested=target, reached=progress,
                                       ok=(action == "done"), reason=action,
                                       elapsed=elapsed)
                if since_window >= stall_window:
                    window_start = time.monotonic()
                    window_progress = progress
                try:
                    self.cmd_pub.publish(twist)
                except Exception:
                    return DriveResult(requested=target, reached=progress,
                                       ok=False, reason="publish_failed",
                                       elapsed=elapsed)
                # 20Hz 发布，是底盘 600ms 看门狗窗口的一半
                rclpy.spin_once(self, timeout_sec=self.publish_period)
        except BaseException:
            # 任何异常/中断都必须先停车再往上抛
            self.publish_zero(seconds=0.3)
            raise
        finally:
            self.stop_robot()
        return DriveResult(requested=target, reached=progress_of(), ok=True,
                           reason="ok", elapsed=time.monotonic() - start)

    def drive_distance(self, target_m, speed=0.12, *, max_time=None,
                       stall_window=1.0, stall_eps=0.005):
        """前进到里程计累计行程达到 target_m。"""
        if max_time is None:
            max_time = abs(target_m) / max(abs(speed), 1e-3) * 2.0 + 5.0
        twist = Twist()
        twist.linear.x = float(speed)
        return self._drive_loop(twist, target_m, lambda: self.distance,
                                max_time, stall_window, stall_eps)

    def rotate_to_angle(self, target_deg, speed=0.6, *, max_time=None,
                        stall_window=1.0, stall_eps=0.01):
        """原地转到里程计累计转角达到 target_deg（正数=逆时针）。"""
        target = math.radians(target_deg)
        if max_time is None:
            max_time = abs(target) / max(abs(speed), 1e-3) * 2.0 + 5.0
        twist = Twist()
        twist.angular.z = float(speed)
        return self._drive_loop(twist, target, lambda: self.yaw_total,
                                max_time, stall_window, stall_eps)


def get_live_factor(node_name, param, fallback):
    """从运行中的节点读参数，失败时回落。返回 (值, 来源说明或 None)。

    vth_calib_measure.py 里有同样的解析写法，但它用的 check_output 没有超时，
    daemon 卡住时会永久挂起；这里加超时。

    第一次调用常常会失败：ros2 CLI 要冷启动 daemon，可能超过 5s。所以失败后
    用更长的超时重试一次 —— 否则会静默回落到 chassis.json，而那个值可能与
    运行中实际生效的值不同，导致"旧值→新值"的换算基准是错的。
    """
    for timeout in (5.0, 15.0):
        try:
            out = subprocess.run(["ros2", "param", "get", node_name, param],
                                 capture_output=True, timeout=timeout)
            text = out.stdout.decode("utf-8", "replace").strip()
            if out.returncode == 0 and ":" in text:
                return float(text.split(":", 1)[1].strip()), \
                    f"来自运行中的 {node_name} 参数"
        except (OSError, ValueError, subprocess.SubprocessError):
            continue
    return fallback, None


def static_selftest(node, grabber, *, seconds=1.0, settle=1.0,
                    mount_xy=(0.15, 0.0), mount_yaw=0.0, r_min=R_MIN,
                    r_max=R_MAX, zero_pub=None):
    """静止自检：不动的情况下采集两帧，估计结果应约为 0。

    用已知答案 0 验证整条链路（QoS、角度模型、裁剪、环境静止、估算器），
    比任何单次估计的质量指标都更能抓住"环境里有人走动"。运动之前必须跑。
    """
    ref, meta_a = capture_reference(node, grabber, seconds=seconds,
                                    r_min=r_min, r_max=r_max,
                                    zero_pub=zero_pub)
    if ref is None:
        return False, f"参考帧采集失败：{meta_a['reason']}"
    deadline = time.monotonic() + settle
    while HAVE_ROS and rclpy.ok() and time.monotonic() < deadline:
        if zero_pub is not None:
            zero_pub()
        rclpy.spin_once(node, timeout_sec=0.05)
    cur, meta_b = capture_points(node, grabber, seconds=seconds,
                                 r_min=r_min, r_max=r_max, zero_pub=zero_pub)
    if cur is None:
        return False, f"复测帧采集失败：{meta_b['reason']}"

    est = estimate_translation(ref, cur, predicted_dx=0.0, mount_xy=mount_xy,
                               mount_yaw=mount_yaw)
    if not est.reliable:
        return False, ("静止状态下估计不可靠，环境或链路有问题：\n"
                       + est.describe("      "))
    if abs(est.distance) >= 0.02 or abs(math.degrees(est.dth)) >= 0.5:
        return False, (f"静止状态下竟测出位移 {est.distance * 1000:.0f} mm / "
                       f"转角 {math.degrees(est.dth):.2f}°（应为 0）。"
                       f"可能是环境里有移动物体、雷达被晃动，"
                       f"或 base_link 不在旋转中心上")
    return True, (f"静止自检通过：位移 {est.distance * 1000:.1f} mm（应≈0）、"
                  f"转角 {math.degrees(est.dth):.3f}°、"
                  f"内点 {est.n_inlier} 点、覆盖 {est.overlap * 100:.0f}%")


# ---------------------------------------------------------------------------
# 离线自测：合成场景
# ---------------------------------------------------------------------------
def _box(cx, cy, half, segs):
    """往 segs 里加一个正方形立柱的 4 条边。"""
    corners = [(cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half)]
    for i in range(4):
        segs.append((corners[i], corners[(i + 1) % 4]))


def _room_asymmetric():
    """6×4m 房间，东侧留切角，北侧加一个 0.8m 见方的立柱 —— 刻意不对称。"""
    segs = [((0.0, 0.0), (5.4, 0.0)),        # 南墙（留出切角）
            ((5.4, 0.0), (6.0, 0.6)),        # 切角
            ((6.0, 0.6), (6.0, 4.0)),        # 东墙
            ((6.0, 4.0), (0.0, 4.0)),        # 北墙
            ((0.0, 4.0), (0.0, 0.0))]        # 西墙
    _box(2.0, 3.2, 0.4, segs)                # 立柱
    return segs


def _room_square():
    """5×5 正方形房间：有 90° 旋转对称（验证对手峰检测）。"""
    return [((0.0, 0.0), (5.0, 0.0)), ((5.0, 0.0), (5.0, 5.0)),
            ((5.0, 5.0), (0.0, 5.0)), ((0.0, 5.0), (0.0, 0.0))]


def _room_rect():
    """6×4 矩形房间：有 180° 旋转对称。"""
    return [((0.0, 0.0), (6.0, 0.0)), ((6.0, 0.0), (6.0, 4.0)),
            ((6.0, 4.0), (0.0, 4.0)), ((0.0, 4.0), (0.0, 0.0))]


def _corridor():
    """两条平行墙、沿 x 延伸很远：沿走廊方向的平移不可观测。"""
    return [((-40.0, -1.5), (40.0, -1.5)), ((-40.0, 1.5), (40.0, 1.5))]


N_RAYS = 503                # 与 STL06N 实际 beam 数一致（0.7156° 角分辨率）
ANGLE_INC = 2.0 * math.pi / N_RAYS


def _cast(segs, base_pose, mount_xy, rng, *, mount_yaw=0.0, n_rays=N_RAYS,
          noise=0.01, dropout=0.0, r_max=R_MAX):
    """从给定车体位姿模拟一次 /scan，返回 ranges 数组（含 NaN）。

    base_pose = (x, y, yaw) 在世界系；雷达世界位姿 = 车体位姿 + 安装位姿
    （位置偏移 mount_xy，朝向再叠加 mount_yaw）。

    角间距取 2π/n_rays：与真实驱动"把 [0,2π] 均分"的行为一致，这样 beam 数
    变化时增量也跟着变（实机就是这样，见 ScanFrame）。
    """
    bx, by, byaw = base_pose
    cy, sy = math.cos(byaw), math.sin(byaw)
    lidar_xy = np.array([bx + cy * mount_xy[0] - sy * mount_xy[1],
                         by + sy * mount_xy[0] + cy * mount_xy[1]])

    angle_increment = 2.0 * math.pi / n_rays
    bearings = np.arange(n_rays) * angle_increment
    world_angles = bearings + byaw + mount_yaw
    u = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)

    segs_arr = np.asarray(segs, dtype=np.float64)      # (S,2,2)
    a = segs_arr[:, 0, :]
    d = segs_arr[:, 1, :] - segs_arr[:, 0, :]

    # 射线 origin + t·u 与线段 a + s·d 的交点（二维叉积参数化）
    ao = a[None, :, :] - lidar_xy[None, None, :]       # (1,S,2)
    ue = u[:, None, :]
    de = d[None, :, :]
    denom = ue[..., 0] * de[..., 1] - ue[..., 1] * de[..., 0]
    num_t = ao[..., 0] * de[..., 1] - ao[..., 1] * de[..., 0]
    num_s = ao[..., 0] * ue[..., 1] - ao[..., 1] * ue[..., 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = num_t / denom
        s = num_s / denom
    valid = (np.abs(denom) > 1e-12) & (t > 1e-6) & (s >= 0.0) & (s <= 1.0)
    t = np.where(valid, t, np.inf)
    ranges = t.min(axis=1)
    ranges[~np.isfinite(ranges)] = np.nan
    ranges[ranges > r_max] = np.nan

    hit = np.isfinite(ranges)
    ranges[hit] += rng.normal(0.0, noise, size=int(hit.sum()))
    if dropout > 0.0:
        ranges[hit & (rng.random(n_rays) < dropout)] = np.nan
    return ranges


def _scan_frames(segs, base_pose, mount_xy, rng, *, n_frames=4,
                 beam_jitter=False, **kw):
    """生成若干 ScanFrame。

    beam_jitter=True 时模拟实机：beam 数在 503~506 之间跳变，且角间距随之变化
    （驱动总是把 [0,2π] 均分）。这是真实底盘上暴露出来的情况，合成场景默认的
    等长帧是覆盖不到的。
    """
    frames = []
    for _ in range(n_frames):
        n = 503 + int(rng.integers(0, 4)) if beam_jitter else kw.get("n_rays",
                                                                    N_RAYS)
        ranges = _cast(segs, base_pose, mount_xy, rng, **dict(kw, n_rays=n))
        frames.append(ScanFrame(ranges, 0.0, 2.0 * math.pi / n))
    return frames


def _ref_from(segs, base_pose, mount_xy, rng, **kw):
    return profile_from_frames(_scan_frames(segs, base_pose, mount_xy, rng, **kw))


def _pts_from(segs, base_pose, mount_xy, rng, **kw):
    return prepare_points(_scan_frames(segs, base_pose, mount_xy, rng, **kw))


def _crop(ranges, lo_deg, hi_deg):
    """模拟驱动把某个扇区置 NaN（裁剪）。"""
    deg = np.degrees(np.arange(ranges.size) * ANGLE_INC) % 360.0
    out = ranges.copy()
    out[(deg >= lo_deg) & (deg <= hi_deg)] = np.nan
    return out


def _selftest_motion_math():
    """驱动判据纯函数的用例。"""
    fails = []
    cases = [
        ("D1 未达标应继续驱动",
         drive_step_action(0.5, 1.0, 1.0, 30.0, 0.2, 0.4, 1.0, 0.005) == "drive"),
        ("D2 达标应判 done",
         drive_step_action(1.02, 1.0, 5.0, 30.0, 0.2, 0.9, 1.0, 0.005) == "done"),
        ("D3 超时应判 timeout",
         drive_step_action(0.3, 1.0, 31.0, 30.0, 0.2, 0.25, 1.0, 0.005) == "timeout"),
        ("D4 停滞应判 stall",
         drive_step_action(0.3, 1.0, 5.0, 30.0, 1.5, 0.298, 1.0, 0.005) == "stall"),
        ("D5 窗口未满不判停滞",
         drive_step_action(0.3, 1.0, 5.0, 30.0, 0.5, 0.298, 1.0, 0.005) == "drive"),
        ("D6 有进展不判停滞",
         drive_step_action(0.7, 1.0, 5.0, 30.0, 1.5, 0.4, 1.0, 0.005) == "drive"),
    ]
    for name, ok in cases:
        if not ok:
            fails.append(name)
    return fails


def _selftest(mc_trials=12, verbose=True):
    """合成场景自测。返回 (通过数, 失败列表)。"""
    rng = np.random.default_rng(20240917)
    mount = (0.15, 0.0)
    failures = []
    passed = 0

    def check(name, cond, detail=""):
        nonlocal passed
        if cond:
            passed += 1
            if verbose:
                print(f"  ✓ {name}")
        else:
            failures.append(f"{name}：{detail}")
            print(f"  ✗ {name}：{detail}")

    room = _room_asymmetric()

    # ---- C1 平移 1.000 m（含 0.5° 航向漂移）----
    # 参考车体位姿取 yaw=0，使世界系 == B0 系，(dx,dy) 含义无歧义
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    cur = _pts_from(room, (4.0, 1.5, math.radians(0.5)), mount, rng)
    est = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    if verbose:
        print("  C1 平移 1.000m:" + "\n" + est.describe("      "))
    check("C1 位移误差<10mm", abs(est.distance - 1.0) < 0.010,
          f"测得 {est.distance:.4f} m")
    check("C1 转角误差<0.3°", abs(math.degrees(est.dth) - 0.5) < 0.3,
          f"测得 {math.degrees(est.dth):.3f}°")
    check("C1 判为可靠", est.reliable, "；".join(est.reasons))

    # ---- C2 旋转 90.3° ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    cur = _pts_from(room, (3.0, 1.5, math.radians(90.3)), mount, rng)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(90.3),
                            mount_xy=mount)
    if verbose:
        print("  C2 旋转 90.3°:" + "\n" + est.describe("      "))
    check("C2 转角误差<0.3°", abs(math.degrees(est.dth) - 90.3) < 0.3,
          f"测得 {math.degrees(est.dth):.3f}°")
    check("C2 判为可靠", est.reliable, "；".join(est.reasons))
    check("C2 残余平移<20mm", math.hypot(est.residual_dx, est.residual_dy) < 0.02,
          f"残余 {math.hypot(est.residual_dx, est.residual_dy) * 1000:.1f} mm")

    # ---- C3 旋转 720.4°：必须还原 720.4° 而不是 0.4° ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    cur = _pts_from(room, (3.0, 1.5, math.radians(720.4)), mount, rng)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(720.4),
                            mount_xy=mount)
    if verbose:
        print("  C3 旋转 720.4°:" + "\n" + est.describe("      "))
    check("C3 还原 720.4°（非 0.4°）",
          abs(math.degrees(est.dth) - 720.4) < 0.3,
          f"测得 {math.degrees(est.dth):.3f}°")
    check("C3 判为可靠", est.reliable, "；".join(est.reasons))

    # ---- C4 正方形房间转 90°：应有等价对手峰 → 不可靠 ----
    ref = _ref_from(_room_square(), (2.5, 2.5, 0.0), mount, rng)
    cur = _pts_from(_room_square(), (2.5, 2.5, math.radians(90.4)), mount, rng)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(90.4),
                            mount_xy=mount)
    check("C4 正方形房间应判不可靠", not est.reliable,
          f"却判为可靠（转角 {math.degrees(est.dth):.2f}°，"
          f"对手峰比 {est.rival_ratio:.2f}）")
    check("C4 原因提到对手峰/对称",
          any(("对手峰" in r) or ("对称" in r) for r in est.reasons),
          f"原因为 {est.reasons}")

    # ---- C5 矩形房间转 90°：180° 镜像解 → 不可靠 ----
    ref = _ref_from(_room_rect(), (3.0, 2.0, 0.0), mount, rng)
    cur = _pts_from(_room_rect(), (3.0, 2.0, math.radians(90.4)), mount, rng)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(90.4),
                            mount_xy=mount)
    check("C5 矩形房间应判不可靠", not est.reliable,
          f"却判为可靠（对手峰比 {est.rival_ratio:.2f}，"
          f"位于 {math.degrees(est.rival_dth) if est.rival_dth else float('nan'):.1f}°）")

    # ---- C6 走廊沿墙平移：不可观测 → 不可靠 ----
    ref = _ref_from(_corridor(), (0.0, 0.0, 0.0), mount, rng)
    cur = _pts_from(_corridor(), (1.0, 0.0, 0.0), mount, rng)
    est = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    check("C6 走廊平移应判不可靠", not est.reliable,
          f"却判为可靠（位移 {est.distance:.3f}，"
          f"sigma {est.sigma_d * 1000:.1f}mm）")

    # ---- C7 40% 射线 NaN（模拟裁剪）仍可靠 ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng, dropout=0.40)
    cur = _pts_from(room, (4.0, 1.5, 0.0), mount, rng, dropout=0.40)
    est = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    check("C7 40%丢点仍可靠", est.reliable, "；".join(est.reasons))
    check("C7 40%丢点仍<20mm", abs(est.distance - 1.0) < 0.020,
          f"测得 {est.distance:.4f} m")

    # ---- C7b 裁剪扇区（驱动实际行为）仍可靠 ----
    def _cropped(pose):
        return [ScanFrame(_crop(f.ranges, 130.0, 229.0), f.angle_min,
                          f.angle_increment)
                for f in _scan_frames(room, pose, mount, rng)]

    ref_c = profile_from_frames(_cropped((3.0, 1.5, 0.0)))
    cur_c = prepare_points(_cropped((4.0, 1.5, 0.0)))
    est = estimate_translation(ref_c, cur_c, predicted_dx=1.0, mount_xy=mount)
    check("C7b 130-229°裁剪仍可靠", est.reliable, "；".join(est.reasons))
    check("C7b 裁剪下仍<20mm", abs(est.distance - 1.0) < 0.020,
          f"测得 {est.distance:.4f} m")

    # ---- C14 实机的变长帧（回归：曾直接让工具崩掉）----
    # 真实底盘上 /scan 的 beam 数在 503~506 之间跳变、角间距随之改变。
    # 早先的实现用第一帧的 increment 去解释所有帧，拼接时直接
    # "all the input array dimensions must match" 崩掉。这里必须既不崩、又准。
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng, beam_jitter=True)
    cur = _pts_from(room, (4.0, 1.5, 0.0), mount, rng, beam_jitter=True)
    est = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    check("C14 变长帧不崩且可靠", est.reliable, "；".join(est.reasons))
    check("C14 变长帧仍<20mm", abs(est.distance - 1.0) < 0.020,
          f"测得 {est.distance:.4f} m")
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng, beam_jitter=True)
    cur = _pts_from(room, (3.0, 1.5, math.radians(90.3)), mount, rng,
                    beam_jitter=True)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(90.3),
                            mount_xy=mount)
    check("C14 变长帧旋转仍<0.3°", abs(math.degrees(est.dth) - 90.3) < 0.3,
          f"测得 {math.degrees(est.dth):.3f}°")

    # ---- C8 3% 点被移动物替换 ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    cur = _pts_from(room, (4.0, 1.5, 0.0), mount, rng)
    k = max(1, int(0.03 * len(cur)))
    blob = rng.normal(0.0, 0.05, size=(k, 2)) + np.array([1.2, 0.5])
    cur = np.vstack([cur[:-k], blob])
    est = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    check("C8 移动物干扰仍可靠", est.reliable, "；".join(est.reasons))
    check("C8 移动物干扰仍<20mm", abs(est.distance - 1.0) < 0.020,
          f"测得 {est.distance:.4f} m")

    # ---- C9 蒙特卡洛：验证 sigma 是否覆盖真实离散 ----
    # 注意区分两件事：sigma 只能预测**随机离散**（spread），预测不了**系统性
    # 偏差**（bias，来自参考廓线网格插值等）。所以这里分开断言，并把实测数字
    # 打出来作为 SIGMA_SAFETY 取值的依据。
    ests, sigmas = [], []
    for _ in range(mc_trials):
        r = np.random.default_rng(int(rng.integers(1 << 30)))
        ref = _ref_from(room, (3.0, 1.5, 0.0), mount, r, n_frames=3)
        cur = _pts_from(room, (4.0, 1.5, 0.0), mount, r, n_frames=3)
        e = estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
        if e.reliable and math.isfinite(e.sigma_d):
            ests.append(e.distance)
            sigmas.append(e.sigma_d)
    if len(ests) >= max(3, mc_trials // 2):
        arr = np.array(ests)
        mean_sigma = float(np.mean(sigmas))
        bias = float(arr.mean() - 1.0)
        spread = float(arr.std(ddof=1))
        check(f"C9 sigma 覆盖离散（{len(ests)} 次，spread ≤ 3σ）",
              spread <= 3.0 * mean_sigma,
              f"离散 {spread * 1000:.2f}mm vs 3σ={3 * mean_sigma * 1000:.2f}mm")
        check("C9 系统性偏差 <10mm", abs(bias) < 0.010,
              f"bias {bias * 1000:+.2f}mm")
        if verbose:
            print(f"      C9 实测：bias {bias * 1000:+.2f}mm，"
                  f"spread {spread * 1000:.2f}mm，报告 σ {mean_sigma * 1000:.2f}mm"
                  f"（spread/σ={spread / mean_sigma:.2f}）")
    else:
        failures.append(f"C9 可靠样本不足（仅 {len(ests)}/{mc_trials}）")

    # ---- C12 蒙特卡洛（旋转）----
    # C9 只验证了位移 sigma；转角 sigma 用的是同一套公式，必须单独验证。
    rests, rsigmas = [], []
    for _ in range(mc_trials):
        r = np.random.default_rng(int(rng.integers(1 << 30)))
        ref = _ref_from(room, (3.0, 1.5, 0.0), mount, r, n_frames=3)
        cur = _pts_from(room, (3.0, 1.5, math.radians(90.3)), mount, r,
                        n_frames=3)
        e = estimate_rotation(ref, cur, predicted_dth=math.radians(90.3),
                              mount_xy=mount)
        if e.reliable and math.isfinite(e.sigma_dth):
            rests.append(math.degrees(e.dth))
            rsigmas.append(math.degrees(e.sigma_dth))
    if len(rests) >= max(3, mc_trials // 2):
        arr = np.array(rests)
        mean_sigma = float(np.mean(rsigmas))
        bias = float(arr.mean() - 90.3)
        spread = float(arr.std(ddof=1))
        check(f"C12 转角 sigma 覆盖离散（{len(rests)} 次，spread ≤ 3σ）",
              spread <= 3.0 * mean_sigma,
              f"离散 {spread:.4f}° vs 3σ={3 * mean_sigma:.4f}°")
        check("C12 转角系统性偏差 <0.5°", abs(bias) < 0.5,
              f"bias {bias:+.4f}°")
        if verbose:
            print(f"      C12 实测：bias {bias:+.4f}°，"
                  f"spread {spread:.4f}°，报告 σ {mean_sigma:.4f}°"
                  f"（spread/σ={spread / mean_sigma:.2f}）")
    else:
        failures.append(f"C12 可靠样本不足（仅 {len(rests)}/{mc_trials}）")

    # ---- C13 mount_yaw != 0（倒装配置）仍能解出正确转角 ----
    # 配置页明确允许把 mount_yaw 设成 3.1416 对应倒装；若模型忽略它，平移项会
    # 整体偏转（可达 0.21m），拟合会变差甚至给错解。这里验证带 ψ 时仍正确。
    yaw_mount = (0.15, 0.0)
    ref = _ref_from(room, (3.0, 1.5, 0.0), yaw_mount, rng, mount_yaw=math.pi)
    cur = _pts_from(room, (3.0, 1.5, math.radians(90.3)), yaw_mount, rng,
                    mount_yaw=math.pi)
    est = estimate_rotation(ref, cur, predicted_dth=math.radians(90.3),
                            mount_xy=yaw_mount, mount_yaw=math.pi)
    check("C13 mount_yaw=π 仍解出 90.3°",
          abs(math.degrees(est.dth) - 90.3) < 0.3,
          f"测得 {math.degrees(est.dth):.3f}°")
    # 平移方向也要验：优化变量就在车体系里，若有人给解"补"一次雷达系→车体系
    # 换算，dx 会翻成 −1.0（多扣了一次传感器偏移）
    cur_t = _pts_from(room, (4.0, 1.5, 0.0), yaw_mount, rng, mount_yaw=math.pi)
    est_t = estimate_translation(ref, cur_t, predicted_dx=1.0,
                                 mount_xy=yaw_mount, mount_yaw=math.pi)
    check("C13b mount_yaw=π 平移解出 +1.0m 车体位移",
          abs(est_t.distance - 1.0) < 0.010 and abs(est_t.dx - 1.0) < 0.02,
          f"测得 distance={est_t.distance:.4f} dx={est_t.dx:+.4f}")

    # ---- C10 退化：点数不足 ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    few = _pts_from(room, (3.0, 1.5, 0.0), mount, rng)[:30]
    est = estimate_translation(ref, few, predicted_dx=1.0, mount_xy=mount)
    check("C10 点数过少应判不可靠", not est.reliable,
          f"却判为可靠（内点 {est.n_inlier}）")

    # ---- C11 运行时预算 ----
    ref = _ref_from(room, (3.0, 1.5, 0.0), mount, rng)
    cur = _pts_from(room, (4.0, 1.5, 0.0), mount, rng)
    t0 = time.monotonic()
    estimate_translation(ref, cur, predicted_dx=1.0, mount_xy=mount)
    dt = time.monotonic() - t0
    check("C11 平移端到端<6s", dt < 6.0, f"耗时 {dt:.2f}s")

    return passed, failures


def _selftest_main():
    print("=" * 66)
    print("calib_common 离线自测（合成场景，无需硬件）")
    print("=" * 66)
    math_fails = _selftest_motion_math()
    total_math = 6
    for item in math_fails:
        print(f"  ✗ {item}")
    print(f"  {'✓' if not math_fails else '✗'} 驱动判据纯函数"
          f"（{total_math - len(math_fails)}/{total_math}）")

    print("\n合成场景估计器用例：")
    passed, failures = _selftest()
    failures = math_fails + failures
    print("\n" + "=" * 66)
    if failures:
        print(f"自测失败：{len(failures)} 项未通过")
        for item in failures:
            print(f"  · {item}")
        return 1
    print(f"自测通过：{passed + total_math - len(math_fails)} 项全部通过")
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest_main())
    print("这是标定工具的共享模块，不是可执行工具。")
    print("用法：python3 calib_common.py --selftest")
    sys.exit(2)
