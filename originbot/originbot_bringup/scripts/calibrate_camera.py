#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机内参标定脚本（OpenCV 棋盘格）——为 camera_info_relay 生成"真兜底"内参。

背景：
  hobot_usb_cam 只在启动瞬间发布一次 /camera_info，收不到时 camera_info_relay
  只能用占位内参（fx=fy=500）兜底。占位内参会让 aruco_localization 的 PnP
  解算产生系统性偏差（距离越远误差越大），且运行时不报错、极难排查。

本脚本用打印的棋盘格实测内参，把兜底从"猜测值"升级为"标定值"：
  1. 棋盘板贴在硬纸板/亚克力上，必须平整。本工程默认按「7 x 9 内角点、
     方格边长 10mm」的标定板设置（即 8 x 10 个方格，约 80 x 100mm）。
     注意：内角点数 = 方格数 - 1；OpenCV 棋盘检测要求两个方向的角点数都必须 > 2。
  2. 方格边长通过 --square 传入（米），默认 0.01（10mm）；
  3. 机器人静止，手持棋盘在 0.15~0.35m 近距离内多角度、覆盖画面四角与中心采集
     （板面约 80x100mm：0.15m 处棋盘约占画面宽度 1/3，0.35m 处方格仍约 10px
     可检出；需靠近且光线充足、对焦实），采满默认 25 帧后自动求解
     （GUI 中按 q / Esc 可提前结束并求解）；
  4. 结果写入 ~/.originbot_delivery/config/camera_calibration.yaml，
     重启 camera_info_relay（或底盘）后自动作为兜底内参生效。

用法：
  # ROS 模式：订阅 /image（需先在 Web 端点击"启动底盘"）
  ros2 run originbot_bringup calibrate_camera
  # 或不经过 colcon 直接运行：
  python3 scripts/calibrate_camera.py

  # 设备模式：不开 ROS，直接读 USB 摄像头（台架标定用）
  python3 scripts/calibrate_camera.py --device /dev/video0

常用参数：
  --board 9x6        棋盘内角点数（列x行）
  --square 0.025     方格边长（米）
  --frames 25        采集帧数
  --interval 0.8     相邻两帧的最小采集间隔（秒），保证姿态多样
  --no-gui           无显示环境（SSH）时禁用预览窗口
  --output PATH      输出路径（默认见上）

判定标准：
  RMS 重投影误差 < 0.5px 为优；0.5~1.0px 可用；> 1.0px 建议重标
  （棋盘必须平整、对焦实、姿态覆盖四角与多倾角）。
  注意：分辨率必须与运行时一致（hobot_usb_cam 为 640x480），换分辨率必须重标。
"""

import argparse
import datetime
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

DEFAULT_OUTPUT = os.path.join(
    os.path.expanduser("~/.originbot_delivery/config"), "camera_calibration.yaml"
)
MIN_FRAMES = 10  # 少于该帧数解算不稳定，直接判失败
STILLNESS_PX = 3.0  # 相邻帧角点最大位移小于该值视为"停稳"，避免采进运动模糊帧
CHESS_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_FILTER_QUADS
)


def parse_board(text):
    """把 "9x6" 解析成 (9, 6)，即 (每行内角点数, 每列内角点数)。"""
    try:
        cols, rows = text.lower().split("x")
        board = (int(cols), int(rows))
        if board[0] < 3 or board[1] < 3:
            raise ValueError
        return board
    except ValueError:
        raise argparse.ArgumentTypeError(f"棋盘格式应为 列x行（如 9x6），收到：{text}")


def gui_available():
    """OpenCV 的 Qt 后端在无显示环境会直接 abort 整个进程（不走 Python 异常，
    无法用 try/except 兜住），所以进 GUI 模式前必须预检 DISPLAY。"""
    if sys.platform in ("win32", "darwin"):
        return True
    return bool(os.environ.get("DISPLAY"))


def find_corners(frame, board):
    """检测棋盘内角点并做亚像素精化；未找到返回 None。"""
    # mono8（单通道）输入直接用，BGR/RGBA 才需要转灰度
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, board, CHESS_FLAGS)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def frames_from_ros(args):
    """ROS 模式：订阅图像话题生成帧，Ctrl+C 或关闭生成器时安全退出。"""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init(args=None)
    node = rclpy.create_node("camera_calibrator")
    bridge = CvBridge()
    latest = {"frame": None}

    def _on_image(msg):
        try:
            latest["frame"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            print(f"警告：图像转换失败 {exc}", file=sys.stderr)

    # best_effort 订阅可同时匹配 best_effort / reliable 的发布端
    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )
    node.create_subscription(Image, args.image_topic, _on_image, qos)
    print(f"已连接 ROS，等待 {args.image_topic} 图像…")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            frame = latest["frame"]
            latest["frame"] = None
            if frame is None:
                time.sleep(0.01)
                continue
            yield frame
    finally:
        node.destroy_node()
        # Ctrl+C 时 rclpy 的 SIGINT 处理器已 shutdown 过上下文，
        # 再调 rclpy.shutdown() 会抛 RCLError；try_shutdown 幂等安全
        rclpy.try_shutdown()


def frames_from_device(args):
    """设备模式：直接读 USB 摄像头生成帧。"""
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 {args.device}")
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("警告：读帧失败，重试…", file=sys.stderr)
                time.sleep(0.05)
                continue
            yield frame
    finally:
        cap.release()


def per_view_errors(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist):
    """逐帧重投影误差（px），用于定位坏帧。"""
    errors = []
    for obj, img, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist)
        err = cv2.norm(img, proj, cv2.NORM_L2) / math.sqrt(len(proj))
        errors.append(float(err))
    return errors


def board_normal_spread(rvecs):
    """所有帧棋盘法线之间的最大夹角（度），衡量采集姿态多样性。"""
    normals = []
    for rvec in rvecs:
        rot, _ = cv2.Rodrigues(rvec)
        normal = rot[:, 2]  # 棋盘 Z 轴在相机系下的方向
        normals.append(normal / np.linalg.norm(normal))
    normals = np.asarray(normals)
    cos = np.clip(normals @ normals.T, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos).max()))


def degeneracy_reasons(hfov_deg, dist, spread_deg):
    """识别"低 RMS 但参数离谱"的退化标定；返回原因列表，空列表表示正常。

    退化机理：棋盘各帧几乎同样正对相机（板面平行于像平面）时，焦距与拍摄
    距离耦合不可分，解算会用夸张的 fx/畸变组合硬拟合，RMS 反而更低。
    """
    reasons = []
    if hfov_deg < 30 or hfov_deg > 165:
        reasons.append(f"水平视场 {hfov_deg:.0f}° 不合理（USB 相机通常 60~120°）")
    k1, k2 = float(dist[0]), float(dist[1])
    if abs(k1) > 1.5 or abs(k2) > 3.0:
        reasons.append(f"畸变系数离谱：k1={k1:.2f}, k2={k2:.2f}（正常 |k1|<1）")
    if spread_deg < 15.0:
        reasons.append(
            f"棋盘姿态跨度仅 {spread_deg:.0f}°（各帧几乎同样正对相机，"
            "焦距-距离退化）")
    return reasons


def save_calibration(path, board, square, rms, camera_matrix, dist, size, frames):
    """原子写入标定结果（先写 .tmp 再 replace，与工程内其它存储一致）。"""
    data = {
        "source": "opencv_chessboard",
        "board": list(board),
        "square_size": float(square),
        "rms_reproj_error": round(float(rms), 4),
        "image_size": [int(size[0]), int(size[1])],
        "frames_used": int(frames),
        "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(v) for v in camera_matrix.flatten()],
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(dist.flatten()),
            "data": [float(v) for v in dist.flatten()],
        },
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    os.replace(tmp_path, path)


def parse_args():
    parser = argparse.ArgumentParser(
        description="OpenCV 棋盘格相机标定，结果供 camera_info_relay 作兜底内参"
    )
    parser.add_argument("--board", type=parse_board, default=(7, 9),
                        help="棋盘内角点数，格式 列x行（默认 7x9，"
                             "即 8x10 个方格、方格边长 10mm 的标定板）")
    parser.add_argument("--square", type=float, default=0.01,
                        help="方格边长，单位米（默认 0.01）")
    parser.add_argument("--frames", type=int, default=25,
                        help="采集帧数（默认 25）")
    parser.add_argument("--interval", type=float, default=0.8,
                        help="相邻两帧最小采集间隔秒（默认 0.8）")
    parser.add_argument("--image-topic", default="/image",
                        help="ROS 模式图像话题（默认 /image）")
    parser.add_argument("--device", default="",
                        help="设备模式：USB 摄像头路径（如 /dev/video0）；"
                             "不指定则使用 ROS 模式")
    parser.add_argument("--width", type=int, default=640,
                        help="设备模式采集宽度（默认 640，0 表示不改）")
    parser.add_argument("--height", type=int, default=480,
                        help="设备模式采集高度（默认 480，0 表示不改）")
    parser.add_argument("--no-gui", action="store_true",
                        help="禁用预览窗口（SSH 等无显示环境）")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"输出 YAML 路径（默认 {DEFAULT_OUTPUT}）")
    return parser.parse_args()


def main():
    args = parse_args()
    board = tuple(args.board)
    square = args.square

    # 棋盘 3D 点：Z=0 平面，X/Y 按方格边长展开
    objp = np.zeros((board[0] * board[1], 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square

    if args.device:
        source = frames_from_device(args)
        print(f"设备模式：{args.device} ({args.width}x{args.height})")
    else:
        source = frames_from_ros(args)
        print(f"标定棋盘：{board[0]}x{board[1]} 内角点，方格 {square * 1000:.0f}mm，"
              f"目标 {args.frames} 帧")

    gui = not args.no_gui
    if gui and not gui_available():
        print("未检测到显示环境（DISPLAY 为空），自动切换为无预览模式"
              "（采集进度见本终端打印）")
        gui = False
    objpoints, imgpoints = [], []
    frame_size = None
    last_accept = 0.0
    last_hint = 0.0
    prev_corners = None
    count = 0

    try:
        for frame in source:
            if frame_size is None:
                frame_size = (frame.shape[1], frame.shape[0])
                print(f"输入分辨率：{frame_size[0]}x{frame_size[1]}")
            elif (frame.shape[1], frame.shape[0]) != frame_size:
                continue  # 分辨率不一致的帧直接丢弃，calibrateCamera 会因此报错

            now = time.monotonic()
            corners = find_corners(frame, board)
            still = (
                corners is not None
                and (prev_corners is None
                     or float(np.abs(corners - prev_corners).max()) < STILLNESS_PX)
            )
            if corners is not None and now - last_accept >= args.interval:
                if still:
                    last_accept = now
                    objpoints.append(objp.copy())
                    imgpoints.append(corners)
                    count += 1
                    print(f"已采集 {count}/{args.frames}")
                    if count in (8, 16):
                        print("提示：请大幅倾斜棋盘（左右±30°、上下俯仰）并远近交替，"
                              "全程正对会导致标定退化")
                elif now - last_hint >= 2.0:
                    last_hint = now
                    print("棋盘在移动中，该帧已跳过（运动模糊会抬高 RMS）——停稳 1 秒再采")
            prev_corners = corners

            if gui:
                disp = frame.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(disp, board, corners, True)
                cv2.putText(
                    disp, f"{count}/{args.frames}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                )
                try:
                    cv2.imshow("camera calibration", disp)
                    key = cv2.waitKey(30) & 0xFF
                except cv2.error:
                    print("无可用显示环境，切换为无预览模式（同 --no-gui）")
                    gui = False
                    key = 0
                if key in (27, ord("q")):
                    print("手动结束采集")
                    break
            elif count < args.frames:
                time.sleep(0.05)

            if count >= args.frames:
                break
    except KeyboardInterrupt:
        print("\n手动中断（Ctrl+C）")

    if count < MIN_FRAMES:
        print(
            f"错误：仅采集到 {count} 帧（至少 {MIN_FRAMES} 帧）。"
            "请检查棋盘是否清晰入镜、对焦是否正常。",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"开始解算（{count} 帧）…")
    rms, camera_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frame_size, None, None
    )
    errors = per_view_errors(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist)
    worst = int(np.argmax(errors))

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    d = dist.flatten()
    print("\n========== 标定结果 ==========")
    print(f"分辨率      : {frame_size[0]}x{frame_size[1]}")
    print(f"fx, fy      : {fx:.2f}, {fy:.2f}")
    print(f"cx, cy      : {cx:.2f}, {cy:.2f}")
    print(f"畸变 k1..k3 : {d[0]:.5f}, {d[1]:.5f}, {d[4]:.5f}")
    print(f"切向 p1, p2 : {d[2]:.5f}, {d[3]:.5f}")
    print(f"RMS 重投影  : {rms:.3f} px")
    print(f"最差帧      : 第 {worst + 1} 帧，误差 {errors[worst]:.3f} px")
    hfov = 2.0 * math.degrees(math.atan(frame_size[0] / (2.0 * fx)))
    spread = board_normal_spread(rvecs)
    print(f"水平视场    : {hfov:.1f}°")
    print(f"姿态跨度    : {spread:.0f}°（各帧棋盘法线最大夹角）")
    print("==============================")
    if rms > 1.0:
        print("警告：RMS > 1.0px，建议重标（棋盘贴平、覆盖画面四角、多角度倾斜）")
    elif rms > 0.5:
        print("提示：RMS 0.5~1.0px，可用于 ArUco 定位；追求更准可重标一次")

    reasons = degeneracy_reasons(hfov, d, spread)
    if reasons:
        # 退化结果不能写正式配置：夸张的 fx 会让 ArUco 的 PnP 测距系统性偏差
        # （fx 偏大 N 倍 → 测距缩小 N 倍），比占位内参更有害
        suspect_path = args.output + ".suspect"
        save_calibration(suspect_path, board, square, rms, camera_matrix, dist,
                         frame_size, count)
        print("\n!! 检测到退化标定，结果不可信，未写入正式配置 !!")
        for reason in reasons:
            print(f"  - {reason}")
        print("  常见原因：棋盘全程几乎正对相机，焦距与拍摄距离耦合不可分，"
              "解算用夸张的 fx/畸变硬拟合（RMS 反而显得更低）。")
        print(f"  本次结果仅存至 {suspect_path}（camera_info_relay 不会读取）。")
        print("  重采要点：大幅倾斜棋盘（左右±30°、上下俯仰）、远近交替"
              "（0.15~0.35m）、覆盖画面中心与四角。")
        return

    save_calibration(args.output, board, square, rms, camera_matrix, dist,
                     frame_size, count)
    print(f"\n已写入 {args.output}")
    print("重启 camera_info_relay（或底盘）后，兜底内参即使用本次标定值。")


if __name__ == "__main__":
    main()
