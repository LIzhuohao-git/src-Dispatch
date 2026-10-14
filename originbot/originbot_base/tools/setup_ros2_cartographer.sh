#!/usr/bin/env bash
#
# RDK X5（TROS 2.x / Ubuntu Jammy / ROS 2 Humble）安装 Cartographer 建图依赖
#
# 背景（2026-09 在 X5 镜像上踩到的两个坑，记录备查）：
#   1. `sudo apt install -y tros-humble-hobot-cartographer` 报：
#        tros-humble-hobot-cartographer : Depends: ros-foxy-cartographer-ros
#        but it is not installable
#      该 deb 是 Foxy 时代打的包（12 KB，只有 ament index / launch / config，
#      不含任何二进制），依赖名在 Jammy 上不存在。真正需要的是标准包
#      ros-humble-cartographer-ros。本方包 fork 自 originbot_navigation，
#      自带 launch/config，因此不需要这个壳包。
#   2. 镜像里 ROS2 仓库的签名密钥已过期（EXPKEYSIG F42ED6FBAB17C654，
#      到期 2025-06-01），`apt-get update` 对 ROS2 源报
#      "The following signatures were invalid"，索引下载失败，
#      于是所有 ros-humble-* 包都查不到。本脚本从镜像的 ros2-apt-source
#      包中取官方刷新后的 keyring（同一把 key，有效期到 2030-06-01）。
#
# 用法：
#   bash setup_ros2_cartographer.sh          # 修密钥 + 装建图依赖
#   bash setup_ros2_cartographer.sh --build  # 额外编译 /userdata/dev_ws 工作空间
#
set -euo pipefail

ROS2_MIRROR=${ROS2_MIRROR:-http://mirrors4.tuna.tsinghua.edu.cn/ros2/ubuntu}
ROS2_SUITE=jammy
ARCH=arm64
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
WORKSPACE=${WORKSPACE:-/userdata/dev_ws}

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }

if [ "$(dpkg --print-architecture)" != "$ARCH" ]; then
  echo "本脚本针对 RDK X5（$ARCH）编写，当前架构为 $(dpkg --print-architecture)" >&2
  exit 1
fi

# 1) ROS2 apt 源（新镜像通常已带，缺失时补上）
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  log "写入 ROS2 源: $ROS2_MIRROR"
  echo "deb [arch=$ARCH signed-by=$KEYRING] $ROS2_MIRROR $ROS2_SUITE main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

# 2) 刷新 ROS2 仓库签名密钥（解决 EXPKEYSIG / "signatures were invalid"）
log "刷新 ROS2 签名密钥"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
curl -fsSL "$ROS2_MIRROR/dists/$ROS2_SUITE/main/binary-$ARCH/Packages.gz" \
  | zcat > "$tmp/Packages"
deb_rel=$(awk '/^Package: ros2-apt-source$/{found=1}
               found && /^Filename:/{print $2; exit}' "$tmp/Packages")
if [ -z "$deb_rel" ]; then
  echo "未能从 $ROS2_MIRROR 找到 ros2-apt-source 包，请检查网络/镜像地址" >&2
  exit 1
fi
curl -fsSL -o "$tmp/ros2-apt-source.deb" "$ROS2_MIRROR/$deb_rel"
dpkg-deb -x "$tmp/ros2-apt-source.deb" "$tmp/extract"
newkey="$tmp/extract/usr/share/keyrings/ros2-archive-keyring.gpg"
[ -s "$newkey" ] || { echo "提取 keyring 失败" >&2; exit 1; }
sudo cp -a "$KEYRING" "$KEYRING.bak.$(date +%Y%m%d%H%M%S)" 2>/dev/null || true
sudo cp "$newkey" "$KEYRING"

# 3) 刷新索引（这一步不再出现 GPG error 才算修好）
log "apt-get update"
sudo apt-get update

# 4) 安装 Cartographer 建图依赖
#    cartographer_ros      提供 cartographer_node / cartographer_occupancy_grid_node
#    robot_localization    提供 ekf_node（originbot_navigation/config/ekf.yaml）
log "安装 ros-humble-cartographer-ros / ros-humble-robot-localization"
sudo apt-get install -y ros-humble-cartographer-ros ros-humble-robot-localization

# 5) 自检
log "自检"
# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash
ros2 pkg prefix cartographer_ros
ros2 pkg prefix robot_localization

# 6) （可选）编译工作空间
if [ "${1:-}" = "--build" ]; then
  log "编译 $WORKSPACE"
  cd "$WORKSPACE"
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
  colcon build --symlink-install
fi

log "完成。验证建图：ros2 launch originbot_navigation cartographer.launch.py"

# 备注：
#   - 不要再用 tros-humble-hobot-cartographer。若确实想装它的 launch/config，
#     只能 sudo dpkg -i --force-depends <deb>（依赖名是错的，但文件能装上）。
#   - 如果需要替代建图算法，ros-humble-slam-toolbox 默认已随镜像安装，
#     Web 端可在 SLAM Toolbox / Cartographer 之间切换。
