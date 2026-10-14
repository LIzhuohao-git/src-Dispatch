#!/usr/bin/env bash
#
# originbot_delivery 开机自启动一键安装脚本 (systemd)
#
# 用法:
#   sudo bash scripts/install_autostart.sh                 # 安装并立即启动
#   sudo bash scripts/install_autostart.sh --no-start      # 安装并设置开机自启，暂不启动
#   sudo bash scripts/install_autostart.sh --uninstall     # 停止并卸载自启动
#   sudo bash scripts/install_autostart.sh --dry-run       # 仅预览生成的配置，不实际安装
#   sudo bash scripts/install_autostart.sh -h              # 显示帮助
#
# 可选环境变量（覆盖自动探测结果）:
#   ROS_SETUP  ROS 环境脚本路径，默认 /opt/ros/humble/setup.bash
#   WS_SETUP   工作空间 install/setup.bash 路径，默认自动探测
#   RUN_USER   服务运行用户，默认 root（或以 sudo 调用时的原用户）
#   RUN_HOME   服务运行用户家目录，默认按 RUN_USER 解析

set -euo pipefail

SERVICE_NAME="originbot-delivery"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_TEMPLATE="${SCRIPT_DIR}/originbot-delivery.service"

usage() {
    cat <<'EOF'
用法:
  sudo bash scripts/install_autostart.sh                 # 安装并立即启动
  sudo bash scripts/install_autostart.sh --no-start      # 安装并设置开机自启，暂不启动
  sudo bash scripts/install_autostart.sh --uninstall     # 停止并卸载自启动
  sudo bash scripts/install_autostart.sh --dry-run       # 仅预览生成的配置，不实际安装

可选参数:
  -h, --help     显示帮助

可选环境变量（覆盖自动探测结果）:
  ROS_SETUP  ROS 环境脚本路径，默认 /opt/ros/humble/setup.bash
  WS_SETUP   工作空间 install/setup.bash 路径，默认自动探测
  RUN_USER   服务运行用户，默认 root（或以 sudo 调用时的原用户）
  RUN_HOME   服务运行用户家目录，默认按 RUN_USER 解析
EOF
}

# ---- 参数解析 ----
MODE="install"
START_SERVICE=1
for arg in "$@"; do
    case "${arg}" in
        -h|--help) usage; exit 0 ;;
        --no-start) START_SERVICE=0 ;;
        --uninstall) MODE="uninstall" ;;
        --dry-run) MODE="dry-run" ;;
        *)
            echo "未知参数: ${arg}" >&2
            usage
            exit 1
            ;;
    esac
done

# ---- root 权限检查（dry-run 不需要） ----
if [[ "${EUID}" -ne 0 && "${MODE}" != "dry-run" ]]; then
    if command -v sudo >/dev/null 2>&1; then
        echo "需要 root 权限，正在通过 sudo 重新运行..."
        exec sudo -E bash "$0" "$@"
    else
        echo "错误: 需要 root 权限，请使用 sudo 运行本脚本。" >&2
        exit 1
    fi
fi

if [[ "${MODE}" != "dry-run" ]] && ! command -v systemctl >/dev/null 2>&1; then
    echo "错误: 未找到 systemctl，本系统可能没有 systemd，无法自启动。" >&2
    exit 1
fi

# ---- 卸载模式 ----
if [[ "${MODE}" == "uninstall" ]]; then
    echo "正在停止并移除 ${SERVICE_NAME}..."
    systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
    rm -f "${SERVICE_DST}"
    systemctl daemon-reload
    echo "完成: ${SERVICE_NAME} 已卸载，开机不再自启动。"
    exit 0
fi

# ---- 路径探测 ----
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
WS_SETUP="${WS_SETUP:-}"

if [[ -z "${WS_SETUP}" ]]; then
    # 优先使用当前 ROS 环境实际指向的安装前缀（与手动 ros2 launch 一致）
    if pkg_prefix="$(ros2 pkg prefix originbot_delivery 2>/dev/null)"; then
        WS_SETUP="$(dirname "${pkg_prefix}")/setup.bash"
    fi
fi

if [[ -z "${WS_SETUP}" ]]; then
    # 兜底: 从脚本所在目录向上查找工作空间的 install/setup.bash
    dir="${SCRIPT_DIR}"
    while [[ "${dir}" != "/" ]]; do
        if [[ -f "${dir}/install/setup.bash" ]]; then
            WS_SETUP="${dir}/install/setup.bash"
            break
        fi
        dir="$(dirname "${dir}")"
    done
fi

[[ -f "${ROS_SETUP}" ]] || {
    echo "错误: 找不到 ROS 环境脚本: ${ROS_SETUP}" >&2
    echo "请通过环境变量 ROS_SETUP 指定正确路径。" >&2
    exit 1
}
[[ -f "${WS_SETUP}" ]] || {
    echo "错误: 找不到工作空间环境脚本: ${WS_SETUP}" >&2
    echo "请先构建工作空间，或通过环境变量 WS_SETUP 指定 install/setup.bash 路径。" >&2
    exit 1
}

WORKSPACE="$(dirname "$(dirname "${WS_SETUP}")")"

# 校验 originbot_delivery 已安装（仅警告，便于排查）
launch_file="$(dirname "${WS_SETUP}")/originbot_delivery/share/originbot_delivery/launch/delivery_bringup.launch.py"
[[ -f "${launch_file}" ]] || {
    echo "警告: 未找到 ${launch_file}，请确认已构建安装 originbot_delivery。" >&2
}

# ---- 服务运行用户 ----
RUN_USER="${RUN_USER:-${SUDO_USER:-root}}"
if [[ -z "${RUN_HOME:-}" ]]; then
    RUN_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6 || true)"
fi
RUN_HOME="${RUN_HOME:-/home/${RUN_USER}}"

render_service() {
    sed -e "s|@ROS_SETUP@|${ROS_SETUP}|g" \
        -e "s|@WS_SETUP@|${WS_SETUP}|g" \
        -e "s|@WORKSPACE@|${WORKSPACE}|g" \
        -e "s|@RUN_USER@|${RUN_USER}|g" \
        -e "s|@RUN_HOME@|${RUN_HOME}|g" \
        "${SERVICE_TEMPLATE}"
}

# ---- dry-run 模式 ----
if [[ "${MODE}" == "dry-run" ]]; then
    echo "==== 即将安装的 systemd 服务配置 (${SERVICE_DST}) ===="
    render_service
    echo "======================================================"
    echo "SERVICE_NAME = ${SERVICE_NAME}"
    echo "ROS_SETUP    = ${ROS_SETUP}"
    echo "WS_SETUP     = ${WS_SETUP}"
    echo "WORKSPACE    = ${WORKSPACE}"
    echo "RUN_USER     = ${RUN_USER}"
    echo "RUN_HOME     = ${RUN_HOME}"
    exit 0
fi

# ---- 安装 ----
SERVICE_CONTENT="$(render_service)"

echo "安装服务文件: ${SERVICE_DST}"
printf '%s\n' "${SERVICE_CONTENT}" > "${SERVICE_DST}.tmp"
install -m 0644 "${SERVICE_DST}.tmp" "${SERVICE_DST}"
rm -f "${SERVICE_DST}.tmp"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}" >/dev/null

if pgrep -f "lib/originbot_delivery/delivery_server" >/dev/null 2>&1; then
    echo "警告: 检测到已有 delivery_server 正在运行。" >&2
    echo "      请先手动停止现有的 delivery_bringup，否则服务会因单实例保护而反复重启。" >&2
fi

if [[ "${START_SERVICE}" -eq 1 ]]; then
    systemctl restart "${SERVICE_NAME}"
    echo "完成: ${SERVICE_NAME} 已安装、设为开机自启并启动。"
    echo "查看状态: systemctl status ${SERVICE_NAME}"
    echo "查看日志: journalctl -u ${SERVICE_NAME} -f"
else
    echo "完成: ${SERVICE_NAME} 已安装并设为开机自启（暂未启动）。"
    echo "启动服务: systemctl start ${SERVICE_NAME}"
fi
