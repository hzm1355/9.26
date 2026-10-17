#!/bin/bash
# ============================================================================
# 智能小车算法仿真 —— 独立应用启动脚本
# ============================================================================
# 对应需求（一）ROS2智能小车算法仿真模块 6 项功能：
#   1. 多车型可配置（尺寸/轴距/最高速度/加速度）
#   2. 虚拟传感器仿真（LiDAR/Camera/IMU/Odom），兼容 Nav2
#   3. Nav2 导航算法验证
#   4. 支持导入自定义 ROS2 包到 Gazebo
#   5. 多车同时运行 + 轨迹记录/回放
#   6. 插件化故障模拟
#
# 用法：
#   ./run_simulation.sh              # 启动仿真（默认 5 辆车）
#   ./run_simulation.sh --nav2       # 同时启动 Nav2 导航栈
#   ./run_simulation.sh --demo       # 启动完整演示（多车驾驶模式）
#   ./run_simulation.sh --headless   # 无 GUI 模式
#   ./run_simulation.sh --selftest   # 运行自检（不需要 Gazebo）
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/v2x_ws"
SRC_DIR="$WS_DIR/src"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }
title() { echo -e "\n${CYAN}========== $* ==========${NC}"; }

# ---- 参数解析 ----
MODE="sim"
NAV2="false"
GUI="true"

for arg in "$@"; do
    case "$arg" in
        --nav2)     NAV2="true" ;;
        --demo)     MODE="demo" ;;
        --headless) GUI="false" ;;
        --selftest) MODE="selftest" ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo "  --nav2      同时启动 Nav2 导航栈"
            echo "  --demo      启动完整演示（多车驾驶模式 + 故障模拟）"
            echo "  --headless  无 GUI 模式（仅 gzserver）"
            echo "  --selftest  运行模块自检（不需要 Gazebo）"
            echo "  --help      显示帮助"
            exit 0
            ;;
        *)
            err "未知参数: $arg"
            exit 1
            ;;
    esac
done

# ---- 自检模式 ----
if [ "$MODE" = "selftest" ]; then
    title "智能小车模块自检"
    source "$WS_DIR/install/setup.bash"

    info "1/3 SDF 生成器自检..."
    ros2 run smart_car_sim car_selftest.py && info "PASS" || { err "FAIL"; exit 1; }

    info "2/3 区域触发自检..."
    ros2 run v2x_demo region_trigger_selftest.py && info "PASS" || { err "FAIL"; exit 1; }

    info "3/3 轨迹记录/回放自检..."
    ros2 run smart_car_sim trajectory_selftest.py && info "PASS" || { err "FAIL"; exit 1; }

    title "全部自检通过"
    exit 0
fi

# ---- 环境检查 ----
title "智能小车算法仿真平台"

if [ ! -d "$WS_DIR/install" ]; then
    err "工作空间未构建: $WS_DIR/install 不存在"
    info "请先执行: cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi

source "$WS_DIR/install/setup.bash"

# 检查 VcXsrv（WSLg 不可用时）
if [ -z "$WAYLAND_DISPLAY" ] && [ -z "$DISPLAY" ]; then
    warn "未检测到显示服务器，请确保 VcXsrv 已启动"
    warn "  启动命令: vcxsrv.exe -multiwindow -clipboard -wgl -ac"
fi

# 确保 Gazebo 模型路径包含 TurtleBot3
export GAZEBO_MODEL_PATH="$SRC_DIR/smart_car_sim/models:$SRC_DIR/v2x_demo/models:${GAZEBO_MODEL_PATH:-}"

info "模式: $MODE | GUI: $GUI | Nav2: $NAV2"
echo ""

# ---- 清理残留进程 ----
if pgrep -x gzserver > /dev/null 2>&1; then
    warn "检测到残留 gzserver，正在清理..."
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    sleep 1
fi

# ---- 启动仿真 ----
if [ "$MODE" = "demo" ]; then
    info "启动完整演示..."
    ros2 launch smart_car_sim full_demo.launch.py gui:=$GUI
elif [ "$MODE" = "sim" ]; then
    info "启动智能小车仿真..."
    ros2 launch smart_car_sim smart_car_sim.launch.py gui:=$GUI nav2:=$NAV2
fi
