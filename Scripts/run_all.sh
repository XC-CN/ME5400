#!/bin/bash
set -e

# 获取项目根目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# 解析参数，主要是为了帮助文档
SEQ_ID="0020"
HEADLESS=false

for arg in "$@"; do
    case $arg in
        -h|--help)
            echo "用法: $0 [选项] [SEQ_ID]"
            echo "选项:"
            echo "  -n, --headless  无可视化模式 (跳过加载 RViz)"
            echo "  -h, --help      显示帮助信息"
            echo ""
            echo "一键执行完整的验证流：先运行纯净 Baseline，再运行带 MCTrack 的优化管线"
            echo "示例:"
            echo "  $0 0020         # 顺序运行序列0020的两套测试并自动生成对比"
            echo "  $0 --headless 0020 # 以 Headless 模式一键测完并出图"
            exit 0
        ;;
        -n|--headless|--no-rviz)
            HEADLESS=true
        ;;
        *)
        if [[ ! $arg =~ ^- ]]; then
            SEQ_ID=$arg
        fi
        ;;
    esac
done

# 统一管理 roscore 的清理
cleanup_all() {
    echo -e "\n[INFO] 正在关闭全局 roscore..."
    if [ ! -z "$GLOBAL_ROSCORE_PID" ] && kill -0 $GLOBAL_ROSCORE_PID 2>/dev/null; then
        kill -9 $GLOBAL_ROSCORE_PID 2>/dev/null || true
    fi
}
trap cleanup_all EXIT

echo "========================================================="
echo "   [ALL] 开始完整验证连跑流: Baseline -> Optimized    "
echo "========================================================="

# 0. 启动全局 roscore
if ! rostopic list > /dev/null 2>&1; then
    echo "[INFO] 启动全局 roscore..."
    roscore &
    GLOBAL_ROSCORE_PID=$!
    sleep 5
else
    echo "[INFO] 检测到外部 roscore 已在运行，将直接复用。"
fi

echo -e "\n>>> [第 1 阶段] 正在运行纯净版基准测试 (run_baseline.sh) ..."
if ! "$SCRIPT_DIR/run_baseline.sh" "$@"; then
    echo "[错误] 基准测试阶段失败！中止后续流程。"
    exit 1
fi

echo -e "\n[INFO] 基准测试执行完毕，准备启动优化验证流 (维持 roscore)..."
sleep 2

echo -e "\n>>> [第 2 阶段] 正在运行深度学习优化管线测试 (run_optimized.sh) ..."
if ! "$SCRIPT_DIR/run_optimized.sh" "$@"; then
    echo "[错误] 优化测试阶段失败！"
    exit 1
fi

echo -e "\n========================================================="
echo "   [ALL] 完整双轨测试流全部执行完成！"
echo "   (定量对比评估结果已在 Results/${SEQ_ID}_results/ 目录下生成)"
echo "========================================================="
exit 0
