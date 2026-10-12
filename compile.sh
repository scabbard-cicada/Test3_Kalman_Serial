#!/usr/bin/env bash
# 编译上位机 ROS 2 工作空间。
#
# 用法：
#   ./compile.sh
#
# 要求当前终端已经 source 对应的 ROS 2 环境：
#   source /opt/ros/jazzy/setup.bash
#   或 source /opt/ros/lyrical/setup.bash
#
# 构建使用 Ninja，并采用 symlink-install，方便修改 launch、YAML 和 Python
# 文件后直接重启而不必重新编译。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "未检测到 ROS_DISTRO。请先 source ROS 2 环境，例如："
  echo "  source /opt/ros/jazzy/setup.bash"
  echo "  source /opt/ros/lyrical/setup.bash"
  exit 1
fi

for tool in colcon cmake ninja; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "缺少工具：$tool"
    case "$tool" in
      colcon) echo "安装：sudo apt install python3-colcon-common-extensions" ;;
      ninja) echo "安装：sudo apt install ninja-build" ;;
      cmake) echo "安装：sudo apt install cmake" ;;
    esac
    exit 1
  fi
done

# 不自动删除用户的构建目录，避免误删其他配置；发现生成器不一致时明确提示。
# colcon 通常把 CMakeCache 放在 build/<package>/，不能只检查根目录。
while IFS= read -r cache; do
  generator="$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' "$cache" | head -n 1)"
  if [[ -n "$generator" && "$generator" != Ninja* ]]; then
    echo "$cache 当前使用的生成器是：$generator"
    echo "本项目要求 Ninja。请确认无须保留旧构建产物后执行："
    echo "  rm -rf build install log"
    exit 1
  fi
done < <(find build -mindepth 2 -maxdepth 2 -name CMakeCache.txt -type f 2>/dev/null)

echo "ROS_DISTRO=$ROS_DISTRO"
echo "Ninja=$(ninja --version)"

colcon build \
  --symlink-install \
  --cmake-args \
    -G Ninja

echo "构建完成：$ROOT"
